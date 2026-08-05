import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import CarControllerParams

VisualAlert = structs.CarControl.HUDControl.VisualAlert
ButtonType = structs.CarState.ButtonEvent.Type
LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)

    self.packer = CANPacker(dbc_names[Bus.pt])

    self.frame = 0
    self.last_steer_frame = 0
    self.last_acc_frame = 0

    self.apply_torque_last = 0

    self.mpc_lkas_counter = 0
    self.mpc_acc_counter = 0
    self.eps_fake318_counter = 0

    self.lkas_req_prepare = 0
    self.lkas_active = 0
    self.lat_safeoff = 0

    self.steer_softstart_limit = 0
    self.steerRateLimActive = False
    self.steerRateLim = 1.0

    # anti-stall protection state
    self.stall_counter = 0
    self.release_counter = 0

    # LOCK3 v6: EPS 请求重握手/退出 (Prepared) 软响应 + 持续超时 full-exit (对齐门总)
    self.eps_prepared_hold = 0     # Prepared 连续=1 的帧数 (v7.2遗留, v9 active分支不再依赖)
    self.lock3_exit_cooldown = 0   # [v7.2遗留, v9 已不用] full-exit 后的冷却帧数
    # LOCK3 v9 (000000b3逐帧逆向): 执行中处于 0xFB(P=1+Cru=1死胡同) 的连续帧数,
    # 超 LOCK3_DEADEND_RELEASE_FRAMES 则放 Act=0 走干净重握手 (复刻seg7成功路径, 见第十一章)
    self.eps_deadend_hold = 0

    # LOCK4: 退出收尾时等 EPS 电机(MainTorque)卸载再松手, 防司机对抗导致 MainTq 滞后锁死
    self.exit_dwell = 0

    # LOCK5: 重接管延迟出力 + 顶不动收回 (对齐门总)
    self.reengage_delay = 0    # 重接管后剩余的"强制0出力"帧数
    self.lkas_active_last = 0  # 上一帧 lkas_active, 用于检测 0->1 重接管沿
    self.stuck_counter = 0     # 大对抗且顶不动的持续帧数
    self.lock5_giveup = False  # 已进入"顶不动收回"放弃状态

    # LOCK6: 低速满扭矩限时封顶 (对齐门总: 允许瞬间满扭矩, 但不许持续死顶)
    self.lock6_hi_counter = 0   # |命令|接近满扭矩的持续帧数
    self.lock6_capped = False   # 是否已进入封顶状态

    # LOCK7 v2: EPS不响应时Out封顶150 (复刻门总闭环, 纯物理判据不看司机对抗)
    self.lock7_stuck_counter = 0  # "EPS不响应(MainTq<10)且Out>150"持续帧数

    self.first_start = True
    self.rfss = 0
    self.sss = 0

    self.apply_accel_last = 0
    
    # 纵向控制优化
    self.speed_hyst_upper = False  # 超速抑制状态


  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    if (self.frame - self.last_steer_frame) >= CarControllerParams.STEER_STEP:

      # Resolve counter mismatch problem
      if self.first_start:
        self.mpc_lkas_counter = int(CS.acc_mpc_state_counter + 1) & 0xF
        self.mpc_acc_counter = int(CS.acc_cmd_counter + 1) & 0xF
        self.eps_fake318_counter = int(CS.eps_state_counter + 1) & 0xF
        self.first_start = False

      apply_torque = 0

      if CC.latActive:
        if self.lkas_active:
          steer_desire = CC.actuators.torque
          self.exit_dwell = 0   # 正在接管出力, 清退出收尾计数, 保证下次退出从0起

          if CarControllerParams.USE_STEERING_SPEED_LIMITER:
            rate_limit = np.interp(CS.out.aEgo, [8.3, 27.8], [132, 64])
            delta_rate = CS.steeringRateDegAbs - rate_limit

            if delta_rate < 0:
              self.steerRateLim -= 0.005 * delta_rate
              if delta_rate < -0.05:
                self.steerRateLimActive = False
              if self.steerRateLim > 1.0:
                self.steerRateLim = 1.0
                self.steerRateLimActive = False
            else:
              if self.steerRateLimActive:
                self.steerRateLim -= 0.005 * delta_rate
              else:
                self.steerRateLim = steer_desire
                self.steerRateLimActive = True
              if self.steerRateLim < 0:
                self.steerRateLim = 0

            new_steer_pu = np.clip(steer_desire, -self.steerRateLim, self.steerRateLim)
          else:
            new_steer_pu = steer_desire

          new_steer = int(round(new_steer_pu * CarControllerParams.STEER_MAX))

          # 低速扭矩封顶: 按车速限制 |扭矩| 上限, 复刻门总 0.98 实测包络,
          # 防止低速大扭矩把方向盘顶到机械限位导致 EPS 过载 TorqueFailed 锁死。
          if CarControllerParams.USE_LOWSPEED_TORQUE_LIMIT:
            tq_ceiling = int(np.interp(CS.out.vEgo,
                                       CarControllerParams.LOWSPEED_TQ_BP,
                                       CarControllerParams.LOWSPEED_TQ_V))
            new_steer = np.clip(new_steer, -tq_ceiling, tq_ceiling)

          if self.steer_softstart_limit < CarControllerParams.STEER_MAX:
            self.steer_softstart_limit = self.steer_softstart_limit + CarControllerParams.STEER_SOFTSTART_STEP
            new_steer = np.clip(new_steer, -self.steer_softstart_limit, self.steer_softstart_limit)

          apply_torque = apply_driver_steer_torque_limits(new_steer, self.apply_torque_last,
                                                          CS.out.steeringTorque, CarControllerParams)

          # 门总接管序列: Act=1 后必须等 EPS CruiseActivated=1 才发扭矩。
          # 在 Cru=0 时发非零扭矩 -> panda 的 steer_req=Active&&CruiseActivated 判为"未接管却
          # 发力" -> 拦截 -> EPS 收到 Active 却收不到扭矩 -> 电机 MainTorque=0 -> 0.7s后
          # TorqueFailed 锁死 (LOCK1 实证: Cru=0 时 OPout 已爬到160, MainTq 全程0)。
          # 故 Cru 未到达前强制扭矩=0 且软起点归零, Cru 到达后从0平滑爬升 (对齐门总)。
          if not CS.eps_cruise_activated:
            apply_torque = 0
            self.steer_softstart_limit = 0
            self.steerRateLimActive = False
            self.steerRateLim = 1.0

          # LOCK5: 重接管"延迟出力 + 顶不动收回" (对齐门总 byd_men_reengage_ramp 实证)
          if CarControllerParams.LOCK5_ENABLE:
            P = CarControllerParams
            # (a) 重接管延迟: lkas_active 0->1 (本帧刚接管) 后, 前 N 帧强制 0 出力,
            #     给 EPS 几帧稳定再出力 (门总大对抗重接管帧+0~+2 出力恒0)。
            if self.lkas_active and not self.lkas_active_last:
              self.reengage_delay = P.LOCK5_REENGAGE_DELAY_FRAMES
              self.stuck_counter = 0
              self.lock5_giveup = False
            if self.reengage_delay > 0:
              self.reengage_delay -= 1
              apply_torque = 0
              self.steer_softstart_limit = 0   # 保证延迟结束后从0慢软起

            # (d) 顶不动收回: 司机大力对抗(|drvTq|大) 且我们命令已出力(|out|>阈值) 但 EPS 电机
            #     (MainTorque)长期顶不上去(<阈值, 说明方向盘被司机压住电机跟不动) -> 判定"顶不动",
            #     持续超 STUCK_FRAMES 帧则把命令收回 0 放弃硬顶 (门总 drv0=-175 样例行为),
            #     避免长时间硬顶触发 TorqueFailed 锁死。待司机松手(drvTq 变小)再解除放弃、重新出力。
            drv_big = abs(int(CS.out.steeringTorque)) >= P.LOCK5_FIGHT_DRV_TQ
            out_big = abs(int(apply_torque)) >= P.LOCK5_STUCK_OUT
            eps_stuck = abs(int(CS.out.steeringTorqueEps)) < P.LOCK5_STUCK_MAINTQ
            if drv_big and out_big and eps_stuck:
              self.stuck_counter += 1
            elif not drv_big:
              self.stuck_counter = max(0, self.stuck_counter - 2)
              if self.stuck_counter == 0:
                self.lock5_giveup = False
            if self.stuck_counter >= P.LOCK5_STUCK_FRAMES:
              self.lock5_giveup = True
            if self.lock5_giveup:
              # 放弃硬顶: 命令按速率收回到0 (不松手退出, 只是不再顶), 待司机松手自然恢复
              apply_torque = apply_driver_steer_torque_limits(0, self.apply_torque_last,
                                                              CS.out.steeringTorque, CarControllerParams)
              print("LOCK5 GIVEUP drv=%d out=%d MainTq=%d stuck=%d -> release (like 门总 -175)" % (
                int(CS.out.steeringTorque), int(apply_torque), int(CS.out.steeringTorqueEps), self.stuck_counter))

          # LOCK6: 低速满扭矩"限时封顶" (对齐门总实测: menacc 22万帧分析)
          # 门总实证: 满扭矩(>=290)持续中位仅 3 帧(0.06s)、最长 31 帧(0.62s), ≥25帧仅1次;
          #   低速(10-20km/h) p99=141~177、max 180~240, 几乎无满扭矩; 极低速(0-10)允许瞬间到300。
          # 段56(我们)相反: 低速无车道线时 -300 持续死顶数秒 -> 激怒 EPS -> 频繁请求退出 -> 临界锁死。
          # 策略(B, 限时而非砍死): 低速时允许瞬间满扭矩(保留起步/大转向力), 但 |命令| 持续处于
          #   高位(>=HI_TORQUE)超过 HI_FRAMES(~0.5s, 门总满扭矩极少超此)时, 进入封顶: 把 |命令|
          #   限到 CAP(~门总低速 p99); 待命令自然降到 HI_TORQUE 以下再解除。只在低速启用(高速不锁)。
          if CarControllerParams.LOCK6_ENABLE and CS.out.vEgo < CarControllerParams.LOCK6_SPEED:
            P = CarControllerParams
            aout = abs(int(apply_torque))
            # 接近满扭矩累计; 命令降到 LO 以下才衰减计数并(计数归0时)解除封顶(滞环防抖)
            if aout >= P.LOCK6_HI:
              self.lock6_hi_counter += 1
            elif aout < P.LOCK6_LO:
              self.lock6_hi_counter = max(0, self.lock6_hi_counter - 2)
              if self.lock6_hi_counter == 0:
                self.lock6_capped = False
            if self.lock6_hi_counter >= P.LOCK6_HOLD_FRAMES:
              self.lock6_capped = True
            if self.lock6_capped:
              # 持续高位太久 -> 回落到低速包络上限, 经速率限制器平滑收敛, 不硬切
              capped = int(np.clip(apply_torque, -P.LOCK6_CEIL, P.LOCK6_CEIL))
              apply_torque = apply_driver_steer_torque_limits(capped, self.apply_torque_last,
                                                              CS.out.steeringTorque, CarControllerParams)
              if self.frame % 25 == 0:
                print("LOCK6 CAP v=%.1f out->%d (hi=%d) 门总低速包络封顶" % (
                  CS.out.vEgo * 3.6, int(apply_torque), self.lock6_hi_counter))

          # LOCK7 v2: EPS不响应时Out封顶150 (复刻门总闭环, 纯物理判据不看司机对抗)
          # 【根因(0000004f/52段)】: MainTq=0(EPS不响应)时我们Out却从16一路爬到234硬灌(开环) ->
          #   Out=234+MainTq=0持续~25帧 -> TorqueFailed锁死。角度才-17°(没打死), 纯"EPS不响应还硬加力"。
          # 【门总铁律(699个MainTq<10连续段, 无例外)】: 门总在EPS不响应时Out最高只到150(中位22),
          #   持续越久压得越低(208帧时Out才13)。即门总"EPS不响应->停止加力->Out压150内->等EPS恢复"。
          # 【判据】: |MainTq|<10(EPS不响应) 且 |Out|>150(超出门总包络) 持续5帧 -> Out封顶到150。
          #   门总正常(MainTq<10但Out<150)不触发; 我们Out冲到234超150才封, 完美区分。经速率限制平滑收敛。
          if CarControllerParams.LOCK7_ENABLE:
            P = CarControllerParams
            eps_stuck = abs(int(CS.out.steeringTorqueEps)) < P.LOCK7_MAINTQ
            out_over = abs(int(apply_torque)) > P.LOCK7_STUCK_CEIL
            if eps_stuck and out_over:
              self.lock7_stuck_counter += 1
            else:
              self.lock7_stuck_counter = max(0, self.lock7_stuck_counter - 1)
            # EPS重新响应(MainTq回升) -> 解除封顶计数
            if abs(int(CS.out.steeringTorqueEps)) > P.LOCK7_RESUME_MAINTQ:
              self.lock7_stuck_counter = 0
            # 持续"EPS不响应且Out>150" -> 把Out封顶到150(复刻门总), 经速率限制平滑收敛
            if self.lock7_stuck_counter >= P.LOCK7_FRAMES:
              capped = int(np.clip(apply_torque, -P.LOCK7_STUCK_CEIL, P.LOCK7_STUCK_CEIL))
              apply_torque = apply_driver_steer_torque_limits(capped, self.apply_torque_last,
                                                              CS.out.steeringTorque, CarControllerParams)
              if self.frame % 10 == 0:
                print("LOCK7 CEIL v=%.1f ang=%.0f out->%d MainTq=%d (EPS不响应, Out封顶150防锁死)" % (
                  CS.out.vEgo * 3.6, CS.out.steeringAngleDeg, int(apply_torque),
                  int(CS.out.steeringTorqueEps)))

          # 无车道线补偿: 已于 20260714 移除 (原按 sign(角度)*120 同向死顶 -> 正反馈锁死 EPS,
          # 详见 values.py LANELESS_ASSIST 注释)。低置信度时保持模型输出(通常≈0), 宁可不出力
          # 也不开环编造扭矩。若日后重做需"衰减模型输出"而非"按角度注入", 且必须经速率限制器。

          # Detect low-speed sustained near-max torque (wheel winding to lock while torque
          # pins at STEER_MAX) and force a brief torque release so the BYD EPS overload
          # timer resets before it asserts TorqueFailed.
          if CarControllerParams.ANTISTALL_ENABLE:
            P = CarControllerParams
            pushing_hard = (CS.out.vEgo < P.ANTISTALL_SPEED and
                            abs(apply_torque) >= P.ANTISTALL_TORQUE)

            if self.release_counter > 0:
              # in release window: hold torque at 0 to let the EPS overload timer reset
              self.release_counter -= 1
              apply_torque = apply_driver_steer_torque_limits(0, self.apply_torque_last,
                                                              CS.out.steeringTorque, CarControllerParams)
              if self.release_counter == 0:
                self.stall_counter = 0
            elif pushing_hard:
              self.stall_counter += 1
              if self.stall_counter >= P.ANTISTALL_TRIGGER_FRAMES:
                # sustained too long: enter release window
                self.release_counter = P.ANTISTALL_RELEASE_FRAMES
            else:
              # torque/speed back to normal: decay the counter
              self.stall_counter = max(0, self.stall_counter - 2)

            # debug: only print while the guard is active to avoid spam
            if self.stall_counter > 0 or self.release_counter > 0:
              print("ANTISTALL v=%.2f tq=%d ang=%.0f | cnt=%d release=%d %s" % (
                CS.out.vEgo, apply_torque, CS.out.steeringAngleDeg,
                self.stall_counter, self.release_counter,
                "<<< RELEASING" if self.release_counter > 0 else ("PUSH?" if pushing_hard else "")))

          # LOCK3 v4: EPS 请求重握手/退出 (Prepared 0->1) -> "软响应"(对齐门总 analyze_menmen_prep_response)
          # 【根因(段29锁死实证)】: 33km/h 正常行驶中 EPS 周期性发 Prepared 请求重握手, 我们若继续
          #   发大扭矩硬顶(Prepared后OPout还从0加到-52), EPS 不满意 -> Prepared 持续25帧不落回 ->
          #   Prepared+MainTq 卡(1,0)0.5s -> TorqueFailed 锁死。
          # 【门总实证】: 门总 Prepared 持续中位3帧、max仅6帧, 从不超7帧; 收扭矩延迟中位0帧(Prepared
          #   时扭矩已≤16); 全量锁死0次。门总不锁死的根本 = 【Prepared 时立即收扭矩不硬顶 -> EPS
          #   满意 -> Prepared 很快落回】, 而非"每次完全退出"(无对抗时70%没退也不锁)。
          # 【v4 软响应】(区别于 v3 一见 Prepared 就完全退出 Active 导致段56断续):
          #   1. Prepared 确认(去抖 PREP_HOLD 帧) -> 按速率把扭矩收到0(每帧-16, 不硬切, 防 LOCK2),
          #      但【保持 lkas_active=1 不完全退出】;
          #   2. Prepared 落回0(短暂事件, 门总≤6帧) -> 退出收尾状态, 下帧起从0慢软起恢复出力(不断续);
          #   3. 仅当 Prepared 持续超 FULL_EXIT 帧(>门总max6, 判定真退出请求) -> 才完全退出 Active。
          if CS.lkas_prepared:
            self.eps_prepared_hold += 1
          else:
            self.eps_prepared_hold = 0

          # LOCK3 v6: SOFT收扭矩(短暂对抗自愈) + 持续超时 full-exit(持续override松手防锁死)
          # 【00000056实证】前4次接管中Prepared(持续12-13帧,DrvTq~100)SOFT收扭矩后司机松手->自愈, 不锁;
          #   第5次(持续25帧,DrvTq234死掰不松)->Out/MainTq归0但Prepared卡1不落回->0.5s后TorqueFailed。
          # v6: SOFT保留(前4次自愈, 不断续); 但Prepared持续超FULL_EXIT(16帧, >13<25)时判定司机【持续
          #   override】-> Act=0完全松手 -> EPS释放不锁死(门总遇持续对抗也是Act=0松手, Prepared max仅6帧)。
          #   退给司机是override本该做的。full-exit后进cooldown(纯递减必到0, 不用会死锁的eps_exit_wait)。
          # LOCK3 v7: SOFT-only (对齐门总 seg16/18/19 原始数据)。
          # 门总遇P=1: 按速率收Out到0(让EPS满意) + 保持Active=1(不撤) -> P=1约9帧内落回 -> 直接恢复出力。
          # 关键: 【不清零 softstart_limit】。v6曾每帧 softstart=0, 导致P=1解除后Out从0慢爬(18帧才到顶),
          #   而门总是Out直接跳回46-69(不softstart)。保留softstart_limit不动, 让恢复靠rate limit快速回,
          #   不额外拖慢。full-exit已在values禁用(FULL_EXIT=9999), 此处保留判断但永不触发。
          # LOCK3 v9 (000000b3逐帧逆向, 第十一章): 执行中(0xFA)遇 P=1 的处理。
          # 关键区分 318 两种 P=1:
          #   0xF9 (P=1, Cru=0)  = 健康的准备态, EPS会自己走向0xFA执行, 【不放手】(收力等它);
          #   0xFB (P=1, Cru=1)  = 死胡同, EPS不会自己走到0xFA, 【收力这么多帧仍不脱离 -> 放Act=0】
          #                        让EPS掉回0xF8, 下次从Cru=0干净两段握手重进(复刻seg7成功路径)。
          # 门总执行中P=1中位3帧、max6帧自落回; 我们超6帧判死胡同放手, 远小于25帧锁死红线。
          if CarControllerParams.LOCK3_ENABLE and CS.lkas_prepared:
            # 收力: SOFT_COLLAPSE_RATE(54, 对齐门总) 快速把Out往0收, 期间保持Act=1等EPS自愈。
            rate = CarControllerParams.LOCK3_SOFT_COLLAPSE_RATE
            last = self.apply_torque_last
            if last > 0:
              apply_torque = max(0, last - rate)
            elif last < 0:
              apply_torque = min(0, last + rate)
            else:
              apply_torque = 0
            self.steerRateLimActive = False
            self.steerRateLim = 1.0
            # 仅 0xFB (P=1 且 Cru=1 死胡同) 才累计并在超时后放手; 0xF9(Cru=0健康准备态)不累计不放手。
            if CS.eps_cruise_activated:
              self.eps_deadend_hold += 1
              if self.eps_deadend_hold >= CarControllerParams.LOCK3_DEADEND_RELEASE_FRAMES:
                self.lkas_active = 0
                self.lkas_req_prepare = 0
                self.steer_softstart_limit = 0
                self.eps_deadend_hold = 0
                print("LOCK3 v9 DEADEND-RELEASE 0xFB held drvTq=%d -> Act=0 干净重握手" % (
                  int(CS.out.steeringTorque)))
            else:
              self.eps_deadend_hold = 0
          else:
            self.eps_deadend_hold = 0

        else:
          # 握手逻辑 (对齐门总, 笔记12/19章验证): 见 EPS Prepared=1 即切 Act=1, 从0软起(每帧+16),
          # 之后 Act 稳定保持直到 Cru=1 (门总 Active 中位维持95帧)。由 LOCK1(Cru未到不发扭矩)+
          # LOCK5(重接管前3帧0出力+慢软起) 保证平滑接管。
          # 注: 已移除 eps_exit_wait 等待逻辑 —— 它是"取消ACC无效/永久失力"的直接原因(LOCK3
          # full-exit 设 True 后, 司机握盘时 Prepared 不落回, 永远解除不了, 且退出分支漏清它)。
          # 门总握手不依赖此等待, 见 Prepared=1 直接接管。
          # LOCK3 v6 冷却: full-exit 松手后, 先冷却几帧不重新接管, 给 EPS/司机稳定
          # (纯递减计数, 必然归0, 不会像 eps_exit_wait 那样永久卡死)。
          # LOCK3 v9 握手状态机 (000000b3逐帧逆向, 第十一章): 以 318 的 P/Cru 双位决定动作,
          # 复刻 seg7 成功路径(空闲Act=0 -> ReqP=1 -> 0xF9(P=1,Cru=0) -> Act=1 -> EPS抬Cru -> 0xFA执行)。
          # 【核心修正】: 只在 0xF9(P=1 且 Cru=0) 干净准备态才置 Act=1;
          #   绝不在 0xFB(P=1 且 Cru=1 死胡同) 挂 Act=1 (那正是seg4卡死的根因)。
          if CS.lkas_prepared and not CS.eps_cruise_activated:
            # 0xF9 干净准备态: EPS已就绪且Cru未抬起 -> 置Act=1, 走向0xFA执行 (seg7成功路径)
            self.lkas_active = 1.0
            self.steerRateLimActive = False
            self.steerRateLim = 1.0
            self.lkas_req_prepare = 0
            self.steer_softstart_limit = 0
            self.eps_deadend_hold = 0
          elif CS.lkas_prepared and CS.eps_cruise_activated:
            # 0xFB 死胡同(P=1+Cru=1但我们未active): 绝不挂Act=1, 放手让EPS掉回0xF8, 下次干净重握手
            self.lkas_active = 0
            self.lkas_req_prepare = 0
            self.steer_softstart_limit = 0
            self.eps_deadend_hold = 0
          elif CS.eps_cruise_activated:
            # 0xFA 执行就绪态(P=0, Cru=1, EPS已授权执行): 直接置Act=1接管 (seg3实证: 从0xFA起Act=1
            # 稳定执行; seg7实证Act=1@0xFA稳2279帧不翻0xFB)。此时Cru已授权, 无需再等Prepared握手。
            self.lkas_active = 1.0
            self.steerRateLimActive = False
            self.steerRateLim = 1.0
            self.lkas_req_prepare = 0
            self.steer_softstart_limit = 0
            self.eps_deadend_hold = 0
          else:
            # 0xF8 空闲(P=0, Cru=0): 保持 Act=0 + 发 ReqPrepare=1 请求准备 (门总/seg7 空闲期均 Act=0)
            self.lkas_active = 0
            self.lkas_req_prepare = 1
            self.eps_deadend_hold = 0




      else:
        # 退出接管 (latActive=0): 门总的退出序列是先把扭矩按下降速率平滑降到接近0, 再松手
        # (Act 1->0)。我们过去在此直接 apply_torque=0 硬清零, 当时若正出满扭矩(-195), EPS
        # 电机实打实出着力, 命令一帧消失 -> 判异常 TorqueFailed 锁死 (LOCK2 实证)。
        # 修复: 若仍在接管且 EPS 巡航仍激活且上帧扭矩还较大, 则保持 lkas_active=1, 用速率限制
        # 把扭矩朝0平滑收敛(每帧≤STEER_DELTA_DOWN); 待扭矩接近0或巡航已退出, 再完全复位握手。
        #
        # LOCK4 (20260702 实证补强): 仅靠"我们的命令降到0"不够。司机用力对抗时 EPS 电机
        # MainTorque 会滞后卡在高位(命令已0, MainTq仍47), 此时松手 Active=0 -> EPS 判"授权撤
        # 销但电机仍出力" -> 锁死。故退出收尾还须等 EPS MainTorque(=steeringTorqueEps)也降到
        # 阈值以下才松手; 命令归0后进入 exit_dwell 挂起等待电机卸载, 超时兜底防无限挂起。
        eps_mt = abs(int(CS.out.steeringTorqueEps))
        cmd_settling = self.lkas_active and CS.eps_cruise_activated and \
            abs(self.apply_torque_last) > CarControllerParams.STEER_DELTA_DOWN
        eps_loaded = (CarControllerParams.LOCK4_ENABLE and self.lkas_active and
                      CS.eps_cruise_activated and eps_mt > CarControllerParams.LOCK4_EPS_RELEASE_TQ and
                      self.exit_dwell < CarControllerParams.LOCK4_EXIT_MAX_FRAMES)
        if cmd_settling or eps_loaded:
          # 保持 active, 命令按速率平滑收敛到0; 若命令已到0但 EPS 电机还在出力, 挂起等待卸载
          apply_torque = apply_driver_steer_torque_limits(0, self.apply_torque_last,
                                                          CS.out.steeringTorque, CarControllerParams)
          self.exit_dwell += 1
          if not cmd_settling and eps_loaded:
            print("LOCK4 EXIT-DWELL cmd=0 wait EPS unload: MainTq=%d drvTq=%d dwell=%d" % (
              eps_mt, int(CS.out.steeringTorque), self.exit_dwell))
        else:
          apply_torque = 0
          self.lkas_req_prepare = 0
          self.steerRateLimActive = False
          self.steerRateLim = 1.0
          self.lkas_active = 0
          self.steer_softstart_limit = 0
          self.stall_counter = 0
          self.release_counter = 0
          self.exit_dwell = 0

      self.apply_torque_last = apply_torque
      # LOCK5: 记录本帧 lkas_active, 供下帧检测 0->1 重接管沿
      self.lkas_active_last = self.lkas_active

      self.mpc_lkas_counter = int(self.mpc_lkas_counter + 1) & 0xF
      self.eps_fake318_counter = int(self.eps_fake318_counter + 1) & 0xF
      self.last_steer_frame = self.frame

      # send steering command, op to esc
      can_sends.append(bydcan.create_steering_control(self.packer, self.CP, CS.cam_lkas,
          self.apply_torque_last, self.lkas_req_prepare, self.lkas_active, CC.hudControl, self.mpc_lkas_counter))

      # Send fake 0x318 (EPS->MPC) to trick MPC into thinking EPS is executing MPC's commands.
      # Without this, MPC detects conflict and may cancel LKAS or generate DTC.
      can_sends.append(bydcan.create_fake_318(self.packer, self.CP, CS.esc_eps,
                                              CS.mpc_laks_output, CS.mpc_laks_reqprepare, CS.mpc_laks_active,
                                              True, self.eps_fake318_counter))

    if (self.frame + 1 - self.last_acc_frame) >= CarControllerParams.ACC_STEP:
      accel = np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

      if CC.longActive:
        # 速度滞环：防止在目标速度附近频繁点刹震荡
        v_cruise = CS.out.cruiseState.speed  # m/s
        v_ego = CS.out.vEgo
        HYSTERESIS = 0.5  # m/s (约1.8km/h 死区)
        
        # 超速检测：真超速(>v_cruise+HYST)才进入减速态
        if v_ego > v_cruise + HYSTERESIS:
          self.speed_hyst_upper = True
        elif v_ego < v_cruise - HYSTERESIS:
          self.speed_hyst_upper = False
        
        # 在超速区间内抑制过激减速，避免频繁刹车泵启动
        if self.speed_hyst_upper and v_cruise < v_ego < v_cruise + HYSTERESIS * 2:
          # 滞环区间：限制减速度，让自然滑行
          accel = max(accel, -0.3)  # 最多轻减速
        
      if CC.longActive:
        stopping = CC.actuators.longControlState == LongCtrlState.stopping
        starting = CC.actuators.longControlState == LongCtrlState.starting
        running = CC.actuators.longControlState == LongCtrlState.pid

        if stopping and accel < -0.1:
            self.rfss = 0
            self.sss = CS.out.standstill

        elif starting and accel > 0.1 and CS.out.vEgo < 0.8:
          self.rfss = CS.out.standstill
          self.sss = 0

        elif running:
          self.rfss = 0
          self.sss = 0

      else:
        accel = 0
        self.sss = 0
        self.rfss = 0

      self.mpc_acc_counter = int(self.mpc_acc_counter + 1) & 0xF
      can_sends.append(bydcan.acc_cmd(self.packer, self.CP, CS.cam_acc, CS.mrr_leading_dist, accel, self.rfss, self.sss, CC.longActive, self.mpc_acc_counter))

      # 接管 813(ACC_HUD_ADAS) + 815(ACC_AEB), 与 814 共用同一套连续 counter (对齐门总 0.98)。
      # 门总实证 (probe_menmen_813_815): 门总把摄像头的 813/814/815 全部拦截重发, 三者用
      # 【同一套】连续 counter (813c==814c==815c, mod16 严格+1递增, 0跳变); 数据体 byte0-5
      # 原样透传摄像头值(AEB/AccState 一个bit不改), 仅重算 counter+checksum。
      # 我们过去只发 814(自己的counter), 813/815 靠 panda 透传摄像头(另一套counter) -> ESC 看到
      # ACC 报文组 counter 不同步 -> 车机 ACC 黄灯报错 + 报错期间纵向失效。故复刻门总: 三报文同
      # counter 重发, 数据全透传, 让 ESC 收到一套自洽连续的 ACC 报文组。
      # AEB 安全: 数据体(AEB_Active/AEB_Decel)每帧取摄像头最新值透传, 仅多一帧(20ms)延迟, 门总已验证。
      can_sends.append(bydcan.create_acc_hud_adas(self.packer, self.CP, CS.cam_adas, self.mpc_acc_counter))
      can_sends.append(bydcan.create_acc_aeb(self.packer, self.CP, CS.cam_aeb, self.mpc_acc_counter))

      self.apply_accel_last = accel
      self.last_acc_frame = self.frame + 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.accel = float(self.apply_accel_last)
    new_actuators.steeringAngleDeg = float(CS.out.steeringAngleDeg)

    self.frame += 1
    return new_actuators, can_sends
