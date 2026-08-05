from dataclasses import dataclass, field
from enum import IntFlag
from opendbc.car import Bus, DbcDict, PlatformConfig, Platforms, CarSpecs
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts, SupportType
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_MAX = 300                 # 门总 0.98 confirmed working max; 897 is rejected by EPS (TorqueFailed)
  # 16->18: 门总23接管段全量实测上升/下降rate=18(p99=max=18, analyze_men_rate.py)。
  # 【必须与panda匹配】panda byd.h max_rate_up/down 已同步改为18; Python发18 <= panda18, 不被拦。
  #   (若panda仍16而Python18 -> panda dist_to_meas_check拦截丢帧, 故两层必须一致改。已同改panda。)
  STEER_DELTA_UP = 18             # =panda max_rate_up(18), 门总实测
  STEER_DELTA_DOWN = 18           # =panda max_rate_down(18), 门总实测
  # 注: LOCK3 SOFT收力用 SOFT_COLLAPSE_RATE=54(>18), 但收力是"向0靠拢"(不越过0), panda收力方向
  #   下限 lowest_allowed=-max_rate_up=-18, 收到0(>=-18)在允许内 -> panda放行, 不受18/帧限制。

  # --- 低速扭矩上限 (默认关闭) ---
  # 历史: 曾以为低速大扭矩持续导致 EPS 锁死, 加了低速封顶。但取证(byd_field_diff)证明
  # 真正根因是 LKAS_Config=3 vs 门总=1 (见 bydcan.py)。门总在低速/对抗/打死方向下满扭矩
  # 也不锁, 说明扭矩大小不是根因。故关闭封顶, 恢复满扭矩力气 (对齐门总"任何情况都有力")。
  USE_LOWSPEED_TORQUE_LIMIT = False
  LOWSPEED_TQ_BP = [0.83, 1.4, 2.8]      # m/s  (≈3, 5, 10 km/h) [保留参数, 未启用]
  LOWSPEED_TQ_V  = [150, 170, STEER_MAX]

  # STEER_DRIVER_ALLOWANCE: 驾驶员反向手力"免费额度"。手力超过它, apply_driver_steer_torque_limits
  # 就把 OP 扭矩上限按 (|DrvTq|-allowance)*MULT 压低。
  # 【20260720 实测门总反推, 68->300】此前 68 是"司机反向对抗型锁死"的总根因:
  #   司机反向掰(DrvTq)超过68后, OP扭矩被渐进压低; 掰到~150+时 OP 被压塌到 0 -> EPS电机MainTq
  #   也塌到 0 -> EPS 看到"active却不出力" -> 发 Prepared -> 卡住不落回 -> TorqueFailed 锁死。
  #   (00000056 实证: DrvTq 120-180 时 OP 塌到~30、Prep占39%; DrvTq>180 时 OP=0、Prep占75%。)
  # 【门总实测 allowance≈294≈300】反解门总 00000006 Seg16/17 高对抗帧(司机掰到-314~-331, OP仍
  #   保持182~241且MainTq紧跟), 反解 allowance 高度集中在 292-298 (n多帧一致):
  #     Seg17 OP=241 DrvTq=-314 MainTq=221 -> allow=314-(300-241)/3≈294
  #     Seg16 OP=200 DrvTq=-331 MainTq=206 -> allow≈298
  #   即门总把 allowance 设成 ~STEER_MAX(300), driver限幅几乎禁用: 司机反向掰到300以内 OP 全力顶
  #   不塌, MainTq 一直跟随, EPS 从不判失效 -> 门总接管中永不发 Prepared -> 永不锁死。
  # 【效果】allowance=300 从源头切断"OP被压塌->MainTq塌->Prepared->锁死"主链:
  #   反向对抗时 OP 不塌(00000056锁死峰值DrvTq=235, allow=300时OP正向上限仍+195>0, 不塌到0)。
  #   手感对齐门总: 时刻全力掌控方向盘, 不因你掰而卸力 = 真正的"人和OP对抗"。
  # 【离手/接管】steeringPressed(DrvTq>59持续5帧) 独立判定"手在方向盘上"(过离手检查、不退出横向);
  #   allowance 不影响它。司机想把车带向自己方向 = 用手力持续对抗, 车会缓慢朝受力方向偏(driver
  #   限幅的让步), 但横向全程不退出 —— 这正是用户要的行为。
  # 【panda】BYD safety 是 TorqueMotorLimited(byd.h), 只查 |OP-MainTq|<=150, 无 driver_torque
  #   限幅字段, 故 allowance 纯 Python 层, panda 无需改/刷。allowance=300 后 OP 不塌、MainTq 跟随,
  #   |OP-MainTq| 偏差反而更小(门总实测<20), 不撞 panda 的 150。
  # 300->68 (门总23段/54414帧全量实证修正, 20260721):
  # 【纠正第50章】第50章反推"门总≈294"错误(只挑极端对抗帧算), 改300导致方向盘死硬(反向要掰到~400
  #   OP才让步)。全量54414帧假设验证: allow=68预测OP误差最小(85), 80=97, 300=222(最差)。门总真实
  #   allowance 在 68-80 区间, 绝非300。且门总OP大小主要由模型/torque控制器决定(接管中OP中位60-100),
  #   不是被allowance削出来的; allowance只在反向大力对抗时轻微压低OP。
  # 【为何现在能安全回68】第50章改300是因当时无LOCK3 v7.2、靠allowance硬顶防锁死。现LOCK3 v7.2
  #   (P=1时收力->3帧撤Active->3帧重握手循环)已实车验证防锁死(00000068/69不再锁), 不再依赖allowance
  #   硬顶。故回归68(=openpilot BYD默认、假设验证最优), 恢复门总式"你一使劲OP就让步"的软手感。
  # 【同向不削弱】公式验证: 同向(你帮OP转)时OP上限被STEER_MAX封顶, 不受allowance影响(全量同向OP能到252)。
  #   左转你也往左打=同向, OP左转不受限。allowance只削反向对抗力。
  STEER_DRIVER_ALLOWANCE = 68     # 门总全量实证68-80区间, 68=默认&假设验证最优; 防锁死靠LOCK3 v7.2
  STEER_DRIVER_MULTIPLIER = 3
  STEER_DRIVER_FACTOR = 1
  STEER_ERROR_MAX = 50            # match 0.98 reference

  STEER_STEP = 2  # 100/2=50hz
  # STEER_SOFTSTART_STEP: 重接管后扭矩上限每帧的爬升量。
  # 历史误判: 曾设 300(1帧到顶,等于禁用软起), 以为门总"立即满扭矩接管"。
  # 但 byd_men_reengage_ramp.py 实测门总重接管后是【慢软起】: 每帧步进≈16, 前几帧甚至为0,
  # 大对抗时 6-8 帧才爬到 ~36。300 的瞬间到顶正是 20260703 大对抗重接管锁死的根因之一。
  # 改用 18 (= STEER_DELTA_UP), 与正常行驶上升速率统一。
  STEER_SOFTSTART_STEP = 18

  ACC_STEP = 2  # 50hz

  ACCEL_MAX = 2.0
  ACCEL_MIN = -3.5

  K_DASHSPEED = 0.072636  # 00000006实测值(中位数,n=29316样本,线性良好,各速度段偏差<0.1%)
                         # 分速度段验证: 10-40km/h=0.072642, 40-70km/h=0.072562
                         # DBC标注0.0735偏高约1.4%。用户反馈车机低3-5km/h已通过实测修正。

  USE_STEERING_SPEED_LIMITER = False

  # --- Anti-stall protection (默认关闭) ---
  # 同上: 真正根因是 LKAS_Config (见 bydcan.py), 非扭矩持续。关闭以恢复满扭矩, 对齐门总。
  # 看门狗会继续监控, 若 Config=1 后仍锁再议。
  ANTISTALL_ENABLE = False
  ANTISTALL_SPEED = 0.83           # m/s (≈3km/h) [保留参数, 未启用]
  ANTISTALL_TORQUE = 140
  ANTISTALL_TRIGGER_FRAMES = 20
  ANTISTALL_RELEASE_FRAMES = 15

  # --- LOCK3: EPS 请求退出横向 (Prepared) -> 快速松手退出 (20260714 已停用, 见下) ---
  # ❌ 已停用 (LOCK3_ENABLE=False)。两轮门总/段56 数据分析证明 LOCK3 的设计前提是错的:
  #  [证据1] 门总 Prepared=1 事件深度分析 (analyze_menmen_prep_deep, 57个事件):
  #    门总遇到 Prepared=1 有 81%(46/57) 【不退出】, 扭矩维持原值继续接管 (最大对抗 drvTq=189
  #    时门总 OPout 全程 63 纹丝不动)。门总退出与否看【司机对抗力度 drvTq】: 退出事件 drvTq_max
  #    中位 156, 未退出事件仅 64。即 Prepared 只是 EPS 的伴随状态位, 不是"退出命令"。
  #  [证据2] 段56 退出时刻分析 (analyze_seg56_exit): 段56 中 lkas_active 掉0 共 49 次, 【全部
  #    49 次 latActive 仍=1】(上层从没要退, latActive 全程 97.4%=1) 且 cru 仍=1(EPS没撤授权)。
  #    即那 49 次横向断续【全是 LOCK3 自作主张退出】造成的(OPout 按16/帧降到0是其退出收尾特征),
  #    与门总(81%不退)完全相反, 正是"低速无车道线横向不连续"的直接根因。
  #  结论: LOCK3"一见Prepared就快速退出"弊大于利。防锁死改由 LOCK6(限时封顶, 从源头不激怒EPS)
  #    + LOCK4(退出等电机卸载) + LOCK5(重接管慢软起) 负责; 横向退出回归 openpilot 标准机制
  #    (上层 latActive / steeringPressed→override 决定, 与门总一致——门总退出也是靠 drvTq 大)。
  # LOCK3 v4 (20260714 重启, 软响应对齐门总): 前版(v3)一见Prepared就"收扭矩+完全退出Active"
  # 造成段56断续, 遂于39章误关(LOCK3_ENABLE=False) -> 段29正常行驶(33km/h)EPS周期性重握手发
  # Prepared, 我们不响应继续硬顶 -> Prepared+MainTq卡(1,0)0.5s -> TorqueFailed锁死。
  # 门总实证(analyze_menmen_prep_response, 57事件, 排除未接管): 门总Prepared持续【中位3帧max6帧】,
  #   从不超7帧就落回; 门总收扭矩延迟中位0帧(Prepared出现时扭矩已≤16); 全量锁死0次。
  #   门总不锁死的根本 = 【Prepared时不硬顶、扭矩收得快 -> EPS满意 -> Prepared很快落回】,
  #   而非"每次退出"(无对抗时70%没完全退出也不锁)。
  # v4软响应: Prepared确认(去抖PREP_HOLD帧)-> 按速率收扭矩到0(每帧-16, 不硬切防LOCK2)但【保持
  #   active不完全退出】; Prepared落回 -> 下帧自动从0慢软起恢复(不断续); 仅当Prepared持续超
  #   FULL_EXIT帧(>门总max6, 判定真退出)才完全退出Active。既解段29锁死(不硬顶)又不致段56断续。
  # ❌❌❌ 20260718 最终停用 (LOCK3_ENABLE=False): can_full_00000047 数据彻底证明 LOCK3 是错的。
  # 【铁证】门总接管中(Act=1,Cru=1) 遇到 Prepared 0->1 事件 = 0 个; 门总 Active=1 持续中位 95帧、
  #   <=2帧的run占0%。我们 Active=1 持续中位仅 1帧、<=2帧的run占 100%(1521个)。
  # 【根因链】LOCK3 把"握手中正常的 Prepared=1"误判为"EPS要退出" -> eps_prepared_hold累加到8 ->
  #   FULL-EXIT: lkas_active=0 + eps_exit_wait=True -> Active被打回0 -> 握手中断 -> EPS再发Prepared
  #   -> 无限振荡; 且 eps_exit_wait 要求 Prepared落回0 才解除, 司机握盘时 Prepared 持续 -> 永久卡死
  #   (失力段 Cru=0 持续252秒, Prep=1时87%的RP=0=eps_exit_wait在卡)。退出分支不清 eps_exit_wait
  #   -> 取消ACC重激活无效 -> 只能离线/在线重建CarController才恢复。
  # --- LOCK3 v5: Prepared=1 时只收扭矩(SOFT), 不退出(FULL-EXIT已移除), 对齐门总 ---
  # 【门总Seg18逐帧实证(20260720)】: 0xFB(Prep=1+Cru=1)出现时 DTq=-160(司机大力) → 门总3帧
  #   内收扭矩到0(Act保持1) → EPS自己退0xF8 → 40-140ms重激活 → 280ms恢复到0xFA。从不锁死。
  # 【我们00000054实证】: 0xFB+OP=-201 → 不收扭矩 → |OP-MTq|>150 → panda safety block →
  #   controlsMismatch → TorqueFailed。根因是Prep时扭矩没收, panda安全层收刀。
  # 【v5修复】SOFT保留(Prep>=2帧按速率平滑收扭矩到0, 保持Act=1, EPS自己决定退不退出);
  #   FULL-EXIT彻底移除(不设Act=0, 不设eps_exit_wait, 不等Prep落回, 不卡重握手)。
  #   门总做法就是LOCK3 SOFT去掉FULL-EXIT, 分毫不差。
  # --- LOCK3 v6 (20260720, 基于 00000056 实证): SOFT收扭矩 + 持续超时 full-exit ---
  # 【00000056 实证(当前代码 e9b948df8 录制)】Config=3 下 EPS 仍会在司机大力反向对抗时抬 Prepared:
  #   本段888帧(17.7s)稳定接管中共5次接管中Prepared事件, 全部由司机大力掰触发(DrvTq max 97~234),
  #   无一由OP出力触发(司机不对抗时低速OP出力EPS跟随率99%, 从不抬Prepared)。
  #   前4次(持续12-13帧, DrvTq~100, 车速7km/h)SOFT收扭矩后司机松手->EPS放回Prepared->恢复, 不锁死。
  #   第5次(持续25帧, DrvTq234死掰不松, 车速14.6km/h)->Out/MainTq都归0但Prepared卡1不落回->0.5s
  #   (~25帧)后TorqueFailed锁死。全段仅此1次锁死。
  # 【根因】LOCK3 v5 SOFT-only 只收扭矩保持Act=1, EPS在等OP松手(Act=0)确认释放; 司机【持续】大力
  #   对抗时Prepared一直=1、EPS等不到释放 -> 自保TorqueFailed。门总遇持续对抗会Act=0松手(所以门总
  #   Prepared max仅6帧就落回), EPS立即释放不锁死。SOFT对"短暂对抗"够用(前4次自愈), 对"持续死掰"无效。
  # 【v6修法】SOFT(短暂对抗自愈, 不断续) + 超时full-exit(持续对抗时Act=0松手, 防锁死):
  #   Prepared持续>=PREP_HOLD(2帧): 按速率收扭矩到0、保持Act=1 (短暂对抗靠此自然恢复, 前4次场景);
  #   Prepared持续>=FULL_EXIT(16帧, >前4次自愈的13帧、<锁死点25帧, 两边都有余量): 判定司机【持续
  #     override】-> Act=0完全松手 -> EPS释放 -> 退出接管(方向交回司机, 这本就是override该做的)。
  #   full-exit后进cooldown(EXIT_COOLDOWN帧, 纯递减计数必到0, 【不用】第46章那个会死锁的eps_exit_wait):
  #   cooldown期间不重新握手接管, 给EPS/司机几帧稳定; cooldown归0后正常握手(见Prepared重新接管)。
  # 【为何不误伤低速正常转弯(用户核心顾虑, 00000056已验证)】: 司机不对抗时低速(10-20km/h)OP出力,
  #   EPS跟随率99%、|OP-MainTq|均值仅5、从不抬Prepared -> 不进本逻辑。Prepared只在司机大力掰时出现,
  #   那是override, 退给司机合理。故日常低速转弯(不主动大力抢方向)顺滑不卡, full-exit只兜"持续死掰"。
  # --- LOCK3 v7 (20260720, 门总seg16/18/19原始数据实证): SOFT-only, 彻底禁用full-exit ---
  # 【门总原始数据铁证(seg16/18/19, 分析脚本analyze_men_seg.py)】:
  #   1) 门总接管中【一样频繁发P=1】(seg16=11次/seg18=11次/seg19=15次), 司机掰得越狠发越多。
  #      => P=1是EPS对"大力对抗"的固有反应, 门总我们都会触发, 无法从316避免(316字段+Out跳变已对比,
  #         静态字段全同, Out门总平滑≤18/帧, 我们P=1前也平滑 -> P=1纯由司机持续对抗触发)。
  #   2) 门总遇P=1的反应 = 【立刻收Out到0(84~100%) + 保持Active=1不撤(74~80%)】:
  #      seg18逐帧: P=1出现下1帧门总Out 54->0, 但A保持1; P=1仅持续~9帧(180ms)门总就恢复Out=46。
  #      门总【收Out让EPS满意 -> P=1很快落回 -> 立刻恢复出力】, 全程Active基本不撤, 不锁死
  #      (seg16/18/19 tqfailed_frames全=0)。
  # 【我们的错误(v6 full-exit)】: v6在P=1持续16帧后 full-exit撤Active -> cooldown不接管 -> 重接管
  #   -> 又被对抗 -> 又P=1 -> 循环, Active段中位仅12帧 = "动一下归零/一卡一卡"。门总根本不撤Active。
  # 【v7修正】: 保留SOFT(收Out到0, 对齐门总, 让EPS满意), 【彻底禁用full-exit】(FULL_EXIT=9999永不触发,
  #   Active全程保持, 靠收Out解除P=1而非撤Active)。allowance=300已让OP不塌, SOFT收Out后EPS会像门总
  #   一样几帧内放回P=1, 不锁死(门总实证tqf=0)。这才是门总真实做法(既非v6撤Active, 也非"完全不理P=1")。
  LOCK3_ENABLE = True          # v7: 开启, 做SOFT收Out(对齐门总), 但下面FULL_EXIT禁用
  LOCK3_PREP_HOLD_FRAMES = 2   # Prepared连续>=此帧才触发SOFT收力(去抖, 滤1帧噪声)。v6->v7改动时曾漏定义
                               # 导致 carcontroller AttributeError 崩溃(controlsd反复崩->safety回落19/一堆报错)
  # v7.2(门总18段逐帧实证, 复刻门总"每步极短时间完成"的反射循环):
  # 【门总真实机制(00000006 seg18逐帧, 用真实时间ms不受丢帧影响)】:
  #   1. Prep=1 -> 立即收扭矩(54/帧), ~28ms到0 (不等待);
  #   2. 扭矩到0稳1-2帧 -> 撤Active (Prep出现后~60ms=3帧);
  #   3. 撤Active -> Prep立即落回0 (~20ms=1帧, 撤Active是解除Prep的直接手段);
  #   4. 条件满足 -> 立即主动重新举Active握手 (撤后~60-80ms), 不被动等;
  #   5. 若司机还在掰 -> 回到步骤1再来一轮 (快速循环~8帧/160ms); 司机停手 -> 恢复正常出力。
  # 【关键】全程"不等待、每步最短时间完成"。之前锁死(00000068/69, Prep持续25帧->EPS超时)是因为
  #   v7保持Active不撤; 之前一卡一卡(v6)是因为cooldown=10太长, 违背门总"立即重握手"。
  # 【阈值3】: 门总Prep出现后~3帧(60ms)撤Active。远小于EPS超时25帧, 稳防锁死; 配合下面cooldown=3
  #   立即重握手, 复刻门总快速循环(不是死等, 不是长cooldown拖慢)。
  LOCK3_FULL_EXIT_FRAMES = 3    # [v7.2遗留, v9已不用] Prep持续>=3帧撤Active
  LOCK3_EXIT_COOLDOWN = 3      # [v7.2遗留, v9已不用] 撤Active后冷却帧数
  # ★★★ LOCK3 v9 (20260804, 000000b3逐帧逆向根因修正, 未上车验证) ★★★
  # 【根因(第十一章)】: 318状态机 门总走 0xF9(P=1,Cru=0)->0xFA执行; 我们卡 0xFB(P=1,Cru=1)死胡同->
  #   0xF8, 永不进0xFA。铁证: 空闲期我们一直挂 Act=1 (v7.2/v8的FULL_EXIT+cooldown循环所致), Cru一抬起
  #   EPS见"Act=1+Cru=1却没做过Prepared握手"->直接给0xFB死胡同。seg7(Act=0空闲)成功进0xFA出力2279帧、
  #   0次0xFB; seg4(Act=1空闲)从没进0xFA、653帧全卡0xFB零出力。
  # 【v9修法(复刻seg7成功路径)】: 非执行态(空闲/死胡同)保持 Act=0 走干净两段握手(ReqP=1->0xF9->Act=1);
  #   遇 0xFB 死胡同立刻放 Act=0, 让EPS掉回0xF8再干净重握手, 绝不在0xFB硬挂Act=1。执行中(0xFA)遇P=1
  #   短暂收力(对齐门总), 仅当0xFB(P=1且Cru=1)持续超此帧数才放手重握(0xF9即P=1&Cru=0健康态不放)。
  LOCK3_DEADEND_RELEASE_FRAMES = 6   # 执行中遇0xFB(P=1+Cru=1)收力这么多帧仍不脱离 -> 放Act=0干净重握手
                                     # (门总执行中P=1≤6帧自落回; 超6帧判死胡同, 远小于25帧锁死红线)
  # LOCK3_SOFT_COLLAPSE_RATE: SOFT收力(P=1时把Out收到0)的每帧下降速率, 【只用于SOFT收力】,
  # 正常行驶下降仍受 STEER_DELTA_DOWN=18 限制。
  # 【门总23段全量实证】: 门总遇P=1需收力时, 单帧下降能到 54~77(中位54), 2帧从64收到0;
  #   而正常行驶下降门总也≤18(收力放宽是特例)。收力快=OP更快停止和司机/EPS对抗=更安全方向。
  # 【安全依据】: 下降(收力/让步)方向快 = OP需要让步时更快松手, 是偏安全的; 上升(出力)保持18不放宽。
  #   门总实测能降63且驾驶平顺 -> EPS/车身受得住此下降速率。取54(门总收力中位)。
  LOCK3_SOFT_COLLAPSE_RATE = 54  # SOFT收力每帧下降上限(对齐门总收力中位54), 仅SOFT收力用, 快速让步

  # --- LOCK4: 退出时等 EPS 电机实际出力(MainTorque)归零再松手 (默认开启) ---
  # 20260702_013051 实证新型锁死 (既非 LOCK1 Cru=0发扭矩, 亦非 LOCK3 Prepared0->1):
  # 司机全程用力对抗方向盘(drvTq -49~-81), 我们接管后 OPtq 快爬到71 与司机对顶, EPS 电机
  # MainTq 也被顶到 47~70。MADS 检测到 override -> latActive 掉0 -> 我们按 LOCK2 把"命令"
  # 平滑降 48->32->16->0(3帧, 这步对的), 但命令到0那帧【同时】把 Active=0 松手, 而此刻 EPS
  # 实际电机 MainTq 仍冻在 47(司机在顶, 电机滞后未跟随命令回落) -> Active 一撤电机还在出力
  # -> EPS 判"授权撤销但电机仍在出力" -> TorqueFailed 锁死。LOCK2 当时能过是因那次 MainTq 跟着
  # 命令一起归0; 本次司机对抗使 MainTq 滞后卡高位, 而退出条件只看"我们的命令"没看"EPS 实际出力"。
  # 修复: 退出收尾保持 Active=1 且命令=0, 直到 EPS MainTorque 也降到阈值以下才干净松手;
  # 设超时上限防止司机持续对抗时无限挂起(超时后仍松手兜底, 此时命令已持续0, MainTq通常已回落)。
  LOCK4_ENABLE = True
  LOCK4_EPS_RELEASE_TQ = 10     # EPS MainTorque 降到 |x|<=此值 视为电机已卸载, 可安全松手
  LOCK4_EXIT_MAX_FRAMES = 30    # 退出收尾最长挂起帧数(~0.6s), 超时强制松手兜底

  # --- LOCK5: 重接管"延迟出力 + 顶不动收回" (对齐门总 0.98, 默认开启) ---
  # byd_men_reengage_ramp.py 实证 (41段, 195次重接管): 门总在 Active 0->1 重新接管后:
  #  (a) 前 3 帧 LKAS_Output 恒=0 (尤其大对抗 |drv|>=60 那 128 次: 帧+0/+1/+2 均值全 0.0),
  #      即先给 EPS 几帧稳定再出力;
  #  (b) 之后每帧 +16 慢软起 (见 STEER_SOFTSTART_STEP);
  #  (c) 大对抗时出力封顶 ~36~44, 不猛涨;
  #  (d) 顶不动就收回: 样例 drv0=-175 out=[0,0,0,16,21,12,0,0,0,0,0,0] —— 门总试出力发现
  #      顶不动(司机扭矩大且 EPS 电机跟不动)立即把命令收回 0 放弃硬顶。
  # 20260703 锁死正因缺此逻辑: 重接管瞬间冲到 -54 持续硬顶司机 -150 对抗 ~1s -> TorqueFailed。
  LOCK5_ENABLE = True
  LOCK5_REENGAGE_DELAY_FRAMES = 3   # 重接管后强制 0 出力的帧数 (门总帧+0~+2 恒0)
  # 顶不动收回: 司机大力对抗 & 我们出力已达一定值但 EPS 电机(MainTorque)顶不上去 且方向盘几乎不动
  # 20260720: allowance=300 后 LOCK5(d) 顶不动收回已多余且有害:
  # - 旧(allow=68): OP被压塌,MainTq也小,MainTq<30说明真被压住 -> 收回有意义
  # - 新(allow=300): OP全力顶,MainTq跟随大,但EPS正常1-3帧滞后时MainTq瞬间<30
  #   + stuck_counter累积 -> 误判"顶不动" -> giveup扭矩归零 -> 一卡一卡的真凶
  # 用 FIGHT_DRV_TQ=9999 把(d)实质关闭(drv_big永远False); LOCK5(a)重接管延迟不受影响,保留。
  LOCK5_FIGHT_DRV_TQ = 9999         # 实质关闭(d)顶不动收回 (allowance=300后此逻辑有害)
  LOCK5_STUCK_FRAMES = 50
  LOCK5_STUCK_OUT = 40
  LOCK5_STUCK_MAINTQ = 30

  # --- LOCK7: 满扭矩顶不动收回 (20260718 新增, 独立于LOCK5的drv对抗判据) ---
  # 【根因(0000004c段7锁死实证)】: 低速5-7km/h路口极限转弯, 方向盘打到454°(接近机械限位),
  #   车轮转不动->EPS电机MainTorque=0(放弃执行), 但模型仍要求满曲率->OP发Out=300硬顶,
  #   Out=300+MainTq=0持续25帧(0.5s)-> EPS过载 TorqueFailed 锁死。
  # 【门总铁律(53段10.9万帧实证)】:
  #   - 门总满扭矩(>=290)最长68帧(1.36s), 但那时 MainTq 一定跟随(EPS真在转);
  #   - 门总使劲(|Out|>=50)时 MainTq<10(顶不动) = 0%(从不出现);
  #   - 即门总【从不出现"满扭矩+MainTq=0"组合】。我们段7这个组合是病态独有。
  # 【与LOCK5区别】: LOCK5需 drv_big(司机对抗>100), 但段7司机扭矩56~162波动没连续>100 -> 没触发。
  #   顶不动(机械限位)跟司机对抗无关, 故LOCK7用纯物理判据: |Out|大 且 MainTq持续≈0 就收, 不看drv。
  # 【判据设计】: 判据含 MainTq<阈值, 所以只拦"顶不动"的满扭矩, 门总正常满扭矩(MainTq跟随)永不触发,
  #   完美区分。N=12帧(0.24s)远短于锁死点25帧, 又不误伤(正常满扭矩MainTq>0根本不进此判据)。
  # 【20260718 v2 重大修正: 从"满扭矩顶不动收0"改为"EPS不响应时Out封顶150"(复刻门总闭环)】
  # 【门总铁律(53段699个'接管中MainTq<10'连续段实证, 无一例外)】:
  #   门总在 MainTq<10(EPS不响应)持续期间, Out最高只到150, 中位仅22; 且持续越久Out压得越低
  #   (208帧4.16s时Out才13, 120帧时26)。即门总在EPS不响应时【停止加力, Out压在150内, 等EPS恢复】。
  # 【我们病态(0000004f/52段实证)】: MainTq=0持续时, Out却从16一路线性爬到234硬灌(开环, 无视
  #   EPS是否响应) -> Out=234+MainTq=0持续~25帧 -> TorqueFailed锁死。角度才-17°(没打死方向),
  #   纯粹是"EPS不响应还硬加力"。
  # 【v1(收到0)为何不够】: v1判据 Out>=260 才触发, 但段52 Out最高234没到260 -> 没触发。且门总
  #   MainTq<10时Out<150是常态(699段), 收到0会误伤; 正确做法是【封顶到150】(门总实测max)而非收0。
  # 【v2判据】: MainTq<10(EPS不响应) 且 |Out|>150(超出门总包络) 持续5帧 -> 把|Out|封顶到150,
  #   经速率限制器平滑收敛。门总正常(MainTq<10但Out<150)不触发, 完美区分。等MainTq回升自动放开。
  LOCK7_ENABLE = True
  LOCK7_MAINTQ = 10            # |MainTorque| < 此值 = EPS未响应(门总此时Out也压得很低)
  LOCK7_STUCK_CEIL = 150       # EPS不响应时 |Out| 封顶值 (门总699段实测max=150, 中位仅22)
  LOCK7_FRAMES = 5             # "EPS不响应且Out>150"持续超此帧 -> 封顶 (超出门总包络即干预)
  LOCK7_RESUME_MAINTQ = 20     # MainTq 回升到 >此值 视为EPS重新响应, 解除封顶

  # --- 无车道线辅助 (20260713 新增 -> 20260714 停用) ---
  # ❌ 已停用并从 carcontroller 移除。原实现在低速转弯(角度>15°)时按
  #    hold_torque = sign(steer_angle) * 120 直接赋值(绕过速率限制/softstart), 与方向盘偏角
  #    【同向】输出大扭矩 = 正反馈自锁: 角度越大越同向顶 -> 顶到机械限位死压 120Nm -> EPS 过载
  #    TorqueFailed 锁死(需断电重启)。且与 LOCK5"顶不动收回"互相触发 0.3~0.5s 震荡, 造成低速
  #    控制不连续。违背笔记28~30章结论(低置信度应"衰减输出"而非"按角度开环编造扭矩")。
  # 若将来要做无车道线辅助: 必须是"衰减/限幅模型输出", 经 apply_driver_steer_torque_limits,
  #    且方向为反向小回中, 角度大时减小而非增大力度。详见笔记。
  LANELESS_ASSIST_ENABLE = False    # 永久停用危险实现
  LANELESS_ASSIST_SPEED = 5.5       # [保留参数, 未启用]

  # --- LOCK6: 低速满扭矩"限时封顶" (对齐门总 0.98 实测包络, 默认开启) ---
  # 门总 22万帧实证 (analyze_menmen_deep):
  #  [1] 满扭矩(|out|>=290)连续段中位仅 3 帧(0.06s), 最长 31 帧(0.62s), >=25帧只 1 次
  #      -> 门总允许【瞬间】满扭矩(起步/路口大转向需要), 但几乎从不持续顶 >0.5s。
  #  [2] 低速包络: 10-15km/h max=240 p99=177; 15-20km/h max=180 p99=141 (无满扭矩帧);
  #      0-10km/h 才允许到 300(短暂)。
  #  段56锁死: 我们低速(7km/h)持续 -300 死顶数秒 -> EPS 过载 TorqueFailed。
  # 策略(B, 限时而非砍死上限): 低速时允许瞬间满扭矩, 但 |命令| 持续接近满(>=HI)超过
  #  HOLD_FRAMES 帧, 就把上限回落到 CEIL, 直到 |命令| 自然降到 <LO 才解除封顶。
  #  既保留门总式瞬间大扭矩, 又杜绝段56式持续死顶。仅低速启用(高速本不锁)。
  LOCK6_ENABLE = True
  LOCK6_SPEED = 5.5            # m/s (≈20km/h) 低于此速度才启用限时封顶 (门总低速包络区)
  LOCK6_HI = 260              # |命令|>=此值视为"接近满扭矩"(门总低速 p99~177, 260 留余量给瞬间尖峰)
  LOCK6_LO = 200             # 封顶后 |命令| 自然降到 <此值 才解除封顶
  LOCK6_HOLD_FRAMES = 25     # 接近满扭矩持续超此帧数(~0.5s, 门总最长满扭矩0.62s) -> 触发回落
  LOCK6_CEIL = 200           # 触发后的扭矩上限 (门总 10-20km/h max 180~240 的中段)

  # op long control
  K_accel_jerk_upper = 0.1
  K_accel_jerk_lower = 0.5
  K_jerk_xp =            [   4,   10,   20,   40,   80]
  K_jerk_base_lower_fp = [-2.3, -1.8, -1.4, -1.0, -0.4]
  K_jerk_base_upper_fp = [ 0.8,  0.7,  0.6,  0.3,  0.2]
  
  # 跟车距离档位映射（原车4档，对应813 SetDistance值 1-4）
  # time_gap单位：秒，公式 safe_distance = v_ego * time_gap + 5m
  # 实测标定建议：原车ACC各档位实车跟车测量真实距离反推
  GAP_TIME_TABLE = {
    1: 1.0,   # 近距离
    2: 1.4,   # 中近
    3: 1.8,   # 中远（默认）
    4: 2.3,   # 远距离
  }

  def __init__(self, CP):
    pass


class BydSafetyFlags(IntFlag):
  HAN_TANG_DMEV = 0x1


@dataclass
class BydCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))
  support_type: SupportType = SupportType.COMMUNITY


@dataclass
class BydPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "byd_tang_dm_2018"})


class CAR(Platforms):
  BYD_TANG_DM = BydPlatformConfig(
    [BydCarDocs("BYD TANG DM")],
    CarSpecs(mass=2250., wheelbase=2.820, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )


class LKASConfig:
  DISABLE = 0
  ALARM = 1
  LKA = 2
  ALARM_AND_LKA = 3


class CanBus:
  ESC = 0
  MRR = 1
  MPC = 2


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=CanBus.ESC,
    ),
  ],
)

PLATFORM_HANTANG_DMEV = {CAR.BYD_TANG_DM}

MPC_ACC_CAR = {CAR.BYD_TANG_DM}
PT_RADAR_CAR = {CAR.BYD_TANG_DM}
TORQUE_LAT_CAR = {CAR.BYD_TANG_DM}
EXP_LONG_CAR = {CAR.BYD_TANG_DM}

DBC = CAR.create_dbc_map()
