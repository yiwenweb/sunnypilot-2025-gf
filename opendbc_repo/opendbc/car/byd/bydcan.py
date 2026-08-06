import numpy as np
from opendbc.car import structs
from opendbc.car.byd.values import CanBus, CarControllerParams

GearShifter = structs.CarState.GearShifter
VisualAlert = structs.CarControl.HUDControl.VisualAlert


def byd_checksum(dat):
    # Nibble-based algorithm verified from earlier BYD openpilot adaptations (byte_key=0xAF)
    byte_key = 0xAF
    first_bytes_sum = sum(byte >> 4 for byte in dat)
    second_bytes_sum = sum(byte & 0xF for byte in dat)
    remainder = second_bytes_sum >> 4
    second_bytes_sum += byte_key >> 4
    first_bytes_sum += byte_key & 0xF
    first_part = ((-first_bytes_sum + 0x9) & 0xF)
    second_part = ((-second_bytes_sum + 0x9) & 0xF)
    return (((first_part + (-remainder + 5)) << 4) + second_part) & 0xFF


# MPC -> Panda -> EPS
def create_steering_control(packer, CP, cam_msg: dict, req_torque, req_prepare, active, hud_control, counter):
    values = {s: cam_msg[s] for s in [
        "AutoFullBeamState",
        "LeftLaneState",
        "LKAS_Config",
        "SETME2_0x1",
        "MPC_State",
        "AutoFullBeam_OnOff",
        "LKAS_Output",
        "LKAS_Active",
        "SETME3_0x0",
        "TrafficSignRecognition_OnOff",
        "SETME4_0x0",
        "SETME5_0x1",
        "RightLaneState",
        "LKAS_State",
        "TrafficSignRecognition_Result",
        "LKAS_AlarmType",
        "SETME7_0x3",
    ]}

    values["ReqHandsOnSteeringWheel"] = 0
    values["LKAS_ReqPrepare"] = req_prepare
    values["Counter"] = counter

    # 强制 LKAS_Config=3 (ALARM_AND_LKA), 对齐门总实测。
    # ★★★ 20260718 重大更正: 之前(第39章)错误地设为1, 并写"门总恒发1"——【实测数据推翻】。
    # 【铁证】verify_lkas_config.py 解码门总7个bus全部日志:
    #   门总接管(Active=1)时 LKAS_Config 恒定=3 (ALARM_AND_LKA), 8336帧无一例外;
    #   我们(旧)恒发1(ALARM), 摄像头原始src=2也是1。
    # 【DBC】LKAS_Config: 0=禁用 1=报警 2=LKA 3=报警+LKA (byte0 bit6-7, 即 (byte0>>6)&0x3)。
    # 【这是所有EPS问题的总根因】: Config=1(仅报警,无LKA位) -> EPS认为"这只是报警系统不是正经
    #   LKA" -> 不完全信任横向 -> 接管中周期性发 Prepared=1 要求重新确认 -> 我们无论响应(收扭矩->
    #   死锁,46章)还是硬顶(->TorqueFailed锁死,49章)都出问题。
    # 【门总Config=3的效果】: EPS认为"正经LKA系统"无条件信任 -> 接管中【永不发Prepared】
    #   (门总Cru=1的8314帧, Prepared=0次!) -> Active稳定维持(中位95帧) -> 永不锁死。
    # 【历史教训】: LOCK1~6、LOCK3 v1-v4 折腾数月, 都在治"收到Prepared怎么办"的症状; 真正根因是
    #   Config发错让EPS不信任才发Prepared。改Config=3后EPS不发Prepared, LOCK3彻底不需要(已停用)。
    values["LKAS_Config"] = 3

    if active:
        values.update({
            "LKAS_Output": req_torque,
            "LKAS_Active": 1,
            "LKAS_State": 7,  # 门总 0.98 sends LKAS_State=7 constantly (confirmed: 3000/3000 frames). Previously 2.
            "LeftLaneState": 3 if hud_control.leftLaneDepart else int(hud_control.leftLaneVisible) + 1,
            "RightLaneState": 3 if hud_control.rightLaneDepart else int(hud_control.rightLaneVisible) + 1,
        })
    else:
        values.update({
            "LKAS_Output": 0,
            "LKAS_Active": 0,
            "LKAS_State": 7,  # 门总 keeps State=7 even when inactive (during OP operation)
        })

    data = packer.make_can_msg("ACC_MPC_STATE", CanBus.ESC, values)[1]
    values["CheckSum"] = byd_checksum(data)
    return packer.make_can_msg("ACC_MPC_STATE", CanBus.ESC, values)


# op long control
def acc_cmd(packer, CP, cam_msg: dict, mrr_leaddist, accel, rfss, sss, longActive, counter):
    values = {s: cam_msg[s] for s in [
        "AccelCmd",
        "ComfortBandUpper",
        "ComfortBandLower",
        "JerkUpperLimit",
        "SETME1_0x1",
        "JerkLowerLimit",
        "ResumeFromStandstill",
        "StandstillState",
        "BrakeBehaviour",
        "AccReqNotStandstill",
        "AccControlActive",
        "AccOverrideOrStandstill",
        "EspBehaviour",
        "SETME2_0xF",
    ]}

    jerk_base_upper = np.interp(mrr_leaddist, CarControllerParams.K_jerk_xp, CarControllerParams.K_jerk_base_upper_fp)
    jerk_base_lower = np.interp(mrr_leaddist, CarControllerParams.K_jerk_xp, CarControllerParams.K_jerk_base_lower_fp)

    if accel < 0:
        jerk_upper = jerk_base_upper
        jerk_lower = jerk_base_lower + accel * CarControllerParams.K_accel_jerk_lower
    else:
        jerk_upper = jerk_base_upper + accel * CarControllerParams.K_accel_jerk_upper
        jerk_lower = jerk_base_lower

    values["Counter"] = counter
    
    if longActive:
        values.update({
            "AccelCmd": accel,
            "ComfortBandUpper": 0,  # raw 100; 门总实证恒0 (99.4%), 与我方一致
            "ComfortBandLower": 0,  # raw 100; 门总实证恒0 (99.4%)
            "JerkUpperLimit": jerk_upper,
            "JerkLowerLimit": jerk_lower,
            "ResumeFromStandstill": rfss,
            "StandstillState": sss,
        })

    data = packer.make_can_msg("ACC_CMD", CanBus.ESC, values)[1]
    values["CheckSum"] = byd_checksum(data)
    return packer.make_can_msg("ACC_CMD", CanBus.ESC, values)


# 接管 813 (ACC_HUD_ADAS): 透传摄像头数据体, 只重编 Counter + 重算 checksum。
# 门总 0.98 实证 (probe_menmen_813_815): 门总 src=128 的 813 与摄像头 src=2 的 byte0-5
# 完全相同(数据体不改), 仅 byte6(Counter)/byte7(CheckSum) 不同; 且 813/814/815 用同一套
# 连续 counter (813c-814c=0, 全程 mod16 +1 无跳变)。车机检测 ACC 报文组 counter 连续性,
# 我们过去只发 814(自己counter)、813/815 透传摄像头(另一套counter) -> 三者不同步 -> 车机
# 判 ACC 报文组不健康 -> 黄灯报错 + 纵向失效。故接管 813/815, 与 814 共用连续 counter。
def create_acc_hud_adas(packer, CP, cam_msg: dict, counter):
    values = {s: cam_msg[s] for s in [
        "SetSpeed",
        "HasLead",
        "SetDistance",
        "LeadingDistance",
        "AEB",
        "FCW",
        "SETME1_0x1",
        "AccState",
        "AccOn1",
        "CloseWarning",
        "SETME2_0x1",
        "Notify",
        "Status",
        "SETME3_0xFFF",
        "SETME4_0xF",
    ]}
    values["Counter"] = counter

    data = packer.make_can_msg("ACC_HUD_ADAS", CanBus.ESC, values)[1]
    values["CheckSum"] = byd_checksum(data)
    return packer.make_can_msg("ACC_HUD_ADAS", CanBus.ESC, values)


# 接管 815 (ACC_AEB): 同 813, 纯数据透传 + 重编 counter/checksum。
# 门总 src=128 的 815 与摄像头 src=2 byte0-5 完全相同(0580020fffff), AEB 指令原样转发到 ESP,
# 仅晚一帧(20ms), 门总已实车验证 AEB 功能正常。数据体不改, 不影响 AEB/FCW 安全功能。
def create_acc_aeb(packer, CP, cam_msg: dict, counter):
    values = {s: cam_msg[s] for s in [
        "AEB_Active",
        "AEB_Decel",
        "SETME_0xF",
    ]}
    values["Counter"] = counter

    data = packer.make_can_msg("ACC_AEB", CanBus.ESC, values)[1]
    values["CheckSum"] = byd_checksum(data)
    return packer.make_can_msg("ACC_AEB", CanBus.ESC, values)


# send fake torque feedback from eps to trick MPC
# 门总 0.98 behaviour (confirmed from rlog, src=130 vs src=0): the fake 318 sent to the
# MPC is a PURE PASS-THROUGH of the real EPS 318 - LKAS_Prepared, CruiseActivated and
# MainTorque all match the real EPS frame-for-frame (3002/3002). Only the Counter is
# re-stamped (and checksum recomputed). Previously we overrode these fields with fabricated
# values (Prepared=0/Cruise=1/MainTorque=mpc_output), which diverged from the real EPS state
# and could make the MPC detect a conflict and cancel LKAS.
def create_fake_318(packer, CP, esc_msg: dict, faketorque, laks_reqprepare, laks_active, enabled, counter):
    values = {s: esc_msg[s] for s in [
        "LKAS_Prepared",
        "CruiseActivated",
        "TorqueFailed",
        "SETME1_0x1",
        "SteerWarning",
        "SteerErrorCode",
        "MainTorque",
        "SETME3_0x1",
        "SETME4_0x3",
        "SteerDriverTorque",
        "SETME5_0xFF",
        "SETME6_0xFFF",
    ]}

    # Pass through real EPS state unchanged, including the real EPS counter.
    # 门总 0.98: fake counter == real EPS counter frame-for-frame (3002/3002), i.e. the
    # fake 318 is a byte-faithful relay of the real EPS 318 onto the MPC bus. Using our own
    # counter here would desync from the real EPS stream the MPC also partially sees.
    values["ReportHandsNotOnSteeringWheel"] = 0
    values["Counter"] = esc_msg["Counter"]

    data = packer.make_can_msg("ACC_EPS_STATE", CanBus.MPC, values)[1]
    values["CheckSum"] = byd_checksum(data)
    return packer.make_can_msg("ACC_EPS_STATE", CanBus.MPC, values)
