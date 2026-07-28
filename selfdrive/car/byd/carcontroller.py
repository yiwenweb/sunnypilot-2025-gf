#!/usr/bin/env python3
"""
BYD CarController - 纵向控制命令 CAN 发送
将纵向控制器输出的 accel 命令转换为原车 ACC CAN 报文
"""

from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.car import apply_driver_steer_torque_limits
from openpilot.selfdrive.car.byd import bydcan
from openpilot.selfdrive.car.byd.values import CAR, DBC, CANBUS, CarControllerParams
from openpilot.selfdrive.controls.lib.longcontrol_byd import LongitudinalController
from opendbc.can.packer import CANPacker

# 常量
ACCEL_SCALE = 100  # accel -> CAN 缩放因子 (需根据实际DBC调整)
ACCEL_OFFSET = 127  # 偏移量 (无符号字节, 127=0加速度)

class CarController:
    def __init__(self, dbc_name, CP, VM):
        self.CP = CP
        self.VM = VM
        self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
        
        # 纵向控制器
        self.long_control = LongitudinalController(CP)
        
        # CAN 帧计数器
        self.frame = 0
        self.accel_steady = 0.0
        
        # 原车按钮状态追踪
        self.acc_resume_button = False
        self.acc_cancel_button = False
        
        # 调试
        self.last_accel_cmd = 0.0
        
    def update(self, CC, CS, now_nanos):
        """
        主更新 - 每帧调用
        Args:
            CC: CarControl (来自 controlsd)
            CS: CarState
            now_nanos: 当前时间戳
        Returns:
            can_sends: list of CAN messages
        """
        can_sends = []
        
        # 1) 纵向控制更新
        if CC.enabled and CC.longActive:
            # 从 CC 获取前车信息 (modelLeads 或 radarState)
            lead_one = None
            if hasattr(CC, 'modelLeads') and CC.modelLeads:
                lead_one = CC.modelLeads[0]
            elif hasattr(CC, 'radarState') and CC.radarState and CC.radarState.leadOne.status:
                lead_one = CC.radarState.leadOne
            
            # 计算目标加速度
            accel_cmd, stopping, collision_warning = self.long_control.update(
                CS, lead_one, CC.vCruise
            )
            
            # 碰撞预警 HUD
            if collision_warning:
                # 触发仪表警告 (通过 ACC_HUD 报文)
                pass  # TODO: 实现 HUD 警告
            
        else:
            # 未激活: 重置控制器
            accel_cmd = 0.0
            stopping = False
            if self.frame % 100 == 0:  # 每5秒重置一次
                self.long_control.reset()
        
        # 2) 生成 ACC 命令 CAN 报文
        if CC.enabled and CC.longActive:
            # ACC_CMD 报文 (地址需根据实际 DBC 确认, 假设 0x316)
            # 格式: [accel_raw, status, checksum, counter, ...]
            accel_raw = self.accel_to_can(accel_cmd)
            
            values = {
                "ACCEL_CMD": accel_raw,
                "ACC_ACTIVE": 1,
                "ACC_CANCEL_REQ": 0,
                "CHECKSUM": 0,  # TODO: 计算校验和
                "COUNTER": self.frame % 16,
            }
            
            can_sends.append(bydcan.create_acc_command(
                self.packer, CANBUS.pt, accel_raw, CC.enabled, self.frame
            ))
        
        else:
            # 未激活: 发送空闲报文 (保持总线活跃)
            can_sends.append(bydcan.create_acc_idle(
                self.packer, CANBUS.pt, self.frame
            ))
        
        # 3) HUD 信息 (显示设定速度、跟车距离档位等)
        if self.frame % 5 == 0:  # 4Hz
            distance_setting = int(self.long_control.follow_distance)
            comfort_mode = int(self.long_control.comfort_mode)
            
            can_sends.append(bydcan.create_acc_hud(
                self.packer, CANBUS.pt,
                CC.enabled,
                CC.vCruise,
                distance_setting,
                comfort_mode,
                self.frame
            ))
        
        # 4) 按钮处理 (ACC恢复/取消)
        if CS.accButtons:  # 原车按钮状态
            if CS.accButtons == car.CarState.ButtonEvent.Type.accelCruise:
                # RES+ 按钮: 恢复或增加速度
                self.acc_resume_button = True
            elif CS.accButtons == car.CarState.ButtonEvent.Type.cancel:
                # CANCEL 按钮: 取消 ACC
                self.acc_cancel_button = True
        
        self.frame += 1
        self.last_accel_cmd = accel_cmd
        
        return can_sends
    
    def accel_to_can(self, accel: float) -> int:
        """
        将加速度 (m/s²) 转换为 CAN 原始值
        需根据实际 DBC 定义调整 scale/offset
        
        门总实测范围: -2.03 ~ +1.38 m/s²
        假设 CAN: 0-255, 127=0加速度, scale=0.02 m/s²/LSB
        """
        # 示例转换 (需根据实际 DBC 调整)
        accel_limited = clip(accel, -2.5, 2.0)
        raw = int(accel_limited * 50 + 127)  # 50 = 1/0.02
        raw = clip(raw, 0, 255)
        return raw
    
    def can_to_accel(self, raw: int) -> float:
        """CAN 原始值 -> 加速度 (反向转换, 用于解析)"""
        return (raw - 127) / 50.0
