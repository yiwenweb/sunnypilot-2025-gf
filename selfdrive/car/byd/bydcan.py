#!/usr/bin/env python3
"""
BYD CAN 报文构建 - ACC 纵向控制相关报文
基于门总实测逆向 + 常见原车 ACC 协议
"""

from openpilot.selfdrive.car.byd.values import CANBUS

def create_acc_command(packer, bus, accel_raw, enabled, frame):
    """
    ACC 加速度命令报文
    
    地址: 0x316 (790) - 需根据实际 DBC 确认
    频率: 20Hz
    格式 (假设):
      byte0: ACCEL_CMD (0-255, 127=0)
      byte1: STATUS (bit0=enabled, bit1=active, ...)
      byte2-3: RESERVED
      byte4: COUNTER (0-15)
      byte5: CHECKSUM
    
    TODO: 根据实际逆向的 DBC 更新字段定义
    """
    values = {
        "ACCEL_CMD": accel_raw,
        "ACC_ENABLED": 1 if enabled else 0,
        "ACC_ACTIVE": 1 if enabled else 0,
        "COUNTER": frame % 16,
    }
    
    # 计算校验和 (需根据原车协议)
    # 示例: 简单异或校验
    checksum = 0
    for v in [accel_raw, int(enabled), frame % 16]:
        checksum ^= v
    values["CHECKSUM"] = checksum & 0xFF
    
    return packer.make_can_msg("ACC_CMD", bus, values)


def create_acc_idle(packer, bus, frame):
    """
    ACC 空闲报文 (未激活时发送, 保持总线活跃)
    """
    values = {
        "ACCEL_CMD": 127,  # 0加速度
        "ACC_ENABLED": 0,
        "ACC_ACTIVE": 0,
        "COUNTER": frame % 16,
        "CHECKSUM": 0x7F,  # 固定校验和
    }
    return packer.make_can_msg("ACC_CMD", bus, values)


def create_acc_hud(packer, bus, enabled, v_cruise_kph, distance_setting, comfort_mode, frame):
    """
    ACC HUD 信息报文 (显示到仪表)
    
    地址: 0x32E (814) - 需确认
    频率: 4Hz
    内容:
      - 设定速度
      - 跟车距离档位 (1-4档指示灯)
      - 舒适模式 (ECO/COMFORT/SPORT)
      - ACC 状态图标
    """
    values = {
        "ACC_SET_SPEED": int(v_cruise_kph),
        "ACC_DISTANCE_SETTING": distance_setting,  # 1-4
        "ACC_COMFORT_MODE": comfort_mode,          # 0-2
        "ACC_ICON_ACTIVE": 1 if enabled else 0,
        "COUNTER": (frame // 5) % 16,  # 4Hz
    }
    
    # 校验和
    checksum = sum(values.values()) & 0xFF
    values["CHECKSUM"] = checksum
    
    return packer.make_can_msg("ACC_HUD", bus, values)


def create_acc_cancel(packer, bus, frame):
    """
    ACC 取消命令 (用户按 CANCEL 或系统主动取消)
    """
    values = {
        "ACC_CANCEL_REQ": 1,
        "COUNTER": frame % 16,
    }
    return packer.make_can_msg("ACC_CANCEL", bus, values)


def create_collision_warning(packer, bus, warning_level, frame):
    """
    碰撞预警报文 (FCW)
    
    Args:
        warning_level: 0=无, 1=警告, 2=紧急
    """
    values = {
        "FCW_WARNING_LEVEL": warning_level,
        "FCW_ACTIVE": 1 if warning_level > 0 else 0,
        "FCW_DISTANCE": 0,  # TODO: 填入实际距离
        "COUNTER": frame % 16,
    }
    return packer.make_can_msg("FCW_WARNING", bus, values)


# ============ 辅助函数 ============

def calculate_byd_checksum(data: bytes, msg_id: int) -> int:
    """
    BYD CAN 校验和计算 (需根据实际协议逆向)
    
    常见算法:
      1. XOR 校验: data[0] ^ data[1] ^ ... ^ msg_id
      2. SUM 校验: (sum(data) + msg_id) & 0xFF
      3. CRC-8
    
    TODO: 用实际总线抓包验证
    """
    # 示例: 简单 XOR
    checksum = msg_id & 0xFF
    for b in data:
        checksum ^= b
    return checksum


def counter_match(expected: int, actual: int) -> bool:
    """验证 counter 是否匹配 (允许少量丢包)"""
    diff = (actual - expected) % 16
    return diff <= 2  # 允许丢2帧


# ============ 解析函数 (用于读取原车报文) ============

def parse_acc_status(data: bytes) -> dict:
    """
    解析原车 ACC 状态报文 (用于监控原车系统)
    
    返回:
      {
        'enabled': bool,
        'active': bool,
        'set_speed': float (km/h),
        'distance_setting': int (1-4),
      }
    """
    # TODO: 根据实际 DBC 实现
    return {
        'enabled': (data[1] & 0x01) != 0,
        'active': (data[1] & 0x02) != 0,
        'set_speed': data[2],
        'distance_setting': (data[3] & 0x03) + 1,
    }


def parse_driver_buttons(data: bytes) -> dict:
    """
    解析方向盘按钮 (RES+, SET-, CANCEL, DISTANCE等)
    
    返回:
      {
        'resume': bool,
        'set': bool,
        'cancel': bool,
        'distance': bool,
        'gap_adjust': int (-1/0/+1),
      }
    """
    # TODO: 根据实际 DBC 实现
    buttons = data[0]
    return {
        'resume': (buttons & 0x01) != 0,
        'set': (buttons & 0x02) != 0,
        'cancel': (buttons & 0x04) != 0,
        'distance': (buttons & 0x08) != 0,
        'gap_adjust': 0,  # TODO: 解析增减档
    }
