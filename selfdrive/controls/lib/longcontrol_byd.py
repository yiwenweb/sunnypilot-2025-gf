#!/usr/bin/env python3
"""
BYD 纵向控制器 - 基于门总实测数据的现代化自适应巡航控制
特性:
  - 自适应跟车距离(4档可调: 近/中/远/超远)
  - MPC预测控制 + PID混合(平顺性优化)
  - 多级舒适性模式(经济/舒适/运动)
  - 智能刹停逻辑(creep + hold)
  - 安全冗余(碰撞预警 + 紧急制动)
  
基于门总段25实测标定:
  - TTC基准: 4.0s (中位), 范围 2.9-6.2s
  - 加速度: +1.38 / -2.03 m/s²
  - 速度范围: 0-18 m/s (0-65 km/h)
"""

import numpy as np
from enum import IntEnum
from collections import deque
from cereal import log, car
from openpilot.common.params import Params
from openpilot.common.numpy_fast import interp, clip
from openpilot.selfdrive.controls.lib.pid import PIDController

# 常量定义
CONTROL_N = 17  # MPC预测步数
DT_MDL = 0.05   # 模型dt (20Hz)

class FollowDistance(IntEnum):
    """跟车距离档位 - 对应原车按钮"""
    CLOSE = 1      # 近: TTC 2.5s (运动/激进)
    MEDIUM = 2     # 中: TTC 3.5s (舒适)
    FAR = 3        # 远: TTC 4.5s (默认/门总实测)
    EXTRA_FAR = 4  # 超远: TTC 6.0s (保守/新手)

class ComfortMode(IntEnum):
    """舒适性模式"""
    ECO = 0        # 经济: 缓加速, 早减速, 省油优先
    COMFORT = 1    # 舒适: 平顺优先 (默认)
    SPORT = 2      # 运动: 响应快, 加速强

class LongitudinalController:
    """BYD 纵向控制器 - MPC+PID混合"""
    
    def __init__(self, CP):
        self.CP = CP
        self.params = Params()
        
        # 用户设置
        self.follow_distance = FollowDistance.FAR  # 默认远档 (TTC 4.5s)
        self.comfort_mode = ComfortMode.COMFORT
        
        # 门总实测标定参数 (段25)
        self.MENZON_ACCEL_MAX = 1.38   # 最大加速度 m/s²
        self.MENZON_ACCEL_MIN = -2.03  # 最大减速度 m/s²
        self.MENZON_TTC_BASE = 4.0     # 基准TTC (中位)
        
        # 跟车距离策略: TTC随速度分段调整
        # 基于门总数据: 低速保守(6s), 中速正常(3s), 高速稳定(4s)
        self.ttc_by_speed = {
            # (vEgo_max, ttc_multiplier) - 相对基准TTC的倍数
            2.0:  1.5,   # 0-2 m/s:   TTC×1.5 = 超保守(防刹停追尾)
            5.0:  0.9,   # 2-5 m/s:   TTC×0.9 = 略紧凑
            9.0:  0.75,  # 5-9 m/s:   TTC×0.75 = 正常跟车
            15.0: 1.0,   # 9-15 m/s:  TTC×1.0 = 基准
            999:  1.0,   # >15 m/s:   TTC×1.0 = 高速稳定
        }
        
        # 舒适性参数矩阵
        self.comfort_params = {
            ComfortMode.ECO: {
                'accel_max': 1.0,      # 限制加速 (省油)
                'accel_min': -1.8,     # 早减速 (滑行)
                'jerk_max': 0.8,       # 柔和
                'response_gain': 0.7,  # 响应慢
            },
            ComfortMode.COMFORT: {
                'accel_max': 1.38,     # 门总实测
                'accel_min': -2.03,
                'jerk_max': 1.2,       # 平顺
                'response_gain': 1.0,
            },
            ComfortMode.SPORT: {
                'accel_max': 1.8,      # 放宽限制
                'accel_min': -2.5,     # 强制动
                'jerk_max': 2.0,       # 允许急动
                'response_gain': 1.3,  # 响应快
            },
        }
        
        # PID控制器 (fallback + 低速精细控制)
        self.pid_speed = PIDController(
            k_p=0.3, k_i=0.05, k_d=0.0,
            pos_limit=self.MENZON_ACCEL_MAX,
            neg_limit=self.MENZON_ACCEL_MIN,
            rate=1 / DT_MDL
        )
        
        # 刹停逻辑
        self.stopping = False
        self.stopped = False
        self.stop_hold_timer = 0.0
        self.STOP_THRESHOLD = 0.3      # m/s, 判定停车阈值
        self.CREEP_SPEED = 0.5         # m/s, creep速度
        self.STOP_DISTANCE = 3.0       # m, 停车目标距离
        
        # 安全冗余
        self.collision_warning = False
        self.emergency_brake = False
        self.TTC_WARNING = 1.5         # s, 碰撞预警阈值
        self.TTC_EMERGENCY = 0.8       # s, 紧急制动阈值
        
        # 状态历史 (平滑 + 异常检测)
        self.accel_history = deque(maxlen=10)
        self.last_accel_cmd = 0.0
        
        # MPC (简化版 - 二次规划求解器)
        self.mpc_solution = None
        
    def update_user_settings(self):
        """从Params读取用户设置"""
        try:
            # 跟车距离档位 (1-4)
            distance = int(self.params.get("BydFollowDistance", encoding='utf-8') or "3")
            self.follow_distance = FollowDistance(clip(distance, 1, 4))
            
            # 舒适性模式 (0-2)
            comfort = int(self.params.get("BydComfortMode", encoding='utf-8') or "1")
            self.comfort_mode = ComfortMode(clip(comfort, 0, 2))
        except:
            pass  # 保持默认值
    
    def get_target_following_distance(self, v_ego: float) -> float:
        """
        计算目标跟车距离
        Args:
            v_ego: 自车速度 m/s
        Returns:
            目标距离 m
        """
        # 1) 基准TTC (根据用户档位调整)
        ttc_base = {
            FollowDistance.CLOSE: 2.5,
            FollowDistance.MEDIUM: 3.5,
            FollowDistance.FAR: 4.5,      # 门总实测接近
            FollowDistance.EXTRA_FAR: 6.0,
        }[self.follow_distance]
        
        # 2) 速度段修正 (门总实测: 低速保守, 中速紧凑)
        ttc_mult = 1.0
        for v_max, mult in sorted(self.ttc_by_speed.items()):
            if v_ego < v_max:
                ttc_mult = mult
                break
        
        ttc = ttc_base * ttc_mult
        
        # 3) 目标距离 = TTC × v_ego + 静止安全距离
        d_target = ttc * v_ego + self.STOP_DISTANCE
        
        return max(d_target, self.STOP_DISTANCE)  # 最小3m
    
    def compute_mpc_accel(self, x0_lead: float, v_ego: float, a_ego: float, 
                          v_lead: float = None) -> float:
        """
        MPC预测控制 - 求解最优加速度轨迹
        简化实现: 单步优化 (完整MPC需要cvxpy求解器)
        
        Args:
            x0_lead: 前车距离 m
            v_ego: 自车速度 m/s
            a_ego: 当前加速度 m/s²
            v_lead: 前车速度 m/s (可选, 用于预测)
        Returns:
            最优加速度 m/s²
        """
        d_target = self.get_target_following_distance(v_ego)
        d_error = x0_lead - d_target  # 正=距离大, 负=太近
        
        # 如果有前车速度, 计算相对速度
        if v_lead is not None:
            v_rel = v_lead - v_ego  # 正=前车快, 负=前车慢
        else:
            v_rel = 0.0
        
        # MPC简化: 比例-微分控制 + 前馈
        # a_cmd = Kp * d_error + Kd * v_rel + Kff * (期望加速度)
        comfort = self.comfort_params[self.comfort_mode]
        response_gain = comfort['response_gain']
        
        # 距离误差反馈
        Kp = 0.15 * response_gain  # 距离增益
        Kd = 0.5 * response_gain   # 速度增益
        
        a_mpc = Kp * d_error + Kd * v_rel
        
        # 前馈: 如果前车在减速, 我们也该减速
        if v_lead is not None and v_rel < -0.5:  # 前车明显慢
            a_mpc += v_rel * 0.3  # 前馈项
        
        # Jerk限制 (平顺性)
        jerk_max = comfort['jerk_max']
        if self.last_accel_cmd is not None:
            da_max = jerk_max * DT_MDL
            a_mpc = clip(a_mpc, 
                        self.last_accel_cmd - da_max, 
                        self.last_accel_cmd + da_max)
        
        return a_mpc
    
    def compute_pid_accel(self, x0_lead: float, v_ego: float) -> float:
        """
        PID控制器 (低速精细控制 + MPC失效时fallback)
        Args:
            x0_lead: 前车距离 m
            v_ego: 自车速度 m/s
        Returns:
            PID加速度 m/s²
        """
        d_target = self.get_target_following_distance(v_ego)
        d_error = x0_lead - d_target
        
        # 目标速度 = 当前速度 + 根据距离误差调整
        # 如果距离大, 可以加速; 距离小, 要减速
        v_target_delta = d_error * 0.2  # 简单比例
        v_target = v_ego + v_target_delta
        v_target = clip(v_target, 0, 25)  # 限制最高速度
        
        a_pid = self.pid_speed.update(v_target - v_ego)
        return a_pid
    
    def update(self, CS, lead_one, v_cruise_setpoint):
        """
        主更新函数 - 每帧调用
        Args:
            CS: CarState
            lead_one: modelLeads[0] 或 radarState.leadOne
            v_cruise_setpoint: 巡航设定速度 m/s
        Returns:
            (accel_cmd, stopping, collision_warning)
        """
        self.update_user_settings()
        
        v_ego = CS.vEgo
        a_ego = CS.aEgo
        comfort = self.comfort_params[self.comfort_mode]
        
        # 1) 判断是否有前车
        has_lead = lead_one is not None and hasattr(lead_one, 'x') and lead_one.prob > 0.5
        
        if not has_lead:
            # 无前车: 巡航到设定速度
            v_error = v_cruise_setpoint - v_ego
            a_cruise = clip(v_error * 0.5, comfort['accel_min'], comfort['accel_max'])
            a_cmd = a_cruise
            self.stopping = False
            self.stopped = False
            self.collision_warning = False
            self.emergency_brake = False
        
        else:
            # 有前车: 跟车逻辑
            x0 = lead_one.x if hasattr(lead_one, 'x') else lead_one.dRel
            v_lead = lead_one.v if hasattr(lead_one, 'v') else (lead_one.vLead if hasattr(lead_one, 'vLead') else None)
            
            # 2) 安全检查: TTC碰撞预警
            v_rel = (v_lead - v_ego) if v_lead is not None else 0
            if v_rel < -0.1:  # 我们比前车快
                ttc = x0 / (v_ego - v_lead) if v_lead is not None else x0 / v_ego
                if ttc < self.TTC_EMERGENCY and x0 < 15:
                    self.emergency_brake = True
                    self.collision_warning = True
                elif ttc < self.TTC_WARNING and x0 < 25:
                    self.collision_warning = True
                else:
                    self.collision_warning = False
                    self.emergency_brake = False
            else:
                self.collision_warning = False
                self.emergency_brake = False
            
            # 3) 刹停逻辑
            if x0 < self.STOP_DISTANCE + 2 and v_ego < 1.0:
                self.stopping = True
                if v_ego < self.STOP_THRESHOLD:
                    self.stopped = True
                    self.stop_hold_timer += DT_MDL
                    # Hold刹车2秒后允许creep
                    if self.stop_hold_timer > 2.0 and x0 > self.STOP_DISTANCE:
                        a_cmd = (self.CREEP_SPEED - v_ego) * 2  # creep
                    else:
                        a_cmd = -1.0  # hold
                else:
                    # 接近停车: 柔和减速
                    a_cmd = -(v_ego ** 2) / (2 * max(x0 - self.STOP_DISTANCE, 0.5))
                    a_cmd = clip(a_cmd, -1.5, 0)
            
            else:
                self.stopping = False
                self.stopped = False
                self.stop_hold_timer = 0.0
                
                # 4) 正常跟车: MPC主导, PID辅助
                if v_ego > 3.0:
                    # 中高速: MPC
                    a_mpc = self.compute_mpc_accel(x0, v_ego, a_ego, v_lead)
                    a_cmd = a_mpc
                else:
                    # 低速: MPC + PID混合 (精细控制)
                    a_mpc = self.compute_mpc_accel(x0, v_ego, a_ego, v_lead)
                    a_pid = self.compute_pid_accel(x0, v_ego)
                    # 低速时PID权重更大
                    w_pid = interp(v_ego, [0, 3], [0.6, 0.2])
                    a_cmd = a_mpc * (1 - w_pid) + a_pid * w_pid
                
                # 5) 紧急制动覆盖
                if self.emergency_brake:
                    a_cmd = min(a_cmd, -3.0)  # 强制重刹
            
            # 6) 速度限制: 不超过巡航设定
            if v_ego > v_cruise_setpoint - 0.5:
                a_cmd = min(a_cmd, 0)  # 禁止加速
        
        # 7) 全局限幅
        a_cmd = clip(a_cmd, comfort['accel_min'], comfort['accel_max'])
        
        # 8) 平滑输出 (防抖动)
        if self.last_accel_cmd is not None:
            alpha = 0.3  # 一阶滤波系数
            a_cmd = alpha * a_cmd + (1 - alpha) * self.last_accel_cmd
        
        self.last_accel_cmd = a_cmd
        self.accel_history.append(a_cmd)
        
        return a_cmd, self.stopping, self.collision_warning
    
    def reset(self):
        """重置控制器状态"""
        self.pid_speed.reset()
        self.stopping = False
        self.stopped = False
        self.stop_hold_timer = 0.0
        self.last_accel_cmd = 0.0
        self.accel_history.clear()
        self.collision_warning = False
        self.emergency_brake = False
