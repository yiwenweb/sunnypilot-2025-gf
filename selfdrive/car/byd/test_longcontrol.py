#!/usr/bin/env python3
"""
BYD 纵向控制器单元测试
不需要真车，纯软件仿真验证控制逻辑
"""

import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import namedtuple

# Mock 依赖
class MockParams:
    def __init__(self):
        self.data = {}
    def get(self, key, encoding=None):
        return self.data.get(key, "3" if "Distance" in key else "1")
    def put(self, key, value):
        self.data[key] = value

sys.modules['openpilot.common.params'] = type(sys)('mock_params')
sys.modules['openpilot.common.params'].Params = MockParams

# 简化导入
from longcontrol_byd import LongitudinalController, FollowDistance, ComfortMode

# Mock CarParams
CP = namedtuple('CarParams', ['carFingerprint'])('BYD_TANG_DM')

# Mock CarState
class MockCarState:
    def __init__(self, vEgo=0, aEgo=0):
        self.vEgo = vEgo
        self.aEgo = aEgo

# Mock Lead
class MockLead:
    def __init__(self, x=50, v=10, prob=1.0):
        self.x = x
        self.v = v
        self.prob = prob

def test_scenario_1_cruise_no_lead():
    """场景1: 无前车巡航到设定速度"""
    print("\n=== 场景1: 无前车巡航 ===")
    controller = LongitudinalController(CP)
    
    v_ego = 0.0
    v_cruise = 15.0  # 54 km/h
    dt = 0.05
    t_sim = 20.0  # 20秒
    
    history = {'t': [], 'v': [], 'a_cmd': []}
    
    for i in range(int(t_sim / dt)):
        t = i * dt
        CS = MockCarState(vEgo=v_ego, aEgo=0)
        
        a_cmd, stopping, warning = controller.update(CS, None, v_cruise)
        
        # 简单运动学更新
        v_ego += a_cmd * dt
        v_ego = max(0, min(v_ego, v_cruise + 1))
        
        history['t'].append(t)
        history['v'].append(v_ego)
        history['a_cmd'].append(a_cmd)
        
        if i % 20 == 0:  # 每1秒打印
            print(f"  t={t:.1f}s: v={v_ego:.2f} m/s, a_cmd={a_cmd:.2f} m/s²")
    
    # 验证结果
    final_v = history['v'][-1]
    assert abs(final_v - v_cruise) < 0.5, f"未达到巡航速度: {final_v} vs {v_cruise}"
    print(f"✓ 最终速度: {final_v:.2f} m/s (目标 {v_cruise:.2f})")
    
    return history

def test_scenario_2_follow_steady():
    """场景2: 稳态跟车（前车匀速）"""
    print("\n=== 场景2: 稳态跟车 ===")
    controller = LongitudinalController(CP)
    controller.follow_distance = FollowDistance.FAR  # 远档, TTC=4.5s
    
    v_ego = 10.0
    v_lead = 10.0
    x_lead = 50.0  # 初始距离
    v_cruise = 20.0
    dt = 0.05
    t_sim = 30.0
    
    history = {'t': [], 'v': [], 'x': [], 'a_cmd': [], 'd_target': []}
    
    for i in range(int(t_sim / dt)):
        t = i * dt
        CS = MockCarState(vEgo=v_ego, aEgo=0)
        lead = MockLead(x=x_lead, v=v_lead)
        
        a_cmd, stopping, warning = controller.update(CS, lead, v_cruise)
        
        # 更新自车
        v_ego += a_cmd * dt
        v_ego = max(0, v_ego)
        
        # 更新相对距离（前车匀速）
        x_lead += (v_lead - v_ego) * dt
        
        d_target = controller.get_target_following_distance(v_ego)
        
        history['t'].append(t)
        history['v'].append(v_ego)
        history['x'].append(x_lead)
        history['a_cmd'].append(a_cmd)
        history['d_target'].append(d_target)
        
        if i % 40 == 0:
            print(f"  t={t:.1f}s: v={v_ego:.2f}, x={x_lead:.1f}m, d_target={d_target:.1f}m, a={a_cmd:.2f}")
    
    # 验证: 最终速度≈前车, 距离≈目标
    final_v = history['v'][-1]
    final_x = history['x'][-1]
    final_d_target = history['d_target'][-1]
    
    assert abs(final_v - v_lead) < 0.5, f"速度未稳定: {final_v} vs {v_lead}"
    assert abs(final_x - final_d_target) < 5, f"距离未稳定: {final_x} vs {final_d_target}"
    
    print(f"✓ 稳态: v={final_v:.2f} m/s, x={final_x:.1f}m (目标 {final_d_target:.1f}m)")
    
    return history

def test_scenario_3_stop_and_go():
    """场景3: 前车刹停 + 重新起步"""
    print("\n=== 场景3: 刹停 + 起步 ===")
    controller = LongitudinalController(CP)
    
    v_ego = 10.0
    v_lead = 10.0
    x_lead = 30.0
    dt = 0.05
    t_sim = 40.0
    
    history = {'t': [], 'v_ego': [], 'v_lead': [], 'x': [], 'a_cmd': [], 'stopping': []}
    
    for i in range(int(t_sim / dt)):
        t = i * dt
        
        # 前车行为: 10s后刹停, 30s后起步
        if t > 10 and t < 30:
            v_lead = max(0, v_lead - 1.5 * dt)  # 前车减速
        elif t >= 30:
            v_lead = min(10, v_lead + 1.0 * dt)  # 前车起步
        
        CS = MockCarState(vEgo=v_ego, aEgo=0)
        lead = MockLead(x=x_lead, v=v_lead)
        
        a_cmd, stopping, warning = controller.update(CS, lead, 20.0)
        
        # 更新
        v_ego += a_cmd * dt
        v_ego = max(0, v_ego)
        x_lead += (v_lead - v_ego) * dt
        
        history['t'].append(t)
        history['v_ego'].append(v_ego)
        history['v_lead'].append(v_lead)
        history['x'].append(x_lead)
        history['a_cmd'].append(a_cmd)
        history['stopping'].append(stopping)
        
        if i % 40 == 0:
            print(f"  t={t:.1f}s: v_ego={v_ego:.2f}, v_lead={v_lead:.2f}, x={x_lead:.1f}m, stop={stopping}")
    
    # 验证: 能否停住 + 重新跟随
    min_v = min(history['v_ego'])
    assert min_v < 0.5, f"未能停车: min_v={min_v}"
    final_v = history['v_ego'][-1]
    assert final_v > 5, f"未能重新起步: final_v={final_v}"
    
    print(f"✓ 刹停成功(min_v={min_v:.2f}), 重新起步(final_v={final_v:.2f})")
    
    return history

def test_scenario_4_distance_settings():
    """场景4: 跟车距离档位测试"""
    print("\n=== 场景4: 距离档位 ===")
    controller = LongitudinalController(CP)
    
    v_ego = 10.0  # 36 km/h
    results = {}
    
    for dist_setting in [FollowDistance.CLOSE, FollowDistance.MEDIUM, 
                         FollowDistance.FAR, FollowDistance.EXTRA_FAR]:
        controller.follow_distance = dist_setting
        d_target = controller.get_target_following_distance(v_ego)
        ttc = d_target / v_ego if v_ego > 0 else 99
        results[dist_setting.name] = (d_target, ttc)
        print(f"  档位{dist_setting.value} ({dist_setting.name}): d={d_target:.1f}m, TTC={ttc:.2f}s")
    
    # 验证: 档位越高, 距离越远
    assert results['CLOSE'][0] < results['MEDIUM'][0] < results['FAR'][0] < results['EXTRA_FAR'][0]
    print("✓ 档位递增, 距离递增")
    
    return results

def test_scenario_5_collision_warning():
    """场景5: 碰撞预警触发"""
    print("\n=== 场景5: 碰撞预警 ===")
    controller = LongitudinalController(CP)
    
    # 危险场景: 距离10m, 自车15m/s, 前车5m/s
    v_ego = 15.0
    v_lead = 5.0
    x_lead = 10.0
    
    CS = MockCarState(vEgo=v_ego, aEgo=0)
    lead = MockLead(x=x_lead, v=v_lead)
    
    a_cmd, stopping, warning = controller.update(CS, lead, 20.0)
    
    # TTC = 10 / (15-5) = 1.0s -> 应触发警告
    assert warning, "未触发碰撞预警"
    assert controller.emergency_brake, "未触发紧急制动"
    assert a_cmd < -2.0, f"紧急制动力度不足: {a_cmd}"
    
    print(f"✓ 碰撞预警触发, 紧急制动 a={a_cmd:.2f} m/s²")

def plot_results(scenarios):
    """绘制测试结果"""
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('BYD 纵向控制器测试结果', fontsize=16)
    
    # 场景1: 无前车巡航
    ax = axes[0, 0]
    s1 = scenarios['cruise']
    ax.plot(s1['t'], s1['v'], label='v_ego')
    ax.axhline(15, color='r', linestyle='--', label='v_cruise')
    ax.set_title('场景1: 无前车巡航')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('速度 (m/s)')
    ax.legend(); ax.grid(True, alpha=0.3)
    
    ax = axes[0, 1]
    ax.plot(s1['t'], s1['a_cmd'])
    ax.set_title('场景1: 加速度命令')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('accel (m/s²)')
    ax.axhline(0, color='k', linewidth=0.5)
    ax.grid(True, alpha=0.3)
    
    # 场景2: 稳态跟车
    ax = axes[1, 0]
    s2 = scenarios['follow']
    ax.plot(s2['t'], s2['x'], label='实际距离')
    ax.plot(s2['t'], s2['d_target'], 'r--', label='目标距离')
    ax.set_title('场景2: 稳态跟车距离')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('距离 (m)')
    ax.legend(); ax.grid(True, alpha=0.3)
    
    ax = axes[1, 1]
    ax.plot(s2['t'], s2['v'], label='v_ego')
    ax.axhline(10, color='r', linestyle='--', label='v_lead')
    ax.set_title('场景2: 速度跟随')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('速度 (m/s)')
    ax.legend(); ax.grid(True, alpha=0.3)
    
    # 场景3: 刹停起步
    ax = axes[2, 0]
    s3 = scenarios['stop_go']
    ax.plot(s3['t'], s3['v_ego'], label='v_ego')
    ax.plot(s3['t'], s3['v_lead'], label='v_lead')
    ax.set_title('场景3: 刹停 + 起步')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('速度 (m/s)')
    ax.legend(); ax.grid(True, alpha=0.3)
    
    ax = axes[2, 1]
    ax.plot(s3['t'], s3['x'])
    ax.axhline(3, color='r', linestyle='--', label='停车距离')
    ax.set_title('场景3: 跟车距离')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('距离 (m)')
    ax.legend(); ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('c:/Users/yiwen/Desktop/1/_longcontrol_test.png', dpi=150)
    print("\n✓ 测试图表已保存: _longcontrol_test.png")

if __name__ == "__main__":
    print("=" * 60)
    print("BYD 纵向控制器单元测试")
    print("=" * 60)
    
    scenarios = {}
    
    try:
        scenarios['cruise'] = test_scenario_1_cruise_no_lead()
        scenarios['follow'] = test_scenario_2_follow_steady()
        scenarios['stop_go'] = test_scenario_3_stop_and_go()
        test_scenario_4_distance_settings()
        test_scenario_5_collision_warning()
        
        print("\n" + "=" * 60)
        print("✓ 所有测试通过")
        print("=" * 60)
        
        # 绘图
        plot_results(scenarios)
        
    except AssertionError as e:
        print(f"\n✗ 测试失败: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ 异常: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
