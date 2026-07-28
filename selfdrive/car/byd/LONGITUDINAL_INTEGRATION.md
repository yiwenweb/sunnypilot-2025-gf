# BYD 纵向控制集成指南

## 概述

基于门总实测数据（段25纯视觉纵向）开发的现代化自适应巡航控制器，包含：
- **自适应跟车距离**（4档可调：近/中/远/超远）
- **MPC预测控制 + PID混合**（平顺性优化）
- **多级舒适性**（经济/舒适/运动模式）
- **智能刹停**（creep + hold）
- **安全冗余**（碰撞预警 + 紧急制动）

## 门总实测标定参数（段25）

| 参数 | 值 | 来源 |
|------|-----|------|
| TTC基准 | 4.0s（中位）/ 4.5s（均值） | 实测统计 |
| 最大加速度 | +1.38 m/s² | 实测峰值 |
| 最大减速度 | -2.03 m/s² | 实测峰值 |
| 控制模式 | PID | longControlState |
| 速度范围 | 1.5~17.7 m/s | 实测范围 |

## 文件结构

```
selfdrive/car/byd/
├── carcontroller.py          # CarController主类（已更新）
├── bydcan.py                 # CAN报文构建（新增）
├── longcontrol_byd.py        # 纵向控制器核心（新增）
├── values.py                 # 车型参数（需更新）
├── carstate.py               # 状态解析（需更新）
└── interface.py              # 接口层（需更新）
```

## 集成步骤

### 1. 更新 values.py

添加纵向控制相关常量：

```python
# selfdrive/car/byd/values.py

class CANBUS:
    pt = 0        # 动力总成CAN
    radar = 1     # 雷达CAN（如果有）
    body = 2      # 车身CAN

class CarControllerParams:
    # ACC CAN地址（需根据实际DBC确认）
    ACC_CMD_ADDR = 0x316      # 加速度命令
    ACC_HUD_ADDR = 0x32E      # 仪表显示
    ACC_STATUS_ADDR = 0x32D   # 状态反馈
    FCW_WARNING_ADDR = 0x341  # 碰撞预警
    
    # 发送频率
    ACC_CMD_FREQ = 20         # Hz
    ACC_HUD_FREQ = 4          # Hz
    
    # 纵向控制参数（门总实测）
    ACCEL_MAX = 1.38          # m/s²
    ACCEL_MIN = -2.03         # m/s²
    JERK_LIMIT_COMFORT = 1.2  # m/s³
```

### 2. 更新 carstate.py

添加ACC按钮和状态解析：

```python
# selfdrive/car/byd/carstate.py

class CarState(CarStateBase):
    def update(self, pt_cp, cam_cp, loopback_cp):
        ret = car.CarState.new_message()
        
        # ... 现有代码 ...
        
        # ACC按钮解析
        if self.CP.carFingerprint in (CAR.BYD_TANG_DM, CAR.BYD_TANG_EV):
            # 方向盘按钮（地址需确认）
            acc_buttons = pt_cp.vl[0x???]["ACC_BUTTONS"]
            if acc_buttons == 1:  # RES+
                ret.cruiseState.buttonEvents.append(
                    car.CarState.ButtonEvent.new_message(
                        type=car.CarState.ButtonEvent.Type.accelCruise
                    )
                )
            elif acc_buttons == 2:  # CANCEL
                ret.cruiseState.buttonEvents.append(
                    car.CarState.ButtonEvent.new_message(
                        type=car.CarState.ButtonEvent.Type.cancel
                    )
                )
            elif acc_buttons == 4:  # DISTANCE
                ret.cruiseState.buttonEvents.append(
                    car.CarState.ButtonEvent.new_message(
                        type=car.CarState.ButtonEvent.Type.gapAdjustCruise
                    )
                )
        
        # 原车ACC状态（用于监控/接管检测）
        ret.cruiseState.available = True  # TODO: 解析实际状态
        
        return ret
```

### 3. 更新 interface.py

启用纵向控制：

```python
# selfdrive/car/byd/interface.py

class CarInterface(CarInterfaceBase):
    @staticmethod
    def _get_params(ret, candidate, fingerprint, car_fw, disable_openpilot_long, experimental_long):
        ret.carName = "byd"
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.byd)]
        
        # 启用纵向控制
        ret.openpilotLongitudinalControl = True
        ret.pcmCruise = False  # 不使用原车ACC
        
        # 纵向参数（门总实测）
        ret.longitudinalTuning.kpBP = [0., 5., 35.]
        ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
        ret.longitudinalTuning.kiBP = [0., 35.]
        ret.longitudinalTuning.kiV = [0.18, 0.12]
        
        # 停车参数
        ret.stoppingControl = True
        ret.startingState = True
        ret.vEgoStopping = 0.3  # m/s
        ret.vEgoStarting = 0.3  # m/s
        ret.stoppingDecelRate = 0.8  # m/s² (柔和刹停)
        
        # 加速度限制（门总实测）
        ret.longitudinalActuatorDelay = 0.15  # s (执行延迟)
        
        return ret
    
    def _update(self, c):
        ret = self.CS.update(self.cp, self.cp_cam, self.cp_loopback)
        
        # 纵向激活条件
        ret.cruiseState.enabled = ret.cruiseState.available and self.CS.out.cruiseState.enabled
        
        return ret
```

### 4. 创建 DBC 定义

需要根据实际总线抓包逆向，创建或更新 `byd_tang_dm_2018.dbc`：

```dbc
BO_ 790 ACC_CMD: 8 EON
 SG_ ACCEL_CMD : 7|8@0+ (0.02,-2.54) [-2.54|2.54] "m/s^2" VCU
 SG_ ACC_ENABLED : 8|1@0+ (1,0) [0|1] "" VCU
 SG_ ACC_ACTIVE : 9|1@0+ (1,0) [0|1] "" VCU
 SG_ COUNTER : 35|4@0+ (1,0) [0|15] "" VCU
 SG_ CHECKSUM : 39|8@0+ (1,0) [0|255] "" VCU

BO_ 814 ACC_HUD: 8 EON
 SG_ ACC_SET_SPEED : 7|8@0+ (1,0) [0|255] "km/h" IC
 SG_ ACC_DISTANCE_SETTING : 15|2@0+ (1,0) [1|4] "" IC
 SG_ ACC_COMFORT_MODE : 17|2@0+ (1,0) [0|2] "" IC
 SG_ ACC_ICON_ACTIVE : 19|1@0+ (1,0) [0|1] "" IC
 SG_ COUNTER : 35|4@0+ (1,0) [0|15] "" IC
 SG_ CHECKSUM : 39|8@0+ (1,0) [0|255] "" IC
```

### 5. 用户参数配置

通过 Params 暴露给用户：

```python
# 在 UI 或通过 SSH 设置

# 跟车距离档位 (1=近, 2=中, 3=远, 4=超远)
params.put("BydFollowDistance", "3")

# 舒适性模式 (0=经济, 1=舒适, 2=运动)
params.put("BydComfortMode", "1")
```

## 测试流程

### 阶段1：空档台架测试（安全）

1. **CAN发送验证**
   ```bash
   # SSH 进 comma 设备
   cd /data/openpilot
   python selfdrive/car/byd/test_can_send.py
   ```
   - 验证 ACC_CMD 报文能正确发送到总线
   - 确认 counter/checksum 正确
   - 不会触发原车报错

2. **DBC定义验证**
   ```bash
   # 用 cabana 查看实时 CAN
   tools/cabana/cabana --dbc opendbc/byd_tang_dm_2018.dbc
   ```
   - 确认报文地址、字段位置、scale/offset 正确

### 阶段2：停车场低速测试

1. **P档/N档怠速**
   - 踩刹车，挂P档，启动OP
   - 观察是否有异常报文/故障码
   - 确认仪表无警告灯

2. **D档蠕行测试（<5km/h）**
   - 空旷停车场，无障碍物
   - 启动ACC，设定速度10km/h
   - 观察车辆响应：
     * 能否平稳起步
     * 加速度是否柔和
     * 无前车时能否巡航到设定速度

3. **跟车测试（静止障碍物）**
   - 对着停车场柱子/墙壁，距离20m
   - 启动ACC，缓慢接近
   - 验证：
     * 5m时开始减速
     * 3m停车（STOP_DISTANCE）
     * 停稳后hold住不溜车

### 阶段3：封闭道路实车验证

1. **中低速跟车（20-60km/h）**
   - 跟随前车，保持档位3（远档）
   - 验证跟车距离符合预期（TTC≈4.5s）
   - 前车加减速，观察响应平顺性

2. **刹停测试**
   - 前车刹停，验证能否柔和停住
   - 停车距离约3m
   - creep功能（停2秒后缓慢前进）

3. **距离档位切换**
   - 行驶中切换1-4档
   - 验证跟车距离实时调整
   - 无突兀加减速

4. **舒适性模式测试**
   - 经济模式：缓加速，省油
   - 舒适模式：平顺
   - 运动模式：响应快，加速强

### 阶段4：公开道路长测

- 累计1000km+实际道路测试
- 监控：
  * 异常报错日志
  * 用户反馈（舒适性/安全性）
  * 边界case处理（急刹车/cut-in等）

## 调试工具

### 1. 实时监控脚本

```python
# debug_long_control.py
from openpilot.common.params import Params
import time

params = Params()
while True:
    # 读取控制器状态
    distance = params.get("BydFollowDistance", encoding='utf-8')
    comfort = params.get("BydComfortMode", encoding='utf-8')
    
    print(f"Distance: {distance}, Comfort: {comfort}")
    time.sleep(1)
```

### 2. CAN报文记录

```bash
# 记录30秒ACC激活时的CAN数据
candump -l can0,0x316:7FF,0x32E:7FF -T 30000
```

### 3. 参数调优工具

如果实测效果不理想，可调整：
- `longcontrol_byd.py` 的 Kp/Kd 增益
- TTC倍数（ttc_by_speed）
- 加速度限制（comfort_params）

## 已知限制和TODO

### 当前版本限制

1. **DBC 未完全逆向**
   - ACC_CMD 地址/字段是假设值
   - 需用实际总线抓包验证
   - checksum 算法待确认

2. **MPC 是简化版**
   - 未使用完整二次规划求解器（cvxpy）
   - 可升级为完整MPC（需要计算资源）

3. **原车按钮集成**
   - DISTANCE 按钮切换档位（未实现）
   - SET/RES 调速度（部分实现）

4. **雷达融合**
   - 当前纯视觉（modelLeads）
   - 后续可加入原车雷达融合

### 后续增强方向

1. **完整 MPC**
   ```python
   # 使用 cvxpy 求解
   import cvxpy as cp
   # 定义优化问题...
   ```

2. **学习用户驾驶风格**
   - 记录用户手动驾驶数据
   - 自适应调整 Kp/TTC

3. **地图辅助**
   - 弯道提前减速
   - 限速标志识别

4. **V2X 集成**
   - 前车刹车灯信号
   - 交通信号灯状态

## 支持和反馈

- 问题反馈：GitHub Issues
- 技术讨论：comma.ai Discord #byd频道
- 数据贡献：上传实测日志到 comma connect

## 许可证

本代码基于 MIT 许可证开源，与 openpilot 主项目保持一致。

---

**安全提示**：本系统为辅助驾驶，不可替代人类驾驶员。使用时需全程监控路况，随时准备接管。
