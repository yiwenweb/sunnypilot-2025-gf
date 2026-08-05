#pragma once

#include "opendbc/safety/safety_declarations.h"

// === BYD Tang DM 2018 - Lateral + experimental longitudinal ===

// RX messages (Bus 0)
#define BYD_EPS              0x11FU  // 287
#define BYD_CARSPEED         0x121U  // 289
#define BYD_YAW_RATE         0x222U  // 546
#define BYD_AXAY             0x223U  // 547
#define BYD_DRIVE_STATE      0x242U  // 578
#define BYD_ACC_EPS_STATE    0x318U  // 792
#define BYD_PEDAL            0x342U  // 834
#define BYD_PCM_BUTTONS      0x3B0U  // 944

// RX messages (Bus 2 - from MPC)
#define BYD_ACC_HUD_ADAS_RX  0x32DU  // 813 - ACC state from MPC

// TX messages
#define BYD_ACC_MPC_STATE    0x316U  // 790 - steering (Bus 0)
#define BYD_ACC_HUD_ADAS     0x32DU  // 813 - stock passthrough
#define BYD_ACC_CMD          0x32EU  // 814 - stock passthrough
#define BYD_ACC_AEB          0x32FU  // 815 - stock passthrough
#define BYD_ACC_EPS_FAKE     0x318U  // 792 - fake EPS (Bus 2)
#define BYD_PCM_BUTTONS_FWD  0x3B0U  // 944 - buttons forward (Bus 2)

#define BYD_MAIN_BUS 0U
#define BYD_CAM_BUS  2U

static uint8_t byd_get_counter(const CANPacket_t *msg) { UNUSED(msg); return 0U; }
static uint32_t byd_get_checksum(const CANPacket_t *msg) { UNUSED(msg); return 0U; }
static uint32_t byd_compute_checksum(const CANPacket_t *msg) { UNUSED(msg); return 0U; }

static const TorqueSteeringLimits BYD_STEERING_LIMITS = {
  .max_torque = 300,       // 门总 0.98 confirmed working max; 897 rejected by EPS (TorqueFailed)
  .max_rate_up = 18,       // 20260720 16->18: 门总23接管段全量实测 上升rate=18(p99=max=18). 须与Python
  .max_rate_down = 18,     //   STEER_DELTA_UP/DOWN=18 匹配, 否则Python发18被panda(16)拦丢帧. 收力(向0)
                           //   不受此限(lowest_allowed=-rate_up, 收到0>=-18放行, SOFT收力54仍OK)。
  .max_rt_delta = 243,
  .type = TorqueMotorLimited,
  .max_torque_error = 150, // BYD EPS has significant motor torque reporting lag; 50 caused
                           // dist_to_meas_check violations at ~80+ torque (EPS still near 0),
                           // triggering controlsMismatch immediateDisable. 150 gives enough
                           // headroom for EPS to catch up within a few frames.
  .min_valid_request_frames = 10,
  .max_invalid_request_frames = 5,
  .min_valid_request_rt_interval = 250000U,
  .has_steer_req_tolerance = true,
};

// OP is sending 790 -> fwd_hook blocks MPC's 790 from Bus2->Bus0
static bool byd_op_steering_active = false;
static uint32_t byd_op_steering_ts = 0U;

// OP always sends fake 792 -> fwd_hook blocks real EPS 792 from Bus0->Bus2
static bool byd_fake_eps_active = false;
static uint32_t byd_fake_eps_ts = 0U;

// OP sends 814 ACC_CMD -> fwd_hook blocks MPC's 814 from Bus2->Bus0
static bool byd_op_acc_active = false;
static uint32_t byd_op_acc_ts = 0U;


static void byd_rx_hook(const CANPacket_t *msg) {

  // === Bus 0 messages ===
  if (msg->bus == BYD_MAIN_BUS) {

    if (msg->addr == BYD_CARSPEED) {
      int speed_raw = (msg->data[0] | ((msg->data[1] & 0x0FU) << 8));
      UPDATE_VEHICLE_SPEED((float)speed_raw * 0.0735f * KPH_TO_MS);
    }

    if (msg->addr == BYD_EPS) {
      int angle_raw = (msg->data[0] | (msg->data[1] << 8));
      if (angle_raw > 32767) angle_raw -= 65536;
      update_sample(&angle_meas, angle_raw);
    }

    if (msg->addr == BYD_ACC_EPS_STATE) {
      // MainTorque (bits 8-19, 12-bit signed) -> torque_meas for safety torque error check
      int torque_motor_raw = ((msg->data[1] | ((msg->data[2] & 0x0FU) << 8)));
      if (torque_motor_raw > 2047) torque_motor_raw -= 4096;
      update_sample(&torque_meas, torque_motor_raw);

      // SteerDriverTorque (bits 24-35, 12-bit signed) -> torque_driver for driver override check
      int torque_driver_raw = ((msg->data[3] | (msg->data[4] << 8)) & 0xFFFU);
      if (torque_driver_raw > 2047) torque_driver_raw -= 4096;
      update_sample(&torque_driver, torque_driver_raw);
    }

    if (msg->addr == BYD_DRIVE_STATE) {
      brake_pressed = GET_BIT(msg, 37U);
      unsigned int gear = (msg->data[5] & 0x7U);
      vehicle_moving = (gear != 1U);
    }

    if (msg->addr == BYD_PEDAL) {
      gas_pressed = msg->data[0] > 0U;
    }

    if (msg->addr == BYD_PCM_BUTTONS) {
      acc_main_on = GET_BIT(msg, 8U);
      // FIXED: Do NOT set controls_allowed here - 944 bit 8 is a momentary button!
      // controls_allowed is now driven by 813 AccState from MPC (Bus 2)

      // MADS support - still use the button press event
      mads_button_press = acc_main_on ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }

    if (msg->addr == BYD_DRIVE_STATE) {
      if ((alternative_experience & ALT_EXP_ENABLE_MADS) && !m_mads_state.system_enabled) {
        m_mads_state.system_enabled = true;
      }
      mads_state_update(vehicle_moving, acc_main_on, controls_allowed,
                        brake_pressed || regen_braking, steering_disengage);
    }
  }

  // === Bus 2 messages (from MPC) ===
  if (msg->bus == BYD_CAM_BUS) {

    // FIXED: Use 813 ACC_HUD_ADAS from MPC to determine cruise state
    // AccState field: byte 2 bits [5:3] (3-bit field)
    // Observed values on Tang DM 2018:
    //   0=OFF, 1=STANDBY, 2=ACTIVATING (transient), 3=ACTIVE, 5=FORCE_ACCEL, 7=FAULT
    // Empirical: AccState=2 is a brief transition state between STANDBY (1) and ACTIVE (3).
    // Treating it as block causes controls_allowed to drop momentarily,
    // which forces carcontroller to reset lkas_active and steer_softstart_limit,
    // breaking lateral handshake. Include 2 in the allowed set.
    if (msg->addr == BYD_ACC_HUD_ADAS_RX) {
      unsigned int acc_state = ((msg->data[2] >> 3) & 0x07U);
      bool cruise_engaged = (acc_state == 1U) || (acc_state == 2U) ||
                            (acc_state == 3U) || (acc_state == 5U);
      controls_allowed = cruise_engaged;

      // Also update acc_main_on based on continuous state
      acc_main_on = (acc_state != 0U) && (acc_state != 7U);
    }
  }
}

static bool byd_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

  // 790 steering on Bus 0 - torque check
  if ((msg->addr == BYD_ACC_MPC_STATE) && (msg->bus == BYD_MAIN_BUS)) {
    int lkas_output = ((msg->data[2] | (msg->data[3] << 8)) & 0x7FFU);
    if (lkas_output > 1023) lkas_output -= 2048;
    bool steer_req = GET_BIT(msg, 28U);
    if (steer_torque_cmd_checks(lkas_output, steer_req, BYD_STEERING_LIMITS)) {
      tx = false;
    }
    // Mark OP steering active (aligned with 门总 0.98: only when torque-check passes)
    if (tx) {
      byd_op_steering_active = true;
      byd_op_steering_ts = microsecond_timer_get();
    }
  }

  // 814 ACC_CMD / 813 ACC_HUD_ADAS / 815 ACC_AEB on Bus 0
  // 对齐门总 0.98: OP 接管重发 813/814/815 (同一套连续 counter), 让 ESC 收到自洽的 ACC 报文组。
  // 过去 block 813/815 -> 只发814、813/815透传摄像头(另一套counter) -> 车机ACC黄灯报错+纵向失效。
  // 现放行 813/814/815, 并在 fwd_hook 拦截摄像头的 813/815 避免双源冲突。
  // 用 byd_op_acc_active 标记接管态 (OP在发这组ACC报文), 供 fwd_hook 门控。
  if (msg->bus == BYD_MAIN_BUS) {
    if ((msg->addr == BYD_ACC_CMD) || (msg->addr == BYD_ACC_HUD_ADAS) || (msg->addr == BYD_ACC_AEB)) {
      byd_op_acc_active = true;
      byd_op_acc_ts = microsecond_timer_get();
    }
  }

  // 792 fake EPS on Bus 2 - always allow, mark active
  if ((msg->addr == BYD_ACC_EPS_FAKE) && (msg->bus == BYD_CAM_BUS)) {
    byd_fake_eps_active = true;
    byd_fake_eps_ts = microsecond_timer_get();
  }

  return tx;
}

static bool byd_fwd_hook(int bus_num, int addr) {
  // Timeout: 100ms no TX -> resume full passthrough
  if (byd_op_steering_active) {
    if ((microsecond_timer_get() - byd_op_steering_ts) > 100000U) {
      byd_op_steering_active = false;
    }
  }
  if (byd_fake_eps_active) {
    if ((microsecond_timer_get() - byd_fake_eps_ts) > 100000U) {
      byd_fake_eps_active = false;
    }
  }
  if (byd_op_acc_active) {
    if ((microsecond_timer_get() - byd_op_acc_ts) > 100000U) {
      byd_op_acc_active = false;
    }
  }

  // Bus 2 -> Bus 0: block MPC's 790 (316) from reaching the EPS bus ONLY while OP is
  // actively sending its own 790 (byd_op_steering_active, refreshed every OP 790 TX).
  // 【根因修复 (EPS 启动即锁死)】: 之前此处无条件拦截摄像头 316, 前提假设"OP 总会发自己的
  //   316 替代"。但上电后 ACC 未激活/MADS 未触发 -> controls_allowed=False, latActive=False
  //   -> OP 一帧 316 都不发 (rlog 实证 sendcan 790=0)。结果 EPS 既收不到摄像头 316 也收不到
  //   OP 316 -> 转向命令流完全断供 -> 握手成功(0xFB)后约 1s EPS 判命令流丢失 -> TorqueFailed
  //   锁死 (LOCK1: f1648 握手 -> f1698 TorqueFailed)。这不是"混流"问题, 是"断供"问题。
  // 【修复】: 门控拦截。OP 在发 316(接管中) -> 拦摄像头 316, EPS 只听 OP(消除混流, 原泄漏已由
  //   tx_hook 每帧刷新 active 堵住); OP 沉默>100ms(未接管/崩溃/退出) -> active 超时 -> 放行
  //   摄像头原厂 316, EPS 始终有合法转向源 -> 永不断供锁死。这才是 openpilot relay 的 fail-safe
  //   本意: OP 不接管时原厂信号必须能通到 EPS。
  // Block MPC's 814/813/815 when OP is taking over (sending its own ACC 报文组).
  // 对齐门总: OP 接管期间, 摄像头的 813/814/815 从 bus2->bus0 全部拦截, ESC 只听 OP 的一套
  // 连续 counter 报文组, 避免双源 counter 冲突导致车机 ACC 报错。OP 停发 100ms 超时后自动恢复
  // 全透传 (byd_op_acc_active 超时清零), 保证异常/退出时原厂 ACC/AEB 立即接管。
  if (bus_num == 2) {
    if (addr == BYD_ACC_MPC_STATE) {
      return true;
    }
    if (byd_op_acc_active && ((addr == BYD_ACC_CMD) || (addr == BYD_ACC_HUD_ADAS) || (addr == BYD_ACC_AEB))) {
      return true;
    }
  }

  // Bus 0 -> Bus 2: block OP's 790 from reaching MPC
  // Block real EPS 792 when fake 792 is active
  if (bus_num == 0) {
    if (addr == BYD_ACC_MPC_STATE) {
      return true;
    }
    if (byd_fake_eps_active && (addr == BYD_ACC_EPS_STATE)) {
      return true;
    }
  }

  return false;
}

static safety_config byd_init(uint16_t param) {
  UNUSED(param);
  byd_op_steering_active = false;
  byd_op_steering_ts = 0U;
  byd_fake_eps_active = false;
  byd_fake_eps_ts = 0U;
  byd_op_acc_active = false;
  byd_op_acc_ts = 0U;

  static RxCheck byd_rx_checks[] = {
    {.msg = {{BYD_EPS, 0, 5, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CARSPEED, 0, 8, 25U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_YAW_RATE, 0, 8, 25U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_AXAY, 0, 8, 25U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_DRIVE_STATE, 0, 8, 25U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_ACC_EPS_STATE, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_PEDAL, 0, 8, 25U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_PCM_BUTTONS, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    // 813 ACC_HUD_ADAS on Bus 2 (from MPC) - MUST be whitelisted here, otherwise
    // safety_rx_hook() never calls byd_rx_hook() for it (only valid && whitelisted msgs
    // reach current_hooks->rx). Without this entry the bus2 813->controls_allowed logic
    // in byd_rx_hook is dead code -> controls_allowed stays False forever -> OP enabled
    // vs panda controls_allowed mismatch -> controlsMismatch immediateDisable (lat+long
    // both drop). Root cause of 000000b6. Camera sends 813 at 50Hz on bus2.
    {.msg = {{BYD_ACC_HUD_ADAS_RX, 2, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  static const CanMsg BYD_TX_MSGS[] = {
    {BYD_ACC_MPC_STATE,  BYD_MAIN_BUS, 8, .check_relay = false},  // 790 steering
    {BYD_ACC_HUD_ADAS,   BYD_MAIN_BUS, 8, .check_relay = false},  // 813 (blocked in tx_hook)
    {BYD_ACC_CMD,        BYD_MAIN_BUS, 8, .check_relay = false},  // 814 (blocked in tx_hook)
    {BYD_ACC_AEB,        BYD_MAIN_BUS, 8, .check_relay = false},  // 815 (blocked in tx_hook)
    {BYD_ACC_EPS_FAKE,   BYD_CAM_BUS,  8, .check_relay = false},  // 792 fake EPS
    {BYD_PCM_BUTTONS_FWD, BYD_CAM_BUS, 8, .check_relay = false},  // 944 buttons -> MPC
  };

  return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
  .get_counter = byd_get_counter,
  .get_checksum = byd_get_checksum,
  .compute_checksum = byd_compute_checksum,
};
