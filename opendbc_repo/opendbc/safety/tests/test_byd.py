#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.byd.values import BydSafetyFlags, CanBus
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda

# --- message addresses (see opendbc/safety/modes/byd.h) ---
MSG_EPS            = 0x11F  # 287  steering angle (RX, bus0)
MSG_CARSPEED      = 0x121  # 289  vehicle speed (RX, bus0)
MSG_DRIVE_STATE   = 0x242  # 578  brake / gear (RX, bus0)
MSG_ACC_EPS_STATE = 0x318  # 792  MainTorque / SteerDriverTorque (RX bus0) + fake EPS (TX bus2)
MSG_PEDAL         = 0x342  # 834  gas pedal (RX, bus0)
MSG_ACC_MPC_STATE = 0x316  # 790  LKAS steering command (TX, bus0)
MSG_ACC_HUD_ADAS  = 0x32D  # 813  ACC state (RX bus2 -> controls_allowed) + stock passthrough (TX bus0)
MSG_ACC_CMD       = 0x32E  # 814  ACC accel command (TX, bus0)
MSG_ACC_AEB       = 0x32F  # 815  ACC AEB (TX, bus0)
MSG_PCM_BUTTONS   = 0x3B0  # 944  cruise buttons (RX bus0) + forward (TX bus2)

# CarDisplaySpeed raw -> m/s conversion mirrors byd_rx_hook:
#   speed_ms = raw * 0.0735 * KPH_TO_MS
SPEED_FACTOR = 0.0735 / 3.6


class TestBydSafety(common.PandaCarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  TX_MSGS = [
    [MSG_ACC_MPC_STATE, 0],
    [MSG_ACC_HUD_ADAS, 0],
    [MSG_ACC_CMD, 0],
    [MSG_ACC_AEB, 0],
    [MSG_ACC_EPS_STATE, 2],
    [MSG_PCM_BUTTONS, 2],
  ]
  # All BYD TX messages use check_relay=false (OP fully synthesizes the ACC/steering
  # report group), so the standard "stock msg seen on our TX bus" relay-malfunction
  # trigger never fires. See BYD_TX_MSGS in byd.h.
  RELAY_MALFUNCTION_ADDRS: dict = {}
  # At init (no OP TX yet) only the steering cmd 0x316 is blocked in both directions.
  # 0x32D/0x32E/0x32F (bus2) are only blocked once OP starts sending its ACC group
  # (byd_op_acc_active), and real EPS 0x318 (bus0) only when the fake EPS is active.
  FWD_BLACKLISTED_ADDRS = {0: [MSG_ACC_MPC_STATE], 2: [MSG_ACC_MPC_STATE]}

  STANDSTILL_THRESHOLD = 0.0
  GAS_PRESSED_THRESHOLD = 0

  # --- torque limits, must match TorqueSteeringLimits BYD_STEERING_LIMITS in byd.h ---
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RATE_UP = 18
  MAX_RATE_DOWN = 18
  MAX_RT_DELTA = 243
  MAX_TORQUE_ERROR = 150
  TORQUE_MEAS_TOLERANCE = 0

  # has_steer_req_tolerance = true in byd.h
  MIN_VALID_STEERING_FRAMES = 10
  MAX_INVALID_STEERING_FRAMES = 5
  STEER_STEP = 2

  LKAS_ACTIVE_VALUE = 1

  # LKAS_Output is an 11-bit signed CAN signal ([-1024, 1023]); torque values beyond
  # that wrap on the bus, so the representable command range is narrower than the
  # generic ±(MAX_TORQUE + 1000) sweep. Anything within ±1023 fully exercises the
  # absolute limit at ±300.
  LKAS_OUTPUT_MIN = -1024
  LKAS_OUTPUT_MAX = 1023

  @property
  def MIN_VALID_STEERING_RT_INTERVAL(self):
    # byd.h sets min_valid_request_rt_interval = 250000us explicitly, which does not
    # equal the generic (frames+1)*step*10000*0.9 formula; pin it to the C value.
    return 250000

  def setUp(self):
    self.packer = CANPackerPanda("byd_tang_dm_2018")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, BydSafetyFlags.HAN_TANG_DMEV)
    self.safety.init_tests()

  # --- required message builders ---

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque, "LKAS_Active": self.LKAS_ACTIVE_VALUE if steer_req else 0}
    return self.packer.make_can_msg_panda("ACC_MPC_STATE", 0, values)

  def _torque_meas_msg(self, torque):
    values = {"MainTorque": torque}
    return self.packer.make_can_msg_panda("ACC_EPS_STATE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"SteerDriverTorque": torque}
    return self.packer.make_can_msg_panda("ACC_EPS_STATE", 0, values)

  def _speed_msg(self, speed):
    values = {"CarDisplaySpeed": round(speed / SPEED_FACTOR)}
    return self.packer.make_can_msg_panda("CARSPEED", 0, values)

  def _vehicle_moving_msg(self, speed: float):
    # BYD derives vehicle_moving from the gear field (gear == 1 => not moving),
    # not from a speed threshold.
    values = {"Gear": 1 if speed <= self.STANDSTILL_THRESHOLD else 2}
    return self.packer.make_can_msg_panda("DRIVE_STATE", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BrakePressed": 1 if brake else 0}
    return self.packer.make_can_msg_panda("DRIVE_STATE", 0, values)

  def _user_gas_msg(self, gas):
    # byd_rx_hook: gas_pressed = PEDAL.data[0] > 0; AcceleratorPedal scale is 0.01.
    values = {"AcceleratorPedal": gas}
    return self.packer.make_can_msg_panda("PEDAL", 0, values)

  def _pcm_status_msg(self, enable):
    # controls_allowed is driven by AccState in 0x32D on the camera bus (bus 2).
    # AccState in {1,2,3,5} engages; use 3 (ACTIVE) for enable, 0 (OFF) for disable.
    values = {"AccState": 3 if enable else 0}
    return self.packer.make_can_msg_panda("ACC_HUD_ADAS", CanBus.MPC, values)

  # --- overrides for BYD-specific signal widths ---

  def test_torque_absolute_limits(self):
    # Overridden: LKAS_Output is 11-bit signed, so sweep only the representable range.
    for controls_allowed in [True, False]:
      for torque in np.arange(self.LKAS_OUTPUT_MIN, self.LKAS_OUTPUT_MAX + 1, self.MAX_RATE_UP):
        self.safety.set_controls_allowed(controls_allowed)
        self.safety.set_rt_torque_last(torque)
        self.safety.set_torque_meas(torque, torque)
        self.safety.set_desired_torque_last(torque - self.MAX_RATE_UP)

        if controls_allowed:
          send = (-self.MAX_TORQUE <= torque <= self.MAX_TORQUE)
        else:
          send = torque == 0

        self.assertEqual(send, self._tx(self._torque_cmd_msg(torque)))

  # --- BYD-specific behavior ---

  def test_controls_allowed_from_acc_state(self):
    # 0x32D AccState on bus 2 gates controls_allowed. {1,2,3,5} engage, others disengage.
    for acc_state in range(8):
      self.safety.set_controls_allowed(False)
      values = {"AccState": acc_state}
      self._rx(self.packer.make_can_msg_panda("ACC_HUD_ADAS", CanBus.MPC, values))
      should_engage = acc_state in (1, 2, 3, 5)
      self.assertEqual(should_engage, self.safety.get_controls_allowed(), f"AccState={acc_state}")

  def test_acc_state_only_from_camera_bus(self):
    # The same 0x32D on bus 0 must not engage controls (only bus 2 is the MPC source).
    self.safety.set_controls_allowed(False)
    values = {"AccState": 3}
    self._rx(self.packer.make_can_msg_panda("ACC_HUD_ADAS", 0, values))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_steering_cmd_always_blocked_from_camera(self):
    # MPC's 0x316 must never be forwarded to the EPS bus (blocked unconditionally).
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, MSG_ACC_MPC_STATE))
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, MSG_ACC_MPC_STATE))


if __name__ == "__main__":
  unittest.main()
