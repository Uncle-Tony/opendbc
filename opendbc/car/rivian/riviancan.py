def checksum(data, poly, xor_output):
  crc = 0
  for byte in data:
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = (crc << 1) ^ poly
      else:
        crc <<= 1
      crc &= 0xFF
  return crc ^ xor_output


def create_angle_steering(packer, frame, angle_deg, active):
  """Pack ACM_SteeringControl (0x110) for angle-based control. DBC: ACM_SteeringAngleRequest 23|15@0+ (0.1, -1638.4) deg."""
  # Clamp to DBC range so packer produces valid 15-bit raw
  angle_clip = max(-1638.4, min(1638.3, float(angle_deg)))
  values = {
    "ACM_SteeringControl_Counter": frame % 15,
    "ACM_EacEnabled": 2 if active else 0,  # 2 = EAC enabled
    "ACM_HapticRequired": 0,
    "ACM_SteeringAngleRequest": angle_clip,
  }
  data = packer.make_can_msg("ACM_SteeringControl", 0, values)[1]
  values["ACM_SteeringControl_Checksum"] = checksum(data[1:], 0x1D, 0x41)
  return packer.make_can_msg("ACM_SteeringControl", 0, values)


def create_lka_steering(packer, frame, acm_lka_hba_cmd, active, mads):
  # 0x120 ACM_lkaHbaCmd: passthrough when controls inactive; when active, only override LKA/symbol/lane/warning states (no torque).
  defaults = {"ACM_hbaSysState": 1, "ACM_hbaLamp": 0, "ACM_hbaOpt": 1, "ACM_FailinfoAeb": 0}
  values = {s: acm_lka_hba_cmd.get(s, defaults[s]) for s in defaults} if acm_lka_hba_cmd else defaults.copy()
  if acm_lka_hba_cmd:
    values.update(acm_lka_hba_cmd)  # start from car's message
  values["ACM_lkaHbaCmd_Counter"] = frame % 15

  if active:
    values |= {
      "ACM_HapticRequest": 0,
      "ACM_lkaStrToqReq": 0,  # angle mode: no torque on 0x120
      "ACM_lkaSymbolState": 3 if mads.lka_icon_states else 2,
      "ACM_lkaToiFlt": 0,
      "ACM_lkaActToi": 0,
      "ACM_lkaLaneRecogState": 3 if mads.lka_icon_states else 0,
      "ACM_lkaRHWarning": 0,
      "ACM_lkaLHWarning": 0,
      "ACM_lkaHandsoffSoundWarning": 0,
      "ACM_lkaHandsoffDisplayWarning": 0,
    }

  data = packer.make_can_msg("ACM_lkaHbaCmd", 0, values)[1]
  values["ACM_lkaHbaCmd_Checksum"] = checksum(data[1:], 0x1D, 0x63)
  return packer.make_can_msg("ACM_lkaHbaCmd", 0, values)


def create_wheel_touch(packer, sccm_wheel_touch, enabled):
  values = {s: sccm_wheel_touch[s] for s in (
    "SCCM_WheelTouch_Counter",
    "SCCM_WheelTouch_HandsOn",
    "SCCM_WheelTouch_CapacitiveValue",
    "SETME_X52",
  )}

  # When only using ACC without lateral, the ACM warns the driver to hold the steering wheel on engagement
  # Tell the ACM that the user is holding the wheel to avoid this warning
  if enabled:
    values["SCCM_WheelTouch_HandsOn"] = 1
    values["SCCM_WheelTouch_CapacitiveValue"] = 100  # only need to send this value, but both are set for consistency

  data = packer.make_can_msg("SCCM_WheelTouch", 2, values)[1]
  values["SCCM_WheelTouch_Checksum"] = checksum(data[1:], 0x1D, 0x97)
  return packer.make_can_msg("SCCM_WheelTouch", 2, values)


def create_longitudinal(packer, frame, accel, enabled):
  values = {
    "ACM_longitudinalRequest_Counter": frame % 15,
    "ACM_AccelerationRequest": accel,
    "ACM_PrndRequired": 0,
    "ACM_longInterfaceEnable": 1 if enabled else 0,
    "ACM_VehicleHoldRequired": 0,
  }

  data = packer.make_can_msg("ACM_longitudinalRequest", 0, values)[1]
  values["ACM_longitudinalRequest_Checksum"] = checksum(data[1:], 0x1D, 0x12)
  return packer.make_can_msg("ACM_longitudinalRequest", 0, values)


def create_adas_status(packer, vdm_adas_status, interface_status):
  # Signal names must match rivian_primaryactuatorCAN.dbc VDM_AdasSts (0x162)
  values = {s: vdm_adas_status[s] for s in (
    "VDM_AdasStatus_Checksum",
    "VDM_AdasStatus_Counter",
    "VDM_AdasDecelLimit",
    "VDM_AdasDriverAccelPriorityStatu",
    "VDM_AdasFaultStatus",
    "VDM_AdasAccelLimit",
    "VDM_AdasDriverModeStatus",
    "VDM_AdasAccelRequest",
    "VDM_AdasInterfaceStatus",
    "VDM_AdasAccelRequestAcknowledged",
    "VDM_AdasVehicleHoldStatus",
  )}

  if interface_status is not None:
    values["VDM_AdasInterfaceStatus"] = interface_status

  data = packer.make_can_msg("VDM_AdasSts", 2, values)[1]
  values["VDM_AdasStatus_Checksum"] = checksum(data[1:], 0x1D, 0xD1)
  return packer.make_can_msg("VDM_AdasSts", 2, values)
