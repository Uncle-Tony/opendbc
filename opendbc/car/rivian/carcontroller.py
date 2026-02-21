import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car import structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.rivian.riviancan import create_angle_steering, create_lka_steering, create_longitudinal, create_wheel_touch, create_adas_status
from opendbc.car.rivian.values import CarControllerParams

from opendbc.sunnypilot.car.rivian.mads import MadsCarController


class CarController(CarControllerBase, MadsCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    MadsCarController.__init__(self)
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.cancel_frames = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    MadsCarController.update(self, CC, CC_SP, CS)
    actuators = CC.actuators
    can_sends = []

    # Angle-based only: send ACM_SteeringControl (0x110) and 0x120
    can_sends.append(create_angle_steering(
      self.packer, self.frame,
      actuators.steeringAngleDeg,
      self.mads.lat_active,
    ))
    can_sends.append(create_lka_steering(
      self.packer, self.frame, CS.acm_lka_hba_cmd, CC.latActive, self.mads,
    ))

    if self.frame % 5 == 0:
      can_sends.append(create_wheel_touch(self.packer, CS.sccm_wheel_touch, CC.enabled))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      can_sends.append(create_longitudinal(self.packer, self.frame, accel, CC.enabled))
    else:
      interface_status = None
      if CC.cruiseControl.cancel:
        # if there is a noEntry, we need to send a status of "available" before the ACM will accept "unavailable"
        # send "available" right away as the VDM itself takes a few frames to acknowledge
        interface_status = 1 if self.cancel_frames < 5 else 0
        self.cancel_frames += 1
      else:
        self.cancel_frames = 0

      can_sends.append(create_adas_status(self.packer, CS.vdm_adas_status, interface_status))

    new_actuators = actuators.as_builder()
    self.frame += 1
    return new_actuators, can_sends
