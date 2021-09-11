from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import CONTROL_N
from selfdrive.modeld.constants import T_IDXS
from selfdrive.config import Conversions as CV
from common.params import Params

LongCtrlState = car.CarControl.Actuators.LongControlState

STOPPING_EGO_SPEED = 0.5
STOPPING_TARGET_SPEED_OFFSET = 0.01
STARTING_TARGET_SPEED = 0.5
DECEL_THRESHOLD_TO_PID = 0.2

DECEL_STOPPING_TARGET = 0.5  # apply at least this amount of brake to maintain the vehicle stationary

RATE = 100.0

# As per ISO 15622:2018 for all speeds
ACCEL_MIN_ISO = -3.5 # m/s^2
ACCEL_MAX_ISO = 2.0 # m/s^2


# TODO this logic isn't really car independent, does not belong here
def long_control_state_trans(active, long_control_state, v_ego, v_target, v_pid,
                             output_accel, brake_pressed, cruise_standstill, stop, gas_pressed,  min_speed_can):
  """Update longitudinal control state machine"""
  stopping_target_speed = min_speed_can + STOPPING_TARGET_SPEED_OFFSET
  stopping_condition = stop or (v_ego < 2.0 and cruise_standstill) or \
                       (v_ego < STOPPING_EGO_SPEED and
                        ((v_pid < stopping_target_speed and v_target < stopping_target_speed) or
                         brake_pressed))

  starting_condition = v_target > STARTING_TARGET_SPEED and not cruise_standstill or gas_pressed

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if active:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.starting

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif output_accel >= -DECEL_THRESHOLD_TO_PID:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl():
  def __init__(self, CP):
    self.long_control_state = LongCtrlState.off  # initialized to off
    self.pid = PIController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                            (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                            rate=RATE,
                            sat_limit=0.8)
    self.v_pid = 0.0
    self.last_output_accel = 0.0

    self.long_log = True

    self.vRel_prev = 0
    self.decel_damping = 1.0
    self.decel_damping2 = 1.0
    self.damping_timer = 0

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update(self, active, CS, CP, long_plan, accel_limits, radarState):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    # TODO estimate car specific lag, use .15s for now
    if len(long_plan.speeds) == CONTROL_N:
      v_target = interp(CP.longitudinalActuatorDelay, T_IDXS[:CONTROL_N], long_plan.speeds)
      v_target_future = long_plan.speeds[-1]
      a_target = 2 * (v_target - long_plan.speeds[0])/CP.longitudinalActuatorDelay - long_plan.accels[0]

      # Only use lag compensation for braking
      a_target = min(a_target, long_plan.accels[0])
      v_target = min(v_target, long_plan.speeds[0])
    else:
      v_target = 0.0
      v_target_future = 0.0
      a_target = 0.0

    # TODO: This check is not complete and needs to be enforced by MPC
    a_target = clip(a_target, ACCEL_MIN_ISO, ACCEL_MAX_ISO)

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    # Update state machine
    output_accel = self.last_output_accel

    if radarState is None:
      dRel = 200
      vRel = 0
    else:
      dRel = radarState.leadOne.dRel
      vRel = radarState.leadOne.vRel
    if long_plan.hasLead:
      stop = True if (dRel < 4.0 and radarState.leadOne.status) else False
    else:
      stop = False
    self.long_control_state = long_control_state_trans(active, self.long_control_state, CS.vEgo,
                                                       v_target_future, self.v_pid, output_accel,
                                                       CS.brakePressed, CS.cruiseState.standstill, CP.minSpeedCan, stop, CS.gasPressed)

    v_ego_pid = max(CS.vEgo, CP.minSpeedCan)  # Without this we get jumps, CAN bus reports 0 when speed < 0.3

    if self.long_control_state == LongCtrlState.off or CS.brakePressed or CS.gasPressed:
      self.v_pid = v_ego_pid
      self.pid.reset()
      output_accel = 0.

    # tracking objects and driving
    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      prevent_overshoot = not CP.stoppingControl and CS.vEgo < 1.5 and v_target_future < 0.7
      deadzone = interp(v_ego_pid, CP.longitudinalTuning.deadzoneBP, CP.longitudinalTuning.deadzoneV)

      # opkr
      if self.vRel_prev != vRel and vRel <= 0 and CS.vEgo > 13. and self.damping_timer <= 0: # decel mitigation for a while
        if (vRel - self.vRel_prev)*3.6 < -4:
          self.damping_timer = 45
          self.decel_damping2 = interp(abs((vRel - self.vRel_prev)*3.6), [0, 10], [1, 0.1])
        self.vRel_prev = vRel
      elif self.damping_timer > 0:
        self.damping_timer -= 1
        self.decel_damping = interp(self.damping_timer, [0, 45], [1, self.decel_damping2])

      freeze_integrator = prevent_overshoot

      output_accel = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, deadzone=deadzone, feedforward=a_target, freeze_integrator=freeze_integrator)
      output_accel *= self.decel_damping

      if prevent_overshoot or CS.brakeHold:
        output_accel = min(output_accel, 0.0)

    # Intention is to stop, switch to a different brake control until we stop
    elif self.long_control_state == LongCtrlState.stopping:
      # Keep applying brakes until the car is stopped
      if not CS.standstill or output_accel > -DECEL_STOPPING_TARGET:
        output_accel -= CP.stoppingDecelRate / RATE
      elif CS.cruiseState.standstill and output_accel < -DECEL_STOPPING_TARGET:
        output_accel += CP.stoppingDecelRate / RATE
      output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

      self.reset(CS.vEgo)

    # Intention is to move again, release brake fast before handing control to PID
    elif self.long_control_state == LongCtrlState.starting:
      factor = 1
      if long_plan.hasLead:
        factor = interp(dRel,[4.0,5.0], [1.0,100.0])
      if output_accel < -DECEL_THRESHOLD_TO_PID:
        output_accel += CP.startingAccelRate / RATE * factor
      self.reset(CS.vEgo)

    self.last_output_accel = output_accel
    final_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    if CP.sccBus != 0 and self.long_log:
      str_log3 = 'GS={:01.2f}/{:01.2f}  BK={:01.2f}/{:01.2f}  GB={:+04.2f}  GS={}  RD={:04.1f}  TG={:04.2f}/{:+04.2f}'.format(final_gas, gas_max, abs(final_brake), abs(brake_max), output_gb, int(CS.gasPressed), CS.radarDistance)
      trace1.printf2('{}'.format(str_log3))

    return final_accel
