#this was initiated by atom(conan)
#partially modified by opkr
import math
import numpy as np
import cereal.messaging as messaging

from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.long_mpc import LongitudinalMpc
from selfdrive.controls.lib.lane_planner import TRAJECTORY_SIZE
from selfdrive.car.hyundai.values import Buttons
from common.numpy_fast import interp
from common.params import Params

import common.log as trace1

MAX_SPEED = 255.0
MIN_CURVE_SPEED = 30.

class SpdController():
    def __init__(self, CP=None):
        self.seq_step_debug = 0
        self.long_curv_timer = 0
        self.path_x = np.arange(192)

        self.wait_timer2 = 0

        self.cruise_set_speed_kph = 0
        self.curise_set_first = 0
        self.curise_sw_check = 0
        self.prev_clu_CruiseSwState = 0    

        self.prev_VSetDis  = 0
        
        self.sc_clu_speed = 0
        self.btn_type = Buttons.NONE
        self.active_time = 0

        self.old_model_speed = 0
        self.old_model_init = 0

        self.curve_speed = 0
        self.curvature_gain = 1

        self.params = Params()
        self.cruise_set_mode = int(self.params.get("CruiseStatemodeSelInit", encoding="utf8"))
        self.map_spd_limit_offset = int(self.params.get("OpkrSpeedLimitOffset", encoding="utf8"))

        self.map_spd_enable = False
        self.map_spd_camera = 0
        self.map_enabled = False
        self.second = 0
        self.sm = messaging.SubMaster(['liveMapData'])

    def cal_curve_speed(self, sm, v_ego):
        md = sm['modelV2']
        if md is not None and len(md.position.x) == TRAJECTORY_SIZE and len(md.position.y) == TRAJECTORY_SIZE:
            x = md.position.x
            y = md.position.y
            dy = np.gradient(y, x)
            d2y = np.gradient(dy, x)
            curv = d2y / (1 + dy ** 2) ** 1.5
            curv = curv[5:TRAJECTORY_SIZE-10]
            a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
            v_curvature = np.sqrt(a_y_max / np.clip(np.abs(curv), 1e-4, None))
            model_speed = np.mean(v_curvature) * 0.9 * self.curvature_gain
            self.curve_speed = float(max(model_speed, MIN_CURVE_SPEED * CV.KPH_TO_MS))
            # if model_speed < v_ego:
            #     self.curve_speed = float(max(model_speed, MIN_CURVE_SPEED * CV.KPH_TO_MS))
            # else:
            #     self.curve_speed = MAX_SPEED
            if np.isnan(self.curve_speed):
                self.curve_speed = MAX_SPEED
        else:
            self.curve_speed = MAX_SPEED
        self.curve_speed = min(MAX_SPEED, self.curve_speed * CV.MS_TO_KPH)

        return self.curve_speed

    def update_cruiseSW(self, CS):
        set_speed_kph = int(round(self.cruise_set_speed_kph))
        delta_vsetdis = 0
        if CS.acc_active:
            delta_vsetdis = abs(int(CS.VSetDis) - self.prev_VSetDis)
            if self.prev_clu_CruiseSwState != CS.cruise_buttons:
                if CS.cruise_buttons == Buttons.RES_ACCEL or CS.cruise_buttons == Buttons.SET_DECEL:
                    self.prev_VSetDis = int(CS.VSetDis)
                elif CS.driverOverride:
                    set_speed_kph = int(CS.VSetDis)
                elif self.prev_clu_CruiseSwState == Buttons.RES_ACCEL:   # up 
                    if self.curise_set_first:
                        self.curise_set_first = 0
                        set_speed_kph =  int(CS.VSetDis)
                    elif delta_vsetdis > 0:
                        set_speed_kph = int(CS.VSetDis)
                    elif not self.curise_sw_check:
                        set_speed_kph += 1
                elif self.prev_clu_CruiseSwState == Buttons.SET_DECEL:  # dn
                    if self.curise_set_first:
                        self.curise_set_first = 0
                        set_speed_kph = int(CS.VSetDis)
                    elif delta_vsetdis > 0:
                        set_speed_kph = int(CS.VSetDis)
                    elif not self.curise_sw_check:
                        set_speed_kph -= 1

                self.prev_clu_CruiseSwState = CS.cruise_buttons
            elif (CS.cruise_buttons == Buttons.RES_ACCEL or CS.cruise_buttons == Buttons.SET_DECEL) and delta_vsetdis > 0:
                self.curise_sw_check = True
                set_speed_kph = int(CS.VSetDis)
        else:
            self.curise_sw_check = False
            self.curise_set_first = 1
            self.prev_VSetDis = int(CS.VSetDis)
            set_speed_kph = int(CS.VSetDis)
            if self.prev_clu_CruiseSwState != CS.cruise_buttons:  # MODE Change
                if CS.cruise_buttons == Buttons.GAP_DIST and not CS.acc_active and CS.out.cruiseState.available:
                    self.cruise_set_mode += 1
                if self.cruise_set_mode > 5:
                    self.cruise_set_mode = 0
                self.prev_clu_CruiseSwState = CS.cruise_buttons

        if set_speed_kph <= 20 and CS.is_set_speed_in_mph:
            set_speed_kph = 20
        elif set_speed_kph <= 30 and not CS.is_set_speed_in_mph:
            set_speed_kph = 30

        self.cruise_set_speed_kph = set_speed_kph
        return self.cruise_set_mode, set_speed_kph

    def get_tm_speed(self, CS, set_time, add_val):
        time = int(set_time)
        set_speed = int(CS.VSetDis) + add_val

        return time, set_speed

    def lead_control(self, CS, sm):
        plan = sm['longitudinalPlan']
        dRel = int(plan.dRel1)
        yRel = int(plan.yRel1)
        vRel = int(plan.vRel1 * 3.6 + 0.5)
        active_time = 10
        btn_type = Buttons.NONE
        #lead_1 = sm['radarState'].leadOne
        long_wait_cmd = 500
        set_speed = int(round(self.cruise_set_speed_kph))

        if self.long_curv_timer < 600:
            self.long_curv_timer += 1

        # var cruise
        lead_wait_cmd, lead_set_speed = self.update_lead(sm, CS, dRel, yRel, vRel)

        # curv slowness
        curve_speed = self.curve_speed   #cal_curve_speed(sm, CS.out.vEgo)
        curv_wait_cmd, curv_set_speed = self.update_curv(CS, sm, curve_speed)

        if curv_wait_cmd != 0:
            if lead_set_speed > curv_set_speed:
                set_speed = curv_set_speed
                long_wait_cmd = curv_wait_cmd
            else:
                set_speed = lead_set_speed
                long_wait_cmd = lead_wait_cmd
        else:
            set_speed = lead_set_speed
            long_wait_cmd = lead_wait_cmd

        if set_speed >= int(round(self.cruise_set_speed_kph)):
            set_speed = int(round(self.cruise_set_speed_kph))
        elif set_speed <= 20 and CS.is_set_speed_in_mph:
            set_speed = 20
        elif set_speed <= 30 and not CS.is_set_speed_in_mph:
            set_speed = 30

        # control process
        target_set_speed = set_speed
        delta = int(round(set_speed)) - int(CS.VSetDis)
        dec_step_cmd = 1

        self.second += 1
        if self.second > 200:
            self.map_enabled = self.params.get_bool("OpkrMapEnable")
            self.second = 0

        if self.map_enabled:
            self.sm.update(0)
            self.map_spd_camera = float(self.sm['liveMapData'].speedLimit)
            self.map_spd_enable = True if self.map_spd_camera > 29 else False
        else:
            self.map_spd_camera = CS.out.safetySign
            self.map_spd_enable = True if self.map_spd_camera > 29. else False

        if self.long_curv_timer < long_wait_cmd:
            pass
        elif delta > 0:
            if ((self.map_spd_camera+round(self.map_spd_camera*0.01*self.map_spd_limit_offset)) == int(CS.VSetDis)) and self.map_spd_enable:
                set_speed = int(CS.VSetDis) + 0
                btn_type = Buttons.NONE
                self.long_curv_timer = 0
            else:
                set_speed = int(CS.VSetDis) + dec_step_cmd
                btn_type = Buttons.RES_ACCEL
                self.long_curv_timer = 0
        elif delta < 0:
            set_speed = int(CS.VSetDis) - dec_step_cmd
            btn_type = Buttons.SET_DECEL
            self.long_curv_timer = 0
        if self.cruise_set_mode == 0:
            btn_type = Buttons.NONE

        self.update_log( CS, set_speed, target_set_speed, long_wait_cmd )

        return btn_type, set_speed, active_time


    def update(self, CS, sm):
        self.cruise_set_mode = CS.out.cruiseState.modeSel
        #self.cruise_set_speed_kph = int(round(CS.out.cruiseState.speed * CV.MS_TO_KPH))
        self.cruise_set_speed_kph = int(round(sm['lateralPlan'].vCruiseSet))
        if CS.driverOverride == 2 or not CS.acc_active or CS.cruise_buttons == Buttons.RES_ACCEL or CS.cruise_buttons == Buttons.SET_DECEL:
            self.resume_cnt = 0
            self.btn_type = Buttons.NONE
            self.wait_timer2 = 10
            self.active_timer2 = 0
        elif self.wait_timer2:
            self.wait_timer2 -= 1
        else:
            btn_type, clu_speed, active_time = self.lead_control(CS, sm)   # speed controller spdcontroller.py

            if (0 <= int(CS.clu_Vanz) <= 1 or 7 < int(CS.clu_Vanz) < 15) and sm['longitudinalPlan'].vRel1 <= 0:
                self.btn_type = Buttons.NONE
            elif self.btn_type != Buttons.NONE:
                pass
            elif btn_type != Buttons.NONE:
                self.resume_cnt = 0
                self.active_timer2 = 0
                self.btn_type = btn_type
                self.sc_clu_speed = clu_speed                
                self.active_time = max( 5, active_time )

            if self.btn_type != Buttons.NONE:
                self.active_timer2 += 1
                if self.active_timer2 > self.active_time:
                    self.wait_timer2 = 5
                    self.resume_cnt = 0
                    self.active_timer2 = 0
                    self.btn_type = Buttons.NONE          
                else:
                    return 1
        return 0