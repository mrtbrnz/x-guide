#!/usr/bin/env python3
from __future__ import print_function

import sys
from os import path, getenv

from math import radians, atan2
import time
import numpy as np

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink
import settings
import pprz_connect
from settings_xml_parse import PaparazziACSettings

from pprzlink.ivy import IvyMessagesInterface

from pprzlink.message import PprzMessage

from vector_fields import TrajectoryEllipse, ParametricTrajectory, spheric_geo_fence, repel, Controller

import pdb

from scipy.spatial.transform import Rotation


class Commands():
    def __init__(self, ac_id, interface):
        self._ac_id = ac_id
        self._interface=interface

    def set_guided_mode(self, quad_id = None):
        """
        change mode to GUIDED.
        """
        if self.ap_mode is not None:
            msg = PprzMessage("ground", "DL_SETTING")
            msg['ac_id'] = self._ac_id
            msg['index'] = self.ap_mode.index
            try:
                msg['value'] = self.ap_mode.ValueFromName('Guided')  # AP_MODE_GUIDED
            except ValueError:
                try:
                    msg['value'] = self.ap_mode.ValueFromName('GUIDED')  # AP_MODE_GUIDED
                except ValueError:
                    msg['value'] = 19 # fallback to fixed index
            print("Setting mode to GUIDED: %s" % msg)
            self._interface.send(msg)

    def set_nav_mode(self):
        """
        change mode to NAV.
        """
        if self.ap_mode is not None:
            msg = PprzMessage("ground", "DL_SETTING")
            msg['ac_id'] = self._ac_id
            msg['index'] = self.ap_mode.index
            try:
                msg['value'] = self.ap_mode.ValueFromName('Nav')  # AP_MODE_NAV
            except ValueError:
                try:
                    msg['value'] = self.ap_mode.ValueFromName('NAV')  # AP_MODE_NAV
                except ValueError:
                    msg['value'] = 13 # fallback to fixed index
            print("Setting mode to NAV: %s" % msg)
            self._interface.send(msg)

    def takeoff(self):
        pass    

    def jump_to_block(self, block_id):
        """
        Jump to Flight Block 
        """
        msg = PprzMessage("ground", "JUMP_TO_BLOCK")
        msg['ac_id'] = self._ac_id
        msg['block_id'] = block_id
        print("jumping to block: %s" % block_id)
        print("ac id: %s" % self._ac_id)
        self._interface.send(msg)

    def land(self):
        pass

    def accelerate(self, north=0.0, east=0.0, down=0.0, flag=0):
        print('Accelerating')
        msg = PprzMessage("datalink", "DESIRED_SETPOINT")
        msg['ac_id'] = self._ac_id
        msg['flag'] = flag # 0:2D, 1:full 3D
        msg['ux'] = north
        msg['uy'] = east
        msg['uz'] = down
        self._interface.send(msg)


class FlightStatus(object):
    def __init__(self, ac_id):
        self._ac_id = ac_id
        self._state = None
        self._current_task = None
        self._current_task_key = None
        self._current_task_time = None
        self._current_task_duration = None
        self._current_task_last_time = 0.0
        self._mission_plan={}

    # @property
    def mission_plan(self):
        return self._mission_plan

    # @mission_plan.setter
    def set_mission_plan(self,m_p_dict):
        self._mission_plan = m_p_dict

    def check_current_task(self):
        # Assuming that the dictionaries will be always ordered... hmmm FIX_ME !!!
        # finalized_tasks =  [self._mission_plan[_k]['finalized'] for _k in self._mission_plan.keys()]
        # first_not_finalized_index = next((i for i, j in enumerate(finalized_tasks) if not j), None)
        # self._current_task = self._mission_plan.keys()[first_not_finalized_index]
        for _k in self._mission_plan.keys():
            if self._mission_plan[_k]['finalized'] == False:
                self._current_task = self._mission_plan[_k]
                self._current_task_key = _k
                self._current_task_duration = self._current_task['duration']
                if self._current_task['start'] != None : self._current_task_time = time.time()-self._current_task['start']
                break
        # Decide to finalize task according to its duration:
        if self._current_task['start'] == None :
            self._current_task['start'] = time.time()
        else:
            if time.time()-self._current_task['start'] > self._current_task_duration :
                self._mission_plan[self._current_task_key]['finalized'] = True


        # print(self._current_task_key)
        # return self._current_task
        #print(f'Mission Plan :{_k} is {self._mission_plan[_k]['start']}')
            
    @property
    def task(self):
        self.check_current_task()
        return self._current_task_key

    def get_current_task_time(self):
        self.check_current_task()
        return self._current_task_time

class Circle():
    def __init__(self, radius=1.5, cx=0.0, cy=0.0, cz=4.0):
        self.radius = radius
        self.cx = cx
        self.cy = cy
        self.cz = cz


class Vehicle(object):
    def __init__(self, ac_id, interface):
        self._initialized = False
        self._take_off = False
        self._morphed = False
        self._morph_state = 0.
        self._fault = False
        self._land = False
        self._ac_id = ac_id
        self._interface = interface
        self._position_initial = np.zeros(3) # Initial Position for safely landing after Ctrl+C
        self._initial_heading = 0.
        self._des_heading = 0.
        self._position = np.zeros(3) # Position
        self._velocity = np.zeros(3) # Velocity
        self._quat = np.zeros(4) # Quaternion
        self._euler = np.zeros(3) # phi-theta-psi Angles ! 
        self._w = np.zeros(3)
        self.gvf_parameter = 0
        self.sm = None  # settings manager
        self.timeout = 0
        self.battery_voltage = 12.0 # As a start... FIXME
        self.cmd = Commands(self._ac_id, self._interface)
        self.fs = FlightStatus(self._ac_id)


        self.ka = 1.6 #acceleration setpoint coeff
        self.circle_vel = 0.6 #m/s
        self.belief_map = {}

    def __str__(self):
        conf_str = f'A/C ID {self._ac_id}'
        return conf_str

    @property
    def id(self):
        return self._ac_id

    @property
    def state(self):
        return self._state

    def assign_properties(self):
        # while self._initialized == False :
            print('Initialization :::',self._initialized)
            ex = 0 ; ey = 0 ; ealpha = 0 ; ea = 1.1 ; eb = 1.1
            print(f'We are inside assign properties for id {self._ac_id}')
            # We have the current position of the vehicle and can set it to the circle center
            self.traj = TrajectoryEllipse(np.array([self._position[0], self._position[1]]), ealpha, ea, eb)
            self._circle = Circle(radius=1.1, cx=self._position[0], cy=self._position[1], cz=self._position[2])

            self._position_initial = self._position.copy() # Just for starting point assignement
            self._initial_heading = self._euler[2] #0. # self.sm["nav_heading"].value
            print(f'Heading initialized to : {self._initial_heading}')

            self.ctr = Controller(L=1e-1,beta=1e-2,k1=1e-3,k2=1e-3,k3=1e-3,ktheta=0.5,s=0.50)
            # self.traj_parametric = ParametricTrajectory(XYZ_off=np.array([0.,0.,2.5]),
            #                                             XYZ_center=np.array([1.3, 1.3, -0.6]),
            #                                             XYZ_delta=np.array([0., np.pi/2, 0.]),
            #                                             XYZ_w=np.array([1,1,1]),
            #                                             alpha=0.,
            #                                             controller=self.ctr)
            self.traj_parametric = ParametricTrajectory(XYZ_off=np.array([0.,0.,2.5]),
                                                        XYZ_center=np.array([1.3, 1.3, 0.]),
                                                        XYZ_delta=np.array([np.pi/2., 0. , 0.]),
                                                        XYZ_w=np.array([2,1,1]),
                                                        alpha=np.pi,
                                                        controller=self.ctr)

    def get_vector_field(self,mission_task, position=None):
        V_des = np.zeros(3)
        if position is not None:
            V_des += spheric_geo_fence(position[0], position[1], position[2], x_source=0., y_source=0., z_source=0., strength=-0.07)
        else:
            V_des += spheric_geo_fence(self._position[0], self._position[1], self._position[2], x_source=0., y_source=0., z_source=0., strength=-0.07)
        for _k in self.belief_map.keys():
            # float(self.belief_map[_k]['X'])
            V_des += repel(self._position[0], self._position[1], self._position[2], 
                    x_source=float(self.belief_map[_k]['X']), 
                    y_source=float(self.belief_map[_k]['Y']), 
                    z_source=float(self.belief_map[_k]['Z']), strength=5.0)

        return V_des

    def send_acceleration(self, V_des, A_3D=False, height_2D=2.):
        err = V_des - self._velocity#[:2]
        print(f'Velocity error {err[0]} , {err[1]}, {err[2]}')
        acc = err*self.ka
        if A_3D :
            self.cmd.accelerate(acc[0],acc[1],-acc[2], flag=1)
        else:
            self.cmd.accelerate(acc[0],acc[1],height_2D, flag=0) # Z is fixed to have a constant altitude... FIXME for 3D !
        # return acc


    def calculate_cmd(self, mission_task):
        V_des = self.get_vector_field(self.fs.task)

        def resurrect(self):
            print('Resurrection')
            if self._fault:
                for i in range(3):
                    self.sm["M1"] = 1.0
                    print('Resurrecting M1')
                    self.sm["M2"] = 1.0
                    print('Resurrecting M2')
                    self.sm["M3"] = 1.0
                    print('Resurrecting M3')
                    self.sm["M4"] = 1.0
                    print('Resurrecting M4')
                    self.sm["M5"] = 1.0
                    print('Resurrecting M5')
                    self.sm["M6"] = 1.0
                    print('Resurrecting M6')
                self._fault = False

        def calc_energy_based_errors(self):
            pos_err = self._position_initial - self._position


        def calculate_path_following_error(verbose=False):
            x = self._position[0] - self._circle.cx
            y = self._position[1] - self._circle.cy
            z = self._position[2] - self._circle.cz

            error_r = self._circle.radius - np.sqrt(x**2+y**2)
            error = np.linalg.norm([error_r, z])
            # i=0; error=np.zeros(len(error_r))
            # for er,ez in zip(error_r, z):
            #     error[i] = la.norm([er,ez])
            #     i+=1
            # if verbose :
            #     print('Mean : ',np.mean(error),'Standart deviation: ', np.std(error))
            #     print('Path following error',np.sum(error)/i)
            return error


        def spin_heading(step=0.01):
            ss = np.sign(step)
            self._des_heading = self._des_heading + step
            if ss<0 : # CCW spin
                self._des_heading = 3.14 if self._des_heading< -3.1415 else self._des_heading
            else:
                self._des_heading = -3.14 if self._des_heading>3.1415 else self._des_heading
            self.sm["nav_heading"] = self._des_heading*2**12

        def follow_circle(height_2D=3.):
            print('We are circling!!!')
            V_des = self.traj.get_vector_field(self._position[0], self._position[1], self._position[2])*self.circle_vel
            # Getting and setting the navigation heading of the vehicles
            follow_heading = False #True
            if follow_heading:
                heading_des = (1.5707963267948966-atan2(V_des[0],V_des[1]))*2**12
                # heading_cur = self.sm["nav_heading"].value
                # print(f'Nav heading error is : {heading_des-heading_cur}')
                if self.sm:
                    self.sm["nav_heading"] = heading_des

            self.send_acceleration(V_des, A_3D=False, height_2D=height_2D)

        def follow_panel_path(height_2D=2.):
            print('We are following the path plan streamlines !!!')
            V_des = self.traj.get_vector_field(self._position[0], self._position[1], self._position[2])*self.circle_vel
            # Getting and setting the navigation heading of the vehicles
            follow_heading = False #True
            if follow_heading:
                heading_des = (1.5707963267948966-atan2(V_des[0],V_des[1]))*2**12
                # heading_cur = self.sm["nav_heading"].value
                # print(f'Nav heading error is : {heading_des-heading_cur}')
                if self.sm:
                    self.sm["nav_heading"] = heading_des

            self.send_acceleration(V_des, A_3D=False, height_2D=height_2D)


        def fail_motor(motor='M1', step=1.):
            if not self._fault:
                # for i in range(10):
                cur_eff = self.sm[motor].value
                print(f'Cur_eff : {cur_eff}')
                # pdb.set_trace()
                if cur_eff!=None:
                    self.sm[motor] = np.clip( (cur_eff - step) ,0. , 1.)
                    self._fault = True if self.sm[motor].value == 0.0 else False

        def recover_motor(self, motor='M1', step=0.1):
            if self._fault:
                # for i in range(10):
                self.sm[motor] = np.clip( (self.sm[motor].value + step) ,0. , 1.)
                self._fault = False if self.sm[motor].value == 1.0 else True

        def morph(gamma=1.0):
            gamma = np.clip(gamma, -1., 1.)
            self.sm["morph_common"] = gamma
            self._morph_state = gamma

        def morph_if_needed(pos_threshold=2.5, ang_threshold=1.57, safe_morph=1.0, in_circle=False):
            if in_circle:
                norm_pos_err = calculate_path_following_error()
                print('Calculating pos error in circle', norm_pos_err)
            else:
                pos_err = self._position_initial - self._position
                norm_pos_err = np.linalg.norm(pos_err)

            heading_err = self._initial_heading - self._euler[2]
            print(f'Heading  err : {heading_err}')
                # print(f'Heading  cur : {self._euler[2]}')
                # print(f'Position err : {norm_pos_err}')

            if norm_pos_err > pos_threshold or abs(heading_err) > ang_threshold :
                print('!!! Auto-Morph !!!')
                # self._morph_direction = 
                morph_cmd = safe_morph
                self.sm["morph_common"] = morph_cmd
                self._morph_state = morph_cmd

        def sequence(mtime, motor,step_var,gamma, start_time,morph_phase=5., fail_phase=15, recovery_phase=5):
            if start_time<= mtime < start_time+morph_phase:
                morph(gamma=gamma)
            if start_time+morph_phase<= mtime < start_time+morph_phase+fail_phase:
                print('Motor Failed !')
                fail_motor(motor=motor, step=step_var)
            if start_time+morph_phase+fail_phase<= mtime < start_time+morph_phase+fail_phase+recovery_phase:
                print('Recover the Motor')
                recover_motor(self, motor=motor, step=step_var)
            print(f' Gamma : {gamma:.2f}')
            time = start_time+morph_phase+fail_phase+recovery_phase
            # finished = 1 if mtime >= time
            return time #, finished


        if self.battery_voltage < 9.5:#13.8:
            print('Battery Voltage : ', self.battery_voltage)
            # morph(gamma=1.0)
            mission_task = 'land'

        if mission_task == 'morph':
            print('Morphing')
            if not self._morphed:
                morph_cmd = 1.0
                self.sm["morph_common"] = morph_cmd
                self._morph_state = morph_cmd
                self._morphed = True
                # self.sm["morph_cmd_1"] = morph_cmd
                # self.sm["morph_cmd_2"] = -morph_cmd
                # morph_period, morph_dance

        if mission_task == 'debug_mode':
            print(f'Debug Mode')
            pos_err = self._position_initial - self._position

            # norm_pos_err = np.linalg.norm(pos_err)
            # print(f'Position err : {norm_pos_err}')
            spin_heading(step=0.01)

            heading_err = self._des_heading - self._euler[2]

            # print(f'Heading  err : {heading_err}')
            # print(f'Heading  cur : {self._euler[2]}')

            # print(f'W  cur : {self._euler[0]:.3f}  ,  {self._euler[1]:.3f}  ,  {self._euler[2]:.3f}')
            # print(f' QUAT ::: {self._quat[0]:.2f}  ,  {self._quat[1]:.2f}  ,  {self._quat[2]:.2f}  ,  {self._quat[3]:.2f}')

        if mission_task == 'follow_path_plan':
            follow_circle(height_2D=2.)

        if mission_task == 'Explore_robustness':
            print(f'Exploring Robustness at : {self._morph_state}')

            self._morph_state = np.clip((self._morph_state - 0.005), -1.0, 1.0)  # 0.005 @ 10Hz gives 40s for Y-X-Y transition
            self.sm["morph_common"] = self._morph_state

            # morph_if_needed(pos_threshold=1.5, ang_threshold=1.0, safe_morph=0.6)
            morph_if_needed(pos_threshold=2.5, ang_threshold=1.57, safe_morph=1.0, in_circle=False)


        if mission_task == 'Explore_robustness_circle_step':
            mtime = self.fs.get_current_task_time()
            step_var=1.0
            motor="M6"
            if 0<= mtime < 2.:
                morph(gamma=1.0)
                motor_list=['M1','M2','M3','M4','M5','M6']
                for _M in motor_list:
                    self.sm[_M] = 1.0
            
            start_time = sequence(mtime, motor, step_var, gamma=1.0,  start_time=5.,         morph_phase=1., fail_phase=15., recovery_phase=0.)
            start_time = sequence(mtime, motor, step_var, gamma=0.8,  start_time=start_time, morph_phase=1., fail_phase=15., recovery_phase=0.)
            start_time = sequence(mtime, motor, step_var, gamma=0.6,  start_time=start_time, morph_phase=1., fail_phase=15., recovery_phase=0.)
            start_time = sequence(mtime, motor, step_var, gamma=0.3,  start_time=start_time, morph_phase=1., fail_phase=15., recovery_phase=0.)
            start_time = sequence(mtime, motor, step_var, gamma=0.2,  start_time=start_time, morph_phase=1., fail_phase=15., recovery_phase=0.)
            start_time = sequence(mtime, motor, step_var, gamma=0.1,  start_time=start_time, morph_phase=1., fail_phase=15., recovery_phase=0.)
            start_time = sequence(mtime, motor, step_var, gamma=0.0,  start_time=start_time, morph_phase=1., fail_phase=15., recovery_phase=0.)
            start_time = sequence(mtime, motor, step_var, gamma=-0.1, start_time=start_time, morph_phase=1., fail_phase=15., recovery_phase=0.)
            start_time = sequence(mtime, motor, step_var, gamma=-0.2, start_time=start_time, morph_phase=1., fail_phase=15., recovery_phase=0.)
            start_time = sequence(mtime, motor, step_var, gamma=-0.3, start_time=start_time, morph_phase=1., fail_phase=15., recovery_phase=0.)
            start_time = sequence(mtime, motor, step_var, gamma=-0.6, start_time=start_time, morph_phase=1., fail_phase=15., recovery_phase=0.)
            start_time = sequence(mtime, motor, step_var, gamma=-0.8, start_time=start_time, morph_phase=1., fail_phase=15., recovery_phase=0.)
            start_time = sequence(mtime, motor, step_var, gamma=-1.0, start_time=start_time, morph_phase=1., fail_phase=15., recovery_phase=0.)

            follow_circle(height_2D=4.)

            morph_if_needed(pos_threshold=2.5, ang_threshold=1.57, safe_morph=1.0, in_circle=True)



        if mission_task == 'Explore_robustness_hover_step':
            mtime = self.fs.get_current_task_time()
            step_var=1.0
            motor="M6"
            if 0<= mtime < 2.:
                morph(gamma=1.0)
                motor_list=['M1','M2','M3','M4','M5','M6']
                for _M in motor_list:
                    self.sm[_M] = 1.0

            start_time = sequence(mtime, motor, step_var, gamma=1.0,  start_time=2.,         morph_phase=5., fail_phase=15., recovery_phase=8.)
            start_time = sequence(mtime, motor, step_var, gamma=0.8,  start_time=start_time, morph_phase=5., fail_phase=15., recovery_phase=8.)
            start_time = sequence(mtime, motor, step_var, gamma=0.6,  start_time=start_time, morph_phase=5., fail_phase=15., recovery_phase=8.)
            start_time = sequence(mtime, motor, step_var, gamma=0.3,  start_time=start_time, morph_phase=5., fail_phase=15., recovery_phase=8.)
            start_time = sequence(mtime, motor, step_var, gamma=0.2,  start_time=start_time, morph_phase=5., fail_phase=15., recovery_phase=8.)
            start_time = sequence(mtime, motor, step_var, gamma=0.1,  start_time=start_time, morph_phase=5., fail_phase=15., recovery_phase=8.)
            start_time = sequence(mtime, motor, step_var, gamma=0.0,  start_time=start_time, morph_phase=5., fail_phase=15., recovery_phase=8.)
            start_time = sequence(mtime, motor, step_var, gamma=-0.1, start_time=start_time, morph_phase=5., fail_phase=15., recovery_phase=8.)
            start_time = sequence(mtime, motor, step_var, gamma=-0.2, start_time=start_time, morph_phase=5., fail_phase=15., recovery_phase=8.)
            start_time = sequence(mtime, motor, step_var, gamma=-0.3, start_time=start_time, morph_phase=5., fail_phase=15., recovery_phase=8.)
            start_time = sequence(mtime, motor, step_var, gamma=-0.6, start_time=start_time, morph_phase=5., fail_phase=15., recovery_phase=8.)
            start_time = sequence(mtime, motor, step_var, gamma=-0.8, start_time=start_time, morph_phase=5., fail_phase=15., recovery_phase=8.)
            start_time = sequence(mtime, motor, step_var, gamma=-1.0, start_time=start_time, morph_phase=5., fail_phase=15., recovery_phase=8.)


            morph_if_needed(pos_threshold=2.6, ang_threshold=1.57)
            # morph_if_needed(pos_threshold=2.5, ang_threshold=1.57, safe_morph=1.0, in_circle=False)

            # if 2<= mtime < 12.:
            #     print('Motor Failed !')
            #     fail_motor(motor=motor, step=step_var)
            # # if 6<= mtime < 30.:
            # #     print('Morphing towards 0 !')
            # #     morph(gamma=(self._morph_state - 0.005) )
            # if 12<= mtime < 14.:
            #     print('Recover the Motor')
            #     recover_motor(self, motor=motor, step=step_var)

            # if 14<= mtime < 16.:
            #     morph(gamma=0.6)
            # if 16<= mtime < 26.:
            #     print('Motor Failed !')
            #     fail_motor(motor=motor, step=step_var)
            # if 26<= mtime < 28.:
            #     print('Recover the Motor')
            #     recover_motor(self, motor=motor, step=step_var)

            # if 28<= mtime < 30.:
            #     morph(gamma=0.3)
            # if 30<= mtime < 40.:
            #     print('Motor Failed !')
            #     fail_motor(motor=motor, step=step_var)
            # if 40<= mtime < 42.:
            #     print('Recover the Motor')
            #     recover_motor(self, motor=motor, step=step_var)

            # if 42<= mtime < 44.:
            #     morph(gamma=0.2)
            # if 44<= mtime < 54.:
            #     print('Motor Failed !')
            #     fail_motor(motor=motor, step=step_var)
            # if 54<= mtime < 56.:
            #     print('Recover the Motor')
            #     recover_motor(self, motor=motor, step=step_var)

            # if 56<= mtime < 58.:
            #     morph(gamma=0.1)
            # if 58<= mtime < 68.:
            #     print('Motor Failed !')
            #     fail_motor(motor=motor, step=step_var)
            # if 68<= mtime < 70.:
            #     print('Recover the Motor')
            #     recover_motor(self, motor=motor, step=step_var)

            # if 70<= mtime < 72.:
            #     morph(gamma=0.0)
            # if 72<= mtime < 82.:
            #     print('Motor Failed !')
            #     fail_motor(motor=motor, step=step_var)
            # if 82<= mtime < 84.:
            #     print('Recover the Motor')
            #     recover_motor(self, motor=motor, step=step_var)



            # Periodically do the below things :

            # spin_heading(step=0.03)
            # morph_if_needed(pos_threshold=2.5, ang_threshold=1.57)



        if mission_task == 'M1_fault':
            print('Fault on M1')
            fail_motor('M1')

        if mission_task == 'M2_fault':
            print('Fault on M2')
            if not self._fault:
                for i in range(2):
                    self.sm["M2"] = 0.0
                self._fault=True

        if mission_task == 'M3_fault':
            print('Fault on M3')
            if not self._fault:
                for i in range(2):
                    self.sm["M3"] = 0.0
                self._fault=True

        if mission_task == 'M4_fault':
            print('Fault on M4')
            if not self._fault:
                for i in range(2):
                    self.sm["M4"] = 0.0
                self._fault=True

        if mission_task == 'M5_fault':
            print('Fault on M5')
            if not self._fault:
                for i in range(2):
                    self.sm["M5"] = 0.0
                self._fault=True

        if mission_task == 'M6_fault':
            print('Fault on M6')
            if not self._fault:
                for i in range(2):
                    self.sm["M6"] = 0.0
                self._fault=True

        if mission_task == 'Resurrect1':
            resurrect(self)
        if mission_task == 'Resurrect2':
            resurrect(self)
        if mission_task == 'Resurrect3':
            resurrect(self)
        if mission_task == 'Resurrect4':
            resurrect(self)
        if mission_task == 'Resurrect5':
            resurrect(self)
        if mission_task == 'Resurrect6':
            resurrect(self)
        if mission_task == 'Resurrect7':
            resurrect(self)


        if mission_task == 'takeoff':
            print('TAKE-OFF!!!')
            if not self._take_off :
                self.cmd.jump_to_block(2)
                time.sleep(0.5)
                self.cmd.jump_to_block(3)
                self._take_off = True

        elif mission_task == 'circle':
            print('We are circling!!!')
            V_des += self.traj.get_vector_field(self._position[0], self._position[1], self._position[2])*self.circle_vel
            # Getting and setting the navigation heading of the vehicles
            follow_heading = True
            if follow_heading:
                heading_des = (1.5707963267948966-atan2(V_des[0],V_des[1]))*2**12
                # heading_cur = self.sm["nav_heading"].value
                # print(f'Nav heading error is : {heading_des-heading_cur}')
                if self.sm:
                    self.sm["nav_heading"] = heading_des

            self.send_acceleration(V_des)

        elif mission_task == 'parametric_circle':
            print('We are circling with parametric circle !!!')
            # now = time.time() #self.fs.get_current_task_time()
            # dt = now-self.fs._current_task_last_time
            # self._current_task_last_time = now
            V_des_increment,uw = self.traj_parametric.get_vector_field(self._position[0], self._position[1], self._position[2], self.gvf_parameter)
            # import pdb
            # pdb.set_trace()
            # print(f'Shape of V_des : {V_des_increment.shape}')
            # print(f'dt : {0.1}, parameter : {self.gvf_parameter} ')
            V_des += V_des_increment 
            self.gvf_parameter += -uw[0]*0.1 #dt

            # Getting and setting the navigation heading of the vehicles
            # print(f'Nav heading value is : {self.sm["nav_heading"]}')
            # if self.sm:
            #     self.sm["nav_heading"] = (1.5707963267948966-atan2(V_des[0],V_des[1]))*2**12
            # self.sm["nav_heading"] = 0.0

            self.send_acceleration(V_des, A_3D=True)

        # print(self.belief_map.keys())
        elif mission_task == 'nav2land':
            print('We are going for landing!!!')
            self.send_acceleration(V_des) # This is 2D with fixed 2m altitude height AGL
            if self.fs._current_task_time > 3. : self.cmd.jump_to_block(5)


        elif mission_task == 'land':
            print('We are landing!!!')
            if not self._land :
                self.cmd.jump_to_block(12)
                self._land = True

        elif mission_task == 'safe2land':
            V_des = self.get_vector_field(self.fs.task, position=self._position_initial)
            self.send_acceleration(V_des)
        # else mission_task == 'kill' :


    def run(self):
        # while True:
        print(f'Running the vehicle {self._ac_id} in {self.fs.task} state ')
        self.calculate_cmd(self.fs.task)



# Callback for PprzConnect notify
def new_ac(conf):
    print(conf)

class SingleControl(object):
    def __init__(self, verbose=False, interface=None, quad_ids = None):
        self.verbose = verbose
        self._interface = interface
        # self._connect = pprz_connect.PprzConnect(notify=new_ac, ivy=self._interface, verbose=False)
        # if self._interface == None : self._interface = self._connect.ivy
        # time.sleep(0.5)
        # self._vehicle_id_list={}
        self._vehicle_position_map = {}
        self.update_vehicle_list()
        self.define_interface_callback()
        time.sleep(0.5)

    def define_interface_callback(self):

        def rotorcraft_fp_cb(ac_id, msg): # FIX ME : use single external function for this, instead of repeating...
            # print(self._vehicle_id_list)
            # ac_id = int(msg['ac_id'])
            # print(ac_id)
            if ac_id in self._vehicle_id_list and msg.name == "ROTORCRAFT_FP":
                rc = self.vehicles[self._vehicle_id_list.index(ac_id)]
                i2p = 1. / 2**8     # integer to position
                i2v = 1. / 2**19    # integer to velocity
                i2w = 1. / 2**12     # integer to angle
                rc._position[0] = float(msg['north']) * i2p
                rc._position[1] = float(msg['east']) * i2p
                rc._position[2] = float(msg['up']) * i2p
                rc._velocity[0] = float(msg['vnorth']) * i2v
                rc._velocity[1] = float(msg['veast']) * i2v
                rc._velocity[2] = float(msg['vup']) * i2v
                rc._euler[0] = float(msg['phi']) * i2w
                rc._euler[1] = float(msg['theta']) * i2w
                rc._euler[2] = float(msg['psi']) * i2w
                self._vehicle_position_map[ac_id] = {'X':rc._position[0],'Y':rc._position[1],'Z':rc._position[2]}
                rc.timeout = 0
                rc._initialized = True

        self._interface.callback = rotorcraft_fp_cb
        self._interface.start()

    def update_vehicle_list(self):
        self._vehicle_id_list=[42]#[int(_id) for _id in self._connect.conf_by_id().keys()]
        self.vehicles = [Vehicle(id, self._interface) for id in self._vehicle_id_list]
        # self.vehicle = Vehicle(42,self._interface)
        # self.create_vehicles()

    # def create_vehicles(self):
    #     self._vehicle_id_list=[int(_id) for _id in self._connect.conf_by_id().keys()]
    #     self.vehicles = [Vehicle(id, self._interface) for id in self._vehicle_id_list]

    def assign(self,mission_plan_dict):
        i=0
        for _id in self._vehicle_id_list:
            print(f'Vehicle id :{_id} mission plan updated ! ')
            rc = self.vehicles[self._vehicle_id_list.index(_id)]
            # rc.sm = settings.PprzSettingsManager(self._connect.conf_by_id(str(rc.id)).settings, str(rc.id), self._connect.ivy)
            rc.fs.set_mission_plan(mission_plan_dict)
            rc.gvf_parameter = (len(self._vehicle_id_list)-i)*30.
            i+=1
    
    def assign_vehicle_properties(self):
        for _id in self._vehicle_id_list:
            rc = self.vehicles[self._vehicle_id_list.index(_id)]
            rc.assign_properties()
               
    def update_belief_map(self, vehicle):
        for _k in self._vehicle_position_map.keys():
            if _k != vehicle._ac_id:
                vehicle.belief_map[_k] = self._vehicle_position_map[_k]

    def run_vehicle(self):
        for _id in self._vehicle_id_list:
            rc = self.vehicles[self._vehicle_id_list.index(_id)]
            self.update_belief_map(rc)
            rc.run()

    def shutdown(self):
        if self._interface is not None:
            print("Shutting down THE interface...")
            self._interface.shutdown()
            self._interface = None

    def __del__(self):
        self.shutdown()

class MissionControl(object):
    def __init__(self, verbose=False, interface=None, quad_ids = None):
        self.verbose = verbose
        self._interface = interface
        self._connect = pprz_connect.PprzConnect(notify=new_ac, ivy=self._interface, verbose=False)
        if self._interface == None : self._interface = self._connect.ivy
        time.sleep(0.5)
        # self._vehicle_id_list={}
        self._vehicle_position_map = {}
        self.update_vehicle_list()  # self.create_vehicles()
        self.subscribe_to_msg()
        time.sleep(0.5)
        # self.assign_vehicle_properties()

    def run_every_vehicle(self):
        # Once it is threaded, below lines can be used to start each vehicles runtime
        for _id in self._vehicle_id_list:
            rc = self.vehicles[self._vehicle_id_list.index(_id)]
            # print(f'Vehicle id :{_id} and its index :{self._vehicle_id_list.index(_id)} Position {rc._position[1]}')
            self.update_belief_map(rc)
            rc.run()

    def assign(self,mission_plan_dict):
        i=0
        for _id in self._vehicle_id_list:
            print(f'Vehicle id :{_id} mission plan updated ! ')
            rc = self.vehicles[self._vehicle_id_list.index(_id)]
            rc.sm = settings.PprzSettingsManager(self._connect.conf_by_id(str(rc.id)).settings, str(rc.id), self._connect.ivy)
            rc.fs.set_mission_plan(mission_plan_dict)
            rc.gvf_parameter = (len(self._vehicle_id_list)-i)*30.
            i+=1


    def assign_vehicle_properties(self):
        for _id in self._vehicle_id_list:
            rc = self.vehicles[self._vehicle_id_list.index(_id)]
            rc.assign_properties()

    def update_belief_map(self, vehicle):
        for _k in self._vehicle_position_map.keys():
            if _k != vehicle._ac_id:
                vehicle.belief_map[_k] = self._vehicle_position_map[_k]

    def update_vehicle_list(self):
        self._connect.get_aircrafts() # Not sure if we need that all the time, as it is subscribed for every NEW_AIRCRAFT...
        self.create_vehicles()

    def create_vehicles(self):
        self._vehicle_id_list=[int(_id) for _id in self._connect.conf_by_id().keys()]
        self.vehicles = [Vehicle(id, self._interface) for id in self._vehicle_id_list]

    def subscribe_to_msg(self):
        # bind to ENERGY message
        def energy_cb(ac_id, msg):
            if ac_id in self._vehicle_id_list and msg.name == "ENERGY":
                rc = self.vehicles[self._vehicle_id_list.index(ac_id)]
                rc.battery_voltage = float(msg['voltage'])

        self._interface.subscribe(energy_cb, PprzMessage("telemetry", "ENERGY"))

        # bind to INS message
        def ins_cb(ac_id, msg):
            # if ac_id in self.ids and msg.name == "INS":
            #     rc = self.rotorcrafts[self.ids.index(ac_id)]
            if ac_id in self._vehicle_id_list and msg.name == "INS":
                rc = self.vehicles[self._vehicle_id_list.index(ac_id)]
                i2p = 1. / 2**8     # integer to position
                i2v = 1. / 2**19    # integer to velocity
                rc._position[0] = float(msg['ins_x']) * i2p
                rc._position[1] = float(msg['ins_y']) * i2p
                rc._position[2] = float(msg['ins_z']) * i2p
                rc._velocity[0] = float(msg['ins_xd']) * i2v
                rc._velocity[1] = float(msg['ins_yd']) * i2v
                rc._velocity[2] = float(msg['ins_zd']) * i2v
                rc.timeout = 0
                rc._initialized = True
        # self._interface.subscribe(ins_cb, PprzMessage("telemetry", "INS"))

        #################################################################
        def rotorcraft_fp_cb(ac_id, msg):
            # print(self._vehicle_id_list)
            # ac_id = int(msg['ac_id'])
            # print(ac_id)
            if ac_id in self._vehicle_id_list and msg.name == "ROTORCRAFT_FP":
                rc = self.vehicles[self._vehicle_id_list.index(ac_id)]
                i2p = 1. / 2**8     # integer to position
                i2v = 1. / 2**19    # integer to velocity
                i2w = 1. / 2**12     # integer to angle
                rc._position[0] = float(msg['north']) * i2p
                rc._position[1] = float(msg['east']) * i2p
                rc._position[2] = float(msg['up']) * i2p
                rc._velocity[0] = float(msg['vnorth']) * i2v
                rc._velocity[1] = float(msg['veast']) * i2v
                rc._velocity[2] = float(msg['vup']) * i2v
                rc._euler[0] = float(msg['phi']) * i2w
                rc._euler[1] = float(msg['theta']) * i2w
                rc._euler[2] = float(msg['psi']) * i2w
                self._vehicle_position_map[ac_id] = {'X':rc._position[0],'Y':rc._position[1],'Z':rc._position[2]}
                rc.timeout = 0
                rc._initialized = True
        
        # Un-comment this if the quadrotors are providing state information to use_deep_guidance.py
        # self._interface.subscribe(rotorcraft_fp_cb, PprzMessage("telemetry", "ROTORCRAFT_FP"))
    
        # bind to GROUND_REF message : ENAC Voliere is sending LTP_ENU
        def ground_ref_cb(ground_id, msg):
            ac_id = int(msg['ac_id'])
            if ac_id in self._vehicle_id_list:
                rc = self.vehicles[self._vehicle_id_list.index(ac_id)]
                # X and V in NED
                rc._position[0] = float(msg['pos'][1])
                rc._position[1] = float(msg['pos'][0])
                rc._position[2] = float(msg['pos'][2])
                rc._velocity[0] = float(msg['speed'][1])
                rc._velocity[1] = float(msg['speed'][0])
                rc._velocity[2] = float(msg['speed'][2])
                # Unitary quaternion representing LTP to BODY orientation (qi, qx, qy, qz)
                rc._quat[0] = float(msg['quat'][0])
                rc._quat[1] = float(msg['quat'][2])
                rc._quat[2] = float(msg['quat'][1])
                rc._quat[3] = -float(msg['quat'][3])

                rc._euler = quat2euler(rc._quat)

                rc._w[0] = float(msg['rate'][0])
                rc._w[1] = float(msg['rate'][1])
                rc._w[2] = float(msg['rate'][2])
                # pdb.set_trace()
                # print(f'Ground REF ac_id {ac_id}')
                # print(f'X:{rc._position[0]} Y: {rc._position[1]} Z: {rc._position[2]}')
                self._vehicle_position_map[ac_id] = {'X':rc._position[0],'Y':rc._position[1],'Z':rc._position[2]}
                rc.timeout = 0
                rc._initialized = True
        
        # Un-comment this if optitrack is being used for state information for use_deep_guidance.py **For use only in the Voliere**
        self._interface.subscribe(ground_ref_cb, PprzMessage("ground", "GROUND_REF"))

        ################################################################

    # <message name="ROTORCRAFT_FP" id="147">
    #   <field name="east"     type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
    #   <field name="north"    type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
    #   <field name="up"       type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
    #   <field name="veast"    type="int32" alt_unit="m/s" alt_unit_coef="0.0000019"/>
    #   <field name="vnorth"   type="int32" alt_unit="m/s" alt_unit_coef="0.0000019"/>
    #   <field name="vup"      type="int32" alt_unit="m/s" alt_unit_coef="0.0000019"/>
    #   <field name="phi"      type="int32" alt_unit="deg" alt_unit_coef="0.0139882"/>
    #   <field name="theta"    type="int32" alt_unit="deg" alt_unit_coef="0.0139882"/>
    #   <field name="psi"      type="int32" alt_unit="deg" alt_unit_coef="0.0139882"/>
    #   <field name="carrot_east"   type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
    #   <field name="carrot_north"  type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
    #   <field name="carrot_up"     type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
    #   <field name="carrot_psi"    type="int32" alt_unit="deg" alt_unit_coef="0.0139882"/>
    #   <field name="thrust"        type="int32"/>
    #   <field name="flight_time"   type="uint16" unit="s"/>
    # </message>

    # <message name="INS" id="198">
    #   <field name="ins_x"     type="int32" alt_unit="m"    alt_unit_coef="0.0039063"/>
    #   <field name="ins_y"     type="int32" alt_unit="m"    alt_unit_coef="0.0039063"/>
    #   <field name="ins_z"     type="int32" alt_unit="m"    alt_unit_coef="0.0039063"/>
    #   <field name="ins_xd"    type="int32" alt_unit="m/s"  alt_unit_coef="0.0000019"/>
    #   <field name="ins_yd"    type="int32" alt_unit="m/s"  alt_unit_coef="0.0000019"/>
    #   <field name="ins_zd"    type="int32" alt_unit="m/s"  alt_unit_coef="0.0000019"/>
    #   <field name="ins_xdd"   type="int32" alt_unit="m/s2" alt_unit_coef="0.0009766"/>
    #   <field name="ins_ydd"   type="int32" alt_unit="m/s2" alt_unit_coef="0.0009766"/>
    #   <field name="ins_zdd"   type="int32" alt_unit="m/s2" alt_unit_coef="0.0009766"/>
    # </message>

    def shutdown(self):
        if self._interface is not None:
            print("Shutting down THE interface...")
            self._interface.shutdown()
            self._interface = None

    def __del__(self):
        self.shutdown()

# def rotorcraft_fp_cb(ac_id, msg):
#             # print(self._vehicle_id_list)
#             # ac_id = int(msg['ac_id'])
#             print('BAAM ', ac_id)
#             if ac_id in self._vehicle_id_list and msg.name == "ROTORCRAFT_FP":
#                 rc = self.vehicles[self._vehicle_id_list.index(ac_id)]
#                 i2p = 1. / 2**8     # integer to position
#                 i2v = 1. / 2**19    # integer to velocity
#                 i2w = 1. / 2**12     # integer to angle
#                 rc._position[0] = float(msg['north']) * i2p
#                 rc._position[1] = float(msg['east']) * i2p
#                 rc._position[2] = float(msg['up']) * i2p
#                 rc._velocity[0] = float(msg['vnorth']) * i2v
#                 rc._velocity[1] = float(msg['veast']) * i2v
#                 rc._velocity[2] = float(msg['vup']) * i2v
#                 rc._euler[2] = float(msg['psi']) * i2w
#                 self._vehicle_position_map[ac_id] = {'X':rc._position[0],'Y':rc._position[1],'Z':rc._position[2]}
#                 rc.timeout = 0
#                 rc._initialized = True

def quat2euler(quat):
    '''quat : Unitary quaternion representing LTP to BODY orientation (qi, qx, qy, qz) '''
    # from scipy.spatial.transform import Rotation
    _quat=np.array([quat[1], quat[2], quat[3], quat[0] ])
    rot = Rotation.from_quat(_quat)
    # rot_euler = rot.as_euler('yxz', degrees=False)
    rot_euler = rot.as_euler('xyz', degrees=False)
    return rot_euler


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Mission Control")
    parser.add_argument("-on", "--running-on", help="Where is the code running-on", dest='running_on', default='ground')
    parser.add_argument("-f", "--file", help="path to messages.xml file", default='pprzlink/messages.xml')
    parser.add_argument("-c", "--class", help="message class", dest='msg_class', default='telemetry')
    parser.add_argument("-d", "--device", help="device name", dest='dev', default='/dev/ttyUSB0') #ttyTHS1
    parser.add_argument("-b", "--baudrate", help="baudrate", dest='baud', default=230400, type=int)
    parser.add_argument("-id", "--ac_id", help="aircraft id (receiver)", dest='ac_id', default=42, type=int)
    parser.add_argument("--interface_id", help="interface id (sender)", dest='id', default=0, type=int)
    # parser.add_argument("-ti", "--target_id", dest='target_id', default=2, type=int, help="Target aircraft ID")
    # parser.add_argument("-ri", "--repel_id", dest='repel_id', default=2, type=int, help="Repellant aircraft ID")
    # parser.add_argument("-bi", "--base_id", dest='base_id', default=10, type=int, help="Base aircraft ID")
    args = parser.parse_args()

    if args.running_on == 'ground' :
        interface  = IvyMessagesInterface("PprzConnect")

    if args.running_on == "serial" :
        from pprzlink.serial import SerialMessagesInterface
        interface = SerialMessagesInterface(None, device=args.dev,
                                               baudrate=args.baud, msg_class=args.msg_class, interface_id=args.id, verbose=False)


    mission_plan_dict={# 'takeoff' :{'start':None, 'duration':20, 'finalized':False},
                        # 'circle'  :{'start':None, 'duration':15, 'finalized':False},
                        'parametric_circle'  :{'start':None, 'duration':15, 'finalized':False},
                        # 'nav2land':{'start':None, 'duration':5,  'finalized':False},
                        # 'land'    :{'start':None, 'duration':10, 'finalized':False} } #,
                        # 'kill'    :{'start':None, 'duration':10, 'finalized':False} }
                        }
    # mission_plan_dict={ 'parametric_circle'  :{'start':None, 'duration':15, 'finalized':False} }

    vehicle_parameter_dict={}

    if args.running_on == 'ground' :
        try:
            mc = MissionControl(interface=interface)
            mc.assign(mission_plan_dict)
            mc.assign_vehicle_properties()
            time.sleep(1.5)

            while True:
                mc.run_every_vehicle()
                time.sleep(0.09)

        except (KeyboardInterrupt, SystemExit):
            mission_end_plan_dict={'safe2land'  :{'start':None, 'duration':15, 'finalized':False}, }
            mc.assign(mission_end_plan_dict)  # mc.assign_vehicle_properties()
            time.sleep(0.5)
            for i in range(10):
                mc.run_every_vehicle()
                time.sleep(0.5)
            print('Shutting down...')
            # mc.set_nav_mode()
            mc.shutdown()
            time.sleep(0.6)
            exit()

    if args.running_on == 'serial' :
        try:
            sc = SingleControl(interface=interface)
            sc.assign(mission_plan_dict)
            sc.assign_vehicle_properties()
            time.sleep(1.5)

            while True:
                sc.run_vehicle()
                time.sleep(0.09)

        except (KeyboardInterrupt, SystemExit):
            mission_end_plan_dict={'safe2land'  :{'start':None, 'duration':15, 'finalized':False}, }
            sc.assign(mission_end_plan_dict) # sc.assign_vehicle_properties()
            time.sleep(0.5)
            for i in range(10):
                sc.run_vehicle()
                time.sleep(0.5)
            print('Shutting down...')
            # mc.set_nav_mode()
            sc.shutdown()
            time.sleep(0.6)
            exit()


if __name__ == '__main__':
    main()

#EOF
