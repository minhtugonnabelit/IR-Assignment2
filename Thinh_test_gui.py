
import PySimpleGUI as sg
import roboticstoolbox as rtb
import spatialmath as sm
import spatialmath.base as smb
import numpy as np
from swift import Swift
import spatialgeometry as geometry

from robot.sawyer import Sawyer
from robot.astorino import Astorino
from controller_interface import ControllerInterface
from mission import Mission
from work_cell import WorkCell
from plate import Plate
from rectangularprism import RectangularPrism

import queue
import time
import logging
import threading
import copy
import os
import argparse

# The default log filename
DAY = time.gmtime()
LOG_FILE_NAME = f"{DAY.tm_mday}_{DAY.tm_mon}_{DAY.tm_year}_{os.getlogin()}.log"
# Here is how a log will format to a file
LOG_FORMAT_FILE = "%(asctime)s,%(levelname)s,%(filename)s,%(funcName)s,%(lineno)d,%(message)s"
# Here is how a log will format to the console
LOG_FORMAT_CONSOLE = "%(asctime)s\t%(levelname)s: %(message)s"


def load_args():
    parser = argparse.ArgumentParser(
        description="Logging mode toggle here", formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument('-v', '--verbose', action='store_true',
                        help="Enable debug outputs")

    return parser.parse_args()


class RobotGUI:

    # switch to FULL UPPERCASE?
    _input_size = (10, 1)
    _slider_size = (21, 13)
    _window_size = (900, 700)  # Width x Height
    _cell_center = sm.SE3(0, 0, 0)

    def __init__(self, args=None):

        sg.theme('DarkAmber')

        # Initialize environment and logging system

        self.env = Swift()
        self.env.set_camera_pose([0, 0, 5], self._cell_center.A[0:3, 3])
        self.env.launch(realtime=True)

        # Verbose setup for logging level

        if args.verbose:
            log_level = logging.DEBUG
        else:
            log_level = logging.INFO
        self.log = RobotGUI._init_log(log_level)

        # Init environment object:

        self.work_cell = WorkCell(self.env, self._cell_center)
        self.plate = Plate(self._cell_center @
                           sm.SE3(-0.35, 0.9, 0.905), self.env)

        # Initialize robot

        base_original_robot = self._cell_center @ sm.SE3(
            0.55, 0.8, 0.65) @ sm.SE3.Rz(-np.pi)
        transl2robot = sm.SE3(0, 1.4, 0.163)

        self.sawyer = Sawyer(
            env=self.env,
            base=base_original_robot,
            gripper_ready=True)
        self.sawyer_qlim = np.rad2deg(copy.deepcopy(self.sawyer.qlim))

        self.astorino = Astorino(
            env=self.env,
            base=base_original_robot @ transl2robot,
            gripper_ready=True)
        self.astorino_qlim = np.rad2deg(copy.deepcopy(self.astorino.qlim))

        # Initialize controller

        self.sawyer_controller = ControllerInterface(self.sawyer, self.log)
        self.sawyer_controller.launch()
        self.sawyer_controller.go_to_home()

        self.astorino_controller = ControllerInterface(self.astorino, self.log)
        self.astorino_controller.launch()
        self.astorino_controller.go_to_home()

        # Initialize mission manager

        self.mission = Mission(self.plate,
                               self.work_cell,
                               self.sawyer_controller,
                               self.astorino_controller)

        # set up collision

        self.collision_setup()

        # Initialize GUI

        self.window = self.create_window()
        self.update_gui_thread()

        self.flag_busy_sawyer = False
        self.flag_busy_astorino = False

        self.button_and_slider_keys_sawyer = ['-SLIDER0-', '-SLIDER1-', '-SLIDER2-', '-SLIDER3-', '-SLIDER4-', '-SLIDER5-', '-SLIDER6-',
                                              '-MINUSPITCH-', '-MINUSROLL-', '-MINUSYAW-', '-MINUSX-', '-MINUSY-', '-MINUSZ-', '-PLUSPITCH-', '-PLUSROLL-', '-PLUSX-', '-PLUSY-', '-PLUSYAW-', '-PLUSZ-',
                                              '-END-EFFECTOR-', '-RJOINT-', '-CARTX-', '-CARTY-', '-CARTZ-', '-ROLL-', '-PITCH-', '-YAW-',
                                              '-CONFIRM-', '-ENABLE-', '-ESTOP-', '-GAMEPAD_DISABLE-', '-GAMEPAD_ENABLE-', '-HOME-']

        self.button_and_slider_keys_astorino = ['-A_SLIDER0-', '-A_SLIDER1-', '-A_SLIDER2-', '-A_SLIDER3-', '-A_SLIDER4-', '-A_SLIDER5-',
                                                '-A_MINUSPITCH-', '-A_MINUSROLL-', '-A_MINUSYAW-', '-A_MINUSX-', '-A_MINUSY-', '-A_MINUSZ-', '-A_PLUSPITCH-', '-A_PLUSROLL-', '-A_PLUSX-', '-A_PLUSY-', '-A_PLUSYAW-', '-A_PLUSZ-',
                                                '-A_END-EFFECTOR-', '-A_RJOINT-', '-A_CARTX-', '-A_CARTY-', '-A_CARTZ-', '-A_ROLL-', '-A_PITCH-', '-A_YAW-',
                                                '-A_CONFIRM-', '-A_ENABLE-', '-A_ESTOP-', '-A_GAMEPAD_DISABLE-', '-A_GAMEPAD_ENABLE-', '-A_HOME-']

    def create_window(self):
        tab_group_layout = [
            [sg.TabGroup(
                [[sg.Tab('Sawyer Controller', self.tab1_setup(), background_color='black', key='-TAB1-'),
                  sg.Tab('Astorino Controller', self.tab2_setup(),
                         background_color='black', key='-TAB2-'),
                  sg.Tab('Mission Controller', self.tab3_setup(), background_color='blue')]],
                tab_location='topleft',
                selected_background_color='pink',

                selected_title_color='black',
                title_color='black',
                size=self._window_size,
                enable_events=True
            )]
        ]
        window = sg.Window('System GUI', tab_group_layout, finalize=True,
                           size=self._window_size, location=(0, 0), resizable=False)

        return window

    def update_gui_thread(self):
        self.update_GUI = threading.Thread(
            target=self.gui_updater, daemon=True)
        self.update_GUI.start()

    def gui_updater(self):
        while True:
            # --------------- SAWYER
            new_joint_states_sawyer = np.rad2deg(self.sawyer.get_jointstates())
            self.window.write_event_value(
                '-UPDATE-JOINTS-', new_joint_states_sawyer)
            state_sawyer = self.sawyer_controller.system_state()
            self.window.write_event_value('-UPDATE-STATE-', state_sawyer)
            # Update Real time sliders for sawyer
            if self.sawyer_controller.get_busy_status():
                self.flag_busy_sawyer = True

            if not self.sawyer_controller.get_busy_status() and self.flag_busy_sawyer:
                for i in range(7):
                    value = round(np.rad2deg(self.sawyer.q[i]), 0)
                    self.window['-SLIDER' + str(i) + '-'].update(value=value)
                    self.window.refresh()
                    self.sawyer_controller.update_robot_js()
                    time.sleep(0.01)
                self.flag_busy_sawyer = False

            # ------------------ ASTORINO
            new_joint_states_astorino = np.rad2deg(
                self.astorino.get_jointstates())
            self.window.write_event_value(
                '-A_UPDATE-JOINTS-', new_joint_states_astorino)
            state_astorino = self.astorino_controller.system_state()
            self.window.write_event_value('-A_UPDATE-STATE-', state_astorino)
            # Update Real time sliders for ASTORINO
            if self.astorino_controller.get_busy_status():
                self.flag_busy_astorino = True

            if not self.astorino_controller.get_busy_status() and self.flag_busy_astorino:
                for i in range(6):
                    value = round(np.rad2deg(self.astorino.q[i]), 0)
                    self.window['-A_SLIDER' + str(i) + '-'].update(value=value)
                    self.window.refresh()
                    self.astorino_controller.update_robot_js()
                    time.sleep(0.01)
                self.flag_busy_astorino = False

            time.sleep(0.5)

    def tab1_setup(self):
        headers = [[
            sg.Text(f'X: {self.sawyer.fkine(self.sawyer.q).A[0,3]} m', size=(
                15, 1), justification='right', key='-X-', background_color='black'),
            sg.Text(f'Y: {self.sawyer.fkine(self.sawyer.q).A[1,3]} m', size=(
                15, 1), justification='right', key='-Y-', background_color='black'),
            sg.Text(f'Z: {self.sawyer.fkine(self.sawyer.q).A[2,3]} m', size=(
                15, 1), justification='right', key='-Z-', background_color='black'),
            sg.Text(f'Rx: {np.rad2deg(self.getori(self.sawyer)[0])} deg', size=(
                15, 1), justification='right', key='-RX-', background_color='black'),
            sg.Text(f'Ry: {np.rad2deg(self.getori(self.sawyer)[1])} deg', size=(
                15, 1), justification='right', key='-RY-', background_color='black'),
            sg.Text(f'Rz: {np.rad2deg(self.getori(self.sawyer)[2])} deg', size=(
                15, 1), justification='right', key='-RZ-', background_color='black'),
        ]]

        controller_state = [[
            sg.Text('Status: ', size=(0, 1), justification='right', key='-STATE_LABEL-',
                    background_color='black',  font=('Ubuntu Mono', 12)),
            sg.Text(f'{self.sawyer_controller.system_state()}', size=(
                15, 1), justification='center', key='-STATE-', text_color='black',  background_color=('yellow'))
        ]]

        left_column = [
            [sg.Frame('Joint Space Jogging',
                      [
                          [sg.Slider(range=(self.sawyer_qlim[0, 0], self.sawyer_qlim[1, 0]), default_value=np.rad2deg(self.sawyer.q[0]), orientation='h',
                                     size=self._slider_size, key='-SLIDER0-', enable_events=True, tick_interval=0.1),
                           sg.Text('L1:', size=(2, 1),
                                   background_color='black'),
                           sg.Text(f'{np.rad2deg(self.sawyer.q[0])} deg', size=(
                               6, 1), key='-BASE-', justification='right'),
                           ],

                          [sg.Slider(range=(self.sawyer_qlim[0, 1], self.sawyer_qlim[1, 1]), default_value=np.rad2deg(self.sawyer.q[1]), orientation='h',
                                     size=self._slider_size, key='-SLIDER1-', enable_events=True),
                           sg.Text('L2:', size=(2, 1),
                                   background_color='black'),
                           sg.Text(f'{np.rad2deg(self.sawyer.q[1])} deg', size=(
                               6, 1), key='-J0-', justification='right')
                           ],

                          [sg.Slider(range=(self.sawyer_qlim[0, 2], self.sawyer_qlim[1, 2]), default_value=np.rad2deg(self.sawyer.q[2]), orientation='h',
                                     size=self._slider_size, key='-SLIDER2-', enable_events=True),
                           sg.Text('L3:', size=(2, 1),
                                   background_color='black'),
                           sg.Text(f'{np.rad2deg(self.sawyer.q[2])} deg', size=(
                               6, 1), key='-J1-', justification='right')
                           ],

                          [sg.Slider(range=(self.sawyer_qlim[0, 3], self.sawyer_qlim[1, 3]), default_value=np.rad2deg(self.sawyer.q[3]), orientation='h',
                                     size=self._slider_size, key='-SLIDER3-', enable_events=True),
                           sg.Text('L4:', size=(2, 1),
                                   background_color='black'),
                           sg.Text(f'{np.rad2deg(self.sawyer.q[3])} deg', size=(
                               6, 1), key='-J2-', justification='right')
                           ],

                          [sg.Slider(range=(self.sawyer_qlim[0, 4], self.sawyer_qlim[1, 4]), default_value=np.rad2deg(self.sawyer.q[4]), orientation='h',
                                     size=self._slider_size, key='-SLIDER4-', enable_events=True),
                           sg.Text('L5:', size=(2, 1),
                                   background_color='black'),
                           sg.Text(f'{np.rad2deg(self.sawyer.q[4])} deg', size=(
                               6, 1), key='-J3-', justification='right')
                           ],

                          [sg.Slider(range=(self.sawyer_qlim[0, 5], self.sawyer_qlim[1, 5]), default_value=np.rad2deg(self.sawyer.q[5]), orientation='h',
                                     size=self._slider_size, key='-SLIDER5-', enable_events=True),
                           sg.Text('L6:', size=(2, 1),
                                   background_color='black'),
                           sg.Text(f'{np.rad2deg(self.sawyer.q[5])} deg', size=(
                               6, 1), key='-J4-', justification='right')
                           ],

                          [sg.Slider(range=(self.sawyer_qlim[0, 6], self.sawyer_qlim[1, 6]), default_value=np.rad2deg(self.sawyer.q[6]), orientation='h',
                                     size=self._slider_size, key='-SLIDER6-', enable_events=True),
                           sg.Text('L7:', size=(2, 1),
                                   background_color='black'),
                           sg.Text(f'{np.rad2deg(self.sawyer.q[6])} deg', size=(6, 1), key='-J5-', justification='right')]
                      ], element_justification='center', background_color='black', font=('Ubuntu Mono', 15), title_location=sg.TITLE_LOCATION_TOP)
             ]
        ]

        right_column = [
            [
                sg.Column(
                    [
                        [
                            sg.Frame('TCP Jogging',
                                     [
                                         [sg.Button('+X',    key='-PLUSX-', size=self._input_size), sg.Button('+Y', key='-PLUSY-',
                                                                                                              size=self._input_size), sg.Button('+Z', key='-PLUSZ-', size=self._input_size)],
                                         [sg.Button('-X',    key='-MINUSX-', size=self._input_size), sg.Button(
                                             '-Y', key='-MINUSY-', size=self._input_size), sg.Button('-Z', key='-MINUSZ-', size=self._input_size)],
                                         [sg.Button('+Roll', key='-PLUSROLL-', size=self._input_size), sg.Button('+Pitch', key='-PLUSPITCH-',
                                                                                                                 size=self._input_size), sg.Button('+Yaw', key='-PLUSYAW-', size=self._input_size)],
                                         [sg.Button('-Roll', key='-MINUSROLL-', size=self._input_size), sg.Button(
                                             '-Pitch', key='-MINUSPITCH-', size=self._input_size), sg.Button('-Yaw', key='-MINUSYAW-', size=self._input_size)]
                                     ], element_justification='center', background_color='black', font=('Ubuntu Mono', 15), title_location=sg.TITLE_LOCATION_TOP, vertical_alignment='top')
                        ]
                    ], background_color='black'),

                sg.Column(
                    [
                        [
                            sg.Frame('Input End-effector',
                                     [
                                         [sg.Text('X:',    size=(2, 1), background_color='black', pad=(1, 11)), sg.Input(default_text='0', size=(5, 1), key='-CARTX-'), sg.Text(
                                             'Roll: ', size=(4, 1), background_color='black', pad=(5, 1)), sg.Input(default_text='0', size=(5, 1), key='-ROLL-')],
                                         [sg.Text('Y:',    size=(2, 1), background_color='black', pad=(1, 11)), sg.Input(default_text='0', size=(5, 1), key='-CARTY-'), sg.Text(
                                             'Pitch: ', size=(4, 1), background_color='black', pad=(5, 1)), sg.Input(default_text='0', size=(5, 1), key='-PITCH-')],
                                         [sg.Text('Z:',    size=(2, 1), background_color='black', pad=(1, 11)), sg.Input(default_text='0', size=(
                                             5, 1), key='-CARTZ-'), sg.Text('Yaw: ',  size=(4, 1), background_color='black', pad=(5, 1)), sg.Input(default_text='0', size=(5, 1), key='-YAW-')]
                                     ], element_justification='center', background_color='black', font=('Ubuntu Mono', 15), title_location=sg.TITLE_LOCATION_TOP, vertical_alignment='top')
                        ]
                    ], background_color='black')
            ],

            [sg.Frame('Message',
                      [
                          [sg.Multiline(
                              size=(70, 5), key='-MSG-', background_color='black', text_color='white')]
                      ], background_color='black', font=('Ubuntu Mono', 15), title_location=sg.TITLE_LOCATION_TOP)
             ]
        ]

        type_of_control = [
            [sg.Radio('Real-time Jogging', 'RADIO1', key='-RJOINT-',
                      size=(20, 1), pad=(5, 3), default=True)],
            [sg.Radio('Input End-effector', 'RADIO1',
                      key='-END-EFFECTOR-', size=(20, 1), pad=(5, 3))]

        ]

        # Define the layout for the first tab
        tab1_layout = [
            [
                sg.Frame('SAWYER CONTROLLER', layout=headers, font=(
                    'Ubuntu Mono', 24), background_color='black')
            ],

            [
                sg.Frame('', layout=controller_state, font=(
                    'Ubuntu Mono', 24), background_color='black')
            ],

            [
                sg.Button('ENABLE CONTROLLER', key='-ENABLE-',
                          size=(20, 1), pad=(5, 5)),
                sg.Button('GAMEPAD MODE',
                          key='-GAMEPAD_ENABLE-', size=(15, 1)),
                sg.Button('DISABLE GAMEPAD',
                          key='-GAMEPAD_DISABLE-', size=(18, 1)),
                sg.Button('EMERGENCY STOP', key='-ESTOP-',
                          button_color=('white', 'red'), size=(30, 1))
            ],

            [
                sg.Column(left_column, element_justification='center',
                          background_color='black'),
                sg.Column(right_column, element_justification='center',
                          vertical_alignment='top', background_color='black')
            ],

            [
                sg.Frame('Control Options', layout=type_of_control, font=('Ubuntu Mono', 15),
                         background_color='black', title_location=sg.TITLE_LOCATION_TOP, pad=(55, 5))
            ],

            [
                sg.Button('Confirm', key='-CONFIRM-', size=(16, 3)
                          ), sg.Button('Home', key='-HOME-', size=(16, 3))
            ],
        ]

        return tab1_layout

    def tab2_setup(self):
        headers = [[
            sg.Text(f'X: {self.astorino.fkine(self.astorino.q).A[0,3]} m', size=(
                15, 1), justification='right', key='-A_X-', background_color='black'),
            sg.Text(f'Y: {self.astorino.fkine(self.astorino.q).A[1,3]} m', size=(
                15, 1), justification='right', key='-A_Y-', background_color='black'),
            sg.Text(f'Z: {self.astorino.fkine(self.astorino.q).A[2,3]} m', size=(
                15, 1), justification='right', key='-A_Z-', background_color='black'),
            sg.Text(f'Rx: {np.rad2deg(self.getori(self.astorino)[0])} deg', size=(
                15, 1), justification='right', key='-A_RX-', background_color='black'),
            sg.Text(f'Ry: {np.rad2deg(self.getori(self.astorino)[1])} deg', size=(
                15, 1), justification='right', key='-A_RY-', background_color='black'),
            sg.Text(f'Rz: {np.rad2deg(self.getori(self.astorino)[2])} deg', size=(
                15, 1), justification='right', key='-A_RZ-', background_color='black'),
        ]]

        controller_state = [[
            sg.Text('Status: ', size=(0, 1), justification='right', key='-A_STATE_LABEL-',
                    background_color='black', font=('Ubuntu Mono', 12)),
            sg.Text(f'{self.astorino_controller.system_state()}', size=(
                15, 1), justification='center', key='-A_STATE-', text_color='black', background_color=('yellow'))
        ]]

        left_column = [
            [sg.Frame('Joint Space Jogging',
                      [
                          [sg.Slider(range=(self.astorino_qlim[0, 0], self.astorino_qlim[1, 0]), default_value=np.rad2deg(self.astorino.q[0]), orientation='h',
                                     size=self._slider_size, key='-A_SLIDER0-', enable_events=True, tick_interval=0.1),
                           sg.Text('L1:', size=(2, 1),
                                   background_color='black'),
                           sg.Text(f'{np.rad2deg(self.astorino.q[0])} deg', size=(
                               6, 1), key='-A_BASE-', justification='right'),
                           ],

                          [sg.Slider(range=(self.astorino_qlim[0, 1], self.astorino_qlim[1, 1]), default_value=np.rad2deg(self.astorino.q[1]), orientation='h',
                                     size=self._slider_size, key='-A_SLIDER1-', enable_events=True),
                           sg.Text('L2:', size=(2, 1),
                                   background_color='black'),
                           sg.Text(f'{np.rad2deg(self.astorino.q[1])} deg', size=(
                               6, 1), key='-A_J0-', justification='right')
                           ],

                          [sg.Slider(range=(self.astorino_qlim[0, 2], self.astorino_qlim[1, 2]), default_value=np.rad2deg(self.astorino.q[2]), orientation='h',
                                     size=self._slider_size, key='-A_SLIDER2-', enable_events=True),
                           sg.Text('L3:', size=(2, 1),
                                   background_color='black'),
                           sg.Text(f'{np.rad2deg(self.astorino.q[2])} deg', size=(
                               6, 1), key='-A_J1-', justification='right')
                           ],

                          [sg.Slider(range=(self.astorino_qlim[0, 3], self.astorino_qlim[1, 3]), default_value=np.rad2deg(self.astorino.q[3]), orientation='h',
                                     size=self._slider_size, key='-A_SLIDER3-', enable_events=True),
                           sg.Text('L4:', size=(2, 1),
                                   background_color='black'),
                           sg.Text(f'{np.rad2deg(self.astorino.q[3])} deg', size=(
                               6, 1), key='-A_J2-', justification='right')
                           ],

                          [sg.Slider(range=(self.astorino_qlim[0, 4], self.astorino_qlim[1, 4]), default_value=np.rad2deg(self.astorino.q[4]), orientation='h',
                                     size=self._slider_size, key='-A_SLIDER4-', enable_events=True),
                           sg.Text('L5:', size=(2, 1),
                                   background_color='black'),
                           sg.Text(f'{np.rad2deg(self.astorino.q[4])} deg', size=(
                               6, 1), key='-A_J3-', justification='right')
                           ],

                          [sg.Slider(range=(self.astorino_qlim[0, 5], self.astorino_qlim[1, 5]), default_value=np.rad2deg(self.astorino.q[5]), orientation='h',
                                     size=self._slider_size, key='-A_SLIDER5-', enable_events=True),
                           sg.Text('L6:', size=(2, 1),
                                   background_color='black'),
                           sg.Text(f'{np.rad2deg(self.astorino.q[5])} deg', size=(
                               6, 1), key='-A_J4-', justification='right')
                           ]
                      ], element_justification='center', background_color='black', font=('Ubuntu Mono', 15), title_location=sg.TITLE_LOCATION_TOP)
             ]
        ]

        right_column = [
            [
                sg.Column(
                    [
                        [
                            sg.Frame('TCP Jogging',
                                     [
                                         [sg.Button('+X',    key='-A_PLUSX-', size=self._input_size), sg.Button('+Y', key='-A_PLUSY-',
                                                                                                                size=self._input_size), sg.Button('+Z', key='-A_PLUSZ-', size=self._input_size)],
                                         [sg.Button('-X',    key='-A_MINUSX-', size=self._input_size), sg.Button(
                                             '-Y', key='-A_MINUSY-', size=self._input_size), sg.Button('-Z', key='-A_MINUSZ-', size=self._input_size)],
                                         [sg.Button('+Roll', key='-A_PLUSROLL-', size=self._input_size), sg.Button('+Pitch', key='-A_PLUSPITCH-',
                                                                                                                   size=self._input_size), sg.Button('+Yaw', key='-A_PLUSYAW-', size=self._input_size)],
                                         [sg.Button('-Roll', key='-A_MINUSROLL-', size=self._input_size), sg.Button(
                                             '-Pitch', key='-A_MINUSPITCH-', size=self._input_size), sg.Button('-Yaw', key='-A_MINUSYAW-', size=self._input_size)]
                                     ], element_justification='center', background_color='black', font=('Ubuntu Mono', 15), title_location=sg.TITLE_LOCATION_TOP, vertical_alignment='top')
                        ]
                    ], background_color='black'),

                sg.Column(
                    [
                        [
                            sg.Frame('Input End-effector',
                                     [
                                         [sg.Text('X:',    size=(2, 1), background_color='black', pad=(1, 11)), sg.Input(default_text='0', size=(5, 1), key='-A_CARTX-'), sg.Text(
                                             'Roll: ', size=(4, 1), background_color='black', pad=(5, 1)), sg.Input(default_text='0', size=(5, 1), key='-A_ROLL-')],
                                         [sg.Text('Y:',    size=(2, 1), background_color='black', pad=(1, 11)), sg.Input(default_text='0', size=(5, 1), key='-A_CARTY-'), sg.Text(
                                             'Pitch: ', size=(4, 1), background_color='black', pad=(5, 1)), sg.Input(default_text='0', size=(5, 1), key='-A_PITCH-')],
                                         [sg.Text('Z:',    size=(2, 1), background_color='black', pad=(1, 11)), sg.Input(default_text='0', size=(5, 1), key='-A_CARTZ-'), sg.Text(
                                             'Yaw: ',  size=(4, 1), background_color='black', pad=(5, 1)), sg.Input(default_text='0', size=(5, 1), key='-A_YAW-')]
                                     ], element_justification='center', background_color='black', font=('Ubuntu Mono', 15), title_location=sg.TITLE_LOCATION_TOP, vertical_alignment='top')
                        ]
                    ], background_color='black')
            ],

            [sg.Frame('Message',
                      [
                          [sg.Multiline(
                              size=(70, 5), key='-A_MSG-', background_color='black', text_color='white')]
                      ], background_color='black', font=('Ubuntu Mono', 15), title_location=sg.TITLE_LOCATION_TOP)
             ]
        ]

        type_of_control = [
            [sg.Radio('Real-time Jogging', 'RADIO2', key='-A_RJOINT-',
                      size=(20, 1), pad=(5, 3), default=True)],
            [sg.Radio('Input End-effector', 'RADIO2',
                      key='-A_END-EFFECTOR-', size=(20, 1), pad=(5, 3))]
        ]

        # Define the layout for the second tab
        tab2_layout = [
            [
                sg.Frame('ASTORINO CONTROLLER', layout=headers, font=(
                    'Ubuntu Mono', 24), background_color='black')
            ],

            [
                sg.Frame('', layout=controller_state, font=(
                    'Ubuntu Mono', 24), background_color='black')
            ],

            [
                sg.Button('ENABLE CONTROLLER', key='-A_ENABLE-',
                          size=(20, 1), pad=(5, 5)),
                sg.Button('GAMEPAD MODE',
                          key='-A_GAMEPAD_ENABLE-', size=(15, 1)),
                sg.Button('DISABLE GAMEPAD',
                          key='-A_GAMEPAD_DISABLE-', size=(18, 1)),
                sg.Button('EMERGENCY STOP', key='-A_ESTOP-',
                          button_color=('white', 'red'), size=(30, 1))
            ],

            [
                sg.Column(left_column, element_justification='center',
                          background_color='black'),
                sg.Column(right_column, element_justification='center',
                          vertical_alignment='top', background_color='black')
            ],

            [
                sg.Frame('Control Options', layout=type_of_control, font=('Ubuntu Mono', 15),
                         background_color='black', title_location=sg.TITLE_LOCATION_TOP, pad=(55, 5))
            ],

            [
                sg.Button('Confirm', key='-A_CONFIRM-', size=(16, 3)
                          ), sg.Button('Home', key='-A_HOME-', size=(16, 3))
            ],
        ]

        return tab2_layout

    def tab3_setup(self):
        headers = [[
            sg.Text(f'X: {self.sawyer.fkine(self.sawyer.q).A[0,3]} m', size=(
                15, 1), justification='right', key='-m_X-', background_color='black'),
            sg.Text(f'Y: {self.sawyer.fkine(self.sawyer.q).A[1,3]} m', size=(
                15, 1), justification='right', key='-m_Y-', background_color='black'),
            sg.Text(f'Z: {self.sawyer.fkine(self.sawyer.q).A[2,3]} m', size=(
                15, 1), justification='right', key='-m_Z-', background_color='black'),
            sg.Text(f'Rx: {np.rad2deg(self.getori(self.sawyer)[0])} deg', size=(
                15, 1), justification='right', key='-m_RX-', background_color='black'),
            sg.Text(f'Ry: {np.rad2deg(self.getori(self.sawyer)[1])} deg', size=(
                15, 1), justification='right', key='-m_RY-', background_color='black'),
            sg.Text(f'Rz: {np.rad2deg(self.getori(self.sawyer)[2])} deg', size=(
                15, 1), justification='right', key='-m_RZ-', background_color='black'),
        ],
            [sg.Text(f'X: {self.astorino.fkine(self.astorino.q).A[0,3]} m', size=(
                15, 1), justification='right', key='-mA_X-', background_color='black'),
             sg.Text(f'Y: {self.astorino.fkine(self.astorino.q).A[1,3]} m', size=(
                 15, 1), justification='right', key='-mA_Y-', background_color='black'),
             sg.Text(f'Z: {self.astorino.fkine(self.astorino.q).A[2,3]} m', size=(
                 15, 1), justification='right', key='-mA_Z-', background_color='black'),
             sg.Text(f'Rx: {np.rad2deg(self.getori(self.astorino)[0])} deg', size=(
                 15, 1), justification='right', key='-mA_RX-', background_color='black'),
             sg.Text(f'Ry: {np.rad2deg(self.getori(self.astorino)[1])} deg', size=(
                 15, 1), justification='right', key='-mA_RY-', background_color='black'),
             sg.Text(f'Rz: {np.rad2deg(self.getori(self.astorino)[2])} deg', size=(
                 15, 1), justification='right', key='-mA_RZ-', background_color='black'),]]

        # need a new mission state list
        mission_state = [[
            sg.Text('Status: ', size=(0, 1), justification='right', key='-MISSION_STATE_LABEL-',
                    background_color='black', font=('Ubuntu Mono', 12)),
            # sg.Text(f'{self.mission.system_state()}', size=(
            #     15, 1), justification='center', key='-A_STATE-', text_color='black', background_color=('yellow'))
        ]]


        # Define the layout for the second tab
        tab3_layout = [
            [
                sg.Frame('MISSION MANAGER', layout=headers, font=('Ubuntu Mono', 24), background_color='black')
            ],
            
            [
                sg.Frame('Mission State', layout=mission_state, font=('Ubuntu Mono', 24), background_color='black'), 
            ],

            [
                sg.Button('Home System', key='-MISSION_HOME-', size=(16, 3)), sg.Button('Run Mission', key='-MISSION_RUN-', size=(16, 3))
            ],

            [
                sg.Button('Pause Mission', key='-MISSION_PAUSE-', size=(16, 3)), sg.Button('Resume Mission', key='-MISSION_RESUME-', size=(16, 3))
            ],

            [
                sg.Button('Enable System', key='-MISSION_ENABLE-', size=(16, 3))
            ],

            [
                sg.Button('Emergency Stop', key='-MISSION_STOP-', button_color=('white', 'red'), size=(16, 3))
            ],
        ]

        return tab3_layout

    def run(self):
        self.window['-MSG-'].print('Press ENABLE Button to Start Controlling')
        self.window['-A_MSG-'].print(
            'Press ENABLE Button to Start Controlling')

        # Since lists are mutable, changes made to the list inside the function are reflected outside the function.
        flag_print_once_sawyer = [True]
        flag_print_running_sawyer = [True]

        flag_print_once_astorino = [True]
        flag_print_running_astorino = [True]

        self.mission_thread = threading.Thread(target=self.mission.run)

        while True:
            event, values = self.window.read()
            if event == sg.WIN_CLOSED:
                break
            self.sawyer_teach_pendant(event=event, values=values, flag_print_once_sawyer=flag_print_once_sawyer,
                                      flag_print_running_sawyer=flag_print_running_sawyer)
            self.astorino_teach_pendant(event=event, values=values, flag_print_once_astorino=flag_print_once_astorino,
                                        flag_print_running_astorino=flag_print_running_astorino)
            self.mission_callback(event=event, values=values)
            # self.env.step(0)

        self.sawyer_controller.clean()
        self.astorino_controller.clean()
        self.env.close()
        self.window.close()

    def astorino_teach_pendant(self, event, values, flag_print_once_astorino, flag_print_running_astorino):
        # -------------------------------------------------------------------- astorino

        if values['-A_RJOINT-']:
            for i in range(6):
                if event == f'-A_SLIDER{i}-':
                    self.astorino_controller.set_joint_value(
                        i, values[f'-A_SLIDER{i}-'])
                    self.astorino_controller.update_js()
                    self.env.step(0)
        else:
            for i in range(6):
                self.astorino_controller.set_joint_value(
                    i, values[f'-A_SLIDER{i}-'])

        # Loop through the input keys and convert values to float
        input_values = []
        for key in ['-A_CARTX-', '-A_CARTY-', '-A_CARTZ-', '-A_ROLL-', '-A_PITCH-', '-A_YAW-']:
            input_value = values[key]
            try:
                float_value = float(input_value)
                input_values.append(float_value)
            except ValueError:
                print(f"Invalid input: {input_value}")

        # Convert the input values to a SE3 object and input as Cartesian pose for robot to work out
        pose = self.astorino.base @ sm.SE3(input_values[0], input_values[1], input_values[2]
                                           ) @ sm.SE3.RPY(input_values[3:6], order='xyz', unit='deg')

        self.astorino_controller.set_cartesian_value(pose)

        # -------------------------------------------------------------------- astorino

        # Update for jointstates and cartersian pose values
        if event == '-A_UPDATE-JOINTS-':
            new_joint_states = np.rad2deg(self.astorino.get_jointstates())
            self.window['-A_BASE-'].update(f'{new_joint_states[0]:.2f} ')
            self.window['-A_J0-'].update(f'{new_joint_states[1]:.2f} ')
            self.window['-A_J1-'].update(f'{new_joint_states[2]:.2f} ')
            self.window['-A_J2-'].update(f'{new_joint_states[3]:.2f} ')
            self.window['-A_J3-'].update(f'{new_joint_states[4]:.2f} ')
            self.window['-A_J4-'].update(f'{new_joint_states[5]:.2f} ')

            self.window['-A_X-'].update(
                f'X: {self.astorino.fkine(self.astorino.q).A[0,3]:.2f} m')
            self.window['-A_Y-'].update(
                f'Y: {self.astorino.fkine(self.astorino.q).A[1,3]:.2f} m')
            self.window['-A_Z-'].update(
                f'Z: {self.astorino.fkine(self.astorino.q).A[2,3]:.2f} m')
            self.window['-A_RX-'].update(
                f'Rx: {np.rad2deg(self.getori(self.astorino)[0]):.2f} deg')
            self.window['-A_RY-'].update(
                f'Ry: {np.rad2deg(self.getori(self.astorino)[1]):.2f} deg')
            self.window['-A_RZ-'].update(
                f'Rz: {np.rad2deg(self.getori(self.astorino)[2]):.2f} deg')

        elif event == '-A_UPDATE-STATE-':
            state = self.astorino_controller.system_state()
            self.window['-A_STATE-'].update(f'{state}')

            if state == 'IDLE':
                self.window['-A_STATE-'].update(text_color='purple',
                                                background_color='yellow')
                flag_print_once_astorino[0] = True
                for key in self.button_and_slider_keys_astorino:
                    if key != '-A_ENABLE-':
                        self.window[key].update(disabled=True)

            elif state == 'ENABLED':
                self.window['-A_STATE-'].update(text_color='white',
                                                background_color='green')

                if self.astorino_controller.get_busy_status() == False:
                    flag_print_running_astorino[0] = True
                    for key in self.button_and_slider_keys_astorino:
                        if key == '-A_ENABLE-':
                            self.window[key].update(disabled=True)
                        else:
                            self.window[key].update(disabled=False)
                else:
                    if flag_print_running_astorino[0]:
                        self.window['-A_MSG-'].print("System is running...")
                        flag_print_running_astorino[0] = False
                    for key in self.button_and_slider_keys_astorino:
                        if key == '-A_ESTOP-':
                            self.window[key].update(disabled=False)
                        else:
                            self.window[key].update(disabled=True)

            elif state == 'STOPPED':
                self.window['-A_STATE-'].update(text_color='white',
                                                background_color='red')
                self.window['-A_ESTOP-'].update('Release EMERGENCY STOP')
                if flag_print_once_astorino[0]:
                    self.window['-A_MSG-'].print(
                        'The Astorino Robot has been stopped. Release Button E-stop to Activate')
                    flag_print_once_astorino[0] = False
                for key in self.button_and_slider_keys_astorino:
                    if key != '-A_ESTOP-':
                        self.window[key].update(disabled=True)

            if not values['-A_END-EFFECTOR-'] and not values['-A_RJOINT-']:
                self.window['-A_CONFIRM-'].update(
                    'Choose\n Control Options', disabled=True)
            elif values['-A_END-EFFECTOR-']:
                if not flag_print_running_astorino[0]:
                    self.window['-A_CONFIRM-'].update('Confirm', disabled=True)
                else:
                    self.window['-A_CONFIRM-'].update('Confirm',
                                                      disabled=False)
            elif values['-A_RJOINT-']:
                self.window['-A_CONFIRM-'].update(
                    'Sliders\n Control Mode', disabled=True)

        # event activated with HOME button
        elif event == '-A_HOME-':
            if self.astorino_controller.disable_gamepad is False:
                self.astorino_controller.disable_gamepad()
            self.astorino_controller.send_command('HOME')

        # event enabled with ENABLE button
        elif event == '-A_ENABLE-':
            self.astorino_controller.send_command('ENABLE')
            state = self.astorino_controller.system_state()
            if state == 'IDLE':
                self.window['-A_MSG-'].update('')
                self.window['-A_MSG-'].print('System is Ready')
                self.window['-A_ENABLE-'].update(disabled=True)

        # event activated with E-stop button
        elif event == '-A_ESTOP-':
            state = self.astorino_controller.system_state()

            if not state == 'IDLE':
                self.astorino_controller.update_estop_state()

            if state == 'ENABLED':
                self.window['-A_ESTOP-'].update('Release EMERGENCY STOP')

            elif state == 'STOPPED':
                self.window['-A_MSG-'].print(
                    'Press ENABLE Button to Start Controlling')
                self.window['-A_ENABLE-'].update(disabled=False)
                self.window['-A_ESTOP-'].update('EMERGENCY STOP')

        # event activated with CONFIRM button
        elif event == '-A_CONFIRM-':
            if values['-A_END-EFFECTOR-']:
                self.astorino_controller.send_command('CARTESIAN_POSE')

        elif event == '-A_GAMEPAD_ENABLE-':
            self.astorino_controller.send_command("GAMEPAD_ENABLE")
            # self.astorino_controller.gamepad_control()

        elif event == '-A_GAMEPAD_DISABLE-':
            self.astorino_controller.disable_gamepad()

        # event activated with +X button
        elif event == '-A_PLUSX-':
            self.astorino_controller.send_command('+X')

        # event activated with -X button
        elif event == '-A_MINUSX-':
            self.astorino_controller.send_command('-X')

        # event activated with +Y button
        elif event == '-A_PLUSY-':
            self.astorino_controller.send_command('+Y')

        # event activated with -Y button
        elif event == '-A_MINUSY-':
            self.astorino_controller.send_command('-Y')

        # event activated with +Z button
        elif event == '-A_PLUSZ-':
            self.astorino_controller.send_command('+Z')

        # event activated with -Z button
        elif event == '-A_MINUSZ-':
            self.astorino_controller.send_command('-Z')

        # event activated with +Roll button
        elif event == '-A_PLUSROLL-':
            self.astorino_controller.send_command('+Rx')

        # event activated with -Roll button
        elif event == '-A_MINUSROLL-':
            self.astorino_controller.send_command('-Rx')

        # event activated with +Pitch button
        elif event == '-A_PLUSPITCH-':
            self.astorino_controller.send_command('+Ry')

        # event activated with -Pitch button
        elif event == '-A_MINUSPITCH-':
            self.astorino_controller.send_command('-Ry')

        # event activated with +Yaw button
        elif event == '-A_PLUSYAW-':
            self.astorino_controller.send_command('+Rz')

        # event activated with -Yaw button
        elif event == '-A_MINUSYAW-':
            self.astorino_controller.send_command('-Rz')


    def sawyer_teach_pendant(self, event, values, flag_print_once_sawyer, flag_print_running_sawyer):
        # ------------------------------------------------------------------- sawyer

        if values['-RJOINT-']:
            for i in range(7):
                # self.sawyer_controller.set_joint_value(i, values[f'-SLIDER{i}-'])
                if event == f'-SLIDER{i}-':
                    self.sawyer_controller.set_joint_value(
                        i, values[f'-SLIDER{i}-'])
                    self.sawyer_controller.update_js()
                    self.env.step(0)
        else:
            for i in range(7):
                self.sawyer_controller.set_joint_value(
                    i, values[f'-SLIDER{i}-'])

        # Loop through the input keys and convert values to float
        input_values = []
        for key in ['-CARTX-', '-CARTY-', '-CARTZ-', '-ROLL-', '-PITCH-', '-YAW-']:
            input_value = values[key]
            try:
                float_value = float(input_value)
                input_values.append(float_value)
            except ValueError:
                self.log.warning(f"Invalid input: {input_value}")

        # Convert the input values to a SE3 object and input as Cartesian pose for robot to work out
        pose = sm.SE3(input_values[0], input_values[1], input_values[2]
                      ) @ sm.SE3.RPY(input_values[3:6], order='xyz', unit='deg')

        self.sawyer_controller.set_cartesian_value(pose)
        # -------------------------------------------------------------------- sawyer

        # Update for jointstates and cartersian pose values
        if event == '-UPDATE-JOINTS-':
            new_joint_states = np.rad2deg(self.sawyer.get_jointstates())
            self.window['-BASE-'].update(f'{new_joint_states[0]:.2f} ')
            self.window['-J0-'].update(f'{new_joint_states[1]:.2f} ')
            self.window['-J1-'].update(f'{new_joint_states[2]:.2f} ')
            self.window['-J2-'].update(f'{new_joint_states[3]:.2f} ')
            self.window['-J3-'].update(f'{new_joint_states[4]:.2f} ')
            self.window['-J4-'].update(f'{new_joint_states[5]:.2f} ')
            self.window['-J5-'].update(f'{new_joint_states[6]:.2f} ')

            self.window['-X-'].update(
                f'X: {self.sawyer.fkine(self.sawyer.q).A[0,3]:.2f} m')
            self.window['-Y-'].update(
                f'Y: {self.sawyer.fkine(self.sawyer.q).A[1,3]:.2f} m')
            self.window['-Z-'].update(
                f'Z: {self.sawyer.fkine(self.sawyer.q).A[2,3]:.2f} m')
            self.window['-RX-'].update(
                f'Rx: {np.rad2deg(self.getori(self.sawyer)[0]):.2f} deg')
            self.window['-RY-'].update(
                f'Ry: {np.rad2deg(self.getori(self.sawyer)[1]):.2f} deg')
            self.window['-RZ-'].update(
                f'Rz: {np.rad2deg(self.getori(self.sawyer)[2]):.2f} deg')

        elif event == '-UPDATE-STATE-':
            state = self.sawyer_controller.system_state()
            self.window['-STATE-'].update(f'{state}')

            if state == 'IDLE':
                self.window['-STATE-'].update(text_color='purple',
                                              background_color='yellow')
                flag_print_once_sawyer[0] = True
                for key in self.button_and_slider_keys_sawyer:
                    if key != '-ENABLE-':
                        self.window[key].update(disabled=True)

            elif state == 'ENABLED':
                self.window['-STATE-'].update(text_color='white',
                                              background_color='green')

                if self.sawyer_controller.get_busy_status() == False:
                    flag_print_running_sawyer[0] = True
                    for key in self.button_and_slider_keys_sawyer:
                        if key == '-ENABLE-':
                            self.window[key].update(disabled=True)
                        else:
                            self.window[key].update(disabled=False)
                else:
                    if flag_print_running_sawyer[0]:
                        self.window['-MSG-'].print("System is running...")
                        flag_print_running_sawyer[0] = False
                    for key in self.button_and_slider_keys_sawyer:
                        if key == '-ESTOP-':
                            self.window[key].update(disabled=False)
                        else:
                            self.window[key].update(disabled=True)

            elif state == 'STOPPED':
                self.window['-STATE-'].update(text_color='white',
                                              background_color='red')
                self.window['-ESTOP-'].update('Release EMERGENCY STOP')
                if flag_print_once_sawyer[0]:
                    self.window['-MSG-'].print(
                        'The Sawyer Robot has been stopped. Release Button E-stop to Activate')
                    flag_print_once_sawyer[0] = False
                for key in self.button_and_slider_keys_sawyer:
                    if key != '-ESTOP-':
                        self.window[key].update(disabled=True)

            if not values['-END-EFFECTOR-'] and not values['-RJOINT-']:
                self.window['-CONFIRM-'].update(
                    'Choose\n Control Options', disabled=True)
            elif values['-END-EFFECTOR-']:
                if not flag_print_running_sawyer[0]:
                    self.window['-CONFIRM-'].update('Confirm', disabled=True)
                else:
                    self.window['-CONFIRM-'].update('Confirm', disabled=False)
            elif values['-RJOINT-']:
                self.window['-CONFIRM-'].update(
                    'Sliders\n Control Mode', disabled=True)

        # event activated with HOME button
        elif event == '-HOME-':
            if self.sawyer_controller.disable_gamepad is False:
                self.sawyer_controller.disable_gamepad()
            self.sawyer_controller.send_command('HOME')

        # event enabled with ENABLE button
        elif event == '-ENABLE-':
            self.sawyer_controller.send_command('ENABLE')
            state = self.sawyer_controller.system_state()
            if state == 'IDLE':
                self.window['-MSG-'].update('')
                self.window['-MSG-'].print('System is Ready')
                self.window['-ENABLE-'].update(disabled=True)

        # event activated with E-stop button
        elif event == '-ESTOP-':
            state = self.sawyer_controller.system_state()

            if not state == 'IDLE':
                self.sawyer_controller.update_estop_state()

            if state == 'ENABLED':
                self.window['-ESTOP-'].update('Release EMERGENCY STOP')

            elif state == 'STOPPED':
                self.window['-MSG-'].print(
                    'Press ENABLE Button to Start Controlling')
                self.window['-ENABLE-'].update(disabled=False)
                self.window['-ESTOP-'].update('EMERGENCY STOP')

        # event activated with CONFIRM button
        elif event == '-CONFIRM-':
            if values['-END-EFFECTOR-']:
                self.sawyer_controller.send_command('CARTESIAN_POSE')

        elif event == '-GAMEPAD_ENABLE-':
            self.sawyer_controller.send_command("GAMEPAD_ENABLE")
            # self.sawyer_controller.gamepad_control()

        elif event == '-GAMEPAD_DISABLE-':
            self.sawyer_controller.disable_gamepad()

        # event activated with +X button
        elif event == '-PLUSX-':
            self.sawyer_controller.send_command('+X')

        # event activated with -X button
        elif event == '-MINUSX-':
            self.sawyer_controller.send_command('-X')

        # event activated with +Y button
        elif event == '-PLUSY-':
            self.sawyer_controller.send_command('+Y')

        # event activated with -Y button
        elif event == '-MINUSY-':
            self.sawyer_controller.send_command('-Y')

        # event activated with +Z button
        elif event == '-PLUSZ-':
            self.sawyer_controller.send_command('+Z')

        # event activated with -Z button
        elif event == '-MINUSZ-':
            self.sawyer_controller.send_command('-Z')

        # event activated with +Roll button
        elif event == '-PLUSROLL-':
            self.sawyer_controller.send_command('+Rx')

        # event activated with -Roll button
        elif event == '-MINUSROLL-':
            self.sawyer_controller.send_command('-Rx')

        # event activated with +Pitch button
        elif event == '-PLUSPITCH-':
            self.sawyer_controller.send_command('+Ry')

        # event activated with -Pitch button
        elif event == '-MINUSPITCH-':
            self.sawyer_controller.send_command('-Ry')

        # event activated with +Yaw button
        elif event == '-PLUSYAW-':
            self.sawyer_controller.send_command('+Rz')

        # event activated with -Yaw button
        elif event == '-MINUSYAW-':
            self.sawyer_controller.send_command('-Rz')


    def mission_callback(self, event, values,):

        if self.mission.mission_completed():
            self.mission_thread.join()
            self.mission.reset_mission()
        
        if event == '-A_UPDATE-JOINTS-':

            self.window['-mA_X-'].update(
                f'X: {self.astorino.fkine(self.astorino.q).A[0,3]:.2f} m')
            self.window['-mA_Y-'].update(
                f'Y: {self.astorino.fkine(self.astorino.q).A[1,3]:.2f} m')
            self.window['-mA_Z-'].update(
                f'Z: {self.astorino.fkine(self.astorino.q).A[2,3]:.2f} m')
            self.window['-mA_RX-'].update(
                f'Rx: {np.rad2deg(self.getori(self.astorino)[0]):.2f} deg')
            self.window['-mA_RY-'].update(
                f'Ry: {np.rad2deg(self.getori(self.astorino)[1]):.2f} deg')
            self.window['-mA_RZ-'].update(
                f'Rz: {np.rad2deg(self.getori(self.astorino)[2]):.2f} deg')
            
        if event == '-UPDATE-JOINTS-':

            self.window['-m_X-'].update(
                f'X: {self.sawyer.fkine(self.sawyer.q).A[0,3]:.2f} m')
            self.window['-m_Y-'].update(
                f'Y: {self.sawyer.fkine(self.sawyer.q).A[1,3]:.2f} m')
            self.window['-m_Z-'].update(
                f'Z: {self.sawyer.fkine(self.sawyer.q).A[2,3]:.2f} m')
            self.window['-m_RX-'].update(
                f'Rx: {np.rad2deg(self.getori(self.sawyer)[0]):.2f} deg')
            self.window['-m_RY-'].update(
                f'Ry: {np.rad2deg(self.getori(self.sawyer)[1]):.2f} deg')
            self.window['-m_RZ-'].update(
                f'Rz: {np.rad2deg(self.getori(self.sawyer)[2]):.2f} deg')

        if event == '-MISSION_ENABLE-':
            self.mission.enable_system()

        elif event == '-MISSION_HOME-':
            self.mission._home_system()

        elif event == '-MISSION_STOP-':
            self.mission.stop_system()
            self.mission_thread.join()

        elif event == '-MISSION_RUN-':
            self.mission_thread = threading.Thread(target=self.mission.run)
            self.mission_thread.start()
            # self.mission.run()

    def getori(self, robot):
        ori = smb.tr2rpy(robot.fkine(robot.q).A[0:3, 0:3], order='xyz')
        return ori

    def collision_setup(self):

        side = [0.2, 0.2, 0.2]
        center = self._cell_center @ sm.SE3(-0.2, 0., 0.87)
        viz_object = self.mission.update_collision_object(side, center)
        obj_id = self.env.add(viz_object)

    def blank(size):
        return sg.Text('', size=size, background_color='brown')

    def _init_log(log_level):

        # First, lets get the root logger
        log = logging.getLogger('root')

        # Create a new log file and setup logging to a file
        log_file = RobotGUI._create_new_logfile(LOG_FILE_NAME)
        file_handler = logging.FileHandler(log_file, mode='a')
        formatter_file = logging.Formatter(LOG_FORMAT_FILE)
        file_handler.setFormatter(formatter_file)
        file_handler.setLevel(logging.DEBUG)

        # Create a log file to go to the console
        stream_handler = logging.StreamHandler()
        formatter_console = logging.Formatter(LOG_FORMAT_CONSOLE)
        stream_handler.setFormatter(formatter_console)

        # Here you can optionally set the name of the log
        stream_handler.set_name("console_log")

        # Set the log level for the console output (based on what we passed into this function)
        stream_handler.setLevel(log_level)

        # Set the main logging level to debug so the file handler can support it. The stream handler (to console) will use the level set above.
        log.setLevel(logging.DEBUG)
        log.addHandler(file_handler)
        log.addHandler(stream_handler)
        log.info(f"Saving logs to: {file_handler.baseFilename}")

        return log

    def _create_new_logfile(basename):

        # Create a directory where we want to save the log file
        current_dir = os.path.dirname(__file__)
        log_dir = os.path.join(current_dir, "logs")

        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        full_filename = os.path.join(log_dir, f"{basename}")
        return full_filename


if __name__ == '__main__':
    args = load_args()
    app = RobotGUI(args)
    app.run()
