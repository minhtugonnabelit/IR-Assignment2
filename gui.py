import threading
import PySimpleGUI as sg
import roboticstoolbox as rtb
import spatialmath as sm
import spatialmath.base as smb
import numpy as np
import copy
from swift import Swift
from robot.sawyer import Sawyer
from robot.astorino import Astorino
from controller import Controller


class RobotGUI:

    _input_size = (10, 1)
    _slider_size = (30, 30)

    def __init__(self):

        sg.theme('DarkAmber')
        self.env = Swift()
        self.env.launch(realtime=True)

        self.sawyer = Sawyer(self.env, base=sm.SE3(0, 1, 0))
        self.sawyer_qlim = np.rad2deg(copy.deepcopy(self.sawyer.qlim))

        self.sawyer_controller = Controller(self.sawyer, self.env)
        self.sawyer_controller.launch()
        self.sawyer_controller.go_to_home()

        self.astorino = Astorino(self.env, base=sm.SE3(1, 0, 0))
        self.astorino_qlim = np.rad2deg(copy.deepcopy(self.astorino.qlim))

        self.astorino_controller = Controller(self.astorino, self.env)
        self.astorino_controller.launch()
        self.astorino_controller.go_to_home()

        self.window = self.create_window()
        self.update_gui_thread()

    def create_window(self):
        tab_group_layout = [
            [sg.TabGroup(
                [[sg.Tab('Sawyer Controller', self.tab1_setup(), background_color='black'),
                  sg.Tab('Astorino Controller', self.tab2_setup(), background_color='black'),
                  sg.Tab('Mission Controller', self.tab3_setup(), background_color='blue')]],
                tab_location='topleft',
                selected_background_color='brown',
                selected_title_color='white',
                title_color='black',
                size=(1368, 768),
                enable_events=True
            )]
        ]
        window = sg.Window('System GUI', tab_group_layout, finalize=True,
                           size=(1368, 768), location=(0, 0), resizable=True)
        return window

    def getori(self):
        ori = smb.tr2rpy(self.sawyer.fkine(self.sawyer.q).A[0:3, 0:3])
        return ori

    def blank(self, size):
        return sg.Text('', size=size, background_color='brown')

    def tab1_setup(self):

        # Define the layout for the first tab
        tab1_layout = [
            [
            sg.Text('SAWYER CONTROLLER', justification='center',font=('Cooper Black', 24), background_color='Black', pad=(10,5)),
            sg.Text(f'X: {self.sawyer.fkine(self.sawyer.q).A[0,3]} m', size=(15, 1), justification='right', key='-X-'),
            sg.Text(f'Y: {self.sawyer.fkine(self.sawyer.q).A[1,3]} m', size=(15, 1), justification='right', key='-Y-'),
            sg.Text(f'Z: {self.sawyer.fkine(self.sawyer.q).A[2,3]} m', size=(15, 1), justification='right', key='-Z-'),
            sg.Text(f'Rx: {np.rad2deg(self.getori()[0])} deg', size=(15, 1), justification='right', key='-RX-'),
            sg.Text(f'Ry: {np.rad2deg(self.getori()[1])} deg', size=(15, 1), justification='right', key='-RY-'),
            sg.Text(f'Rz: {np.rad2deg(self.getori()[2])} deg', size=(15, 1), justification='right', key='-RZ-'),
            ],

            [
            sg.Text('Controller state: ', size=(15, 1), justification='right', key='-STATE_LABEL-', background_color='black'), 
            sg.Text(f'{self.sawyer_controller.system_state()}', size=(15, 1), justification='right', key='-STATE-'),  
            sg.Button('ENABLE CONTROLLER', key='-ENABLE-', size=(30,1), pad=(5,5)),
            sg.Button('Gamepad Mode', size=(15, 1), key='-GAMEPAD_ENABLE-' ),
            sg.Button('Disable Gamepad', size=(15, 1), key='-GAMEPAD_DISABLE-' ),
            sg.Button('E-stop', button_color=('white', 'red'), size=(30, 1), key='-ESTOP-' ),              
            ],

            [sg.Text('Joint Space Jogging', key='-JS-', font=('Cooper Black', 15), background_color='black', pad=(10,1))],
            [sg.Slider(range=(self.sawyer_qlim[0, 0], self.sawyer_qlim[1, 0]), default_value=np.rad2deg(self.sawyer.q[0]), orientation='h',
                    size=self._slider_size, key='-SLIDER0-', enable_events=True, tick_interval=0.1), sg.Text(f'{np.rad2deg(self.sawyer.q[0])} deg', size=(10, 1), key='-BASE-', justification='right'),],
            [sg.Slider(range=(self.sawyer_qlim[0, 1], self.sawyer_qlim[1, 1]), default_value=np.rad2deg(self.sawyer.q[1]), orientation='h',
                    size=self._slider_size, key='-SLIDER1-', enable_events=True), sg.Text(f'{np.rad2deg(self.sawyer.q[1])} deg', size=(10, 1), key='-J0-', justification='right')],
            [sg.Slider(range=(self.sawyer_qlim[0, 2], self.sawyer_qlim[1, 2]), default_value=np.rad2deg(self.sawyer.q[2]), orientation='h',
                    size=self._slider_size, key='-SLIDER2-', enable_events=True), sg.Text(f'{np.rad2deg(self.sawyer.q[2])} deg', size=(10, 1), key='-J1-', justification='right')],
            [sg.Slider(range=(self.sawyer_qlim[0, 3], self.sawyer_qlim[1, 3]), default_value=np.rad2deg(self.sawyer.q[3]), orientation='h',
                    size=self._slider_size, key='-SLIDER3-', enable_events=True), sg.Text(f'{np.rad2deg(self.sawyer.q[3])} deg', size=(10, 1), key='-J2-', justification='right')],
            [sg.Slider(range=(self.sawyer_qlim[0, 4], self.sawyer_qlim[1, 4]), default_value=np.rad2deg(self.sawyer.q[4]), orientation='h',
                    size=self._slider_size, key='-SLIDER4-', enable_events=True), sg.Text(f'{np.rad2deg(self.sawyer.q[4])} deg', size=(10, 1), key='-J3-', justification='right')],
            [sg.Slider(range=(self.sawyer_qlim[0, 5], self.sawyer_qlim[1, 5]), default_value=np.rad2deg(self.sawyer.q[5]), orientation='h',
                    size=self._slider_size, key='-SLIDER5-', enable_events=True), sg.Text(f'{np.rad2deg(self.sawyer.q[5])} deg', size=(10, 1), key='-J4-', justification='right')],
            [sg.Slider(range=(self.sawyer_qlim[0, 6], self.sawyer_qlim[1, 6]), default_value=np.rad2deg(self.sawyer.q[6]), orientation='h',
                    size=self._slider_size, key='-SLIDER6-', enable_events=True), sg.Text(f'{np.rad2deg(self.sawyer.q[6])} deg', size=(10, 1), key='-J5-', justification='right')],
        
            [sg.Text('TCP Jogging', key='-TCP-', font=('Cooper Black', 15), background_color='black')],
            [sg.Button('+X',    key='-PLUSX-', size=self._input_size), sg.Button('+Y', key='-PLUSY-', size=self._input_size), sg.Button('+Z', key='-PLUSZ-', size=self._input_size),
             sg.Text('X: ',    size=(5, 1), ), sg.Input(default_text='0', size=(5, 1), key='-CARTX-'),
             sg.Text('Roll: ', size=(5, 1), ), sg.Input(default_text='0', size=(5, 1), key='-ROLL-')],
            [sg.Button('-X',    key='-MINUSX-', size=self._input_size), sg.Button('-Y',key='-MINUSY-', size=self._input_size), sg.Button('-Z', key='-MINUSZ-', size=self._input_size),
             sg.Text('Y: ',    size=(5, 1), ), sg.Input(default_text='0', size=(5, 1), key='-CARTY-'),
             sg.Text('Pitch: ',size=(5, 1), ), sg.Input(default_text='0', size=(5, 1), key='-PITCH-')],
            [sg.Button('+Roll', key='-PLUSROLL-', size=self._input_size), sg.Button('+Pitch', key='-PLUSPITCH-', size=self._input_size), sg.Button('+Yaw', key='-PLUSYAW-', size=self._input_size),
             sg.Text('Z: ',    size=(5, 1), ), sg.Input(default_text='0', size=(5, 1), key='-CARTZ-'),
             sg.Text('Yaw: ',  size=(5, 1), ), sg.Input(default_text='0', size=(5, 1), key='-YAW-')],
            [sg.Button('-Roll', key='-MINUSROLL-', size=self._input_size), sg.Button('-Pitch',key='-MINUSPITCH-', size=self._input_size), sg.Button('-Yaw', key='-MINUSYAW-', size=self._input_size)],

            [sg.Radio('Joint Space Control', 'RADIO1', default=True, key='-JOINT-', size=(18,1)),
            sg.Radio('End Effector Control', 'RADIO1', key='-END-EFFECTOR-', size=(18,1))],

            [sg.Button('Confirm', key='-CONFIRM-', size=(18,1)), sg.Button('Home', key='-HOME-', size=(18,1)),],
            
        ]
        return tab1_layout

    def tab2_setup(self):
        # Define the layout for the first tab
        tab2_layout = [
            [
            sg.Text('Astorino Teach Pendant', justification='center',font=('Cooper Black', 24), background_color='brown'),
            sg.Text(f'X: {self.astorino.fkine(self.astorino.q).A[0,3]} m', size=(15, 1), justification='right', key='-A_X-'),
            sg.Text(f'Y: {self.astorino.fkine(self.astorino.q).A[1,3]} m', size=(15, 1), justification='right', key='-A_Y-'),
            sg.Text(f'Z: {self.astorino.fkine(self.astorino.q).A[2,3]} m', size=(15, 1), justification='right', key='-A_Z-'),
            sg.Text(f'Rx: {np.rad2deg(self.getori()[0])} deg', size=(15, 1), justification='right', key='-A_RX-'),
            sg.Text(f'Ry: {np.rad2deg(self.getori()[1])} deg', size=(15, 1), justification='right', key='-A_RY-'),
            sg.Text(f'Rz: {np.rad2deg(self.getori()[2])} deg', size=(15, 1), justification='right', key='-A_RZ-'),
            ], 

            [sg.Text('Controller state: ', size=(15, 1), justification='right', key='-A_STATE_LABEL-', background_color='black'), 
             sg.Text(f'{self.astorino_controller.system_state()}', size=(15, 2), justification='center', key='-A_STATE-'),
             sg.Button('ENABLE CONTROLLER', key='-A_ENABLE-', size=(30,1), pad=(5,5)),
             sg.Button('E-stop', button_color=('white', 'red'), size=(30, 1), key='-A_ESTOP-' ),],    
            
            [sg.Text('Joint Space Jogging', key='-A_JS-', font=('Cooper Black', 15), background_color='brown'),],
            [sg.Slider(range=(self.astorino_qlim[0, 0], self.astorino_qlim[1, 0]), default_value=np.rad2deg(self.astorino.q[0]), orientation='h',
                    size=self._slider_size, key='-A_SLIDER0-', enable_events=True), sg.Text(f'{np.rad2deg(self.astorino.q[0])} deg', size=(10, 1), key='-A_BASE-', justification='right'),],
            [sg.Slider(range=(self.astorino_qlim[0, 1], self.astorino_qlim[1, 1]), default_value=np.rad2deg(self.astorino.q[1]), orientation='h',
                    size=self._slider_size, key='-A_SLIDER1-', enable_events=True), sg.Text(f'{np.rad2deg(self.astorino.q[1])} deg', size=(10, 1), key='-A_J0-', justification='right')],
            [sg.Slider(range=(self.astorino_qlim[0, 2], self.astorino_qlim[1, 2]), default_value=np.rad2deg(self.astorino.q[2]), orientation='h',
                    size=self._slider_size, key='-A_SLIDER2-', enable_events=True), sg.Text(f'{np.rad2deg(self.astorino.q[2])} deg', size=(10, 1), key='-A_J1-', justification='right')],
            [sg.Slider(range=(self.astorino_qlim[0, 3], self.astorino_qlim[1, 3]), default_value=np.rad2deg(self.astorino.q[3]), orientation='h',
                    size=self._slider_size, key='-A_SLIDER3-', enable_events=True), sg.Text(f'{np.rad2deg(self.astorino.q[3])} deg', size=(10, 1), key='-A_J2-', justification='right')],
            [sg.Slider(range=(self.astorino_qlim[0, 4], self.astorino_qlim[1, 4]), default_value=np.rad2deg(self.astorino.q[4]), orientation='h',
                    size=self._slider_size, key='-A_SLIDER4-', enable_events=True), sg.Text(f'{np.rad2deg(self.astorino.q[4])} deg', size=(10, 1), key='-A_J3-', justification='right')],
            [sg.Slider(range=(self.astorino_qlim[0, 5], self.astorino_qlim[1, 5]), default_value=np.rad2deg(self.astorino.q[5]), orientation='h',
                    size=self._slider_size, key='-A_SLIDER5-', enable_events=True), sg.Text(f'{np.rad2deg(self.astorino.q[5])} deg', size=(10, 1), key='-A_J4-', justification='right')],
            
        
            [sg.Text('TCP Jogging', key='-A_TCP-', font=('Cooper Black', 15), background_color='brown')],
            [sg.Text('X: ',    size=(5, 1), ), sg.Input(default_text='0', size=(5, 1), key='-A_CARTX-'),
             sg.Text('Y: ',    size=(5, 1), ), sg.Input(default_text='0', size=(5, 1), key='-A_CARTY-'),
             sg.Text('Z: ',    size=(5, 1), ), sg.Input(default_text='0', size=(5, 1), key='-A_CARTZ-')],
            [sg.Text('Roll: ', size=(5, 1), ), sg.Input(default_text='0', size=(5, 1), key='-A_ROLL-'),
             sg.Text('Pitch: ',size=(5, 1), ), sg.Input(default_text='0', size=(5, 1), key='-A_PITCH-'),
             sg.Text('Yaw: ',  size=(5, 1), ), sg.Input(default_text='0', size=(5, 1), key='-A_YAW-')],
            [sg.Button('+X',    key='-A_PLUSX-', size=self._input_size), sg.Button('+Y',        key='-A_PLUSY-', size=self._input_size), sg.Button('+Z', key='-A_PLUSZ-', size=self._input_size)],
            [sg.Button('-X',    key='-A_MINUSX-', size=self._input_size), sg.Button('-Y',       key='-A_MINUSY-', size=self._input_size), sg.Button('-Z', key='-A_MINUSZ-', size=self._input_size)],
            [sg.Button('+Roll', key='-A_PLUSROLL-', size=self._input_size), sg.Button('+Pitch', key='-A_PLUSPITCH-', size=self._input_size), sg.Button('+Yaw', key='-A_PLUSYAW-', size=self._input_size)],
            [sg.Button('-Roll', key='-A_MINUSROLL-', size=self._input_size), sg.Button('-Pitch',key='-A_MINUSPITCH-', size=self._input_size), sg.Button('-Yaw', key='-A_MINUSYAW-', size=self._input_size)],

            [sg.Radio('Joint Space Control', 'RADIO1', default=True, key='-A_JOINT-', size=(18,1)),
            sg.Radio('End Effector Control', 'RADIO1', key='-A_END-EFFECTOR-', size=(18,1))],

            [sg.Button('Confirm', key='-A_CONFIRM-', size=(18,1)), sg.Button('Home', key='-A_HOME-', size=(18,1)),
            ],
            
        ]
        return tab2_layout

    def tab3_setup(self):
        #... [omitting for brevity]
        tab3_layout = [
            [sg.Text('This is Tab 3')],
            [sg.Button('Submit Tab 3')]
        ]
        return tab3_layout
    
    def update_gui_thread(self):
        threading.Thread(target=self.gui_updater, daemon=True).start()

    def gui_updater(self):
        while True:
            
            new_joint_states = np.rad2deg(self.sawyer.get_jointstates())
            self.window.write_event_value('-UPDATE-JOINTS-', new_joint_states)

            state = self.sawyer_controller.system_state()
            self.window.write_event_value('-UPDATE-STATE-', state)
            
    def run(self):
        q = np.zeros(7)
        while True:

            event, values = self.window.read()
            system_state = self.sawyer_controller.system_state()

            if event == sg.WIN_CLOSED:
                break
            
            # constantly update the joint values associated with sliders' values
            for i in range(7):
                self.sawyer_controller.set_joint_value(i, values[f'-SLIDER{i}-'])
        
            # Loop through the input keys and convert values to float
            input_values = []
            for key in ['-CARTX-', '-CARTY-', '-CARTZ-', '-ROLL-', '-PITCH-', '-YAW-']:
                input_value = values[key]

                try:
                    float_value = float(input_value)
                    input_values.append(float_value)
                except ValueError:
                    print(f"Invalid input: {input_value}")

            # Convert the input values to a SE3 object and input as Cartesian pose for robot to work out
            pose = sm.SE3(input_values[0], input_values[1], input_values[2]
                            ) @ sm.SE3.RPY(input_values[3:6], order='xyz', unit='deg')
            
            self.sawyer_controller.set_cartesian_value(pose)

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

                self.window['-X-'].update(f'X: {self.sawyer.fkine(self.sawyer.q).A[0,3]:.2f} m')
                self.window['-Y-'].update(f'Y: {self.sawyer.fkine(self.sawyer.q).A[1,3]:.2f} m')
                self.window['-Z-'].update(f'Z: {self.sawyer.fkine(self.sawyer.q).A[2,3]:.2f} m')
                self.window['-RX-'].update(f'Rx: {np.rad2deg(self.getori()[0]):.2f} deg')
                self.window['-RY-'].update(f'Ry: {np.rad2deg(self.getori()[1]):.2f} deg')
                self.window['-RZ-'].update(f'Rz: {np.rad2deg(self.getori()[2]):.2f} deg')
            

            if event == '-UPDATE-STATE-':

                state = self.sawyer_controller.system_state()
                self.window['-STATE-'].update(f'{state}')


            # event activated with E-stop button
            if event == '-ESTOP-':
                if system_state == "STOPPED":
                    self.sawyer_controller.disengage_estop()
                else:
                    self.sawyer_controller.engage_estop()
                
            if event == '-GAMEPAD_ENABLE-':
                self.sawyer_controller.send_command('GAMEPAD_ENABLE')

            if event == '-GAMEPAD_DISABLE-':
                self.sawyer_controller.send_command('GAMEPAD_DISABLE')


            # event activated with HOME button
            if event == '-HOME-':
                if self.sawyer_controller._disable_gamepad is False:
                    self.sawyer_controller.send_command('GAMEPAD_DISABLE')
                self.sawyer_controller.send_command('HOME')

            # event enabled with ENABLE button
            if event == '-ENABLE-':
                self.sawyer_controller.send_command('ENABLE')

            # event activated with +X button
            if event == '-PLUSX-':
                self.sawyer_controller.send_command('+X')
            
            # event activated with -X button
            if event == '-MINUSX-':
                self.sawyer_controller.send_command('-X')

            # event activated with +Y button
            if event == '-PLUSY-':
                self.sawyer_controller.send_command('+Y')

            # event activated with -Y button
            if event == '-MINUSY-':
                self.sawyer_controller.send_command('-Y')

            # event activated with +Z button
            if event == '-PLUSZ-':
                self.sawyer_controller.send_command('+Z')

            # event activated with -Z button
            if event == '-MINUSZ-':
                self.sawyer_controller.send_command('-Z')


            # event activated with CONFIRM button
            if event == '-CONFIRM-':
                if values['-JOINT-']:
                    self.sawyer_controller.send_command('JOINT_ANGLES')

                elif values['-END-EFFECTOR-']:
                    self.sawyer_controller.send_command('CARTESIAN_POSE')

        self.sawyer_controller.clean()
        self.astorino_controller.clean()
        self.window.close()


if __name__ == '__main__':
    app = RobotGUI()
    app.run()
