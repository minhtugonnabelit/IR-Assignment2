import threading
import time

import copy
import numpy as np
import PySimpleGUI as sg
import roboticstoolbox as rtb
import spatialmath as sm
import spatialmath.base as smb
from swift import Swift

from robot.sawyer import Sawyer
from controller import Controller

# Define the 5size of the sliders
input_size = (10, 1)
slider_size = (30, 30)

# Define the theme for the GUI
sg.theme('DarkAmber')

# Create the environment and robot
env = Swift()
env.launch(realtime=True)

# Create the Sawyer robot and set the joint limits
robot = Sawyer(env)
sawyer_controller = Controller(robot, env)
sawyer_controller.go_to_home()
sawyer_qlim = copy.deepcopy(robot.qlim)
sawyer_qlim = np.rad2deg(sawyer_qlim)

def getori():
    ori = smb.tr2rpy(robot.fkine(robot.q).A[0:3, 0:3])
    return ori

def blank(size):
    return sg.Text('', size=size, background_color='brown')

def tab1_setup():
    # Define the layout for the first tab
    tab1_layout = [
        [
        sg.Text('Sawyer Teach Pendant', justification='center',font=('Cooper Black', 24), background_color='brown'),
        sg.Text(f'X: {robot.fkine(robot.q).A[0,3]} m', size=(15, 1), justification='right', key='-X-'),
        sg.Text(f'Y: {robot.fkine(robot.q).A[1,3]} m', size=(15, 1), justification='right', key='-Y-'),
        sg.Text(f'Z: {robot.fkine(robot.q).A[2,3]} m', size=(15, 1), justification='right', key='-Z-'),
        sg.Text(f'Rx: {np.rad2deg(getori()[0])} deg', size=(15, 1), justification='right', key='-RX-'),
        sg.Text(f'Ry: {np.rad2deg(getori()[1])} deg', size=(15, 1), justification='right', key='-RY-'),
        sg.Text(f'Rz: {np.rad2deg(getori()[2])} deg', size=(15, 1), justification='right', key='-RZ-'),
        sg.Button('E-stop', button_color=('white', 'red'), size=(30, 1), key='-ESTOP-' ),],

        [blank((3,1)),sg.Text('Joint Space Jogging', key='-JS-', font=('Cooper Black', 15), background_color='brown'), blank((120,1)), sg.Button('ENABLE CONTROLLER', key='-ENABLE-', size=(30,1))],
        [sg.Slider(range=(sawyer_qlim[0, 0], sawyer_qlim[1, 0]), default_value=np.rad2deg(robot.q[0]), orientation='h',
                size=slider_size, key='-SLIDER0-', enable_events=True, pad=(20,5), ), sg.Text(f'{np.rad2deg(robot.q[0])} deg', size=(10, 1), key='-BASE-', justification='right'),],
        [sg.Slider(range=(sawyer_qlim[0, 1], sawyer_qlim[1, 1]), default_value=np.rad2deg(robot.q[1]), orientation='h',
                size=slider_size, key='-SLIDER1-', enable_events=True, pad=(20,5)), sg.Text(f'{np.rad2deg(robot.q[1])} deg', size=(10, 1), key='-J0-', justification='right')],
        [sg.Slider(range=(sawyer_qlim[0, 2], sawyer_qlim[1, 2]), default_value=np.rad2deg(robot.q[2]), orientation='h',
                size=slider_size, key='-SLIDER2-', enable_events=True), sg.Text(f'{np.rad2deg(robot.q[2])} deg', size=(10, 1), key='-J1-', justification='right')],
        [sg.Slider(range=(sawyer_qlim[0, 3], sawyer_qlim[1, 3]), default_value=np.rad2deg(robot.q[3]), orientation='h',
                size=slider_size, key='-SLIDER3-', enable_events=True), sg.Text(f'{np.rad2deg(robot.q[3])} deg', size=(10, 1), key='-J2-', justification='right')],
        [sg.Slider(range=(sawyer_qlim[0, 4], sawyer_qlim[1, 4]), default_value=np.rad2deg(robot.q[4]), orientation='h',
                size=slider_size, key='-SLIDER4-', enable_events=True), sg.Text(f'{np.rad2deg(robot.q[4])} deg', size=(10, 1), key='-J3-', justification='right')],
        [sg.Slider(range=(sawyer_qlim[0, 5], sawyer_qlim[1, 5]), default_value=np.rad2deg(robot.q[5]), orientation='h',
                size=slider_size, key='-SLIDER5-', enable_events=True), sg.Text(f'{np.rad2deg(robot.q[5])} deg', size=(10, 1), key='-J4-', justification='right')],
        [sg.Slider(range=(sawyer_qlim[0, 6], sawyer_qlim[1, 6]), default_value=np.rad2deg(robot.q[6]), orientation='h',
                size=slider_size, key='-SLIDER6-', enable_events=True), sg.Text(f'{np.rad2deg(robot.q[6])} deg', size=(10, 1), key='-J5-', justification='right')],
    
        [sg.Text('TCP Jogging', key='-TCP-', font=('Cooper Black', 15), background_color='brown')],
        [sg.Text('X: ',    size=(5, 1), ), sg.Input(default_text='0', size=(10, 1), key='-CARTX-'),
         sg.Text('Y: ',    size=(5, 1), ), sg.Input(default_text='0', size=(10, 1), key='-CARTY-'),
         sg.Text('Z: ',    size=(5, 1), ), sg.Input(default_text='0', size=(10, 1), key='-CARTZ-')],
        [sg.Text('Roll: ', size=(5, 1), ), sg.Input(default_text='0', size=(10, 1), key='-ROLL-'),
         sg.Text('Pitch: ',size=(5, 1), ), sg.Input(default_text='0', size=(10, 1), key='-PITCH-'),
         sg.Text('Yaw: ',  size=(5, 1), ), sg.Input(default_text='0', size=(10, 1), key='-YAW-')],
        [sg.Button('+X',    key='-PLUSX-', size=input_size), sg.Button('+Y',        key='-PLUSY-', size=input_size), sg.Button('+Z', key='-PLUSZ-', size=input_size)],
        [sg.Button('-X',    key='-MINUSX-', size=input_size), sg.Button('-Y',       key='-MINUSY-', size=input_size), sg.Button('-Z', key='-MINUSZ-', size=input_size)],
        [sg.Button('+Roll', key='-PLUSROLL-', size=input_size), sg.Button('+Pitch', key='-PLUSPITCH-', size=input_size), sg.Button('+Yaw', key='-PLUSYAW-', size=input_size)],
        [sg.Button('-Roll', key='-MINUSROLL-', size=input_size), sg.Button('-Pitch',key='-MINUSPITCH-', size=input_size), sg.Button('-Yaw', key='-MINUSYAW-', size=input_size)],

        [sg.Radio('Joint Space Control', 'RADIO1', default=True, key='-JOINT-', size=(18,1)),
         sg.Radio('End Effector Control', 'RADIO1', key='-END-EFFECTOR-', size=(18,1))],

        [sg.Button('Confirm', key='-CONFIRM-', size=(18,1)), sg.Button('Home', key='-HOME-', size=(18,1)),],
        
    ]
    return tab1_layout


# Define the layout for the second tab
tab2_layout = [
    [sg.Text('This is Tab 2')],
    [sg.Checkbox('Check me')],
    [sg.Button('Submit Tab 2')]
]

# Define the layout for the third tab
tab3_layout = [
    [sg.Text('This is Tab 3')],
    [sg.Button('Submit Tab 3')]
]

tab_group_layout = [
    [sg.TabGroup(
        [[sg.Tab('Tab 1', tab1_setup(), background_color='brown',),
          sg.Tab('Tab 2', tab2_layout, background_color='blue',),
          sg.Tab('Tab 3', tab3_layout, background_color='blue',),
          ]],
        tab_location='topleft',
        selected_background_color='brown',
        selected_title_color='white',
        title_color='black',
        size=(1368, 768),
        enable_events=True,

    )
    ]]

# Create the PySimpleGUI window with the TabGroup
window = sg.Window('System GUI', tab_group_layout, finalize=True,
                   size=(1680, 1050), location=(0, 0), resizable=True)

def update_gui_thread():
    while True:
        new_joint_states = np.rad2deg(robot.get_jointstates())
        window.write_event_value('-UPDATE-JOINTS-', new_joint_states)


SLIDER_UPDATE_THREAD = threading.Thread(target=update_gui_thread, daemon=True).start()
input_values = []

q=np.zeros(7)
while True:

    event, values = window.read()
    system_state = sawyer_controller.system_state()

    if event == sg.WIN_CLOSED:
        break
    
    # constantly update the joint values associated with sliders' values
    for i in range(7):
        sawyer_controller.set_joint_value(i, values[f'-SLIDER{i}-'])

    input_values = []

    # Loop through the input keys and convert values to float
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
    
    sawyer_controller.set_cartesian_value(pose)

    # event activated with E-stop button
    if event == '-ESTOP-':
        if system_state == "STOPPED":
            sawyer_controller.send_command('DISENGAGE')
            # sawyer_controller.disengage_estop()
        else:
            sawyer_controller.send_command('STOPPED')
            # sawyer_controller.engage_estop()
        
    # event activated with HOME button
    if event == '-HOME-':
        sawyer_controller.send_command('HOME')

    if event == '-ENABLE-':
        sawyer_controller.enable_system()

    # event activated with +X button
    if event == '-PLUSX-':
        sawyer_controller.send_command('+X')
    
    # event activated with -X button
    if event == '-MINUSX-':
        sawyer_controller.send_command('-X')

    # event activated with +Y button
    if event == '-PLUSY-':
        sawyer_controller.send_command('+Y')

    # event activated with -Y button
    if event == '-MINUSY-':
        sawyer_controller.send_command('-Y')

    # event activated with +Z button
    if event == '-PLUSZ-':
        sawyer_controller.send_command('+Z')

    # event activated with -Z button
    if event == '-MINUSZ-':
        sawyer_controller.send_command('-Z')

    if event == '-UPDATE-JOINTS-':

        new_joint_states = np.rad2deg(robot.get_jointstates())
        window['-BASE-'].update(f'{new_joint_states[0]:.2f} ')
        window['-J0-'].update(f'{new_joint_states[1]:.2f} ')
        window['-J1-'].update(f'{new_joint_states[2]:.2f} ')
        window['-J2-'].update(f'{new_joint_states[3]:.2f} ')
        window['-J3-'].update(f'{new_joint_states[4]:.2f} ')
        window['-J4-'].update(f'{new_joint_states[5]:.2f} ')
        window['-J5-'].update(f'{new_joint_states[6]:.2f} ')

        window['-X-'].update(f'X: {robot.fkine(robot.q).A[0,3]:.2f} m')
        window['-Y-'].update(f'Y: {robot.fkine(robot.q).A[1,3]:.2f} m')
        window['-Z-'].update(f'Z: {robot.fkine(robot.q).A[2,3]:.2f} m')
        window['-RX-'].update(f'Rx: {np.rad2deg(getori()[0]):.2f} deg')
        window['-RY-'].update(f'Ry: {np.rad2deg(getori()[1]):.2f} deg')
        window['-RZ-'].update(f'Rz: {np.rad2deg(getori()[2]):.2f} deg')


    # event activated with CONFIRM button
    if event == '-CONFIRM-':
        if values['-JOINT-']:
            sawyer_controller.send_command('JOINT_ANGLES')

        elif values['-END-EFFECTOR-']:
            sawyer_controller.send_command('CARTESIAN_POSE')
        




