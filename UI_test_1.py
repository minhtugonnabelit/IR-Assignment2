import copy
import numpy as np
import PySimpleGUI as sg
import roboticstoolbox as rtb
import spatialmath as sm
import spatialmath.base as smb
from swift import Swift

from Sawyer_model.sawyer import Sawyer

# Create the environment and robot
env = Swift()
env.launch(realtime=True)

# Create the Sawyer robot and set the joint limits
robot = Sawyer(env)
sawyer_qlim = copy.deepcopy(robot.qlim)
sawyer_qlim = np.rad2deg(sawyer_qlim)

# Define the size of the sliders
input_size = (5, 1)
slider_size = (30, 30)

# Define the theme for the GUI
sg.theme('DarkAmber')

# Define the layout for the first tab
tab1_layout = [
    [sg.Button('E-stop', button_color=('white', 'red'), size=(7, 2), key='-ESTOP-', use_ttk_buttons=True, ),
     sg.Text('Sawyer Teach Pendant', justification='center',font=('Cooper Black', 20), background_color='brown')],

    [sg.Slider(range=(sawyer_qlim[0, 0], sawyer_qlim[1, 0]), default_value=robot.q[0], orientation='h',
               size=slider_size, key='-SLIDER0-', enable_events=True), sg.Input(default_text='base', size=(10, 1))],
    [sg.Slider(range=(sawyer_qlim[0, 1], sawyer_qlim[1, 1]), default_value=robot.q[1], orientation='h',
               size=slider_size, key='-SLIDER1-', enable_events=True), sg.Input(default_text='j0', size=(10, 1))],
    [sg.Slider(range=(sawyer_qlim[0, 2], sawyer_qlim[1, 2]), default_value=robot.q[2], orientation='h',
               size=slider_size, key='-SLIDER2-', enable_events=True), sg.Input(default_text='j1', size=(10, 1))],
    [sg.Slider(range=(sawyer_qlim[0, 3], sawyer_qlim[1, 3]), default_value=robot.q[3], orientation='h',
               size=slider_size, key='-SLIDER3-', enable_events=True), sg.Input(default_text='j2', size=(10, 1))],
    [sg.Slider(range=(sawyer_qlim[0, 4], sawyer_qlim[1, 4]), default_value=robot.q[4], orientation='h',
               size=slider_size, key='-SLIDER4-', enable_events=True), sg.Input(default_text='j3', size=(10, 1))],
    [sg.Slider(range=(sawyer_qlim[0, 5], sawyer_qlim[1, 5]), default_value=robot.q[5], orientation='h',
               size=slider_size, key='-SLIDER5-', enable_events=True), sg.Input(default_text='j4', size=(10, 1))],
    [sg.Slider(range=(sawyer_qlim[0, 6], sawyer_qlim[1, 6]), default_value=robot.q[6], orientation='h',
               size=slider_size, key='-SLIDER6-', enable_events=True), sg.Input(default_text='j5', size=(10, 1))],

    [sg.Text('X: ', size=(5, 1), ), sg.Input(default_text='0', size=(10, 1), key='-CARTX-'),
     sg.Text('Y: ', size=(5, 1), ), sg.Input(
         default_text='0', size=(10, 1), key='-CARTY-'),
     sg.Text('Z: ', size=(5, 1), ), sg.Input(default_text='0', size=(10, 1), key='-CARTZ-')],

    [sg.Text('Roll: ', size=(5, 1), ), sg.Input(default_text='0', size=(10, 1), key='-ROLL-'),
     sg.Text('Pitch: ', size=(5, 1),), sg.Input(
         default_text='0', size=(10, 1), key='-PITCH-'),
     sg.Text('Yaw: ', size=(5, 1), ),  sg.Input(default_text='0', size=(10, 1), key='-YAW-')],

    [sg.Radio('Joint Control', 'RADIO1', default=True, key='-JOINT-'),
     sg.Radio('End Effector Control', 'RADIO1', key='-END-EFFECTOR-')],

    [sg.Button('Confirm', key='-CONFIRM-'), sg.Button('Home', key='-HOME-')],
    []
]

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
        [[sg.Tab('Tab 1', tab1_layout, background_color='brown',),
          sg.Tab('Tab 2', tab2_layout, background_color='blue',),
          sg.Tab('Tab 3', tab3_layout, background_color='blue',),
          ]],
        tab_location='topleft',
        selected_background_color='brown',
        selected_title_color='white',
        title_color='black',
        size=(1024, 768),
    )
    ]]

# Create the PySimpleGUI window with the TabGroup
window = sg.Window('System GUI', tab_group_layout, finalize=True,
                   size=(1280, 800), location=(0, 0), resizable=True)

while True:
    event, values = window.read()

    if event == sg.WIN_CLOSED:
        break

    # constantly update the joint values associated with sliders' values
    for i in range(7):
        robot.set_joint_value(i, values[f'-SLIDER{i}-'])

    # event activated with HOME button
    if event == '-HOME-':
        robot.home()

    # event activated with CONFIRM button
    if event == '-CONFIRM-':
        if values['-JOINT-']:
            robot.move()
        elif values['-END-EFFECTOR-']:

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
                          ) @ sm.SE3.RPY(input_values[3:6], order='xyz')
            robot.go_to_CartesianPose(pose, time=5)

window.close()
