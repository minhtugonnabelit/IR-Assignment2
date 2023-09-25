import copy
import numpy as np
import PySimpleGUI as sg
import roboticstoolbox as rtb
from swift import Swift

from Sawyer_model.sawyer import Sawyer


env = Swift()
env.launch(realtime=True)
robot = Sawyer(env)

sawyer_qlim = copy.deepcopy(robot.qlim)
sawyer_qlim = np.rad2deg(sawyer_qlim)

sg.theme('DarkAmber')

sliders = []
for i in range(7):
    sliders.append(sg.Slider(
        range=(sawyer_qlim[0, i], sawyer_qlim[1, i]),
        default_value=0,  # Set initial value to 0
        orientation='h',
        size=(20, 20),
        key=f'-SLIDER{i}-',
        enable_events=True
    ))



# Define the layout for the first tab
tab1_layout = [
    [sg.Text('Sawyer teach pendant', justification='center', font=('Arial Bold', 20), background_color='brown')],
    [sg.Slider(range=(sawyer_qlim[0,0], sawyer_qlim[1,0]), default_value=robot.q[0], orientation='h', size=(20, 20), key='-SLIDER0-', enable_events=True), sg.Spin([sz for sz in range(6, 172)], font=('Helvetica 20'), initial_value=robot.q[0], change_submits=True, key='spin'),],
    [sg.Slider(range=(sawyer_qlim[0,1], sawyer_qlim[1,1]), default_value=robot.q[1], orientation='h', size=(20, 20), key='-SLIDER1-', enable_events=True)],
    [sg.Slider(range=(sawyer_qlim[0,2], sawyer_qlim[1,2]), default_value=robot.q[2], orientation='h', size=(20, 20), key='-SLIDER2-', enable_events=True)],
    [sg.Slider(range=(sawyer_qlim[0,3], sawyer_qlim[1,3]), default_value=robot.q[3], orientation='h', size=(20, 20), key='-SLIDER3-', enable_events=True)],
    [sg.Slider(range=(sawyer_qlim[0,4], sawyer_qlim[1,4]), default_value=robot.q[4], orientation='h', size=(20, 20), key='-SLIDER4-', enable_events=True)],
    [sg.Slider(range=(sawyer_qlim[0,5], sawyer_qlim[1,5]), default_value=robot.q[5], orientation='h', size=(20, 20), key='-SLIDER5-', enable_events=True)],
    [sg.Slider(range=(sawyer_qlim[0,6], sawyer_qlim[1,6]), default_value=robot.q[6], orientation='h', size=(20, 20), key='-SLIDER6-', enable_events=True)],
    [sg.Radio('Joint Control', 'RADIO1', default=True, key='-JOINT-'), sg.Radio('End Effector Control', 'RADIO1', key='-END-EFFECTOR-')],
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
        )
     ]]

# Create the PySimpleGUI window with the TabGroup
window = sg.Window('System GUI', tab_group_layout, finalize=True, size=(1024, 768), location=(0, 0))

while True:
    event, values = window.read()

    if event == sg.WIN_CLOSED:
        break
    
    if event == '-CONFIRM-':
        robot.move()

    if event == '-HOME-':
        robot.home()


    for i in range(7):
        robot.set_joint_value(i, values[f'-SLIDER{i}-'])
        # robot.q[i] = values[f'-SLIDER{i}-']

    sz = values['spin']
    window['spin'].update(sz)
    window['-SLIDER0-'].update(sz)
    # robot.q = robot.q   


    # # Event handling for each tab's elements
    # if event == 'Submit Tab 1':
    #     # Handle actions for Tab 1
    #     pass
    # elif event == 'Submit Tab 2':
    #     # Handle actions for Tab 2
    #     pass
    # elif event == 'Submit Tab 3':
    #     # Handle actions for Tab 3
    #     pass

window.close()
