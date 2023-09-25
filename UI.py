import PySimpleGUI as sg
import random  # Replace this with actual code to communicate with your robot

# Define initial values
initial_joint_value = 50
initial_mode = 'Joint Control'

# Define the layout for the GUI
layout = [
    [sg.Text('Robot Control', size=(20, 1), justification='center', font=('Arial Bold', 20))],
    [sg.Slider(range=(0, 100), default_value=initial_joint_value, orientation='h', size=(20, 20), key='-SLIDER-', 
               enable_events=True), sg.Radio('Joint Control', 'RADIO1', default=True, key='-JOINT-'), 
               sg.Radio('End Effector Control', 'RADIO1', key='-END-EFFECTOR-')],
    [sg.Text(f'Joint Value: {initial_joint_value}', size=(20, 1), key='-VALUE-')],
    [sg.Button('Exit')]
]

# Create the PySimpleGUI window
window = sg.Window('Robot Control', layout, finalize=True)
mode = initial_mode

while True:
    event, values = window.read()
    
    if event in (sg.WIN_CLOSED, 'Exit'):
        break
    
    if event == '-JOINT-':
        mode = 'Joint Control'
    elif event == '-END-EFFECTOR-':
        mode = 'End Effector Control'
    
    # Simulate robot data (replace with actual robot communication)
    robot_joint_value = random.randint(0, 100)

    if mode == 'Joint Control':
        # Update the slider's value based on robot joint data
        window['-SLIDER-'].update(robot_joint_value)
        window['-SLIDER-'].get()
        
    # Update the displayed joint value
    window['-VALUE-'].update(f'Joint Value: {robot_joint_value}')

window.close()