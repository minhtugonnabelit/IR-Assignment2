import PySimpleGUI as sg
import time

# Define the layout for the GUI
layout = [
    [sg.Text('Continuous Execution Example')],
    [sg.Text('', key='-OUTPUT-')],
    [sg.Button('Start Continuous Execution'), sg.Button('Stop Continuous Execution')],
    [sg.Button('Exit')],
]

# Create the PySimpleGUI window
window = sg.Window('Continuous Execution Example', layout)

# Flag to control continuous execution
continuous_execution_enabled = False

while True:
    event, values = window.read(timeout=100)  # Adjust the timeout as needed
    
    if event in (sg.WIN_CLOSED, 'Exit'):
        break

    if event == 'Start Continuous Execution':
        continuous_execution_enabled = True
    elif event == 'Stop Continuous Execution':
        continuous_execution_enabled = False

    # Check if continuous execution is enabled and perform some continuous task
    # if continuous_execution_enabled:
    current_time = time.strftime('%H:%M:%S')
    window['-OUTPUT-'].update(f'Continuous Task: {current_time}')

window.close()
