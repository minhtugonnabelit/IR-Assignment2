from time import sleep
import PySimpleGUI as sg

layout = [
    [sg.Slider(range=(1, 100), orientation='h', size=(20, 20), key = "slider")],
    [sg.Button("Show after all done"), sg.Button("Show each step")]]

window = sg.Window("Window", layout)
while True:
    event, values = window.read()
    if event in (sg.WINDOW_CLOSED, "Exit"):
        break
    elif event == "Show after all done":
        for i in range(0, 100, 10):
            window['slider'].update(value = 10+i)
            sleep(0.01)
    elif event == "Show each step":
        for i in range(0, 100, 10):
            window['slider'].update(value = 10+i)
            window.refresh()
            sleep(0.01)

window.close()