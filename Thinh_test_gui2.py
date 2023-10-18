import PySimpleGUI as sg
import threading
import time
from queue import Queue

# Define the GUI layout
layout = [
    [sg.Text("Counter: 0", key="-COUNTER-")],
    [sg.Button("Start", key="-START-"), sg.Button("Stop", key="-STOP-"), sg.Button("Exit")]
]

window = sg.Window("GUI Update Example", layout, finalize= True, size= (500,500), location= (0,0), resizable= True)
event_queue = Queue()

# Function to update the counter
def update_counter_thread():
    counter = 0
    while True:
        if not event_queue.empty():
            event = event_queue.get()
            if event == "stop":
                break
        counter += 1
        window["-COUNTER-"].update(f"Counter: {counter}")
        time.sleep(1)  # Simulate some work

# Create and start the update thread
update_thread = None

while True:
    event, values = window.read()

    if event in (sg.WIN_CLOSED, "Exit"):
        event_queue.put("stop")  # Signal the update thread to stop
        if update_thread is not None:
            update_thread.join()  # Wait for the thread to finish
        break
    if event == "-START-":
        if update_thread is None or not update_thread.is_alive():
            update_thread = threading.Thread(target=update_counter_thread)
            update_thread.start()
    if event == "-STOP-":
        if update_thread is not None:
            event_queue.put("stop")  # Signal the update thread to stop
            update_thread.join()  # Wait for the thread to finish

window.close()
