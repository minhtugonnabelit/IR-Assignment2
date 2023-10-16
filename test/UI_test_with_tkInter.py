import threading
import time
import tkinter as tk
from tkinter import ttk
# import ttkbootstrap as ttks
import roboticstoolbox as rtb
import spatialmath as sm
# from Sawyer_model.sawyer import Sawyer
from robot.sawyer import Sawyer
from controller import Controller
import numpy as np
import copy

from swift import Swift

from PIL import Image
Image.CUBIC = Image.BICUBIC

# Create the environment and robot
env = Swift()
env.launch(realtime=True)

# Create the Sawyer robot and set the joint limits
robot = Sawyer(env)
sawyer_controller = Controller(robot, env)
sawyer_controller.go_to_home()
sawyer_qlim = copy.deepcopy(robot.qlim)
sawyer_qlim = np.rad2deg(sawyer_qlim)

class SawyerGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        
        self.title("Sawyer Teach Pendant")
        style = ttks.Style(theme='cyborg')
        style.theme_use('cyborg')

        self.tab_control = ttk.Notebook(self)
        self.tab1 = ttk.Frame(self.tab_control)
        self.tab2 = ttk.Frame(self.tab_control)
        self.tab3 = ttk.Frame(self.tab_control)
        
        self.tab_control.add(self.tab1, text='Tab 1')
        self.tab_control.add(self.tab2, text='Tab 2')
        self.tab_control.add(self.tab3, text='Tab 3')
        
        self.tab_control.pack(expand=1, fill='both')

        self.setup_tab1()

    def setup_tab1(self):
        self.slider0 = tk.Scale(self.tab1, from_=sawyer_qlim[0, 0], to=sawyer_qlim[1, 0], orient='horizontal', length=200, resolution=0.1, command=self.on_slider0)
        # self.meter0 = ttks.Meter(self.tab1, amountused=robot.q[0], interactive=True, bootstyle='success', metertype='semi', stripethickness=10)
        self.slider0.grid(column=0, row=1)
        self.slider0_label = ttk.Label(self.tab1, text=f'{np.rad2deg(robot.q[0])} deg')
        self.slider0_label.grid(column=1, row=1)

        # ... add other widgets similarly ...
        self.estop_button = ttk.Button(self.tab1, text="E-stop", command=self.on_estop)
        self.estop_button.grid(column=0, row=2)

    def on_slider0(self, slider0_value):
        self.slider0_label.configure(text=f'{slider0_value} deg')
        robot.q[0] = np.deg2rad(float(slider0_value))
        # robot.q = robot.q   
        # self.meter0.configure(amountused=self.slider0.ge

    def on_estop(self):
        # ... handle E-stop button press ..
        pass

if __name__ == "__main__":
    gui = SawyerGUI()
    gui.mainloop()
