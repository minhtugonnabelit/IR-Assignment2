class RobotControlGUI:
    def __init__(self):
        # Create a GUI with two tabs: X robot and Y robot
        self.create_tabs()
        self.active_robot = "X"  # Initialize with X robot as active

    def switch_tab(self, tab):
        # Update the active robot when switching tabs
        if tab == "X":
            self.active_robot = "X"
        elif tab == "Y":
            self.active_robot = "Y"

    def handle_key_event(self, key):
        # Check the active robot and trigger the corresponding action
        if self.active_robot == "X":
            if key == "MoveForward":
                self.x_robot.move_forward()
            # Handle other X robot commands here
        elif self.active_robot == "Y":
            if key == "MoveForward":
                self.y_robot.move_forward()
            # Handle other Y robot commands here

    def create_tabs(self):
        # Create the X robot tab with buttons and key bindings
        self.x_robot_tab = Tab("X Robot")
        self.x_robot_tab.add_button("Move Forward", self.handle_key_event)

        # Create the Y robot tab with buttons and key bindings
        self.y_robot_tab = Tab("Y Robot")
        self.y_robot_tab.add_button("Move Forward", self.handle_key_event)
        
    def run(self):
        # Your GUI main loop here

gui = RobotControlGUI()
gui.run()
