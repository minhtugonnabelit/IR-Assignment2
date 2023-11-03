# 3D Printing Automation with Robotic Manipulators

## Overview

This project provides a visualisation of two robotic manipulators working in tandem on a 3D printing automation task. Once a print is completed, one robot picks up the plate and moves it to the second robot. Both robots then grip the plate and bend it in unison to detach the printed object from the plate.

## Key Features

**Dual Robotic Manipulators**: Collaboratively working to ensure the smooth detachment of 3D printed objects.  

**3D Printing Automation**: Enhances the efficiency of the 3D printing process by automating the post-printing task.  

**Real-time Visualisation**: Observe the manipulators in action in a visualised environment.  

**Interactive Testing GUI:** Found in the main.py script, this GUI mimics user interaction similar to the functionality of a teach pendant on other robot models. The interface includes draggable buttons and sliders. By default, buttons are vertically aligned; for custom arrangements, modifications are required by delving into the SwiftElement package.  

**Collision Avoidance and Detection:** The system is equipped with advanced algorithms for:

- **Collision Avoidance:** This ensures that the robotic manipulators steer clear of obstructions, preventing potential damage or delays.

- **Collision Detection:** If a collision does occur, the system can detect it and respond appropriately to minimize damage or disruptions.  

**Safety Features:** Ensuring user and equipment safety is paramount in the design of this robotic system. The safety features integrated into the system include:  

- **Emergency Stop Button on GUI**  

  For each Robot Controller and Mission Controller Tab, an `Emergency Stop` (e-stop) button is prominently displayed on the GUI. This feature operates as follows:

  Engage: Pressing the e-stop button during any motion will immediately stop the current movement of the robots. The system then transitions to a `Stopped` state.

  Release: Upon releasing the e-stop, the system enters an 'IDLE' state. Users can then press `Enable` to reactivate the system.

  Press `Run Mission` to continue operations from where the robots were interrupted.  

- **Light Curtain**  

  A light curtain is installed around the workspace to ensure human safety:

  Purpose: This feature prevents unauthorized or accidental entry into the robot's working area.

  Testing: Users can test this feature by controlling the human model in the visualisation. If the human model breaches the light curtain's boundary:

  The system immediately transitions to the `Stopped` state, akin to the e-stop being pressed.
  
  The operation remains halted until the user intervenes to ensure safety and resume operations.

## Operating Modes

The system operates in two primary modes:

- Teach Pendant (Interactive Testing GUI)  
   
  This mode offers an interactive GUI that mimics the interaction of a user in a similar way to how a teach pendant works on other robot models. This GUI allows for real-time testing and interaction with the robotic system.

  To initialize this mode, go to the corresponding Controller tab of either robot in the GUI and adjust sliders to control the robot joints 

- Mission Mode  

  In mission mode, the robotic manipulators automatically execute the task of gripping the plate and bending it in unison. This is to detach the printed object from the plate once the printing process is completed.

To initialize this mode, go to the Mission Controller tab

Both modes can be accessed and initialized directly from the GUI. Refer to the GUI documentation (or the GUI section of this README) for further instructions.
## Visualization Environment
The visualization of the manipulators and their operations is presented in the [Swift](https://github.com/jhavl/swift) environment. It provides a real-time, visually rich representation of the manipulators in action.

## Robotic Motion Calculation

Motion calculations for the manipulators are handled by the renowned [robotictoolbox](https://github.com/petercorke/roboticstoolbox-python) library developed by Peter Corke. This ensures precise and reliable calculations for robot movements.

## Dependencies
To set up and run the visualisation, the following dependencies are required:

`numpy`

`pygame`

`robotictoolbox`

`PySimpleGUI`

`OpenGL`

Please ensure all dependencies are installed before running the visualisation.

Thank you for your interest in our 3D Printing Automation project. We hope you find this simulation insightful and valuable
