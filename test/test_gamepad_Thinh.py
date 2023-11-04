import pygame
import sys
import os  # Import the os module for clearing the terminal
import time
# Initialize the pygame library
pygame.init()

gamepad_list = []
 
# Setup joystick
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    raise Exception('No joystick found')
else:
    joystick_1 = pygame.joystick.Joystick(0)  # NOTE: Change 0 to another number if multiple joysticks present
    joystick_1.init()
    gamepad_list.append(joystick_1)
    joystick_2 = pygame.joystick.Joystick(1)
    joystick_2.init()
    gamepad_list.append(joystick_2)
    
 
for i in range(len(gamepad_list)):
    joy_name = gamepad_list[i].get_name()
    joy_axes = gamepad_list[i].get_numaxes()
    joy_buttons = gamepad_list[i].get_numbuttons()-4
    print(f'Your joystick_{i}: {joy_name} has:')
    print(f' - {joy_buttons} buttons')
    print(f' - {joy_axes} axes')

# Print joystick information
# joy_name = joystick_1.get_name()
# joy_axes = joystick_1.get_numaxes()
# joy_buttons = joystick_1.get_numbuttons()-4
 
# print(f'Your joystick_1 ({joy_name}) has:')
# print(f' - {joy_buttons} buttons')
# print(f' - {joy_axes} axes')
print('Os name = ', os.name)
time.sleep(1)
# Main loop to check joystick functionality
while True:
    # Clear the terminal
    os.system('cls' if os.name == 'nt' else 'clear')
    
    for x in range(len(gamepad_list)):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        
        joy_buttons = gamepad_list[i].get_numbuttons()-4
        # Filter
        axes_threshold = 0.05
        # Print buttons/axes info to the console
        button_info = [f'Button[{i}]:{gamepad_list[x].get_button(i)}' for i in range(joy_buttons)]
        axis_info = [f'Axes[{i}]: {gamepad_list[x].get_axis(i):.3f}' if abs(gamepad_list[x].get_axis(i)) >= axes_threshold else f'Axes[{i}]: 0.000' for i in range(gamepad_list[x].get_numaxes())]
        info_str = '--------------\n' + '\n'.join(button_info + axis_info) + '\n--------------'
        print(info_str)
 
    pygame.time.delay(50)  # Pause for 50 ms (equivalent to pause(0.05) in MATLAB)