# import pygame
# from pygame.locals import *
# from OpenGL.GL import *
# from OpenGL.GLUT import *
# from OpenGL.GLU import *

# # Initialize Pygame
# pygame.init()

# # Set up the Pygame window
# window_width, window_height = 800, 600
# pygame.display.set_mode((window_width, window_height), DOUBLEBUF | OPENGL)


# # Example mesh rendering

# # Main game loop
# running = True
# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False

#     keys = pygame.key.get_pressed()
#     if keys[pygame.K_LEFT]:
#         # Update mesh transformation for left arrow key
#         print('left')
#     if keys[pygame.K_RIGHT]:
#         # Update mesh transformation for right arrow key
#         print('right')
#     if keys[pygame.K_UP]:
#         # Update mesh transformation for up arrow key
#         print('up')
#     if keys[pygame.K_DOWN]:
#         # Update mesh transformation for down arrow key
#         print('down')

#     pygame.time.wait(400)

# pygame.quit()



# import pygame module in this program
import pygame

# activate the pygame library
# initiate pygame and give permission
# to use pygame's functionality.
pygame.init()

# define the RGB value for white,
# green, blue colour.
white = (255, 255, 255)
green = (0, 255, 0)
blue = (0, 0, 128)

# assigning values to X and Y variable
X = 250
Y = 50

# create the display surface object
# of specific dimension (X, Y).
display_surface = pygame.display.set_mode((X, Y))

# set the pygame window name
pygame.display.set_caption('Show Text')

# create a font object.
# 1st parameter is the font file
# which is present in pygame.
# 2nd parameter is the size of the font
font = pygame.font.Font('freesansbold.ttf', 18)

# create a text surface object,
# on which text is drawn on it.
text = font.render('Press here to control', True, green, blue)

# create a rectangular object for the
# text surface object
textRect = text.get_rect()

# set the center of the rectangular object.
textRect.center = (X // 2, Y // 2)

# infinite loop
running = True
while running:
    # completely fill the surface object
    # with white color
    display_surface.fill(white)

    # copying the text surface object
    # to the display surface object
    # at the center coordinate.
    display_surface.blit(text, textRect)

    keys = pygame.key.get_pressed()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    if keys[pygame.K_LEFT]:
        # Update mesh transformation for left arrow key
        print('left')
    if keys[pygame.K_RIGHT]:
        # Update mesh transformation for right arrow key
        print('right')
    if keys[pygame.K_UP]:
        # Update mesh transformation for up arrow key
        print('up')
    if keys[pygame.K_DOWN]:
        # Update mesh transformation for down arrow key
        print('down')

    pygame.time.wait(100)

    # Draws the surface object to the screen.
    pygame.display.update()

# deactivates the pygame library
pygame.quit()

# quit the program.
quit()
