import argparse

# Toggle here which mapping method to use (activating both can be slow):
USE_GRIDS_MAPPING = True
USE_GEOMETRY_MAPPING = True

# Get the folder from which we should get the LIDAR captures using the command line:
parser = argparse.ArgumentParser(
                    prog='DynamicMapping',
                    description='Dynamic mapping algorithms for 2D LIDARs')
parser.add_argument('-f', '--folder', type=str, 
                    help="Relative path of the folder where the recorded data is")
args = parser.parse_args()

# Do other imports after the argument parsing, for a faster reaction of the command
# line when we are just printing the help message of the program
import os
import pygame as py

from scene.scene import Scene

# Default folder where to search for LIDAR data:
DIR_NAME = os.path.dirname(os.path.realpath(__file__))
DEFAULT_DATAFOLDER = os.path.join(DIR_NAME, "../../LidarCaptures/DynamicCapture_0")

# Set this to True to run the simulation frame by frame (pressing the right 
# arrow key to go to the next frame):
FRAME_MODE = False

# Dimensions of the screen in pixels
WIDTH = 800
HEIGHT = 400

# Target Framerate:
FPS = 60

if args.folder is None:
    datafolder = DEFAULT_DATAFOLDER
    print("\nUsing the default datafolder:", datafolder)
else:
    datafolder = os.path.join(os.getcwd(), args.folder)
    print("\nUsing the recorded data located at:", datafolder)

# initialize pygame and create screen
py.init()
py.font.init()
screen = py.display.set_mode((WIDTH, HEIGHT))
clock = py.time.Clock()

# Create a scene to manage all the entities:
scene = Scene(screen, 0, 0, 10, datafolder, USE_GRIDS_MAPPING, USE_GEOMETRY_MAPPING)

running = True
while running:
    clock.tick(FPS)    
    screen.fill((255, 255, 255))

    # By default, the scene is updated only if we are not in frame mode:
    update_scene = not FRAME_MODE

    # Manage events:
    for event in py.event.get():
        if event.type == py.QUIT:
            running = False

        # On mouse wheel event, zoom in the scene:
        elif event.type == py.MOUSEWHEEL:
            mouse_x, mouse_y = py.mouse.get_pos()
            scene.zoom(1.1 ** event.y, mouse_x, mouse_y)

        elif event.type == py.MOUSEBUTTONDOWN and event.button == 1:
            py.mouse.get_rel()

        elif event.type == py.KEYDOWN:
            # If we are in the frame mode, we update the scene only if we press the 
            # right arrow key:
            if FRAME_MODE and event.key == py.K_RIGHT:
                update_scene = True
                
            scene.on_key_down(event.key)

    if py.mouse.get_pressed(num_buttons=3)[0]:
        delta_x, delta_y = py.mouse.get_rel()
        scene.move(delta_x, -delta_y)

    # Update the scene:
    if update_scene:
        scene.update(FRAME_MODE)

    # Draw the scene:
    scene.draw(clock)

    # Flip the display after drawing everything:
    py.display.flip()

py.quit()