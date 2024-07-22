import pygame as py
from scene import Scene

# define constants
WIDTH = 800
HEIGHT = 400
FPS = 60 

# initialize pygame and create screen
py.init()  
screen = py.display.set_mode((WIDTH , HEIGHT))
clock = py.time.Clock()

# Create a scene to manage all the entities:
scene = Scene(screen, 0, 0, 10)

running = True
while running:
    clock.tick(FPS)
    screen.fill((255, 255, 255))

    # check for the exit
    for event in py.event.get():
        if event.type == py.QUIT:
            running = False

        # On mouse wheel event, zoom in the scene:
        elif event.type == py.MOUSEWHEEL:
            mouse_x, mouse_y = py.mouse.get_pos()
            scene.zoom(1.1 ** event.y, mouse_x, mouse_y)

        elif event.type == py.MOUSEBUTTONDOWN and event.button == 1:
            py.mouse.get_rel()

    if py.mouse.get_pressed(num_buttons=3)[0]:
        delta_x, delta_y = py.mouse.get_rel()
        scene.move(delta_x, -delta_y)

    # Update the scene:
    scene.update()

    # Draw the scene:
    scene.draw()

    # Flip the display after drawing everything:
    py.display.flip()

py.quit()