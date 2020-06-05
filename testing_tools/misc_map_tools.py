
import numpy as np
import cv2 as cv


def make_map(width, height, grid_spacing):
    _map = np.ones((height, width, 3), np.uint8) * 255

    # Draw the grid:
    for x in np.arange(grid_spacing, width, grid_spacing):
        cv.line(_map, (x, 0), (x, height), (255,200,150), 2)
    for y in np.arange(grid_spacing, height, grid_spacing):
        cv.line(_map, (0, y), (width, y), (255,200,150), 2)

    # Radial gridlines:
    max_radius = np.sqrt(height**2 + (width/2)**2)
    x_center = int(round(width/2))
    y_center = int(round(height/2))
    for r in np.arange(grid_spacing, max_radius, grid_spacing):
        cv.circle(_map, (x_center, y_center), int(r), (255,200,150), 2)

    # The position of the rover:
    #cv.rectangle(_map, (x_center-25, height-10), (x_center+25, height), (0,0,0), -1)
    return _map

