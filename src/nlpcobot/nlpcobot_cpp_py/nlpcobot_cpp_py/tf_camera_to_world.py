#!/usr/bin/env python3
import numpy as np
from math import tan


class TfCamera2World:
    def __init__(self, x_min=-0.1, x_max=0.7, y_min=-0.5, y_max=0.5, image_width=640, image_height=480):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

        self.image_width = image_width
        self.image_height = image_height

    def pixel_to_world(self, u, v, camera_x, camera_y, camera_z):
        """
        Convert pixel coordinates to world coordinates relative to the camera.

        :param u: Pixel x-coordinate.
        :param v: Pixel y-coordinate.
        :param camera_x: Camera's x-coordinate in world space.
        :param camera_y: Camera's y-coordinate in world space.
        :param camera_z: Camera's z-coordinate in world space.
        :return: World coordinates (x_world, y_world, z_world) in m.
        """
        # Map pixel coordinates to world coordinates
        x_world = self.x_max - (v / self.image_height) * (self.x_max - self.x_min)

        # midpoint = self.image_width / 2
        # if u > midpoint:
        #     y_world = (u / midpoint) * self.y_max
        # else:
        #     y_world = ((u - midpoint) / midpoint) * self.y_min
        y_world = self.y_max - (u / self.image_width) * (self.y_max - self.y_min)

        # Assuming a fixed z height (for now)
        z_world = 0.25

        return x_world, y_world, z_world


# Example usage
if __name__ == "__main__":
    # Initialize the transformer
    transformer = TfCamera2World()

    # Input pixel coordinates and camera position
    u, v = 163.405, 263.485  # Pixel coordinates
    # Camera position in world coordinates
    camera_x, camera_y, camera_z = 0.4, 0.0, 1.0

    # Transform pixel coordinates to world coordinates
    x_world, y_world, z_world = transformer.pixel_to_world(
        u, v, camera_x, camera_y, camera_z)

    # Output the result
    print(f"World Coordinates: ({x_world:.2f}, {y_world:.2f}, {z_world:.2f})")
