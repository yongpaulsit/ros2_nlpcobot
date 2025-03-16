#!/usr/bin/env python3
import numpy as np
from math import tan
class TfCamera2World:
    def __init__(self, workspace_width=0.60, workspace_height=0.40, image_width=640, image_height=480):
        """
        Initialize the transformer with workspace and image dimensions.
        
        :param workspace_width: Width of the workspace in m (default: 0.60 m).
        :param workspace_height: Height of the workspace in m (default: 0.40 m).
        :param image_width: Width of the camera image in pixels (default: 640 px).
        :param image_height: Height of the camera image in pixels (default: 480 px).
        """
        self.workspace_width = workspace_width
        self.workspace_height = workspace_height
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
        # Normalize pixel coordinates to [-1, 1]
        u_norm = (2 * u / self.image_width) - 1
        v_norm = 1 - (2 * v / self.image_height)

        # Scale normalized coordinates to workspace dimensions
        x_world = (u_norm * self.workspace_width / 2) + camera_x
        y_world = (v_norm * self.workspace_height / 2) + camera_y

        # The z-coordinate is the same as the workspace height (0 m)
        z_world = 0.0

        return x_world, y_world, z_world


# Example usage
if __name__ == "__main__":
    # Initialize the transformer
    transformer = TfCamera2World()

    # Input pixel coordinates and camera position
    u, v = 163.405, 263.485  # Pixel coordinates
    camera_x, camera_y, camera_z = 0.4, 0.0, 1.0  # Camera position in world coordinates

    # Transform pixel coordinates to world coordinates
    x_world, y_world, z_world = transformer.pixel_to_world(u, v, camera_x, camera_y, camera_z)

    # Output the result
    print(f"World Coordinates: ({x_world:.2f}, {y_world:.2f}, {z_world:.2f}) cm")