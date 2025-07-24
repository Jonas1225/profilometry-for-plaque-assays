import numpy as np


def compute_depth(unwrapped_phi, Scalelength=0.062, angle_deg=60.0):
    """
    Convert unwrapped phase to depth (in mm) using triangulation.

    Parameters:
        unwrapped_phi : 2D ndarray of unwrapped phase values (radians)
            The unwrapped phase map obtained from phase shifting algorithm.
        Scalelength   : float, optional
            The physical length of the scale in meters (default: 0.062 m).
        angle_deg     : float, optional
            The angle between the projector and the camera in degrees (default: 60.0°).

    Returns:
        depth_mm : 2D ndarray of float
            Reconstructed depth map in millimeters.
    """
    # Step 1: Calculate pitch of the projected pattern in meters
    pitch_m = Scalelength / 22  # 22 fringes in the scale region

    # Step 2: Convert angle to radians
    angle_rad = np.deg2rad(angle_deg)

    # Step 3: Apply triangulation formula
    depth_m = (pitch_m / (2 * np.pi * np.tan(angle_rad))) * unwrapped_phi

    # Step 4: Convert to millimeters
    return depth_m * 1000.0
