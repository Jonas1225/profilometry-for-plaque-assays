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
    # Display geometry
    N_pixels = 22  # measured number of pixels between fringes
    P_pixel = (Scalelength * 1000) / 480.0  # single pixel size in mm
    λ_fringe = N_pixels * P_pixel           # Fringe period (mm)

    # Setup distances
    d_display = 150.0  # mm
    d_camera  = 100.0  # mm
    G = (d_display + d_camera) / d_camera  # Geometric factor

    # Reflection correction
    R = 1.0 / np.sin(np.deg2rad(angle_deg))

    # Optical magnification (set from your system)
    M = 100

    # Conversion factor (mm/rad)
    K = (λ_fringe * G * R) / (2.0 * np.pi * M)

    # Depth map in µm
    depth_um = K * unwrapped_phi * 1000.0
    return depth_um
