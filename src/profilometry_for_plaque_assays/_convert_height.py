import numpy as np


def compute_depth(unwrapped_phase, Scalelength=0.062):
    """
    Convert unwrapped phase to depth (in µm) using triangulation.
    
    Parameters:
        unwrapped_phase : 2D ndarray of unwrapped phase values (radians)
        Scalelength   : the width of TFT display (meters)

    Returns:
        Depth map in micrometers (µm)
    """
    # Display geometry
    N_pixels = 22  # measured number of fringes
    P_fringe = (Scalelength * 1000) /  N_pixels  # single pixel size in mm


    # Setup distances
    d = 100.0  # The distance from optical centers to the reference plane d mm
    l  = 100.0  #  The distance between the optical centers l mm

    # Optical magnification (set from my system)
    M = 0.0373

    # Conversion factor (mm/rad)
    K = (P_fringe * M * d) / (2.0 * np.pi * l)

    # Depth map in µm
    depth_um = K * unwrapped_phase * 1000.0
    return depth_um
