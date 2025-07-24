import numpy as np


def calculate_wrapped_phase(I1, I2, I3):
    """
    Apply 3-step phase shift algorithm to compute wrapped phase in [0, 2π).
    """
    phi = np.arctan2(np.sqrt(3) * (I1 - I3), 2 * I2 - I1 - I3)
    return np.mod(phi, 2 * np.pi)
