import numpy as np


def remove_plane(Z):
    """
    Fit and subtract a plane from the depth map Z to flatten the surface.
    """
    height, width = Z.shape
    X, Y = np.meshgrid(np.arange(width), np.arange(height))
    A = np.vstack([X.ravel(), Y.ravel(), np.ones_like(X.ravel())]).T
    coeffs, _, _, _ = np.linalg.lstsq(A, Z.ravel(), rcond=None)
    plane = coeffs[0] * X + coeffs[1] * Y + coeffs[2]
    Z_flat = Z - plane
    return Z_flat, plane
