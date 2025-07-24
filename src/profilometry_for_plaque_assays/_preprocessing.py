import numpy as np
from skimage import exposure
from skimage.color import rgb2gray


def to_gray(image):
    """
    Convert image to 2D grayscale if needed.
    Supports grayscale, RGB, or RGBA images.
    """
    if image.ndim == 3:
        if image.shape[-1] == 4:  # RGBA
            image = image[..., :3]
        image = rgb2gray(image)  # Converts to float64 grayscale
    elif image.ndim != 2:
        raise ValueError(
            "Unsupported image shape: must be 2D or 3D with 3/4 channels"
        )
    return image


def enhance_contrast(img):
    """
    Stretch contrast between 2nd and 98th percentiles.
    """
    p2, p98 = np.percentile(img, (2, 98))
    return exposure.rescale_intensity(img, in_range=(p2, p98))


def preprocess_image(image):
    """
    Preprocess image (NumPy array): convert to grayscale and enhance contrast.
    """
    gray = to_gray(image)
    return enhance_contrast(gray)
