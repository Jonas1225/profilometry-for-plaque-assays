"""
Open Napari and load three images

Create a Shapes layer (rectangle or polygon)

Use the shapes tool to draw an ROI

Open and run your widget

Select the "Phase 3-Step Processing" tool

Choose the three images and ROI layer

Set the scale length and projection angle

Click the "Run" button

Execute!
"""

import time

import napari
import numpy as np
from magicgui import magic_factory
from napari.types import ImageData
from napari.utils.notifications import show_error, show_info
from skimage.restoration import unwrap_phase

from ._convert_height import compute_depth
from ._phase_utils import calculate_wrapped_phase
from ._plane_tools import remove_plane
from ._preprocessing import preprocess_image


def extract_roi_from_shapes(viewer):
    """
    Try to extract the most recent rectangular ROI from Shapes layer.
    """
    for layer in viewer.layers:
        if isinstance(layer, napari.layers.Shapes):
            print("Found Shapes layer. Shape types:", layer.shape_type)
            # Ensure layer.shape_type is a list (each shape has a type)
            if len(layer.data) > 0 and len(layer.shape_type) > 0:
                # Find the last rectangle (or compatible shape)
                for i in reversed(range(len(layer.data))):
                    shape_type = layer.shape_type[i]
                    if shape_type in ("rectangle", "rect"):
                        shape = layer.data[i]
                        y_coords = shape[:, 0]
                        x_coords = shape[:, 1]
                        x1, x2 = int(np.min(x_coords)), int(np.max(x_coords))
                        y1, y2 = int(np.min(y_coords)), int(np.max(y_coords))
                        return x1, y1, x2, y2
    return None


@magic_factory(
    I1_layer={"label": "Image I1"},
    I2_layer={"label": "Image I2"},
    I3_layer={"label": "Image I3"},
    Scalelength={"label": "Scale Length (m)", "min": 0.001, "step": 0.001},
    angle_deg={"label": "Projection Angle (°)", "min": 0.0, "max": 90.0},
)
def phase_3step_widget(
    viewer: "napari.viewer.Viewer",
    I1_layer: ImageData,
    I2_layer: ImageData,
    I3_layer: ImageData,
    Scalelength: float = 0.062,
    angle_deg: float = 60.0,
):
    try:
        # Step 1: Extract ROI coordinates from shapes layer
        roi = extract_roi_from_shapes(viewer)
        print("Extracted ROI:", roi)
        if roi is None:
            # Prompt user to add a rectangle if none found
            shapes_layer = None
            for layer in viewer.layers:
                if isinstance(layer, napari.layers.Shapes):
                    shapes_layer = layer
                    break

            if shapes_layer is None:
                shapes_layer = viewer.add_shapes(
                    name="ROI", shape_type="rectangle"
                )
            shapes_layer.mode = "add_rectangle"
            show_info("Please draw a rectangular ROI on the image and run again.")
            return

        x1, y1, x2, y2 = roi

        # Step 2: Crop and preprocess the ROI region in each image
        I1_roi = I1_layer[y1:y2, x1:x2]
        I2_roi = I2_layer[y1:y2, x1:x2]
        I3_roi = I3_layer[y1:y2, x1:x2]

        I1p = preprocess_image(I1_roi)
        I2p = preprocess_image(I2_roi)
        I3p = preprocess_image(I3_roi)

        # Step 3: Phase processing
        phi_wrapped = calculate_wrapped_phase(I1p, I2p, I3p)
        phi_unwrapped = unwrap_phase(phi_wrapped)
        Z = compute_depth(phi_unwrapped, Scalelength, angle_deg)
        Z_flat, plane = remove_plane(Z)

        # Step 4: Display results with timestamp to avoid layer name conflicts
        timestamp = int(time.time())
        viewer.add_image(
            I1p, name=f"I1 (ROI) {timestamp}", colormap="gray", visible=False
        )
        viewer.add_image(
            phi_wrapped, name=f"Wrapped Phase {timestamp}", colormap="twilight"
        )
        viewer.add_image(
            phi_unwrapped,
            name=f"Unwrapped Phase {timestamp}",
            colormap="twilight",
        )
        viewer.add_image(
            Z, name=f"Height (mm) {timestamp}", colormap="viridis"
        )
        viewer.add_image(
            Z_flat, name=f"Height - Plane {timestamp}", colormap="inferno"
        )

        show_info("3-Step Phase Processing completed successfully!")

    except Exception as e:
        show_error(f"Processing failed:\n{str(e)}")
