"""
This file is part of the Napari plugin: Profilometry for Plaque Assays.
It enables 3-step phase reconstruction and profilometry depth mapping of biological samples such as plaque assays, using fringe image sets and region-of-interest (ROI) selections, with hardware integration for automated imaging using a TFT display and a HIKROBOT industrial camera.
It is designed to work with the napari image viewer and provides a user-friendly interface for capturing and processing images.

It includes features for:
- Capturing phase-shifted fringe images from a TFT display and a HIKROBOT camera.
- Automatically preprocessing images, computing wrapped and unwrapped phase,
- Generating depth maps with and without planar background.
- Visualizing every intermediate result directly in napari.
- Saving results in various formats including TIFF and PNG.
- Integrating with napari's GUI for easy interaction and visualization.
- Allowing users to draw ROIs for focused analysis.
- Saving intermediate results for further analysis.
- Auto-selecting image layers I1 I2 I3 based on naming conventions.
- Adding a button to capture images directly from the camera viewer and TFT display.

"""

import os
import time

import imageio.v3 as iio
import matplotlib.pyplot as plt
import napari
import numpy as np
from magicgui import magic_factory, widgets
from magicgui.widgets import FileEdit
from napari.types import ImageData
from napari.utils.notifications import show_error, show_info
from qtpy.QtWidgets import QWidget
from skimage.restoration import unwrap_phase

from ._capture_pipeline import capture_three_phase_images
from ._convert_height import compute_depth
from ._phase_utils import calculate_wrapped_phase
from ._plane_tools import remove_plane
from ._preprocessing import preprocess_image


def extract_roi_from_shapes(viewer):
    for layer in viewer.layers:
        if isinstance(layer, napari.layers.Shapes):
            if len(layer.data) > 0:
                for i in reversed(range(len(layer.data))):
                    if layer.shape_type[i] in ("rectangle", "rect"):
                        shape = layer.data[i]
                        y_coords = shape[:, 0]
                        x_coords = shape[:, 1]
                        x1, x2 = int(np.min(x_coords)), int(np.max(x_coords))
                        y1, y2 = int(np.min(y_coords)), int(np.max(y_coords))
                        return x1, y1, x2, y2
    return None


def auto_select_layers(viewer):
    def find_layer(name):
        for layer in viewer.layers:
            if isinstance(layer, napari.layers.Image) and layer.name == name:
                data = layer.data
                # Layer: type=<class 'numpy.ndarray'>, shape=(2048, 3072, 4), dtype=uint8, color=RGBA
                if data.ndim == 3 and data.shape[-1] == 3:
                    color_type = "RGB"
                elif data.ndim == 3 and data.shape[-1] == 4:
                    color_type = "RGBA"
                elif data.ndim == 2:
                    color_type = "Grayscale"
                else:
                    color_type = f"Unknown (shape={data.shape})"
                print(
                    f"[Info] Layer '{name}': type={type(data)}, shape={data.shape}, dtype={data.dtype}, color={color_type}"
                )
                return data
        show_info(f"Auto-selecting layer '{name}'")
        show_error(f"Required layer '{name}' not found.")
        return None

    I1 = find_layer("I1")
    I2 = find_layer("I2")
    I3 = find_layer("I3")

    return I1, I2, I3


def float_to_uint16(image):
    """Convert a float image to uint16 after normalization."""
    image = np.nan_to_num(image)
    image = image - np.min(image)
    if np.max(image) > 0:
        image = image / np.max(image)
    return (image * 65535).astype(np.uint16)


def process_and_display(
    viewer, I1, I2, I3, scale_length, angle_deg, save_dir, save_intermediates
):
    """Shared processing for captured or selected images."""

    def save_colormap_png(image, path, cmap="viridis"):
        """Save a float image as a PNG with colormap."""
        image = np.nan_to_num(image)
        image = image - np.min(image)
        if np.max(image) > 0:
            image = image / np.max(image)
        plt.imsave(path, image, cmap=cmap)

    # Step 1: Preprocessing
    I1p = preprocess_image(I1)
    I2p = preprocess_image(I2)
    I3p = preprocess_image(I3)

    # Step 2: Phase calculation
    phi_wrapped = calculate_wrapped_phase(I1p, I2p, I3p)
    phi_unwrapped = unwrap_phase(phi_wrapped)

    # Step 3: Height map
    Z = compute_depth(phi_unwrapped, scale_length, angle_deg)
    Z_flat, _ = remove_plane(Z)

    # Step 4: Saving
    timestamp = int(time.time())
    os.makedirs(save_dir, exist_ok=True)

    if save_intermediates:
        # Save preprocessed images as TIFF (uint16)
        iio.imwrite(
            os.path.join(save_dir, f"I1_crop_{timestamp}.tiff"),
            float_to_uint16(I1p),
        )
        iio.imwrite(
            os.path.join(save_dir, f"I2_crop_{timestamp}.tiff"),
            float_to_uint16(I2p),
        )
        iio.imwrite(
            os.path.join(save_dir, f"I3_crop_{timestamp}.tiff"),
            float_to_uint16(I3p),
        )

        # Save intermediate float maps as colored PNGs
        save_colormap_png(
            phi_wrapped,
            os.path.join(save_dir, f"Wrapped_Phase_{timestamp}.png"),
            cmap="twilight",
        )
        save_colormap_png(
            phi_unwrapped,
            os.path.join(save_dir, f"Unwrapped_Phase_{timestamp}.png"),
            cmap="twilight",
        )
        save_colormap_png(
            Z,
            os.path.join(save_dir, f"Z_Original_{timestamp}.png"),
            cmap="viridis",
        )
        save_colormap_png(
            Z_flat,
            os.path.join(save_dir, f"Z_Flat_{timestamp}.png"),
            cmap="inferno",
        )

        # Also save raw data as .npy
        np.save(os.path.join(save_dir, f"Z_Flat_{timestamp}.npy"), Z_flat)
        show_info(f"Processing complete. Results saved to: {save_dir}")

    # Step 5: Display results in viewer
    viewer.add_image(I1p, name=f"I1 {timestamp}", colormap="gray")
    viewer.add_image(
        phi_wrapped, name=f"Wrapped Phase {timestamp}", colormap="twilight"
    )
    viewer.add_image(
        phi_unwrapped, name=f"Unwrapped Phase {timestamp}", colormap="twilight"
    )
    viewer.add_image(
        Z, name=f"Original Height (mm) {timestamp}", colormap="viridis"
    )
    viewer.add_image(
        Z_flat,
        name=f"Height - Plane Removed (mm) {timestamp}",
        colormap="inferno",
    )


def has_dock_widget(viewer, name: str):
    """Check if a dock widget with the given name exists (public, no warnings)."""
    return viewer.window._qt_window.findChild(QWidget, name) is not None


@magic_factory(
    I1_layer={
        "label": "Image I1",
        "tooltip": "Auto-selects by name if not specified",
    },
    I2_layer={"label": "Image I2"},
    I3_layer={"label": "Image I3"},
    Scalelength={"label": "TFT Scale Length (m)", "min": 0.001, "step": 0.001},
    angle_deg={"label": "Projection Angle (°)", "min": 0.0, "max": 90.0},
    save_dir={"widget_type": FileEdit, "label": "Save Directory", "mode": "d"},
    save_intermediates={
        "label": "Save Intermediate Results",
        "widget_type": "CheckBox",
        "value": True,
    },
)
def phase_3step_widget(
    viewer: "napari.viewer.Viewer",
    I1_layer: ImageData = None,
    I2_layer: ImageData = None,
    I3_layer: ImageData = None,
    Scalelength: float = 0.062,  # Default TFT display length in meters
    angle_deg: float = 60.0,
    save_dir: str = ".",
    save_intermediates: bool = True,
):
    # Step 1: Auto-select image layers if any are missing
    if I1_layer is None or I2_layer is None or I3_layer is None:
        I1_auto, I2_auto, I3_auto = auto_select_layers(viewer)
        I1_layer = I1_layer or I1_auto
        I2_layer = I2_layer or I2_auto
        I3_layer = I3_layer or I3_auto

    # Step 2: Add capture button only once
    if not has_dock_widget(viewer, "📸 Capture Button"):
        capture_button = widgets.PushButton(
            label="📸 Capture from camera + TFT display"
        )
        capture_button.native.setObjectName("📸 Capture Button")
        viewer.window.add_dock_widget(
            capture_button, area="right", name="📸 Capture Button"
        )

        @capture_button.clicked.connect
        def on_capture():
            try:
                show_info("Capturing images...")
                I1, I2, I3 = capture_three_phase_images(viewer)

                timestamp = int(time.time())
                os.makedirs(save_dir, exist_ok=True)
                if save_intermediates:
                    iio.imwrite(
                        os.path.join(save_dir, f"I1_raw_{timestamp}.tiff"),
                        float_to_uint16(I1),
                    )
                    iio.imwrite(
                        os.path.join(save_dir, f"I2_raw_{timestamp}.tiff"),
                        float_to_uint16(I2),
                    )
                    iio.imwrite(
                        os.path.join(save_dir, f"I3_raw_{timestamp}.tiff"),
                        float_to_uint16(I3),
                    )

                # viewer.add_image(I1, name=f"I1")
                # viewer.add_image(I2, name=f"I2")
                # viewer.add_image(I3, name=f"I3")

                roi = extract_roi_from_shapes(viewer)
                if roi is None:
                    shapes_layer = next(
                        (
                            l
                            for l in viewer.layers
                            if isinstance(l, napari.layers.Shapes)
                        ),
                        None,
                    )
                    if shapes_layer is None:
                        shapes_layer = viewer.add_shapes(
                            name="ROI", shape_type="rectangle"
                        )
                    shapes_layer.mode = "add_rectangle"
                    show_info("Please draw a rectangular ROI and run again.")
                    return

                x1, y1, x2, y2 = roi
                I1_crop = I1[y1:y2, x1:x2]
                I2_crop = I2[y1:y2, x1:x2]
                I3_crop = I3[y1:y2, x1:x2]

                process_and_display(
                    viewer,
                    I1_crop,
                    I2_crop,
                    I3_crop,
                    Scalelength,
                    angle_deg,
                    save_dir,
                    save_intermediates,
                )

            except Exception as e:
                show_error(f"Capture or processing failed:\n{e}")

        _ = on_capture  # avoid IDE warning

    # Step 3: Run if all image layers are available
    if I1_layer is not None and I2_layer is not None and I3_layer is not None:
        try:
            roi = extract_roi_from_shapes(viewer)
            if roi is None:
                shapes_layer = next(
                    (
                        l
                        for l in viewer.layers
                        if isinstance(l, napari.layers.Shapes)
                    ),
                    None,
                )
                if shapes_layer is None:
                    shapes_layer = viewer.add_shapes(
                        name="ROI", shape_type="rectangle"
                    )
                shapes_layer.mode = "add_rectangle"
                show_info("Please draw a rectangular ROI and run again.")
                return

            x1, y1, x2, y2 = roi
            I1_crop = I1_layer[y1:y2, x1:x2]
            I2_crop = I2_layer[y1:y2, x1:x2]
            I3_crop = I3_layer[y1:y2, x1:x2]

            process_and_display(
                viewer,
                I1_crop,
                I2_crop,
                I3_crop,
                Scalelength,
                angle_deg,
                save_dir,
                save_intermediates,
            )

        except Exception as e:
            show_error(f"Processing failed:\n{e}")
