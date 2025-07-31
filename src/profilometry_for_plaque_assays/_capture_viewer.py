import numpy as np
from napari.viewer import Viewer


class ViewerImageCapturer:
    def __init__(self, viewer: Viewer, layer_name="Live: WidefieldCamera"):
        self.viewer = viewer
        self.layer_name = layer_name

    def capture_and_add_layer(self, new_layer_name: str) -> np.ndarray:
        """
        Captures a screenshot of the specified live layer and adds it
        to the viewer as a new static image layer.

        Parameters:
            new_layer_name (str): Name for the new image layer.

        Returns:
            np.ndarray: Captured image array.
        """
        target_layer = None
        original_visibility = {}

        # Store original visibility state and isolate the target layer
        for layer in self.viewer.layers:
            if layer.name == self.layer_name:
                target_layer = layer
            original_visibility[layer.name] = layer.visible
            layer.visible = layer.name == self.layer_name

        if target_layer is None:
            raise RuntimeError(f"Layer '{self.layer_name}' not found.")

        # Capture a high-resolution screenshot (same size as raw image)
        size = (3072, 2048)
        image = self.viewer.screenshot(
            canvas_only=True, flash=False, size=size
        )

        # Restore original visibility states
        for layer in self.viewer.layers:
            layer.visible = original_visibility[layer.name]

        # Add the captured image as a new layer
        self.viewer.add_image(image, name=new_layer_name)

        return image

    def close(self):
        """
        Reserved method for releasing resources (currently unused).
        """
