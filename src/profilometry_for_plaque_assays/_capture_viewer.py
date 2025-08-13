# _capture_viewer.py
import numpy as np
import time
from napari.viewer import Viewer
from napari.layers import Image
from qtpy.QtWidgets import QApplication


class ViewerImageCapturer:
    """Enhanced version with better live layer refresh"""
    def __init__(self, viewer: Viewer, layer_name: str):
        self.viewer = viewer
        self.layer_name = layer_name

    def _enhanced_refresh(self, layer):
        """Enhanced refresh with multiple methods"""
        
        # Method 1: Force Qt event processing
        QApplication.processEvents()
        time.sleep(0.05)
        
        # Method 2: Trigger viewer refresh
        if hasattr(self.viewer, 'reset_view'):
            try:
                self.viewer.reset_view()
            except:
                pass
        
        # Method 3: Layer-specific refresh
        if hasattr(layer, 'refresh'):
            try:
                layer.refresh()
            except:
                pass
        elif hasattr(layer, '_refresh'):
            try:
                layer._refresh()
            except:
                pass

        
        # Method 4: Force redraw
        try:
            layer.events.set_data()
        except:
            pass
        
        # Final Qt processing
        QApplication.processEvents()
        time.sleep(0.1)  # Allow time for refresh

    def capture_and_add_layer(self, new_layer_name: str) -> np.ndarray:
        """Enhanced capture with multiple refresh attempts"""
        target_layer = None
        original_visibility = {}

        # Find target layer
        for layer in self.viewer.layers:
            if layer.name == self.layer_name:
                target_layer = layer
            original_visibility[layer.name] = layer.visible
            layer.visible = layer.name == self.layer_name

        if target_layer is None:
            raise RuntimeError(f"Layer '{self.layer_name}' not found.")

        # Enhanced refresh sequence
        self._enhanced_refresh(target_layer)

        # Capture screenshot
        #size = (3072, 2048)
        #image = self.viewer.screenshot(
        #    canvas_only=True, flash=False, size=size
        #)

        # Capture raw layer data
        image = target_layer.data.copy()

        # Restore original visibility states
        for layer in self.viewer.layers:
            layer.visible = original_visibility[layer.name]

        # Add the captured image as a new layer
        self.viewer.add_image(image, name=new_layer_name)

        return image
    

    def close(self):
        """Cleanup resources if needed."""
        pass
