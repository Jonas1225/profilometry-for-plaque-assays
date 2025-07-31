# capture_pipeline.py
import time

from ._capture_viewer import ViewerImageCapturer


def capture_three_phase_images(viewer):
    camera = ViewerImageCapturer(viewer, layer_name="Live: WidefieldCamera")

    images = []
    for i in range(3):

        img = camera.capture_and_add_layer(new_layer_name=f"I{i + 1}")

        if i < 2:
            time.sleep(
                1.4
            )  #  Adjust delay to match desired phase difference 4pi/3

        images.append(img)

    camera.close()

    return images
