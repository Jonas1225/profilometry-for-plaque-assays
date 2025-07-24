"""
打开 Napari 并加载三幅图像

创建一个 Shapes 图层（矩形或多边形）

使用形状工具绘制一个 ROI 区域

打开并运行你的 widget

选择 "Phase 3-Step Processing" 工具

选择三幅图像和 ROI 图层

设置比例长度和投影角度

点击 "Run" 按钮

执行！

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
    for layer in viewer.layers:
        if isinstance(layer, napari.layers.Shapes):
            print("Found Shapes layer. Shape types:", layer.shape_type)
            # 确保 layer.shape_type 是一个列表（每个形状一个类型）
            if len(layer.data) > 0 and len(layer.shape_type) > 0:
                # 找到最后一个矩形（或其他形状也可以兼容）
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
        # Step 1: ROI 提取
        roi = extract_roi_from_shapes(viewer)
        print("Extracted ROI:", roi)
        if roi is None:
            # 引导用户添加矩形
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
            show_info("请在图像上拖动绘制一个矩形 ROI，然后再次运行处理。")
            return

        x1, y1, x2, y2 = roi

        # Step 2: 裁剪 & 预处理
        I1_roi = I1_layer[y1:y2, x1:x2]
        I2_roi = I2_layer[y1:y2, x1:x2]
        I3_roi = I3_layer[y1:y2, x1:x2]

        I1p = preprocess_image(I1_roi)
        I2p = preprocess_image(I2_roi)
        I3p = preprocess_image(I3_roi)

        # Step 3: 相位处理
        phi_wrapped = calculate_wrapped_phase(I1p, I2p, I3p)
        phi_unwrapped = unwrap_phase(phi_wrapped)
        Z = compute_depth(phi_unwrapped, Scalelength, angle_deg)
        Z_flat, plane = remove_plane(Z)

        # Step 4: 显示图层，使用时间戳避免覆盖
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

        show_info("3-Step Phase Processing 完成！")

    except Exception as e:
        show_error(f"处理失败:\n{str(e)}")
