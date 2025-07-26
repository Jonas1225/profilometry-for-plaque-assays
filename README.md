# Napari Plugin: Profilometry for Plaque Assays

This **napari plugin** enables 3-step phase reconstruction and profilometry depth mapping of biological samples such as plaque assays, using fringe image sets and region-of-interest (ROI) selections, with **hardware integration** for automated imaging using a **TFT display** and a **HIKROBOT industrial camera**.

---

## ✨ Features

- Capture phase-shifted fringe images from:
  - 🖥️ TFT screen (phase shift: 0, π/3, 2π/3)
  - 📸 HIKROBOT camera (MV-CE060-10UC)
- Automatically:
  - Preprocess images
  - Compute wrapped and unwrapped phase
  - Generate depth maps with and without planar background
- Visualize every intermediate result directly in napari

---

## 📦 Installation

Install this plugin using `pip`:

```bash
pip install profilometry-for-plaque-assays
```
Or install directly from a cloned GitHub repository:

```bash
git clone https://github.com/Jonas1225/profilometry-for-plaque-assays.git
cd profilometry-for-plaque-assays
pip install -e .
```
---

## 🧪 Requirements
- Python 3.8–3.11
- HIKROBOT MVS SDK (for USB camera control)
- Arduino (for TFT phase control via serial)
- Dependencies:
    - numpy, scikit-image, opencv-python, pyserial, magicgui, napari

---

## 🚀 Usage
- Connect the HIKROBOT camera and Arduino-controlled TFT screen.
- Open napari (with or without imSwitch).
- Load the plugin:
- 3-Step Phase Shift Reconstruction (Capture from TFT + HIK)
- Click "📸 Capture from Hardware" to begin automated acquisition.
- View wrapped/unwrapped phase and height maps directly in napari.

---

## 🧰 Hardware Notes
### TFT Phase Display
- Controlled via Arduino using RA8875 driver
- Receives serial commands like:
    - phase:0 → 0 rad
    - phase:1 → 2π/3 rad
    - phase:2 → 4π/3 rad
### Camera
- Model: MV-CE060-10UC
- SDK: HIKROBOT MVS Python SDK (MvImport)


---

## 🛠️ Developer Notes
profilometry-for-plaque-assays/
- src/
    - profilometry_for_plaque_assays/
        - _dock_widget
        - _control_display.py
        - _control_camera_hik.py
        - ...
    - napari.yaml
    - pyproject.toml
    - README.md

---

## Author
- Jonas1225
- GitHub: @Jonas1225



