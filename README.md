# Napari Plugin: Profilometry for Plaque Assays

This **napari plugin** enables 3-step phase reconstruction and profilometry depth mapping of biological samples such as plaque assays, using fringe image sets and region-of-interest (ROI) selections.

---

## ✨ Features

- Load 3-step fringe images (I1, I2, I3)
- Preprocess and enhance contrast
- Select ROI using rectangle shapes
- Compute wrapped and unwrapped phase
- Calculate height map (depth)
- Plane removal to flatten height surface
- Easy visualization inside Napari

---

## 🔧 Installation

You can install the plugin from source (recommended for development) or package:



### From GitHub (source install)

```bash
git clone https://github.com/Jonas1225/profilometry-for-plaque-assays.git
cd profilometry-for-plaque-assays
pip install -e .

# Or install the plugin directly  
pip install napari-profilometry-plaque-assays
