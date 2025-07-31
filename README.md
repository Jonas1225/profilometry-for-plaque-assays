# Napari Plugin: Profilometry for Plaque Assays

This plugin enables **3-step phase reconstruction** and **profilometry depth mapping** of biological samples (e.g., plaque assays) using fringe image sets and region-of-interest (ROI) selection.
It integrates with hardware — a TFT display and a HIKROBOT industrial camera — for automated image capture, and works within the Napari image viewer for an interactive experience.


---

## ✨ Features

- Capture phase-shifted fringe images from:
  - 🖥️ TFT screen (phase shift: 0, π/3, 2π/3)
  - 📸 HIKROBOT camera (MV-CE060-10UC)
- Automatically:
  - Preprocess images
  - Compute wrapped and unwrapped phase
  - Generate depth maps with and without planar background
- Visualize every intermediate result directly in napar
- Save raw and processed images in TIFF, PNG, and Numpy .npy formats
- Region-of-interest selection via rectangle shapes in Napari
- Option to enable/disable saving of intermediate results

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
```bash
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_RA8875.h>

// â€” Pin wiring â€”
#define RA8875_SCK    18   // VSPI SCK
#define RA8875_MISO   19   // VSPI MISO
#define RA8875_MOSI   23   // VSPI MOSI
#define RA8875_CS      5   // CS
#define RA8875_RESET  22   // RST

// Only the 2-argument constructor is available
Adafruit_RA8875 tft(RA8875_CS, RA8875_RESET);

// Pattern parameters
const int lineThickness = 4;
const int lineSpacing   = 20;
int offsetY = 0;

void setup() {
  Serial.begin(115200);

  // 1) Pulse RST for a clean start
  pinMode(RA8875_RESET, OUTPUT);
  digitalWrite(RA8875_RESET, LOW);
  delay(50);
  digitalWrite(RA8875_RESET, HIGH);
  delay(50);

  // 2) Remap the default SPI bus to your VSPI pins + CS
  //    (SCK, MISO, MOSI, SS)
  SPI.begin(RA8875_SCK, RA8875_MISO, RA8875_MOSI, RA8875_CS);

  // 3) Initialize the RA8875 at 800Ã—480 (change if your panel differs)
  if (!tft.begin(RA8875_800x480)) {
    Serial.println("RA8875 not detected!");
    while (1);
  }
  Serial.println("RA8875 initialized");

  // 4) Enable display + full backlight
  tft.displayOn(true);
  tft.GPIOX(true);
  tft.PWM1config(true, RA8875_PWM_CLK_DIV1024);
  tft.PWM1out(255);

  // 5) Clear the screen
  tft.fillScreen(RA8875_BLACK);
}

void loop() {
  // 1) Clear previous frame
  tft.fillScreen(RA8875_BLACK);

  // 2) Draw horizontal stripes shifted by offsetY
  int step = lineThickness + lineSpacing;
  for (int y = (offsetY % step); y < tft.height(); y += step) {
    tft.fillRect(0, y, tft.width(), lineThickness, RA8875_WHITE);
  }

  // 3) Advance and wrap the offset
  offsetY = (offsetY + 1) % step;

  // 4) Control scroll speed
  delay(50);
}

```
- Dependencies:
    - numpy, scikit-image, opencv-python, pyserial, magicgui, napari

---

## 🚀 Usage
- Connect the HIKROBOT camera and Arduino-controlled TFT screen.
- Open napari (with imSwitch) and load your fringe images (or later capture them using the integrated hardware).
- Select three image layers named or containing "I1", "I2", and "I3" respectively.
- Draw a rectangular ROI using the Napari Shapes layer if needed.
- Open the plugin widget (Profilometry for Plaque Assays).
- Set parameters:
  - TFT Scale Length (m): physical length of your TFT display (default: 0.062 m).
  - Projection Angle (°): angle of projection (default: 60°).
  - Save Directory: folder where outputs will be saved.
  - Save Intermediate Results: enable or disable saving of intermediate TIFF and PNG images.
- Click the Capture from camera + TFT display button to acquire new images or run the processing on existing layers.
- Processed results and intermediate images will be displayed in Napari and saved to the specified directory.

---

## Notes
- The plugin automatically detects and crops images based on the rectangular ROI drawn in Napari.
- If no ROI is detected, the plugin will prompt the user to draw one.
- Intermediate images include preprocessed fringes, wrapped/unwrapped phase maps, and height maps.
- Depth maps are computed in millimeters.
- The plugin depends on the napari, magicgui, numpy, scikit-image, and imageio packages.

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
    - MvImport
    - profilometry_for_plaque_assays/
        - _dock_widget
        - _control_display.py
        - _control_camera_hik.py
        - ...
    - napari.yaml
    - pyproject.toml
    - README.md

- _capture_pipeline.py: camera and TFT image acquisition logic.
- _preprocessing.py: image preprocessing steps.
- _phase_utils.py: phase calculation functions.
- _convert_height.py: height/depth map computation.
- _plane_tools.py: planar background removal.

---

## Author
- Jonas1225
- GitHub: [@Jonas1225](https://github.com/Jonas1225)
