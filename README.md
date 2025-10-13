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
- ESP32 Arduino using RA8875 driver (for TFT display phase control)    
```bash
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_RA8875.h>

// Pin wiring
#define RA8875_SCK    18   // VSPI SCK
#define RA8875_MISO   19   // VSPI MISO
#define RA8875_MOSI   23   // VSPI MOSI
#define RA8875_CS      5   // CS
#define RA8875_RESET  22   // RST

Adafruit_RA8875 tft(RA8875_CS, RA8875_RESET);

// Pattern parameters
const int lineThickness = 4;
const int lineSpacing   = 20;
const int step = lineThickness + lineSpacing;  // Total period = 24 pixels

// Phase control
int currentPhase = 0;        // 0, 1, or 2 for three phases
bool patternChanged = true;  // Flag to indicate when to redraw
unsigned long lastPhaseTime = 0;
const unsigned long PHASE_DURATION = 1500;  // 1.5 seconds per phase (matches your Python timing)

// Serial command processing
String serialCommand = "";
bool commandComplete = false;

// 🔹 Function prototypes
void drawPhasePattern(int phase);
void checkSerialCommands();
void processCommand(String command);

void setup() {
  Serial.begin(115200);
  Serial.println("3-Step Profilometry Display Controller");
  Serial.println("Commands: 'P0', 'P1', 'P2' for phases, 'AUTO' for automatic cycling");
  
  // Initialize display
  pinMode(RA8875_RESET, OUTPUT);
  digitalWrite(RA8875_RESET, LOW);
  delay(50);
  digitalWrite(RA8875_RESET, HIGH);
  delay(50);
  
  SPI.begin(RA8875_SCK, RA8875_MISO, RA8875_MOSI, RA8875_CS);
  
  if (!tft.begin(RA8875_800x480)) {
    Serial.println("RA8875 not detected!");
    while (1);
  }
  
  Serial.println("RA8875 initialized");
  
  // Enable display + full backlight
  tft.displayOn(true);
  tft.GPIOX(true);
  tft.PWM1config(true, RA8875_PWM_CLK_DIV1024);
  tft.PWM1out(255);
  
  // Draw initial phase
  drawPhasePattern(currentPhase);
  Serial.println("Ready - Phase 0 displayed");
}

void loop() {
  // Check for serial commands
  checkSerialCommands();
  
  // Auto-cycling mode (comment out if you want manual control only)
  if (millis() - lastPhaseTime > PHASE_DURATION) {
    currentPhase = (currentPhase + 1) % 3;
    patternChanged = true;
    lastPhaseTime = millis();
    Serial.print("Auto-switched to Phase ");
    Serial.println(currentPhase);
  }
  
  // Redraw pattern if phase changed
  if (patternChanged) {
    drawPhasePattern(currentPhase);
    patternChanged = false;
  }
  
  delay(10);  // Small delay to prevent overwhelming the system
}

void drawPhasePattern(int phase) {
  // Clear screen
  tft.fillScreen(RA8875_BLACK);
  
  // Calculate phase offset in pixels
  // For 3-step profilometry: Phase 0 = 0°, Phase 1 = 120°, Phase 2 = 240°
  // Convert to pixel offset: (phase * step) / 3
  int phaseOffset = (phase * step) / 3;  // 0, 8, 16 pixels for phases 0,1,2
  
  // Draw horizontal stripes with phase offset
  for (int y = phaseOffset; y < tft.height(); y += step) {
    tft.fillRect(0, y, tft.width(), lineThickness, RA8875_WHITE);
  }
  
  // Debug output
  Serial.print("Phase ");
  Serial.print(phase);
  Serial.print(" drawn with offset ");
  Serial.print(phaseOffset);
  Serial.println(" pixels");
}

void checkSerialCommands() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      commandComplete = true;
    } else {
      serialCommand += inChar;
    }
    
    if (commandComplete) {
      processCommand(serialCommand);
      serialCommand = "";
      commandComplete = false;
    }
  }
}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  if (command == "P0") {
    currentPhase = 0;
    patternChanged = true;
    Serial.println("Manual: Phase 0 set");
  }
  else if (command == "P1") {
    currentPhase = 1;
    patternChanged = true;
    Serial.println("Manual: Phase 1 set");
  }
  else if (command == "P2") {
    currentPhase = 2;
    patternChanged = true;
    Serial.println("Manual: Phase 2 set");
  }
  else if (command == "AUTO") {
    lastPhaseTime = millis();
    Serial.println("Auto-cycling enabled");
  }
  else if (command == "STOP") {
    // Stop auto-cycling by setting a very long phase duration
    lastPhaseTime = millis() + 1000000;  // Effectively disable auto-cycling
    Serial.println("Auto-cycling stopped");
  }
  else if (command == "STATUS") {
    Serial.print("Current phase: ");
    Serial.println(currentPhase);
    Serial.print("Pattern: ");
    Serial.print(lineThickness);
    Serial.print("px lines, ");
    Serial.print(lineSpacing);
    Serial.println("px spacing");
  }
  else {
    Serial.println("Unknown command. Use: P0, P1, P2, AUTO, STOP, STATUS");
  }
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
- Run the processing on existing layers or click the "Capture from camera + TFT display" button to acquire new images. 
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
        - ...
    - napari.yaml
    - pyproject.toml
    - README.md

- _synchronized_capture_pipeline.py: fringe sample image acquisition logic.
- _capture_viewer.py: capture viewer.
- _preprocessing.py: image preprocessing steps.     
- _phase_utils.py: phase calculation functions.
- _convert_height.py: height/depth map computation.
- _plane_tools.py: planar background removal.

---

## Author
- Jonas1225
- GitHub: [@Jonas1225](https://github.com/Jonas1225)

```bash
""" 
Reconstructing the phase (and thus surface height) from a single fringe pattern using a 2D Poisson equation solver is a classic approach in optical metrology and phase retrieval. 
This method typically uses Fourier-based demodulation to extract phase gradients, followed by solving the Poisson equation to recover the continuous phase or surface height. 
1.Input: A single fringe pattern image (e.g., sinusoidal intensity pattern) 
2.Preprocessing: Convert to grayscale, enhance contrast 
3.Compute gradients: Estimate partial derivatives (∂φ/∂x, ∂φ/∂y) 
4.Poisson solver: Solve ∇²φ = div(grad φ) to reconstruct φ 
5.(Optional): Remove plane for flattening 
6.3D surface drawing 
"""
import numpy as np
import matplotlib.pyplot as plt
import imageio.v2 as imageio
from scipy.ndimage import sobel
from scipy.fftpack import fft2, ifft2
from skimage import exposure
from matplotlib.widgets import RectangleSelector
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

# -------------------------------
# Convert image to grayscale
# -------------------------------
def to_gray(img):
    if img.ndim == 4 and img.shape[0] == 1:
        img = img[0]
    if img.ndim == 3:
        gray = np.dot(img[..., :3], [0.2989, 0.5870, 0.1140])  # RGB to grayscale
    else:
        gray = img
    return gray.astype(np.float32)

# -------------------------------
# Enhance contrast using histogram stretch
# -------------------------------
def enhance_contrast(img):
    p2, p98 = np.percentile(img, (2, 98))
    return exposure.rescale_intensity(img, in_range=(p2, p98))

# -------------------------------
# Compute phase gradients
# -------------------------------
def compute_gradients(img):
    grad_x = sobel(img, axis=1)
    grad_y = sobel(img, axis=0)
    return grad_x, grad_y

# -------------------------------
# Poisson equation solver (∇²φ = div(grad φ))
# -------------------------------
def poisson_solver(grad_x, grad_y):
    h, w = grad_x.shape
    fx = np.zeros_like(grad_x)
    fy = np.zeros_like(grad_y)
    
    fx[:, :-1] = grad_x[:, :-1] - grad_x[:, 1:]
    fy[:-1, :] = grad_y[:-1, :] - grad_y[1:, :]
    
    div = fx + fy
    yy, xx = np.meshgrid(np.fft.fftfreq(h), np.fft.fftfreq(w), indexing='ij')
    denom = (2 * np.pi * 1j * xx)**2 + (2 * np.pi * 1j * yy)**2
    denom[0, 0] = 1.0  # avoid division by zero
    
    F_div = fft2(div)
    phi = np.real(ifft2(F_div / denom))
    phi -= np.min(phi)  # normalize phase origin
    return phi

# -------------------------------
# Convert phase to real-world height (in mm)
# -------------------------------
def compute_depth(unwrapped_phi, Scalelength=0.062, angle_deg=60.0):
    """
    Convert unwrapped phase to depth (in µm) using triangulation.
    
    Parameters:
        unwrapped_phi : 2D ndarray of unwrapped phase values (radians)
        Scalelength   : the width of TFT display (meters)
        angle_deg     : angle between projection and camera (degrees)

    Returns:
        Depth map in micrometers (µm)
    """
    # Display geometry
    N_pixels = 22  # measured number of pixels between fringes
    P_pixel = (Scalelength * 1000) / 480.0  # single pixel size in mm
    λ_fringe = N_pixels * P_pixel           # Fringe period (mm)

    # Setup distances
    d_display = 150.0  # mm
    d_camera  = 100.0  # mm
    G = (d_display + d_camera) / d_camera  # Geometric factor

    # Reflection correction
    R = 1.0 / np.sin(np.deg2rad(angle_deg))

    # Optical magnification (set from your system)
    M = 100

    # Conversion factor (mm/rad)
    K = (λ_fringe * G * R) / (2.0 * np.pi * M)

    # Depth map in µm
    depth_um = K * unwrapped_phi * 1000.0
    return depth_um

# -------------------------------
# Plot 3D surface
# -------------------------------
def plot_3d_surface(height):
    h, w = height.shape
    X, Y = np.meshgrid(np.arange(w), np.arange(h))
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    skip = 4  # downsampling for speed
    ax.plot_surface(X[::skip, ::skip], Y[::skip, ::skip], height[::skip, ::skip],
                    cmap=cm.viridis, edgecolor='none')
    ax.set_title('3D Reconstructed Surface (Height in mm)')
    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Y (pixels)')
    ax.set_zlabel('Height (mm)')
    ax.view_init(elev=45, azim=135)
    plt.tight_layout()
    plt.show()

# -------------------------------
# ROI selection callback
# -------------------------------
roi_coords = []

def line_select_callback(eclick, erelease):
    x1, y1 = int(eclick.xdata), int(eclick.ydata)
    x2, y2 = int(erelease.xdata), int(erelease.ydata)
    roi_coords.clear()
    roi_coords.append((min(x1, x2), min(y1, y2), abs(x2 - x1), abs(y2 - y1)))
    plt.close()

# -------------------------------
# Show image for manual ROI selection
# -------------------------------
def select_roi(image):
    fig, ax = plt.subplots()
    ax.imshow(image, cmap='gray')
    ax.set_title("Draw ROI with mouse, then close the window")
    toggle_selector = RectangleSelector(ax, line_select_callback,
                                        useblit=True,
                                        button=[1], minspanx=5, minspany=5,
                                        spancoords='pixels', interactive=True)
    plt.show()
    return roi_coords[0]

# -------------------------------
# Main reconstruction pipeline
# -------------------------------
def reconstruct_from_single_fringe(path):
    img = imageio.imread(path)
    gray = to_gray(img)
    contrast = enhance_contrast(gray)

    print("Please select the ROI area...")
    x, y, w, h = select_roi(contrast)
    roi = contrast[y:y + h, x:x + w]

    grad_x, grad_y = compute_gradients(roi)
    phi = poisson_solver(grad_x, grad_y)
    height = phase_to_height(phi, Scalelength=0.062, angle_deg=60.0)
    return roi, grad_x, grad_y, height

# -------------------------------
# Plot 2D results: original, gradients, height
# -------------------------------
def plot_all(fringe, grad_x, grad_y, height):
    plt.figure(figsize=(14, 8))
    titles = ['ROI Fringe', 'Gradient X', 'Gradient Y', 'Reconstructed Height (mm)']
    images = [fringe, grad_x, grad_y, height]
    cmaps = ['gray', 'seismic', 'seismic', 'jet']
    for i in range(4):
        plt.subplot(2, 2, i + 1)
        plt.imshow(images[i], cmap=cmaps[i])
        plt.title(titles[i])
        plt.colorbar()
        plt.axis('off') 
    plt.tight_layout()
    plt.show()

# -------------------------------
# Main Entry
# -------------------------------
if __name__ == "__main__":
    # Replace this path with your fringe image
    image_path = "C:\\Users\\Jiang\\Desktop\\sample1\\16.tiff"  # Modify as needed
    fringe, grad_x, grad_y, height = reconstruct_from_single_fringe(image_path)
    plot_all(fringe, grad_x, grad_y, height)
    plot_3d_surface(height)

```











