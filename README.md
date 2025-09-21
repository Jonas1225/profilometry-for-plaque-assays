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
import numpy as np
import imageio.v2 as imageio
import matplotlib.pyplot as plt
from matplotlib.widgets import RectangleSelector
from skimage.restoration import unwrap_phase
from skimage import exposure
from mpl_toolkits.mplot3d import Axes3D  # Required for 3D plotting

# --- Image Processing Utilities ---

def to_gray(img):
    """
    Convert input image to grayscale using standard luminance weights.
    If image has 4D (e.g., batch dimension), extract the first image.
    """
    if img.ndim == 4:
        img = img[0]
    if img.ndim == 3:
        # Use standard grayscale conversion formula
        img = np.dot(img[..., :3], [0.2989, 0.5870, 0.1140])
    return img.astype(np.float32)

def enhance_contrast(img):
    """
    Enhance contrast using 2nd and 98th percentiles (percentile stretch).
    """
    p2, p98 = np.percentile(img, (2, 98))
    return exposure.rescale_intensity(img, in_range=(p2, p98))

# --- Phase to Depth Calculation ---

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

# --- Plane Removal ---

def remove_plane(Z):
    """
    Fit a plane Z = ax + by + c to the depth map and subtract it.
    
    Returns:
        Z_flattened: height map with fitted plane removed
        plane      : the fitted plane
    """
    height, width = Z.shape
    X, Y = np.meshgrid(np.arange(width), np.arange(height))
    A = np.vstack([X.ravel(), Y.ravel(), np.ones_like(X.ravel())]).T
    coeffs, _, _, _ = np.linalg.lstsq(A, Z.ravel(), rcond=None)  # Solve Ax = Z
    plane = (coeffs[0] * X + coeffs[1] * Y + coeffs[2])
    return Z - plane, plane

# --- Wrapped Phase Calculation ---

def calculate_wrapped_phase(I1, I2, I3):
    """
    Calculate wrapped phase using the 3-step phase shifting algorithm.
    
    Formula:
        φ_wrapped = atan2(√3 * (I1 - I3), 2*I2 - I1 - I3)
    
    Returns:
        Wrapped phase in [0, 2π)
    """
    phi_wrapped = np.arctan2(np.sqrt(3) * (I1 - I3), 2 * I2 - I1 - I3)
    return np.mod(phi_wrapped, 2 * np.pi)

# --- Image Preprocessing Pipeline ---

def preprocess_image(path):
    """
    Load image, convert to grayscale, and enhance contrast.
    
    Parameters:
        path : file path to image

    Returns:
        Preprocessed grayscale image
    """
    img = imageio.imread(path)
    gray = to_gray(img)
    contrast = enhance_contrast(gray)
    return contrast  # return full image (no cropping)

# --- ROI Selection ---

def select_roi(image):
    """
    Launch an interactive matplotlib window for user to select ROI.
    
    Returns:
        ROI coordinates: (x1, y1, x2, y2)
    """
    fig, ax = plt.subplots()
    ax.imshow(image, cmap='gray')
    plt.title("Drag to select ROI, then close window")

    roi_coords = []

    def onselect(eclick, erelease):
        # Get rectangle corners from click & release
        x1, y1 = int(eclick.xdata), int(eclick.ydata)
        x2, y2 = int(erelease.xdata), int(erelease.ydata)
        roi_coords.clear()
        roi_coords.append((min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)))

    # Interactive rectangle tool
    rect_selector = RectangleSelector(ax, onselect,
                                      useblit=True,
                                      button=[1],  # left click
                                      minspanx=5, minspany=5,
                                      spancoords='pixels',
                                      interactive=True)
    plt.show()

    if roi_coords:
        return roi_coords[0]
    else:
        h, w = image.shape
        return (0, 0, w, h)

# --- Main Processing Function ---

def process_phase_images(image_paths, Scalelength=0.062, angle_deg=60.0, roi=None):
    """
    Full pipeline: load images, calculate phase, unwrap, compute depth.
    
    Parameters:
        image_paths : list of 3 file paths (I1, I2, I3)
        Scalelength : scale bar length in meters
        angle_deg   : projection angle in degrees
        roi         : optional region of interest (x1, y1, x2, y2)

    Returns:
        Dictionary with images, phase maps, depth map, and plane-removed result
    """
    if len(image_paths) != 3:
        raise ValueError("Exactly 3 phase-shifted images are required.")

    # Load and preprocess images
    I1, I2, I3 = [preprocess_image(p) for p in image_paths]

    # Select ROI if not provided
    if roi is None:
        roi = select_roi(I1)
    x1, y1, x2, y2 = roi

    # Crop to ROI
    I1_roi = I1[y1:y2, x1:x2]
    I2_roi = I2[y1:y2, x1:x2]
    I3_roi = I3[y1:y2, x1:x2]

    # Phase processing
    phi_wrapped = calculate_wrapped_phase(I1_roi, I2_roi, I3_roi)
    phi_unwrapped = unwrap_phase(phi_wrapped)

    # Convert to depth
    Z = compute_depth(phi_unwrapped, Scalelength, angle_deg)

    # Remove plane for visualization
    Z_flattened, plane = remove_plane(Z)

    return {
        'I1': I1_roi, 'I2': I2_roi, 'I3': I3_roi,
        'phi_wrapped': phi_wrapped,
        'phi_unwrapped': phi_unwrapped,
        'Z': Z,
        'plane': plane,
        'Z_flattened': Z_flattened,
        'roi': roi
    }

# --- Visualization ---

def plot_results(results):
    """
    Plot intermediate results and 3D surface.
    
    Parameters:
        results : dict returned by `process_phase_images`
    """
    I1, I2, I3 = results['I1'], results['I2'], results['I3']
    phi_wrapped = results['phi_wrapped']
    phi_unwrapped = results['phi_unwrapped']
    Z = results['Z']
    plane = results['plane']
    Z_flattened = results['Z_flattened']
    height, width = Z.shape
    X, Y = np.meshgrid(np.arange(width), np.arange(height))

    # Plot 8 subplots (images + phase + depth)
    plt.figure(figsize=(18, 12))
    titles = [
        'I1 (Enhanced)', 
        'I2 (Enhanced)', 
        'I3 (Enhanced)', 
        'Wrapped Phase φ (0~2π)', 
        'Unwrapped Phase Φ (continuous)', 
        'Original Height Z (μm)', 
        'Fitted Plane a·x + b·y + c', 
        'Flattened Height Z (μm)'
    ]
    images = [I1, I2, I3, phi_wrapped, phi_unwrapped, Z, plane, Z_flattened]
    cmaps = ['gray']*3 + ['twilight', 'twilight'] + ['jet']*3

    for i in range(8):
        plt.subplot(2, 4, i+1)
        plt.title(titles[i])
        im = plt.imshow(images[i], cmap=cmaps[i])
        plt.axis('off')
        plt.colorbar(im, fraction=0.046, pad=0.04)

    plt.tight_layout()
    plt.show()

    # 3D plot of the surface
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    skip = 5  # downsample to speed up rendering
    ax.plot_surface(X[::skip, ::skip], Y[::skip, ::skip], Z_flattened[::skip, ::skip],
                    cmap='viridis', edgecolor='none')
    ax.set_title('3D Surface (Flattened)')
    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Y (pixels)')
    ax.set_zlabel('Height (μm)')
    ax.view_init(elev=45, azim=135)
    ax.set_box_aspect([1, 1, 0.5])
    plt.tight_layout()
    plt.show()

# --- Entry Point (script run mode) ---

if __name__ == "__main__":
    image_paths = [
        'C:\\Users\\Jiang\\Desktop\\samples\\good\\I1_crop_1757595882.tiff', 
        'C:\\Users\\Jiang\\Desktop\\samples\\good\\I2_crop_1757595882.tiff',
        'C:\\Users\\Jiang\\Desktop\\samples\\good\\I3_crop_1757595882.tiff',
    ]

    results = process_phase_images(image_paths)
    print(f"Selected ROI (x1,y1,x2,y2): {results['roi']}")
    plot_results(results)

```









