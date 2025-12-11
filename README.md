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
def compute_residues(phase_wrap):
    """
    Compute residues (±1 or 0) for each pixel
    using the classical 2x2 loop scheme.
    """
    M, N = phase_wrap.shape
    residue = np.zeros((M, N), dtype=np.int8)

    for i in range(M - 1):
        for j in range(N - 1):
            # Phase differences in 2x2 loop
            s = 0.0
            s += wrap_to_pi(phase_wrap[i, j + 1] - phase_wrap[i, j])
            s += wrap_to_pi(phase_wrap[i + 1, j + 1] - phase_wrap[i, j + 1])
            s += wrap_to_pi(phase_wrap[i + 1, j] - phase_wrap[i + 1, j + 1])
            s += wrap_to_pi(phase_wrap[i, j] - phase_wrap[i + 1, j])

            # s is typically ~ ±2π or 0
            if s > 1e-3:
                residue[i, j] = 1   # positive residue
            elif s < -1e-3:
                residue[i, j] = -1  # negative residue
            else:
                residue[i, j] = 0

    return residue

def connect_residues_with_branch_cuts(residue):
    """
    Very simplified Goldstein variant:
    - Collect positive and negative residues.
    - Connect each positive residue to the nearest negative
      with a Manhattan path (horizontal + vertical).
    - All pixels on these paths form branch cuts.
    """
    M, N = residue.shape
    branch_cuts = np.zeros((M, N), dtype=bool)

    pos_list = [(i, j) for i in range(M) for j in range(N) if residue[i, j] == 1]
    neg_list = [(i, j) for i in range(M) for j in range(N) if residue[i, j] == -1]

    used_neg = set()

    def draw_manhattan_path(p1, p2):
        (r1, c1), (r2, c2) = p1, p2
        # first horizontal, then vertical
        step_c = 1 if c2 >= c1 else -1
        for c in range(c1, c2 + step_c, step_c):
            branch_cuts[r1, c] = True

        step_r = 1 if r2 >= r1 else -1
        for r in range(r1, r2 + step_r, step_r):
            branch_cuts[r, c2] = True

    # For each positive residue: find nearest negative
    for p in pos_list:
        if not neg_list:
            break
        distances = [np.hypot(p[0] - n[0], p[1] - n[1]) for n in neg_list]
        k = int(np.argmin(distances))
        n = neg_list[k]
        if n in used_neg:
            continue
        used_neg.add(n)
        draw_manhattan_path(p, n)

    return branch_cuts

def unwrap_goldstein(phase_wrap):
    """
    Simplified Goldstein Branch-Cut unwrapping method:
    1) Compute residues.
    2) Place branch cuts between ±residues.
    3) Flood-fill / region-growing around branch cuts.
    """
    M, N = phase_wrap.shape

    # 1) Residues
    residue = compute_residues(phase_wrap)

    # 2) Branch cuts
    branch_cuts = connect_residues_with_branch_cuts(residue)

    # 3) Flood-fill unwrapping
    unwrapped = np.zeros_like(phase_wrap)
    visited = np.zeros((M, N), dtype=bool)

    # Start pixel – here (0,0)
    start = (0, 0)
    unwrapped[start] = phase_wrap[start]
    visited[start] = True

    from collections import deque
    q = deque([start])

    # 4-neighborhood
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while q:
        i, j = q.popleft()
        for di, dj in neighbors:
            ni, nj = i + di, j + dj
            if ni < 0 or ni >= M or nj < 0 or nj >= N:
                continue
            if visited[ni, nj]:
                continue
            # Skip branch-cut pixels
            if branch_cuts[ni, nj]:
                continue

            # local wrapped difference
            dphi = wrap_to_pi(phase_wrap[ni, nj] - phase_wrap[i, j])
            unwrapped[ni, nj] = unwrapped[i, j] + dphi
            visited[ni, nj] = True
            q.append((ni, nj))

    # Regions not reached due to branch cuts remain 0
    return unwrapped, residue, branch_cuts

```



