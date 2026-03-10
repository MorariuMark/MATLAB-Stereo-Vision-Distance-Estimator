# MATLAB Stereo Vision Distance Estimator

## Overview
This project implements a robust stereo vision pipeline in MATLAB to measure the physical distance to a target object in real-time. Utilizing two standard USB webcams arranged on a fixed stereo baseline, the system calculates depth based on the horizontal displacement (disparity) of the target across the two image planes.

Instead of relying on idealized theoretical focal length formulas—which often fail in practice due to lens distortion on commercial webcams—this system utilizes an empirical calibration approach. It maps pixel disparity directly to real-world centimeters using piecewise cubic Hermite interpolation (`pchip`), ensuring high accuracy across the calibrated operational range.

## Key Technical Features
* **Hardware Resolution Locking:** Enforces a strict `1280x720` hardware resolution across both camera streams to ensure metric consistency and prevent OS-level auto-scaling from corrupting disparity calculations.
* **Robust Feature Extraction (Blob Analysis):** Utilizes binarization and morphological area constraints (bandpass filtering between 50 and 8000 pixels) on the reference frame to isolate the target while rejecting sensor noise and large environmental artifacts (e.g., shadows).
* **Dynamic Template Matching:** Extracts the target's Region of Interest (ROI) from the left frame and locates its corresponding pair in the right frame using Normalized Cross-Correlation (`normxcorr2`). This approach is highly resilient to localized lighting variations and contrast shifts.
* **Empirical Calibration & Interpolation:** Features a live calibration routine where users can map known physical distances to sub-pixel disparities, generating a non-linear calibration curve (`pchip`) to compensate for lens distortion.
* **Interactive GUI:** Custom MATLAB user interface for live video acquisition, interactive point calibration, and real-time depth visualization.

## Prerequisites
* **MATLAB** (Tested on recent releases)
* **Image Processing Toolbox**
* **MATLAB Support Package for USB Webcams** (Available free via the MATLAB Add-On Explorer)
* **Hardware:** 2x USB Webcams mounted on a rigid horizontal baseline.

## System Pipeline
1.  **Acquisition:** Capture synchronized frames from the Left and Right cameras via the `OpenLiveInterface` function.
2.  **Pre-processing:** Convert frames to grayscale and equalize dimensional matrices.
3.  **Reference Detection (Left):** Thresholding $\rightarrow$ Area Filtering $\rightarrow$ Centroid Extraction.
4.  **Target Matching (Right):** Dynamic template cropping $\rightarrow$ Epipolar-constrained search band generation $\rightarrow$ Normalized Cross-Correlation $\rightarrow$ Peak sub-pixel coordinate extraction.
5.  **Depth Calculation:** Calculate disparity ($d = |x_{left} - x_{right}|$) and map $d$ to physical depth ($Z$) using pre-calibrated `pchip` interpolation.

   
#Demo:
<img width="1095" height="904" alt="Image" src="https://github.com/user-attachments/assets/d2b9b687-39b7-4b88-94db-99eb65a74d06" />
![Image](https://github.com/user-attachments/assets/5991159c-0112-4834-a52b-e3f4f8653862)
![Image](https://github.com/user-attachments/assets/f9b2b71a-3c98-4699-8d09-6205ed35a021)
![Image](https://github.com/user-attachments/assets/ee667dd0-9f18-42c1-bd08-560605fa6a80)


## Setup and Usage
1. Mount two webcams parallel to each other on a fixed baseline.
2. Run `projectSWEA.m` in the MATLAB Command Window.
3. **Calibration:** * Select `1. New Live Calibration` from the main menu. 
   * Place an object at known distances (e.g., 70cm, 100cm, 125cm), input the actual distance in the prompt, and capture the points. 
   * Save the data to build the system's disparity-to-depth curve.
4. **Measurement:** * Select `2. Live Measurement` to view real-time distance estimations to your target. The GUI will render the matched coordinates and the calculated depth in centimeters.

## Author
* Morariu Christian-Mark - ETCTI UPT 
