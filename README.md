# ESP32-S3 Camera ROI and Blob Detection Documentation

## Overview
This ESP32-S3 project captures grayscale camera frames, extracts a ring-shaped Region Of Interest (ROI), and performs blob detection for bright IR-like signals.

## Current Behavior
- Board: ESP32-S3 Sense with OV3660 camera
- Resolution: 160x120 pixels (QQVGA)
- Pixel format: grayscale
- Frame capture mode: `CAMERA_GRAB_LATEST` when PSRAM is available
- Serial debug output every 32 frames

## ROI Construction
The ROI is a ring-shaped region centered at (80, 60) and is precomputed once during setup.

### Parameters
- **Image Size**: 160x120 pixels
- **Center**: (80, 60)
- **Outer radius**: 36 pixels
- **Inner radius**: 30 pixels
- **Ring width**: 6 pixels total
- **ROI pixel count**: about 1,600–1,700 pixels

### Construction Process
1. Compute squared distance from image center for each pixel
2. Include pixels where `30² ≤ d² ≤ 36²`
3. Store matching pixel indices in `roi_idx`
4. Use these indices for every frame analysis

## Blob Detection
Blob detection is seeded from the brightest ROI pixel and uses local intensity jumps to identify edges.

### Algorithm Steps
1. Find the brightest ROI pixel with intensity ≥ 40
2. For ROI pixels within a 50-pixel radius of that seed:
   - Require pixel intensity ≥ 40
   - Check 4-connected neighbors for an intensity drop ≥ 12
   - Count such pixels as edge pixels
3. If edge pixels ≥ 3, the blob is considered detected
4. Compute blob center as the average position of the detected edge pixels

### Detection Parameters
- **Minimum bright pixel intensity**: 40
- **Neighbor intensity drop threshold**: 12
- **Minimum edge pixels**: 3
- **Search radius**: 50 pixels around the seed

## Processing Flow
- `setup()` initializes serial output, camera, and ROI
- `loop()` captures each frame, calls `processFrame()`, then returns the framebuffer
- `processFrame()` computes ROI mean/min/max and performs blob-based edge detection
- Output is printed every 32 frames with intensity stats and blob center when detected

## Notes
- Camera initialization adapts for PSRAM availability by using two frame buffers and `CAMERA_GRAB_LATEST`
- When ROI is not available, the code falls back to analyzing a 48x48 center region
- The current implementation is tuned for fast grayscale ROI analysis on ESP32-S3
