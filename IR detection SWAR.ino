/*
* Swarm Bots — Stage 1: Capture + ROI + Intensity + Blob Detection
* Board: ESP32-S3 Sense (OV3660)
* PURPOSE:
*  - Capture grayscale frame
*  - Extract Region Of Interest (ROI)
*  - Compute intensity average for calibration
*  - Detect IR signal blobs based on sudden intensity changes
*/
#include "esp_camera.h"
#include <Arduino.h>
#include <math.h>

// Select camera model
#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "camera_pins.h"

// IMAGE SETTINGS - can be increased or tailored as long as framerate >=32 fps
#define IMG_W 160
#define IMG_H 120

// ROI SETTINGS (tuneable)
// start with center-based circular ROI  -> center of image (fixed reference point)
// cx = IMG_W / 2
// cy = IMG_H / 2 
#define ROI_CX IMG_W / 2
#define ROI_CY IMG_H / 2
// Ring-shaped ROI measured on downscaled 20x15 view (8x magnification)
// Observed: ~9 downscaled pixels diameter, 2-pixel ring thickness
// Scaled to 160x120 frame: 72-pixel diameter, 16-pixel ring thickness
// Full circle: center (80, 60), radius 36 → bounds [44-116, 24-96] - cross peak values
// Detection ring: outer 8 pixels from radius 28 to 36
#define CIRCLE_RADIUS 36
#define ROI_MARGIN 5  // ± pixels from center, spans radius [(36-margin*2) -> 36]
#define ROI_R CIRCLE_RADIUS - ROI_MARGIN      // midpoint of ring (26 to 36)
// ROI storage (global variables)
#define MAX_ROI_PIXELS 2000  // Ring area: π*[36^2-(36-margin*2)^2] ≈ 1608 pixels if margin=3
int roi_count = 0;
int roi_idx[MAX_ROI_PIXELS];
// debug output
#define PRINT_EVERY_FRAME 8

// BLOB DETECTION PARAMETERS (tuneable for IR signal detection)
#define BLOB_MIN_INTENSITY 40       // minimum bright_px intensity to start blob detection
#define BLOB_NEIGHBOR_DIFF 12       // threshold to determine if a neighbor pixel is an edge
#define BLOB_MIN_EDGE_PIXELS 3     // minimum edge pixels required to detect a blob
#define BLOB_CENTER_MARGIN 4        // margin around detected center for final clustering

// CAMERA CONFIG (Seeed XIAO ESP32-S3 Sense / OV3660)
// Pins from Seeed documentation
#define PWDN_GPIO_NUM     -1  // Not used
#define RESET_GPIO_NUM    -1  // Not used

#define XCLK_GPIO_NUM      10 
#define SIOD_GPIO_NUM      40  
#define SIOC_GPIO_NUM      39  

#define Y9_GPIO_NUM       48  
#define Y8_GPIO_NUM       11  
#define Y7_GPIO_NUM       12  
#define Y6_GPIO_NUM       14  
#define Y5_GPIO_NUM       16  
#define Y4_GPIO_NUM       18  
#define Y3_GPIO_NUM       17  
#define Y2_GPIO_NUM       15  

#define VSYNC_GPIO_NUM    38  
#define HREF_GPIO_NUM     47  
#define PCLK_GPIO_NUM     13  


// ROI PRECOMPUTE
void buildROI()
{
  #ifdef ROI_R
  // Ring geometry: pixels where (ROI_R - ROI_MARGIN)^2 ≤ d^2 ≤ (ROI_R + ROI_MARGIN)^2
  // where d^2 = (x - ROI_CX)^2 + (y - ROI_CY)^2
  
  roi_count = 0;

  int rmin = ROI_R - ROI_MARGIN;
  int rmax = ROI_R + ROI_MARGIN;
  int rmin2 = rmin * rmin;
  int rmax2 = rmax * rmax;

  for (int y = 0; y < IMG_H; y++)
  {
    for (int x = 0; x < IMG_W; x++)
    {
      int dx = x - ROI_CX;
      int dy = y - ROI_CY;
      int d2 = dx * dx + dy * dy;

      // Check if pixel falls within ring boundaries
      if (d2 >= rmin2 && d2 <= rmax2)
      {
        if (roi_count < MAX_ROI_PIXELS)
        {
          // Convert 2D coordinates to 1D index for frame buffer
          // Frame is stored as: [y0_x0, y0_x1, ... y0_xW, y1_x0, ...]
          roi_idx[roi_count++] = y * IMG_W + x;
        }
      }
    }
  }
  Serial.print("[ROI] pixels: ");
  Serial.print(roi_count);
  
  // Compute and print ROI bounds for validation
  int roi_min_x = IMG_W;
  int roi_max_x = 0;
  int roi_min_y = IMG_H; 
  int roi_max_y = 0;
  
  for (int i = 0; i < roi_count; i++)
  {
    int idx = roi_idx[i];
    int x = idx % IMG_W;
    int y = idx / IMG_W;
    if (x < roi_min_x){
    roi_min_x = x;
  }
    if (x > roi_max_x){ 
      roi_max_x = x;
    }
    if (y < roi_min_y) {
      roi_min_y = y;
    }
    if (y > roi_max_y) {
      roi_max_y = y;
    }
  }
  // Print bounding box of ROI (should approximate the ring dimensions)\n  Serial.print(" x=[");
  Serial.print(roi_min_x);
  Serial.print("-");
  Serial.print(roi_max_x);
  Serial.print("] y=[");
  Serial.print(roi_min_y);
  Serial.print("-");
  Serial.print(roi_max_y);
  Serial.println("]");
  
  #else
  Serial.println("WARNING: ROI_R not defined -> full frame analysis.");
  roi_count = 0;
  #endif
}

// =========================
// CAMERA INIT
// =========================
bool initCamera()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;

  // IMPORTANT: grayscale for fast processing
  config.pixel_format = PIXFORMAT_GRAYSCALE;

  config.frame_size = FRAMESIZE_QQVGA;  // 160x120 resolution (matches IMG_W and IMG_H)
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with higher JPEG quality for streaming
  if (psramFound()) {
    config.jpeg_quality = 15;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    // Limit the frame size when PSRAM is not available
    config.frame_size = FRAMESIZE_QQVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK)
  {
    Serial.printf("[CAM] init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
    //s->set_ae_level(s,-2);

    // Manual exposure control for consistent IR detection
    s->set_exposure_ctrl(s, 0);  // Disable auto exposure
    s->set_aec_value(s, 5);     // Set manual exposure to 20ms

    // Manual gain control
    s->set_gain_ctrl(s, 0);      // Disable auto gain
    s->set_agc_gain(s, 0);      // Set manual gain to 10
  }

  Serial.println("[CAM] init OK");
  return true;
}

// PROCESS FRAME - Intensity Analysis and Blob Detection
void processFrame(camera_fb_t *fb, int frameId)
{
  uint8_t *img = fb->buf;

  uint32_t sum = 0;        // Sum of all pixel intensities
  uint8_t maxVal = 0;      // Brightest pixel found
  uint8_t minVal = 255;    // Darkest pixel found
  int pixelCount = 0;      // Number of pixels analyzed

  // Analyze either ROI (if available) or center region (during calibration)
  if (roi_count > 0)
  {
    // ROI-based analysis (ring pixels only)
    for (int i = 0; i < roi_count; i++)
    {
      uint8_t px = img[roi_idx[i]];
      sum += px;
      if (px > maxVal) {
        maxVal = px;
      }
      if (px < minVal) {
        minVal = px;
      }
    }
    pixelCount = roi_count;
  }
  else
  {
    // Fallback: analyze center region until ROI is calibrated
    // Center 48x48 region (from pixel 24 to 72 in both axes)
  
    for (int y = 24; y < 72; y++)
    {
      for (int x = 24; x < 72; x++)
      {
        uint8_t px = img[y * IMG_W + x];
        sum += px;
        if (px > maxVal) {
          maxVal = px;
        }
        if (px < minVal) {
          minVal = px;
        }
      }
    }
    pixelCount = 48 * 48;
  }

  float mean = 0.0;
  if (pixelCount > 0) {
    mean = (float)sum / pixelCount;
  }

  // =========================
  // BLOB DETECTION - bright_px-based edge detection with relaxed threshold
  // =========================
  // Step 1: Find bright_px pixel (brightest in ROI)
  uint8_t bright_px_intensity = 0;
  int bright_px_idx = -1;
  int bright_px_x = -1;
  int bright_px_y = -1;
  
  if (roi_count > 0)
  {
    for (int i = 0; i < roi_count; i++)
    {
      int idx = roi_idx[i];
      uint8_t px = img[idx];
      
      // Check if pixel is bright enough AND brighter than current max
      // Both conditions must be true to update the bright_px
      bool meets_min = (px >= BLOB_MIN_INTENSITY);
      bool is_brighter = (px > bright_px_intensity);
      
      if (meets_min && is_brighter)
      {
        bright_px_intensity = px;
        bright_px_idx = idx;
        // Convert 1D index back to 2D coordinates for distance calculation
        // x = index % width, y = index / width
        bright_px_x = idx % IMG_W;
        bright_px_y = idx / IMG_W;
      }
    }
  }

  // Step 2: Count edge pixels around bright_px with relaxed threshold
  int edge_pixels = 0;
  int edge_x = 0;
  int edge_y = 0;
  uint8_t max_edge_intensity = 0;

  if (bright_px_idx >= 0)
  {
    // Search only within reasonable distance from bright_px (limit to ~50 pixel radius to avoid false detections far away)
    int search_radius = 20;
    
    for (int i = 0; i < roi_count; i++)
    {
      int idx = roi_idx[i];
      int x = idx % IMG_W;
      int y = idx / IMG_W;
      uint8_t px = img[idx];

      if (px < BLOB_MIN_INTENSITY) {
        continue;
      }
      
      // Skip if too far from bright_px
      int dx = x - bright_px_x;
      int dy = y - bright_px_y;
      int distance_squared = dx * dx + dy * dy;
      int search_radius_squared = search_radius * search_radius;
      if (distance_squared > search_radius_squared) {
        continue;
      }

      // Check 4-connected neighbors for intensity jump
      // Array: up, down, right, left
      // Edges are detected where a bright pixel has a dark neighbor (intensity drop)
      int neighbor_offsets[4][2] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
      bool has_jump = false;

      for (int n = 0; n < 4; n++)
      {
        int offset_x = neighbor_offsets[n][0];
        int offset_y = neighbor_offsets[n][1];
        int nx = x + offset_x;
        int ny = y + offset_y;
        
        // Check if neighbor is within frame bounds
        bool x_in_bounds = (nx >= 0 && nx < IMG_W);
        bool y_in_bounds = (ny >= 0 && ny < IMG_H);
        if (!x_in_bounds || !y_in_bounds) {
          continue;
        }

        // Convert 2D neighbor coordinates back to 1D index
        int neighbor_idx = ny * IMG_W + nx;
        uint8_t neighbor_px = img[neighbor_idx];
        int intensity_diff = px - neighbor_px;
        
        // Found an edge: significant intensity drop to a neighbor
        if (intensity_diff >= BLOB_NEIGHBOR_DIFF) {
          has_jump = true;
          break;
        }
      }

      if (has_jump)
      {
        edge_pixels++;
        edge_x += x;
        edge_y += y;
        
        if (px > max_edge_intensity) {
          max_edge_intensity = px;
        }
      }
    }
  }

  bool blob_detected = false;
  int blob_center_x = -1;
  int blob_center_y = -1;

  // Calculate blob position only if minimum edge pixels found
  if (edge_pixels >= BLOB_MIN_EDGE_PIXELS) {
    blob_detected = true;
    
    // Calculate center of detected edges
    // Divide accumulated coordinates by number of edge pixels to get average position
    blob_center_x = edge_x / edge_pixels;
    blob_center_y = edge_y / edge_pixels;
  }


  if (frameId % PRINT_EVERY_FRAME == 0)
  {
    Serial.print("[FRAME ");
    Serial.print(frameId);
    Serial.print("] mean=");
    Serial.print(mean, 1);
    Serial.print(" min=");
    Serial.print(minVal);
    Serial.print(" max=");
    Serial.print(maxVal);
    Serial.print(" edge_px=");
    Serial.print(edge_pixels);
    if (blob_detected)
    {
      Serial.print(" center=(");
      Serial.print(blob_center_x);
      Serial.print(",");
      Serial.print(blob_center_y);
      Serial.print(") int=");
      Serial.print(max_edge_intensity);
    }
    Serial.println();
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Initializing Object Det");
  Serial.println("Step 1: Camera init...");
  if (!initCamera())
  {
    Serial.println("FATAL: Camera init failed. Stopping.");
    while (true) delay(100);
  }
  Serial.println("Camera OK");
  Serial.println("Step 2: ROI allocation...");
  buildROI();
  Serial.println("ROI OK");
  Serial.println("Setup complete - starting frame capture and IR detection");
}

int frameCounter = 0;
void loop()
{
  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb)
  {
    Serial.println("[CAM] frame capture failed");
    delay(100);
    return;
  }

  processFrame(fb, frameCounter);

  esp_camera_fb_return(fb);

  frameCounter++;

  delay(5); 
}
