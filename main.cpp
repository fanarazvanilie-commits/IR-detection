/*
 * Swarm Bots — Stage 1: Capture + ROI + Intensity + Blob Detection
 * Board: ESP32-S3 Sense (OV3660)
 * PURPOSE:
 *  - Capture grayscale frame
 *  - Extract Region Of Interest (ROI)
 *  - Compute intensity average for calibration
 *  - Detect IR signal blobs based on sudden intensity changes
 */

#include <Arduino.h>
#include "esp_camera.h"

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "camera_pins.h"

#include "s3capture.h"

void setup()
{
  s3capture::setupSerial();
  Serial.println("Initializing Object Det");

  Serial.println("Step 1: Camera init...");
  if (!s3capture::initCamera())
  {
    Serial.println("FATAL: Camera init failed. Stopping.");
    while (true)
    {
      delay(100);
    }
  }

  if (!s3capture::initSD())
  {
    Serial.println("FATAL: SD init failed. Stopping.");
    while (true)
    {
      delay(100);
    }
  }

  Serial.println("Camera OK");
  Serial.println("Step 2: ROI allocation...");
  s3capture::buildROI();
  Serial.println("ROI OK");

  Serial.println("Setup complete - starting frame capture and IR detection");
  // delay(3000);
}

static int frameCounter = 0;

void loop()
{
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("[CAM] frame capture failed");
    delay(100);
    return;
  }

  s3capture::processFrame(fb, frameCounter);
  s3capture::saveToSD(fb);
  esp_camera_fb_return(fb);
  frameCounter++;
  delay(s3capture::FRAME_DELAY_MS);
}
