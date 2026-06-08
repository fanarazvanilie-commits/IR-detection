#ifndef S3CAPTURE_H
#define S3CAPTURE_H
#include "esp_camera.h"

namespace s3capture {

static const int SERIAL_BAUD_RATE = 115200;

static const int IMAGE_WIDTH = 96;
static const int IMAGE_HEIGHT = 96;

static const int ROI_CENTER_X = IMAGE_WIDTH / 2;
static const int ROI_CENTER_Y = IMAGE_HEIGHT / 2 + 10;
static const int ROI_RADIUS = 22; // ~44 pixel diameter ring
static const int ROI_MARGIN = 7;
static const int ROI_INNER_RADIUS = ROI_RADIUS - ROI_MARGIN;
static const int ROI_OUTER_RADIUS = ROI_RADIUS + ROI_MARGIN - 4;
static const int MAX_ROI_PIXELS = 2000;

static const int CALIBRATION_REGION_HALF = 24;
static const int CALIBRATION_REGION_SIZE = CALIBRATION_REGION_HALF * 2;

static const int FRAME_DELAY_MS = 6.25; // normal runtime: 30ms capture => 33.3fps   ||    need 31.25ms frame-frame delay for 32fps

static const int CAMERA_XCLK_FREQ_HZ = 24000000;
static const camera_fb_location_t CAMERA_DEFAULT_FB_LOCATION = CAMERA_FB_IN_PSRAM;
static const int CAMERA_DEFAULT_JPEG_QUALITY = 12;
static const int CAMERA_PSRAM_JPEG_QUALITY = 15;
static const int CAMERA_PSRAM_FB_COUNT = 2;
static const int CAMERA_NO_PSRAM_FB_COUNT = 1;
static const pixformat_t CAMERA_GRAYSCALE_FORMAT = PIXFORMAT_GRAYSCALE;
static const framesize_t CAMERA_FRAME_SIZE = FRAMESIZE_96X96;

static const int PRINT_EVERY_FRAME = 1;
static const int ANGULAR_SECTIONS = 120;
static const float DEGREES_PER_SECTION = 360.0f / ANGULAR_SECTIONS;

static const float ANGULAR_PEAK_THRESHOLD = 1.5f;
static const int ANGULAR_PEAK_MIN_DISTANCE = 2;
static const float ANGULAR_PEAK_SIGMA = 1.5f;
static const float ANGULAR_PEAK_ABSOLUTE_WEIGHT = 0.35f;
static const int MAX_ANGULAR_PEAKS = 8;

static const int BLOB_MIN_INTENSITY = 2;
static const int BLOB_NEIGHBOR_DIFF = 30;
static const int BLOB_MIN_EDGE_PIXELS = 3;
static const int BLOB_SEARCH_RADIUS = 30;
static const int BLOB_CENTER_MARGIN = 4;

enum DecodeState {
    WAIT_FOR_START,
    READ_BITS,
    COOLDOWN
};
static DecodeState decodeState = WAIT_FOR_START;
static uint8_t decodedBits[3];
static int currentBit = 0;
static unsigned long cooldownStart = 0;
static const int BIT_COUNT = 3;
static const int COOLDOWN_MS = 100;

void setupSerial();
bool initCamera();
bool initSD();
void saveToSD(camera_fb_t *fb);
void buildROI();
void processFrame(camera_fb_t *fb, int frameId);

} // namespace s3capture
#endif // S3CAPTURE_H