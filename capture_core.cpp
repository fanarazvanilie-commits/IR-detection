#include "s3capture.h"
#include <Arduino.h>
#define CAMERA_MODEL_XIAO_ESP32S3
#include "camera_pins.h"
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <cmath>
#include <time.h>
#include <esp_heap_caps.h>
namespace s3capture {

static int roi_count = 0;
static int roi_indices[MAX_ROI_PIXELS];

unsigned long lastCaptureTime = 0;
int imageCount = 1;
bool camera_sign = false;
bool sd_sign = false;
const int SD_PIN_CS = 21;

static const int SD_BATCH_FRAME_COUNT = 30;
static const size_t SD_FRAME_BUFFER_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT;
static const size_t SD_CSV_BUFFER_SIZE = 4096;
static uint8_t *sdBatchBuffer = nullptr;
static char *sdCsvBatchBuffer = nullptr;
static size_t sdCsvBatchLengths[SD_BATCH_FRAME_COUNT] = {0};
static int sdBatchCount = 0;
static bool sdBatchActive = false;
static time_t sdBatchStartTime = 0;

static const int kNeighborOffsets[4][2] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};

static unsigned long sdLastSaveMs = 0;
static const unsigned long SD_IDLE_SAVE_INTERVAL_MS = 2000;
static bool sdInitialized = false;
static float previousAveragesForCsv[ANGULAR_SECTIONS] = {0.0f};
static bool hasPreviousAveragesForCsv = false;
static float previousAveragesForDetection[ANGULAR_SECTIONS] = {0.0f};
static bool hasPreviousAveragesForDetection = false;

static const int MAX_SESSIONS = 3;
static const int SESSION_ZONE_MARGIN = 2;
static const int SESSION_CAPTURE_MARGIN = 3;
static const int SESSION_START_FRAMES = 1;
static const int SESSION_CAPTURE_FRAMES = 6;
static const int SESSION_TIMEOUT_FRAMES = 8;

struct CaptureSession
{
  bool active = false;
  int centerSection = -1;
  int startFrames = 0;
  bool waitingForData = false;
  int captureDelay = 0;
  int captureFrames = 0;
  int lastActiveFrame = -1;
  uint8_t frameBits[SESSION_CAPTURE_FRAMES];
};

static CaptureSession sessions[MAX_SESSIONS];

static bool sectionsOverlap(int sectionA, int sectionB, int margin)
{
  int diff = abs(sectionA - sectionB);
  if (diff > ANGULAR_SECTIONS - diff)
  {
    diff = ANGULAR_SECTIONS - diff;
  }
  return diff <= margin;
}

static bool sectionsOverlap(int sectionA, int sectionB)
{
  return sectionsOverlap(sectionA, sectionB, SESSION_ZONE_MARGIN);
}

static int findSessionForSection(int section)
{
  for (int i = 0; i < MAX_SESSIONS; ++i)
  {
    if (!sessions[i].active)
    {
      continue;
    }
    if (sectionsOverlap(section, sessions[i].centerSection))
    {
      return i;
    }
  }
  return -1;
}

static bool saveGrayscalePgm(camera_fb_t *fb, const char *path)
{
  if (!sdInitialized)
  {
    Serial.println("SD not initialized");
    return false;
  }

  File file = SD.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.printf("SD open failed: %s\n", path);
    return false;
  }

  char header[32];
  int headerLen = snprintf(header, sizeof(header), "P5\n%u %u\n255\n", fb->width, fb->height);
  if (headerLen <= 0 || file.write((const uint8_t *)header, headerLen) != headerLen)
  {
    Serial.println("SD header write failed");
    file.close();
    return false;
  }

  if (file.write(fb->buf, fb->len) != fb->len)
  {
    Serial.println("SD image write failed");
    file.close();
    return false;
  }

  file.close();
  return true;
}

static int getAngularSection(int x, int y)
{
  int dx = x - ROI_CENTER_X;
  int dy = y - ROI_CENTER_Y;
  float angle = atan2f(dy, dx) * 180.0f / PI;
  if (angle < 0)
  {
    angle += 360.0f;
  }
  int section = (int)(angle / DEGREES_PER_SECTION);
  if (section >= ANGULAR_SECTIONS)
  {
    section = ANGULAR_SECTIONS - 1;
  }
  return section;
}
//We loop over all pixels that belong to the ROI. roi_indices[] stores the positions (as 1D indices) of those pixels.
static void computeAngularAverages(const uint8_t *img, float *averages, int numSections) {
  int counts[ANGULAR_SECTIONS] = {0};
  float sums[ANGULAR_SECTIONS] = {0.0f};
  for (int i = 0; i < roi_count; ++i)
  {
    int idx = roi_indices[i];
 // Convert 1D index into (x, y) coordinates
    int x = idx % IMAGE_WIDTH;
    int y = idx / IMAGE_WIDTH;

    int section = getAngularSection(x, y); // Determine which angular section this pixel belongs to
    sums[section] += img[idx];
    counts[section]++;
  }
  
   //fallback no pixels in section
  for (int s = 0; s < numSections; ++s)
  {
    if (counts[s] > 0){
      averages[s] = sums[s] / counts[s];}
    else{
      averages[s] = 0.0f; }
  }
}
// Subtract the previous frame's raw section averages from the current frame's raw section averages.
// The peak section and its neighbors preserve their absolute intensity to anchor the signal
// and prevent collapse to zero over time.
static void computeBaselineSubtractedAverages(const float *averages, float *deltaSubtracted, float *absoluteSubtracted, int numSections, float *previousAverages, bool &hasPreviousAverages)
{
  float minValue = averages[0];
  int peakSection = 0;
  float peakValue = averages[0];
  
  for (int s = 1; s < numSections; ++s)
  {
    if (averages[s] < minValue)
    {
      minValue = averages[s];
    }
    if (averages[s] > peakValue)
    {
      peakValue = averages[s];
      peakSection = s;
    }
  }

  for (int s = 0; s < numSections; ++s)
  {
    if (absoluteSubtracted)
    {
      absoluteSubtracted[s] = averages[s] - minValue;
    }
  }

  if (!hasPreviousAverages)
  {
    for (int s = 0; s < numSections; ++s)
    {
      deltaSubtracted[s] = 0.0f;
    }
    memcpy(previousAverages, averages, sizeof(float) * numSections);
    hasPreviousAverages = true;
    return;
  }

  for (int s = 0; s < numSections; ++s)
  {
    int diff = abs(s - peakSection);
    if (diff > numSections - diff)
    {
      diff = numSections - diff;
    }
    
    if (diff <= 2)
    {
      deltaSubtracted[s] = averages[s];
    }
    else
    {
      deltaSubtracted[s] = averages[s] - previousAverages[s];
      previousAverages[s] = averages[s];
    }
  }
}
static void makeSdFilename(char *filename, size_t size, bool active, time_t timestamp, int index, const char *extension = ".pgm")
{
  const char *prefix = active ? "/scan_" : "/idle_";
  static bool timeWarningShown = false;

  struct tm *tmInfo = localtime(&timestamp);
  bool hasValidTime = tmInfo && tmInfo->tm_year >= 100; // year 2000+ indicates real clock sync

  if (hasValidTime)
  {
    char datetime[32];
    if (strftime(datetime, sizeof(datetime), "%d-%b_%H-%M-%S", tmInfo) > 0)
    {
      snprintf(filename, size, "%s%s_%02d%s", prefix, datetime, index + 1, extension);
      return;
    }
  }

  if (!timeWarningShown)
  {
    Serial.println("WARNING: system time not initialized, using fallback filenames");
    timeWarningShown = true;
  }

  // Fall back to a simple counter when time is unavailable.
  snprintf(filename, size, "%s%06d_%02d%s", prefix, imageCount, index + 1, extension);
}

static bool saveGrayscalePgmBuffer(const uint8_t *buffer, size_t len, unsigned width, unsigned height, const char *path)
{
  if (!sdInitialized)
  {
    Serial.println("SD not initialized");
    return false;
  }

  File file = SD.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.printf("SD open failed: %s\n", path);
    return false;
  }

  char header[32];
  int headerLen = snprintf(header, sizeof(header), "P5\n%u %u\n255\n", width, height);
  if (headerLen <= 0 || file.write((const uint8_t *)header, headerLen) != headerLen)
  {
    Serial.println("SD header write failed");
    file.close();
    return false;
  }

  if (file.write(buffer, len) != len)
  {
    Serial.println("SD image write failed");
    file.close();
    return false;
  }

  file.close();
  return true;
}

static bool saveCsvBuffer(const char *buffer, size_t len, const char *path)
{
  if (!sdInitialized)
  {
    Serial.println("SD not initialized");
    return false;
  }

  File file = SD.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.printf("SD open failed: %s\n", path);
    return false;
  }

  if (file.write((const uint8_t *)buffer, len) != len)
  {
    Serial.println("SD CSV write failed");
    file.close();
    return false;
  }

  file.close();
  return true;
}

static size_t makeSectionAveragesCsv(const float *averages, int numSections, char *buffer, size_t bufferSize)
{
  size_t offset = 0;
  for (int s = 0; s < numSections; ++s)
  {
    int angle = static_cast<int>(s * DEGREES_PER_SECTION);
    int written = snprintf(buffer + offset, bufferSize - offset, "%ddeg,%.2f\n", angle, averages[s]);
    if (written < 0 || static_cast<size_t>(written) >= bufferSize - offset)
    {
      break;
    }
    offset += static_cast<size_t>(written);
  }
  return offset;
}

static size_t makeSectionDeltaCsv(const float *deltas, int numSections, char *buffer, size_t bufferSize)
{
  size_t offset = 0;
  for (int s = 0; s < numSections; ++s)
  {
    int angle = static_cast<int>(s * DEGREES_PER_SECTION);
    float outputValue = deltas[s] > 0.0f ? deltas[s] : 0.0f;
    int written = snprintf(buffer + offset, bufferSize - offset, "%ddeg,%.2f\n", angle, outputValue);
    if (written < 0 || static_cast<size_t>(written) >= bufferSize - offset)
    {
      break;
    }
    offset += static_cast<size_t>(written);
  }
  return offset;
}

static bool initSdBatchBuffer()
{
  if (sdBatchBuffer && sdCsvBatchBuffer)
  {
    return true;
  }

  if (!sdBatchBuffer)
  {
    sdBatchBuffer = (uint8_t *)heap_caps_malloc(SD_BATCH_FRAME_COUNT * SD_FRAME_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!sdBatchBuffer)
    {
      Serial.println("Failed to allocate PSRAM batch buffer");
      return false;
    }
  }

  if (!sdCsvBatchBuffer)
  {
    sdCsvBatchBuffer = (char *)heap_caps_malloc(SD_BATCH_FRAME_COUNT * SD_CSV_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!sdCsvBatchBuffer)
    {
      Serial.println("Failed to allocate PSRAM CSV buffer");
      return false;
    }
  }

  sdBatchCount = 0;
  sdBatchActive = false;
  sdBatchStartTime = 0;
  return true;
}

static void flushSdBatch()
{
  if (sdBatchCount <= 0)
  {
    return;
  }

  if (!sdInitialized && !initSD())
  {
    Serial.println("SD flush failed because SD is not initialized");
    sdBatchCount = 0;
    return;
  }

  Serial.printf("Flushing %d PSRAM buffered frames to SD (active=%d)\n", sdBatchCount, sdBatchActive ? 1 : 0);
  for (int i = 0; i < sdBatchCount; ++i)
  {
    char imageFilename[48];
    char csvFilename[48];
    makeSdFilename(imageFilename, sizeof(imageFilename), sdBatchActive, sdBatchStartTime, i, ".pgm");
    makeSdFilename(csvFilename, sizeof(csvFilename), sdBatchActive, sdBatchStartTime, i, ".csv");

    uint8_t *bufferPtr = sdBatchBuffer + (size_t)i * SD_FRAME_BUFFER_SIZE;
    bool imageSaved = saveGrayscalePgmBuffer(bufferPtr, SD_FRAME_BUFFER_SIZE, IMAGE_WIDTH, IMAGE_HEIGHT, imageFilename);
    bool csvSaved = saveCsvBuffer(sdCsvBatchBuffer + (size_t)i * SD_CSV_BUFFER_SIZE, sdCsvBatchLengths[i], csvFilename);

    if (imageSaved && csvSaved)
    {
      Serial.printf("PSRAM frame %d saved to SD as %s + %s\n", i + 1, imageFilename, csvFilename);
      imageCount++;
    }
    else
    {
      Serial.printf("PSRAM frame %d failed to save to SD as %s or %s\n", i + 1, imageFilename, csvFilename);
    }
  }

  sdBatchCount = 0;
}

static bool bufferFrameToPsram(camera_fb_t *fb, const char *csv, size_t csvLen, bool active)
{
  if (!fb || !fb->buf || fb->len != SD_FRAME_BUFFER_SIZE)
  {
    Serial.println("Invalid frame for PSRAM batch buffer");
    return false;
  }

  if (!initSdBatchBuffer())
  {
    return false;
  }

  if (sdBatchCount == 0)
  {
    sdBatchActive = active;
    sdBatchStartTime = time(NULL);
  }

  if (sdBatchCount < SD_BATCH_FRAME_COUNT)
  {
    memcpy(sdBatchBuffer + (size_t)sdBatchCount * SD_FRAME_BUFFER_SIZE, fb->buf, SD_FRAME_BUFFER_SIZE);
    if (csvLen > SD_CSV_BUFFER_SIZE)
    {
      csvLen = SD_CSV_BUFFER_SIZE;
    }
    memcpy(sdCsvBatchBuffer + (size_t)sdBatchCount * SD_CSV_BUFFER_SIZE, csv, csvLen);
    sdCsvBatchLengths[sdBatchCount] = csvLen;
    Serial.printf("Buffered frame %d/%d to PSRAM (active=%d)\n", sdBatchCount + 1, SD_BATCH_FRAME_COUNT, active ? 1 : 0);
    sdBatchCount++;
  }

  if (sdBatchCount >= SD_BATCH_FRAME_COUNT)
  {
    flushSdBatch();
  }

  return true;
}

bool initSD()
{
  if (sdInitialized)
  {
    return true;
  }
  //  SPI.setFrequency(10000000); // MHz 

   if (!SD.begin(SD_PIN_CS)) {
    Serial.println("Card Mount Failed");
    return false;
}

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    return false;
  }

   Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  sdInitialized = true;
  return true;
}

bool isSessionActive()
{
  for (int i = 0; i < MAX_SESSIONS; ++i)
  {
    if (sessions[i].active)
    {
      return true;
    }
  }
  return false;
}

static char csvTemp[SD_CSV_BUFFER_SIZE];

void saveToSD(camera_fb_t *fb)
{
  bool active = isSessionActive();
  unsigned long now = millis();
  if (!active && (now - lastCaptureTime) < SD_IDLE_SAVE_INTERVAL_MS)
  {
    return;
  }

  float angularAverages[ANGULAR_SECTIONS] = {0.0f};
  float deltaSubtracted[ANGULAR_SECTIONS] = {0.0f};

  if (roi_count > 0)
  {
    float absoluteSubtracted[ANGULAR_SECTIONS] = {0.0f};
    computeAngularAverages(fb->buf, angularAverages, ANGULAR_SECTIONS);
    computeBaselineSubtractedAverages(angularAverages, deltaSubtracted, absoluteSubtracted, ANGULAR_SECTIONS, previousAveragesForCsv, hasPreviousAveragesForCsv);
  }

  size_t csvLen = makeSectionDeltaCsv(deltaSubtracted, ANGULAR_SECTIONS, csvTemp, SD_CSV_BUFFER_SIZE);

  if (!bufferFrameToPsram(fb, csvTemp, csvLen, active))
  {
    Serial.println("Buffering frame to PSRAM failed, trying direct SD save");
    if (!sdInitialized && !initSD())
    {
      return;
    }

    char imageFilename[48];
    char csvFilename[48];
    time_t timestamp = time(NULL);
    makeSdFilename(imageFilename, sizeof(imageFilename), active, timestamp, 0, ".pgm");
    makeSdFilename(csvFilename, sizeof(csvFilename), active, timestamp, 0, ".csv");

    if (saveGrayscalePgmBuffer(fb->buf, fb->len, fb->width, fb->height, imageFilename) &&
        saveCsvBuffer(csvTemp, csvLen, csvFilename))
    {
      lastCaptureTime = now;
      imageCount++;
      Serial.printf("SD saved %s and %s\n", imageFilename, csvFilename);
    }
    else
    {
      Serial.printf("SD failed to save %s or %s\n", imageFilename, csvFilename);
    }
    return;
  }

  lastCaptureTime = now;
}

static int allocateSession(int section, int frameId)
{
  for (int i = 0; i < MAX_SESSIONS; ++i)
  {
    if (!sessions[i].active)
    {
      sessions[i].active = true;
      sessions[i].centerSection = section;
      sessions[i].startFrames = 1;
      sessions[i].waitingForData = false;
      sessions[i].captureDelay = 0;
      sessions[i].captureFrames = 0;
      sessions[i].lastActiveFrame = frameId;
      for (int j = 0; j < SESSION_CAPTURE_FRAMES; ++j)
      {
        sessions[i].frameBits[j] = 0;
      }
      Serial.print("[RX] NEW session ");
      Serial.print(i);
      Serial.print(" section ");
      Serial.println(section);
      return i;
    }
  }
  return -1;
}

static void resetSession(int index)
{
  sessions[index].active = false;
  sessions[index].centerSection = -1;
  sessions[index].startFrames = 0;
  sessions[index].waitingForData = false;
  sessions[index].captureDelay = 0;
  sessions[index].captureFrames = 0;
  sessions[index].lastActiveFrame = -1;
  for (int j = 0; j < SESSION_CAPTURE_FRAMES; ++j)
  {
    sessions[index].frameBits[j] = 0;
  }
}

static void decodeSession(int index)
{
  uint8_t bits[SESSION_CAPTURE_FRAMES / 2];
  Serial.print("[RX] RAW BITS=");
  for (int i = 0; i < SESSION_CAPTURE_FRAMES; ++i)
  {
    Serial.print(sessions[index].frameBits[i]);
  }
  Serial.print(" -> ");
  for (int i = 0; i < SESSION_CAPTURE_FRAMES; i += 2)
  {
    // Use the later sample in each pair to represent the oversampled logical bit.
    bits[i / 2] = sessions[index].frameBits[i + 1];
    Serial.print(bits[i / 2]);
  }
  Serial.print(" (frames: ");
  for (int i = 0; i < SESSION_CAPTURE_FRAMES; ++i)
  {
    Serial.printf("%s%d", i == 0 ? "" : ",", sessions[index].frameBits[i]);
  }
  Serial.println(")");

  uint8_t objectId = (bits[0] << 2) | (bits[1] << 1) | bits[2];
  Serial.print(" [RX] SESSION ");
  Serial.print(index);
  Serial.print(" ID=");
  Serial.print(objectId);
  Serial.print(" section=");
  Serial.println(sessions[index].centerSection);
  resetSession(index);
}

int toIndex(int x, int y)
{
  return y * IMAGE_WIDTH + x;
}

int minInt(int a, int b)
{
  if (a < b)
  {
    return a;
  }
  else
  {
    return b;
  }
}

int maxInt(int a, int b)
{
  if (a > b)
  {
    return a;
  }
  else
  {
    return b;
  }
}

static void analyzeCalibrationRegion(const uint8_t *img, uint32_t &sum, uint8_t &minVal, uint8_t &maxVal, int &pixelCount)
{
  for (int y = CALIBRATION_REGION_HALF; y < CALIBRATION_REGION_HALF + CALIBRATION_REGION_SIZE; y++)
  {
    for (int x = CALIBRATION_REGION_HALF; x < CALIBRATION_REGION_HALF + CALIBRATION_REGION_SIZE; x++)
    {
      uint8_t px = img[toIndex(x, y)];
      sum += px;
      minVal = (uint8_t)minInt(minVal, px);
      maxVal = (uint8_t)maxInt(maxVal, px);
    }
  }

  pixelCount = CALIBRATION_REGION_SIZE * CALIBRATION_REGION_SIZE;
}

static void analyzeROI(const uint8_t *img, uint32_t &sum, uint8_t &minVal, uint8_t &maxVal, int &pixelCount)
{
  for (int i = 0; i < roi_count; ++i)
  {
    uint8_t px = img[roi_indices[i]];
    sum += px;
    minVal = (uint8_t)minInt(minVal, px);
    maxVal = (uint8_t)maxInt(maxVal, px);
  }

  pixelCount = roi_count;
}

static bool findBrightestPixelInROI(const uint8_t *img, uint8_t &outIntensity, int &outIndex, int &outX, int &outY)
{
  for (int i = 0; i < roi_count; ++i)
  {
    int idx = roi_indices[i];
    uint8_t px = img[idx];
    if (px >= BLOB_MIN_INTENSITY && px > outIntensity){
      outIntensity = px;
      outIndex = idx;
      outX = idx % IMAGE_WIDTH;
      outY = idx / IMAGE_WIDTH; }
  }

  return outIndex >= 0;
}





static void smoothAngularSignal(const float *src, float *dst, int numSections, float sigma)
{
  int radius = maxInt(1, (int)(sigma * 3.0f));
  int kernelSize = 2 * radius + 1;
  float kernel[25];
  float norm = 0.0f;

  for (int i = 0; i < kernelSize; ++i)
  {
    float x = (float)(i - radius);
    float w = expf(-0.5f * (x * x) / (sigma * sigma));
    kernel[i] = w;
    norm += w;
  }

  for (int s = 0; s < numSections; ++s)
  {
    float sum = 0.0f;
    for (int i = 0; i < kernelSize; ++i)
    {
      int idx = (s + i - radius + numSections) % numSections;
      sum += kernel[i] * src[idx];
    }
    dst[s] = sum / norm;
  }
}

static void combineAngularSignals(const float *deltaSubtracted, const float *absoluteSubtracted, float *combined, int numSections)
{
  for (int s = 0; s < numSections; ++s)
  {
    float delta = deltaSubtracted[s];
    combined[s] = delta + ANGULAR_PEAK_ABSOLUTE_WEIGHT * absoluteSubtracted[s];
  }
}

static bool isSectionInWindow(int section, int center, int margin, int numSections)
{
  int diff = abs(section - center);
  if (diff > numSections - diff)
  {
    diff = numSections - diff;
  }
  return diff <= margin;
}

static int findAngularPeaks(const float *subtracted, int numSections, int *peakSections, int maxPeaks)
{
  int peaks = 0;
  for (int s = 0; s < numSections; ++s){
    float value = subtracted[s];
    if (value < ANGULAR_PEAK_THRESHOLD){    // Ignore small values (below threshold)
    continue; }


   /*Check if this point is a LOCAL MAXIMUM.
    A local maximum means:- This value is bigger than its neighbors
    - Not just immediate neighbors, but also within ANGULAR_PEAK_MIN_DISTANCE distance
    */
    bool isLocalMax = true;
    for (int d = 1; d <= ANGULAR_PEAK_MIN_DISTANCE; ++d)
    {
      int left = (s - d + numSections) % numSections;
      int right = (s + d) % numSections;
      if (value <= subtracted[left] || value <= subtracted[right]){
        isLocalMax = false;
        break; }
    }

    if (!isLocalMax){ // skip if not lcaol maximum
      continue; }

    if (peaks < maxPeaks){
      peakSections[peaks++] = s; }
  }

  return peaks;
}

static int countEdgePixels(const uint8_t *img, int brightX, int brightY, uint8_t &outMaxEdgeIntensity, int &outEdgeSumX, int &outEdgeSumY)
{
  int edgePixels = 0;
  const int searchRadiusSquared = BLOB_SEARCH_RADIUS * BLOB_SEARCH_RADIUS;

  for (int i = 0; i < roi_count; ++i)
  {
    int idx = roi_indices[i];
    int x = idx % IMAGE_WIDTH;
    int y = idx / IMAGE_WIDTH;
    uint8_t px = img[idx];

    if (px < BLOB_MIN_INTENSITY)
    {
      continue;
    }

    int dx = x - brightX;
    int dy = y - brightY;
    if ((dx * dx + dy * dy) > searchRadiusSquared)
    {
      continue;
    }

    bool hasIntensityJump = false;
    for (int o = 0; o < 4; ++o)
    {
      int nx = x + kNeighborOffsets[o][0];
      int ny = y + kNeighborOffsets[o][1];
      if (nx < 0 || nx >= IMAGE_WIDTH || ny < 0 || ny >= IMAGE_HEIGHT)
      {
        continue;
      }

      int neighborIndex = toIndex(nx, ny);
      uint8_t neighborPx = img[neighborIndex];
      if ((px - neighborPx) >= BLOB_NEIGHBOR_DIFF)
      {
        hasIntensityJump = true;
        break;
      }
    }

    if (hasIntensityJump)
    {
      ++edgePixels;
      outEdgeSumX += x;
      outEdgeSumY += y;
      outMaxEdgeIntensity = (uint8_t)maxInt(outMaxEdgeIntensity, px);
    }
  }

  return edgePixels;
}

static void logFrame(int frameId, float mean, uint8_t minVal, uint8_t maxVal, int edgePixels, bool blobDetected, int blobCenterX, int blobCenterY, uint8_t maxEdgeIntensity, int angularPeaks, const int *peakSections, float blobAngle)
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
  Serial.print(edgePixels);

  if (blobDetected)
  {
    Serial.print(" center=(");
    Serial.print(blobCenterX);
    Serial.print(",");
    Serial.print(blobCenterY);
    Serial.print(") int=");
    Serial.print(maxEdgeIntensity);
    Serial.print(" angle=");
    Serial.print(blobAngle, 1);
    Serial.print(" section=");
    Serial.print((int)(blobAngle / DEGREES_PER_SECTION));
  }

  Serial.print(" angular_peaks=");
  Serial.print(angularPeaks);

  if (angularPeaks > 0)
  {
    Serial.print(" peak_sections=");
    for (int i = 0; i < angularPeaks; ++i)
    {
      Serial.print(peakSections[i]);
      if (i < angularPeaks - 1)
      {
        Serial.print(",");
      }
    }
  }

  Serial.println();
}

void setupSerial()
{
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setDebugOutput(true);
}

bool initCamera()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

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

  config.xclk_freq_hz = CAMERA_XCLK_FREQ_HZ;
  config.pixel_format = CAMERA_GRAYSCALE_FORMAT;
  config.frame_size = CAMERA_FRAME_SIZE;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_DEFAULT_FB_LOCATION;
  config.jpeg_quality = CAMERA_DEFAULT_JPEG_QUALITY;
  config.fb_count = CAMERA_NO_PSRAM_FB_COUNT;

  if (psramFound())
  {
    config.jpeg_quality = CAMERA_PSRAM_JPEG_QUALITY;
    config.fb_count = CAMERA_PSRAM_FB_COUNT;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }
  else
  {
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("[CAM] init failed: 0x%x\n", err);
    return false;
  }

  camera_sign = true;
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != nullptr && sensor->id.PID == OV3660_PID)
  {
    sensor->set_vflip(sensor, 1);
    sensor->set_brightness(sensor, -1);
    sensor->set_ae_level(sensor, -2);
    sensor->set_contrast(sensor, 2);
    sensor->set_saturation(sensor, -2);
    sensor->set_sharpness(sensor, -2);
    sensor->set_denoise(sensor, 1);
    sensor->set_awb_gain(sensor, 0);
    sensor->set_exposure_ctrl(sensor, 0);
    sensor->set_gain_ctrl(sensor, 0);
    sensor->set_aec2(sensor, 0);
    sensor->set_aec_value(sensor, 1000);
    sensor->set_agc_gain(sensor, 20);
    sensor->set_raw_gma(sensor, 0);
    sensor->set_lenc(sensor, 1);

  }

  Serial.println("[CAM] init OK");
  return true;
}

void buildROI()
{
  static_assert(ROI_INNER_RADIUS >= 0, "ROI inner radius must be non-negative");

  roi_count = 0;
  const int rmin2 = ROI_INNER_RADIUS * ROI_INNER_RADIUS;
  const int rmax2 = ROI_OUTER_RADIUS * ROI_OUTER_RADIUS;

  for (int y = 0; y < IMAGE_HEIGHT; ++y)
  {
    for (int x = 0; x < IMAGE_WIDTH; ++x)
    {
      int dx = x - ROI_CENTER_X;
      int dy = y - ROI_CENTER_Y;
      int distanceSquared = dx * dx + dy * dy;
      if (distanceSquared < rmin2 || distanceSquared > rmax2)
      {
        continue;
      }

      if (roi_count < static_cast<int>(MAX_ROI_PIXELS))
      {
        roi_indices[roi_count++] = toIndex(x, y);
      }
    }
  }

  Serial.print("[ROI] pixels: ");
  Serial.print(roi_count);

  int roi_min_x = IMAGE_WIDTH;
  int roi_max_x = 0;
  int roi_min_y = IMAGE_HEIGHT;
  int roi_max_y = 0;

  for (int i = 0; i < roi_count; ++i)
  {
    int index = roi_indices[i];
    int x = index % IMAGE_WIDTH;
    int y = index / IMAGE_WIDTH;
    roi_min_x = minInt(roi_min_x, x);
    roi_max_x = maxInt(roi_max_x, x);
    roi_min_y = minInt(roi_min_y, y);
    roi_max_y = maxInt(roi_max_y, y);
  }

  Serial.print(" x=[");
  Serial.print(roi_min_x);
  Serial.print("-");
  Serial.print(roi_max_x);
  Serial.print("] y=[");
  Serial.print(roi_min_y);
  Serial.print("-");
  Serial.print(roi_max_y);
  Serial.println("]");
}

void splitROIAngularSections(int *pixelSections)
{
  for (int i = 0; i < roi_count; ++i)
  {
    int idx = roi_indices[i];
    int x = idx % IMAGE_WIDTH;
    int y = idx / IMAGE_WIDTH;
    int dx = x - ROI_CENTER_X;
    int dy = y - ROI_CENTER_Y;
    float angle = atan2f(dy, dx) * 180.0f / PI;
    if (angle < 0)
    {
      angle += 360.0f;
    }
    int section = (int)(angle / DEGREES_PER_SECTION);
    if (section >= ANGULAR_SECTIONS)
    {
      section = ANGULAR_SECTIONS - 1;
    }
    pixelSections[i] = section;
  }
}



void processFrame(camera_fb_t *fb, int frameId)
{
  const uint8_t *img = fb->buf;
  uint32_t sum = 0;
  uint8_t maxVal = 0;
  uint8_t minVal = 255;
  int pixelCount = 0;

  if (roi_count > 0)
  {
    analyzeROI(img, sum, minVal, maxVal, pixelCount);
  }
  else
  {
    analyzeCalibrationRegion(img, sum, minVal, maxVal, pixelCount);
  }

  float mean = 0.0f;
  if (pixelCount > 0)
  {
    mean = ((float)sum) / pixelCount;
  }

  uint8_t brightIntensity = 0;
  int brightIndex = -1;
  int brightX = -1;
  int brightY = -1;
  if (roi_count > 0)
  {
    findBrightestPixelInROI(img, brightIntensity, brightIndex, brightX, brightY);
  }

  int edgePixels = 0;
  int edgeSumX = 0;
  int edgeSumY = 0;
  uint8_t maxEdgeIntensity = 0;
  if (brightIndex >= 0)
  {
    edgePixels = countEdgePixels(img, brightX, brightY, maxEdgeIntensity, edgeSumX, edgeSumY);
  }
  bool blobDetected = edgePixels >= BLOB_MIN_EDGE_PIXELS;
  int blobCenterX = -1;
  int blobCenterY = -1;
  if (blobDetected)
  {
    blobCenterX = edgeSumX / edgePixels;
    blobCenterY = edgeSumY / edgePixels;
  }

  float angularAverages[ANGULAR_SECTIONS];
  float deltaSubtracted[ANGULAR_SECTIONS];
  float absoluteSubtracted[ANGULAR_SECTIONS];
  float combinedSignal[ANGULAR_SECTIONS];
  float smoothedSignal[ANGULAR_SECTIONS];
  int peakSections[MAX_ANGULAR_PEAKS];
  int peakValues[MAX_ANGULAR_PEAKS];
  int peakCount = 0;

  if (roi_count > 0)
  {
    computeAngularAverages(img, angularAverages, ANGULAR_SECTIONS);
    computeBaselineSubtractedAverages(angularAverages, deltaSubtracted, absoluteSubtracted, ANGULAR_SECTIONS, previousAveragesForDetection, hasPreviousAveragesForDetection);
    combineAngularSignals(deltaSubtracted, absoluteSubtracted, combinedSignal, ANGULAR_SECTIONS);
    smoothAngularSignal(combinedSignal, smoothedSignal, ANGULAR_SECTIONS, ANGULAR_PEAK_SIGMA);
    peakCount = findAngularPeaks(smoothedSignal, ANGULAR_SECTIONS, peakSections, MAX_ANGULAR_PEAKS);
    if (peakCount == 0)
    {
      peakCount = findAngularPeaks(combinedSignal, ANGULAR_SECTIONS, peakSections, MAX_ANGULAR_PEAKS);
    }
    for (int i = 0; i < peakCount; ++i)
    {
      float deltaValue = deltaSubtracted[peakSections[i]];
      if (deltaValue > 0.0f)
      {
        peakValues[i] = 1;
      }
      else if (deltaValue < 0.0f)
      {
        peakValues[i] = 0;
      }
      else
      {
        peakValues[i] = absoluteSubtracted[peakSections[i]] > ANGULAR_PEAK_THRESHOLD ? 1 : 0;
      }
    }
  }

  float blobAngle = -1.0f;
  if (blobDetected)
  {
    blobAngle = atan2f(blobCenterY - ROI_CENTER_Y, blobCenterX - ROI_CENTER_X) * 180.0f / PI;
    if (blobAngle < 0)
    {
      blobAngle += 360.0f;
    }
  }

  unsigned long now = millis();
  bool peakHandled[MAX_ANGULAR_PEAKS] = {false};

  for (int s = 0; s < MAX_SESSIONS; ++s)
  {
    if (!sessions[s].active)
    {
      continue;
    }

    bool areaSignal = false;
    int overlapMargin = sessions[s].waitingForData ? SESSION_CAPTURE_MARGIN : SESSION_ZONE_MARGIN;
    int signalBitValue = 0;
    for (int i = 0; i < peakCount; ++i)
    {
      if (peakHandled[i])
      {
        continue;
      }
      if (sectionsOverlap(peakSections[i], sessions[s].centerSection, overlapMargin))
      {
        areaSignal = true;
        signalBitValue = peakValues[i];
        sessions[s].centerSection = peakSections[i];
        peakHandled[i] = true;
        break;
      }
    }

    if (sessions[s].waitingForData)
    {
      if (sessions[s].captureDelay > 0)
      {
        sessions[s].captureDelay--;
      }
      else if (sessions[s].captureFrames < SESSION_CAPTURE_FRAMES)
      {
        int bitIndex = sessions[s].captureFrames;
        int bitValue = areaSignal ? signalBitValue : 0;
        sessions[s].frameBits[bitIndex] = bitValue;
        Serial.printf("[RX] session %d capture frame %d/%d id=%d areaSignal=%d bit=%d peakCount=%d center=%d bits=", s, bitIndex + 1, SESSION_CAPTURE_FRAMES, frameId, areaSignal ? 1 : 0, bitValue, peakCount, sessions[s].centerSection);
        for (int j = 0; j <= bitIndex; ++j)
        {
          Serial.print(sessions[s].frameBits[j]);
        }
        Serial.println();
        sessions[s].captureFrames++;
      }

      if (areaSignal)
      {
        sessions[s].lastActiveFrame = frameId;
      }

      if (sessions[s].captureFrames >= SESSION_CAPTURE_FRAMES)
      {
        decodeSession(s);
        continue;
      }

      if ((frameId - sessions[s].lastActiveFrame) > SESSION_TIMEOUT_FRAMES)
      {
        resetSession(s);
      }
    }
    else
    {
      if (areaSignal)
      {
        sessions[s].startFrames++;
        sessions[s].lastActiveFrame = frameId;

        if (sessions[s].startFrames >= SESSION_START_FRAMES)
        {
          sessions[s].waitingForData = true;
          sessions[s].captureDelay = 0; // skip this locking frame and start capture on the next one
          sessions[s].captureFrames = 0;
          Serial.print("[RX] LOCK session ");
          Serial.print(s);
          Serial.print(" area section ");
          Serial.print(sessions[s].centerSection);
          Serial.println(" and start capture");
        }
      }
      else if ((frameId - sessions[s].lastActiveFrame) > SESSION_TIMEOUT_FRAMES)
      {
        resetSession(s);
      }
    }
  }

  for (int i = 0; i < peakCount; ++i)
  {
    if (peakHandled[i])
    {
      continue;
    }

    bool overlaps = false;
    for (int s = 0; s < MAX_SESSIONS; ++s)
    {
      if (!sessions[s].active)
      {
        continue;
      }
      int overlapMargin = sessions[s].waitingForData ? SESSION_CAPTURE_MARGIN : SESSION_ZONE_MARGIN;
      if (sectionsOverlap(peakSections[i], sessions[s].centerSection, overlapMargin))
      {
        overlaps = true;
        break;
      }
    }

    if (!overlaps)
    {
      allocateSession(peakSections[i], frameId);
    }
  }
  if ((frameId % PRINT_EVERY_FRAME) == 0)
  {
    logFrame(frameId, mean, minVal, maxVal, edgePixels, blobDetected, blobCenterX, blobCenterY, maxEdgeIntensity, peakCount, peakSections, blobAngle);
  }
}

} 
