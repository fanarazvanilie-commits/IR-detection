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
static int roi_sections[MAX_ROI_PIXELS];

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
static uint8_t sectionStates[ANGULAR_SECTIONS] = {0};

static const int MAX_SESSIONS = 8;
static const int SESSION_ZONE_MARGIN = 2;
static const int SESSION_CAPTURE_MARGIN = 3;
static const int SESSION_START_FRAMES = 2;
static const int SESSION_CAPTURE_FRAMES = 6;
static const int SESSION_TIMEOUT_FRAMES = 8;
static const int SESSION_MAX_LIFETIME_FRAMES = 8;
static const int SESSION_START_SKIP_FRAMES = 1;

// Allocation / validation tuning to reduce false positives
static const float GAUSS_NORMERR_TIGHT = 0.06f; // strict threshold for very small blobs
static const float GAUSS_NORMERR_LOOSE = 0.13f; // looser threshold for wider blobs
static const float MIN_PEAK_FOR_ALLOC = 0.0f;   // minimum smoothed peak to consider allocating a session
static const float GAUSS_PEAK_RATIO_MIN = 1.8f;  // require the center peak to stand out above the blob average
static const float DETECTION_THRESHOLD_MIN = 0.6f;
static const float DETECTION_THRESHOLD_SCALE = 0.45f;
static const float SESSION_START_RISING_THRESHOLD_MIN = 5.0f;
static const float SESSION_START_RISING_THRESHOLD_SCALE = 0.5f;
static const int MIN_BLOB_SIZE_FOR_LOOSE = 3;   // blob size >= this may use loose normErr



struct CaptureSession
{
  bool active = false;
  int centerSection = -1;
  int startFrames = 0;
  bool waitingForData = false;
  int captureDelay = 0;
  int captureFrames = 0;
  int lastActiveFrame = -1;
  int allocatedFrame = -1;
  int currentBit = 0;
  int pendingTransitionCount = 0;
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

//We loop over all pixels that belong to the ROI. roi_indices[] stores the positions (as 1D indices) of those pixels.
static void computeAngularAverages(const uint8_t *img, float *averages, int numSections) {
  int counts[ANGULAR_SECTIONS] = {0};
  float sums[ANGULAR_SECTIONS] = {0.0f};
  for (int i = 0; i < roi_count; ++i)
  {
    int idx = roi_indices[i];
    int section = roi_sections[i];
    sums[section] += img[idx];
    counts[section]++;
  }

  for (int s = 0; s < numSections; ++s)
  {
    if (counts[s] > 0)
    {
      averages[s] = sums[s] / counts[s];
    }
    else
    {
      averages[s] = 0.0f;
    }
  }
}
// Subtract the previous frame's section averages from the current frame's section averages.
// This removes static reflections and keeps only frame-to-frame intensity changes.
static void computeBaselineSubtractedAverages(const float *averages, float *deltaSubtracted, 
  int numSections, float *previousAverages, bool &hasPreviousAverages)
{
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
    deltaSubtracted[s] = averages[s] - previousAverages[s];
    previousAverages[s] = averages[s];
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
    int written = snprintf(buffer + offset, bufferSize - offset, "%ddeg,%.2f\n", angle, deltas[s]);
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
  if (!active && (millis() - lastCaptureTime) < SD_IDLE_SAVE_INTERVAL_MS)
  {
    return;
  }

  float angularAverages[ANGULAR_SECTIONS] = {0.0f};
  float deltaSubtracted[ANGULAR_SECTIONS] = {0.0f};
  float smoothed[ANGULAR_SECTIONS] = {0.0f};

  if (roi_count > 0)
  {
    computeAngularAverages(fb->buf, angularAverages, ANGULAR_SECTIONS);
    computeBaselineSubtractedAverages(angularAverages, deltaSubtracted, ANGULAR_SECTIONS, previousAveragesForCsv, hasPreviousAveragesForCsv);
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
      lastCaptureTime = millis();
      imageCount++;
      Serial.printf("SD saved %s and %s\n", imageFilename, csvFilename);
    }
    else
    {
      Serial.printf("SD failed to save %s or %s\n", imageFilename, csvFilename);
    }
    return;
  }

  lastCaptureTime = millis();
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
      sessions[i].allocatedFrame = frameId;
      sessions[i].currentBit = 0;
      sessions[i].pendingTransitionCount = 0;
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
  sessions[index].allocatedFrame = -1;
  sessions[index].currentBit = 0;
  sessions[index].pendingTransitionCount = 0;
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




static void updateSectionStates(const float *deltas, int numSections, uint8_t *states, float threshold)
{
  for (int s = 0; s < numSections; ++s)
  {
    if (fabsf(deltas[s]) > threshold)
    {
      states[s] = 1;
    }
    else
    {
      states[s] = 0;
    }
  }
}

static int computeBlobCenter(int start, int end, int numSections)
{
  if (start <= end)
  {
    return (start + end) / 2;
  }

  int wrappedLength = (numSections - start) + (end + 1);
  int centerOffset = wrappedLength / 2;
  int center = start + centerOffset;
  if (center >= numSections)
  {
    center -= numSections;
  }
  return center;
}

static int findSignificantSectionBlobs(const float *deltas, int numSections, float threshold, int *blobStarts, int *blobEnds, int maxBlobs)
{
  bool active[ANGULAR_SECTIONS];
  for (int s = 0; s < numSections; ++s)
  {
    active[s] = fabsf(deltas[s]) > threshold;
  }

  int blobCount = 0;
  int start = -1;

  for (int s = 0; s < numSections; ++s)
  {
    if (active[s])
    {
      if (start < 0)
      {
        start = s;
      }
    }
    else if (start >= 0)
    {
      if (blobCount < maxBlobs)
      {
        blobStarts[blobCount] = start;
        blobEnds[blobCount] = s - 1;
      }
      blobCount++;
      start = -1;
    }
  }

  if (start >= 0)
  {
    if (blobCount < maxBlobs)
    {
      blobStarts[blobCount] = start;
      blobEnds[blobCount] = numSections - 1;
    }
    blobCount++;
  }

  if (blobCount > 1 && active[0] && active[numSections - 1])
  {
    int lastIndex = blobCount - 1;
    int firstStart = blobStarts[0];
    int firstEnd = blobEnds[0];
    int lastStart = blobStarts[lastIndex];
    int lastEnd = blobEnds[lastIndex];

    if (blobCount <= maxBlobs)
    {
      blobStarts[0] = lastStart;
      blobEnds[0] = firstEnd;
      for (int i = 1; i < lastIndex; ++i)
      {
        blobStarts[i] = blobStarts[i + 1];
        blobEnds[i] = blobEnds[i + 1];
      }
    }

    blobCount--;
  }

  return blobCount > maxBlobs ? maxBlobs : blobCount;
}

static bool validateBlobWithGaussianFit(const float *deltas, int start, int end, int numSections, int center)
{
  // Extract blob values and find peak
  float blobValues[ANGULAR_SECTIONS];
  int blobSize = 0;
  float peakValue = 0.0f;

  float signedPeak = 0.0f;
  float signalSum = 0.0f;
  float weightedAngleSum = 0.0f;
  float weightSum = 0.0f;

  if (start <= end)
  {
    for (int s = start; s <= end; ++s)
    {
      float value = deltas[s];
      blobValues[blobSize] = value;
      float absValue = fabsf(value);
      if (absValue > signedPeak)
      {
        signedPeak = absValue;
      }
      float angle = s * DEGREES_PER_SECTION;
      weightedAngleSum += angle * absValue;
      weightSum += absValue;
      blobSize++;
    }
  }
  else
  {
    for (int s = start; s < numSections; ++s)
    {
      float value = deltas[s];
      blobValues[blobSize] = value;
      float absValue = fabsf(value);
      if (absValue > signedPeak)
      {
        signedPeak = absValue;
      }
      float angle = s * DEGREES_PER_SECTION;
      weightedAngleSum += angle * absValue;
      weightSum += absValue;
      blobSize++;
    }
    for (int s = 0; s <= end; ++s)
    {
      float value = deltas[s];
      blobValues[blobSize] = value;
      float absValue = fabsf(value);
      if (absValue > signedPeak)
      {
        signedPeak = absValue;
      }
      float angle = s * DEGREES_PER_SECTION;
      weightedAngleSum += angle * absValue;
      weightSum += absValue;
      blobSize++;
    }
  }

  if (signedPeak <= 0.0f)
  {
    return false;
  }

  float averageAbs = (blobSize > 0) ? (signalSum / blobSize) : 0.0f;
  if (blobSize > 1 && signedPeak < averageAbs * GAUSS_PEAK_RATIO_MIN)
  {
    return false;
  }

  float centerAngle;
  if (weightSum > 0.0f)
  {
    centerAngle = fmodf(weightedAngleSum / weightSum, 360.0f);
    if (centerAngle < 0.0f)
    {
      centerAngle += 360.0f;
    }
  }
  else
  {
    centerAngle = center * DEGREES_PER_SECTION;
  }

  float amplitude = signedPeak;
  float variance = 0.0f;
  int centerIndexInBlob = (start <= end) ? (center - start) : ((center >= start) ? (center - start) : (numSections - start + center));

  for (int i = 0; i < blobSize; ++i)
  {
    float dx = i - centerIndexInBlob;
    variance += fabsf(blobValues[i]) * dx * dx;
  }
  variance /= (weightSum > 0.0f ? weightSum : 1.0f);
  float sigma = sqrtf(variance);
  if (sigma < 0.5f)
  {
    sigma = 0.5f;
  }

  float sumSquaredError = 0.0f;
  for (int i = 0; i < blobSize; ++i)
  {
    float dx = i - centerIndexInBlob;
    float fitted = amplitude * expf(-(dx * dx) / (2.0f * sigma * sigma));
    float error = fabsf(blobValues[i]) - fitted;
    sumSquaredError += error * error;
  }

  float normalizedError = sumSquaredError / (blobSize * amplitude * amplitude);
  Serial.printf("[GAUSS] blob start=%d end=%d size=%d peak=%.2f normErr=%.3f center=%d sigma=%.2f\n", start, end, blobSize, amplitude, normalizedError, center, sigma);

  if (amplitude < MIN_PEAK_FOR_ALLOC)
  {
    return false;
  }

  if (blobSize < MIN_BLOB_SIZE_FOR_LOOSE)
  {
    return normalizedError < GAUSS_NORMERR_TIGHT;
  }
  return normalizedError < GAUSS_NORMERR_LOOSE;
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




static void logFrame(int frameId, float mean, uint8_t minVal, uint8_t maxVal, int activeStateCount)
{
  Serial.print("[FRAME ");
  Serial.print(frameId);
  Serial.print("] mean=");
  Serial.print(mean, 1);
  Serial.print(" min=");
  Serial.print(minVal);
  Serial.print(" max=");
  Serial.print(maxVal);
  Serial.print(" activeStates=");
  Serial.print(activeStateCount);
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
        roi_indices[roi_count] = toIndex(x, y);
        float angle = atan2f(dy, dx) * 180.0f / PI;
        if (angle < 0)
        {
          angle += 360.0f;
        }
        int section = static_cast<int>(angle / DEGREES_PER_SECTION);
        if (section >= ANGULAR_SECTIONS)
        {
          section = ANGULAR_SECTIONS - 1;
        }
        roi_sections[roi_count] = section;
        roi_count++;
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

  float angularAverages[ANGULAR_SECTIONS] = {0.0f};
  float deltaSubtracted[ANGULAR_SECTIONS] = {0.0f};
  float smoothed[ANGULAR_SECTIONS] = {0.0f};
  float detectionThreshold = ANGULAR_PEAK_THRESHOLD;
  float maxDeltaSmoothed = 0.0f;
  float sessionStartThreshold = SESSION_START_RISING_THRESHOLD_MIN;
  if (roi_count > 0)
  {
    computeAngularAverages(img, angularAverages, ANGULAR_SECTIONS);
    computeBaselineSubtractedAverages(
      angularAverages,
      deltaSubtracted,
      ANGULAR_SECTIONS,
      previousAveragesForDetection,
      hasPreviousAveragesForDetection);

    // Small circular 3-point smoothing to reduce jagged noise while keeping peaks thin
    for (int s = 0; s < ANGULAR_SECTIONS; ++s)
    {
      int left = (s - 1 + ANGULAR_SECTIONS) % ANGULAR_SECTIONS;
      int right = (s + 1) % ANGULAR_SECTIONS;
      smoothed[s] = (deltaSubtracted[left] + 2.0f * deltaSubtracted[s] + deltaSubtracted[right]) * 0.25f;
    }

    float maxDeltaRaw = 0.0f;
    float absDeltaSum = 0.0f;
    int positiveSections = 0;
    for (int s = 0; s < ANGULAR_SECTIONS; ++s)
    {
      if (deltaSubtracted[s] > maxDeltaRaw) maxDeltaRaw = deltaSubtracted[s];
      if (smoothed[s] > maxDeltaSmoothed) maxDeltaSmoothed = smoothed[s];
      absDeltaSum += fabsf(smoothed[s]);
      if (smoothed[s] > DETECTION_THRESHOLD_MIN) positiveSections++;
    }

    detectionThreshold = max(DETECTION_THRESHOLD_MIN, maxDeltaSmoothed * DETECTION_THRESHOLD_SCALE);
    if (detectionThreshold < ANGULAR_PEAK_THRESHOLD)
    {
      detectionThreshold = ANGULAR_PEAK_THRESHOLD;
    }

    sessionStartThreshold = (maxDeltaSmoothed < SESSION_START_RISING_THRESHOLD_MIN)
        ? max(detectionThreshold * 2.0f, maxDeltaSmoothed * SESSION_START_RISING_THRESHOLD_SCALE)
        : SESSION_START_RISING_THRESHOLD_MIN;

    updateSectionStates(smoothed, ANGULAR_SECTIONS, sectionStates, detectionThreshold);

    int blobStarts[ANGULAR_SECTIONS];
    int blobEnds[ANGULAR_SECTIONS];
    int blobCount = findSignificantSectionBlobs(smoothed, ANGULAR_SECTIONS, detectionThreshold, blobStarts, blobEnds, ANGULAR_SECTIONS);

    Serial.printf("[DEBUG] frame %d maxRaw=%.2f maxSm=%.2f threshold=%.2f blobs=%d positiveSections=%d\n",
                  frameId, maxDeltaRaw, maxDeltaSmoothed, detectionThreshold, blobCount, positiveSections);

    if (blobCount > 0)
    {
      for (int b = 0; b < blobCount; ++b)
      {
        int centerSection = computeBlobCenter(blobStarts[b], blobEnds[b], ANGULAR_SECTIONS);
        if (findSessionForSection(centerSection) >= 0)
        {
          continue;
        }

        float centerValue = smoothed[centerSection];
        float centerPeak = fabsf(centerValue);
        if (centerValue <= sessionStartThreshold)
        {
          continue;
        }
        if (centerPeak < detectionThreshold || centerPeak < MIN_PEAK_FOR_ALLOC)
        {
          continue;
        }

        if (validateBlobWithGaussianFit(smoothed, blobStarts[b], blobEnds[b], ANGULAR_SECTIONS, centerSection))
        {
          allocateSession(centerSection, frameId);
        }
      }
    }
  }

  for (int s = 0; s < MAX_SESSIONS; ++s)
  {
    if (!sessions[s].active)
    {
      continue;
    }

    int centerSection = sessions[s].centerSection;
    if (centerSection < 0 || centerSection >= ANGULAR_SECTIONS)
    {
      resetSession(s);
      continue;
    }

    float centerDelta = smoothed[centerSection];
    int signalBitValue = (fabsf(centerDelta) > detectionThreshold) ? 1 : 0;
    const float bitDetectThreshold = max(DETECTION_THRESHOLD_MIN, maxDeltaSmoothed * 0.25f);

    if (sessions[s].waitingForData)
    {
      if (sessions[s].captureDelay > 0)
      {
        sessions[s].captureDelay--;
      }
      else if (sessions[s].captureFrames < SESSION_CAPTURE_FRAMES)
      {
        if (fabsf(centerDelta) > bitDetectThreshold)
        {
          int detectedBit = (centerDelta > 0.0f) ? 1 : 0;
          const float strongTransitionThreshold = max(bitDetectThreshold * 3.0f, SESSION_START_RISING_THRESHOLD_MIN);
          if (detectedBit != sessions[s].currentBit)
          {
            if (fabsf(centerDelta) >= strongTransitionThreshold)
            {
              sessions[s].currentBit = detectedBit;
              sessions[s].pendingTransitionCount = 0;
            }
            else
            {
              sessions[s].pendingTransitionCount++;
              if (sessions[s].pendingTransitionCount >= 2)
              {
                sessions[s].currentBit = detectedBit;
                sessions[s].pendingTransitionCount = 0;
              }
            }
          }
          else
          {
            sessions[s].pendingTransitionCount = 0;
          }
        }

        int captureBitValue = sessions[s].currentBit;
        int bitIndex = sessions[s].captureFrames;
        sessions[s].frameBits[bitIndex] = captureBitValue;
        Serial.printf("[RX] session %d capture frame %d/%d bit=%d(captured) state=%d center=%d delta=%.2f bits=",
                      s,
                      bitIndex + 1,
                      SESSION_CAPTURE_FRAMES,
                      captureBitValue,
                      signalBitValue,
                      centerSection,
                      centerDelta);

        for (int j = 0; j <= bitIndex; ++j)
        {
          Serial.print(sessions[s].frameBits[j]);
        }
        Serial.println();
        sessions[s].captureFrames++;
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
      else if ((frameId - sessions[s].allocatedFrame) > SESSION_MAX_LIFETIME_FRAMES)
      {
        Serial.print("[RX] session ");
        Serial.print(s);
        Serial.println(" exceeded max lifetime, resetting");
        resetSession(s);
      }
    }
    else
    {
      bool startSignal = centerDelta > sessionStartThreshold;
      if (startSignal)
      {
        sessions[s].startFrames++;
        sessions[s].lastActiveFrame = frameId;

        if (sessions[s].startFrames >= SESSION_START_FRAMES)
        {
          sessions[s].waitingForData = true;
          sessions[s].captureDelay = SESSION_START_SKIP_FRAMES;
          sessions[s].captureFrames = 0;
          sessions[s].currentBit = 1;
          sessions[s].pendingTransitionCount = 0;
          Serial.print("[RX] LOCK session ");
          Serial.print(s);
          Serial.print(" section ");
          Serial.print(sessions[s].centerSection);
          Serial.println(" start capture");
        }
      }
      else
      {
        sessions[s].startFrames = 0;
      }

      if ((frameId - sessions[s].lastActiveFrame) > SESSION_TIMEOUT_FRAMES)
      {
        resetSession(s);
      }
      else if ((frameId - sessions[s].allocatedFrame) > SESSION_MAX_LIFETIME_FRAMES)
      {
        Serial.print("[RX] session ");
        Serial.print(s);
        Serial.println(" exceeded max lifetime in startup, resetting");
        resetSession(s);
      }
    }
  }

  if ((frameId % PRINT_EVERY_FRAME) == 0)
  {
    int activeStateCount = 0;
    for (int s = 0; s < ANGULAR_SECTIONS; ++s)
    {
      activeStateCount += (sectionStates[s] != 0) ? 1 : 0;
    }
    logFrame(frameId, mean, minVal, maxVal, activeStateCount);
  }
}

}
