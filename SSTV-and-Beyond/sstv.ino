int pictureNumber = 1;

// PD120-Timing (in Mikrosekunden)
const uint32_t syncPulseDuration = 20000;   // 20 ms
const uint32_t porchDuration     = 2080;      // 2,08 ms
const uint32_t scanDuration      = 121600;    // 121,6 ms pro Scanabschnitt

// Bildauflösung für PD120
const int imageWidth = 640;
const int imageHeight = 496;  // muss gerade sein (z.B. 496 Zeilen = 248 Zeilenpaare)

// Dauer pro Pixel in Mikrosekunden
const uint32_t pixelDuration = scanDuration / imageWidth;  // ca. 190 µs

// Unterklasse, die den Canvas-Puffer im PSRAM allokiert
class PSRAMCanvas16 : public GFXcanvas16 {
public:
  PSRAMCanvas16(uint16_t w, uint16_t h) : GFXcanvas16(w, h) {
    if (buffer) {
      free(buffer);
      buffer = nullptr;
    }
    buffer = (uint16_t*)heap_caps_malloc(w * h * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    if (!buffer) {
      Serial.println("PSRAM-Allokation fehlgeschlagen!");
    } else {
      Serial.println("PSRAM-Allokation OK!");
    }
  }
};

// Globaler Canvas (RGB565)
PSRAMCanvas16 *canvas;

// ---------------------- Hardware-Timer & globale Variablen ------------------

// Pixelzähler und Steuerung für den aktuellen Scanabschnitt
volatile int pixelCounter = 0;
volatile bool rowFinished = false;

// Segmenttypen für den Scanabschnitt
enum SegmentType { SEG_Y, SEG_RY, SEG_BY };
volatile SegmentType currentSegment = SEG_Y;

// Für den Y-Scan
volatile int currentRow = 0;

// Für Differenz-Segmente (R-Y, B-Y)
volatile int currentRowOdd = 0;
volatile int currentRowEven = 0;

// ---------------------- Funktionen zur Pixelabfrage und SSTV-Konvertierung ----------------------

// Liest einen Pixel (RGB565) aus dem Canvas und wandelt ihn in 24-Bit RGB um.
void getCanvasPixel(int x, int y, uint8_t &R, uint8_t &G, uint8_t &B) {
  uint16_t pixel = canvas->getBuffer()[y * imageWidth + x];
  uint8_t r5 = (pixel >> 11) & 0x1F;
  uint8_t g6 = (pixel >> 5)  & 0x3F;
  uint8_t b5 = pixel & 0x1F;
  R = (r5 * 255) / 31;
  G = (g6 * 255) / 63;
  B = (b5 * 255) / 31;
}

// Konvertiert RGB in SSTV-Kanäle
// Y = 0.299*R + 0.587*G + 0.114*B
// R-Y = 0.713 * (R - Y)
// B-Y = 0.564 * (B - Y)
void convertToSSTV(uint8_t R, uint8_t G, uint8_t B, float &Y, float &RY, float &BY) {
  Y  = 0.299 * R + 0.587 * G + 0.114 * B;
  RY = 0.713 * (R - Y);
  BY = 0.564 * (B - Y);
}

// Mapped den Y-Wert (0..255) auf den Frequenzbereich 1500Hz bis 2300Hz
uint32_t mapYToFrequency(float Y) {
  return 1500 + (uint32_t)((Y / 255.0) * 800);
}

// Mapped einen Differenzwert (z.B. R-Y oder B-Y; Wertebereich ca. -128..127) auf 1500Hz bis 2300Hz.
uint32_t mapDiffToFrequency(float diff) {
  return 1500 + (uint32_t)(((diff + 128.0) / 255.0) * 800);
}

// ------------------------- ESP-Timer Callback (Pixel-Update) -------------------------
// Diese Funktion wird periodisch alle pixelDuration µs vom esp_timer aufgerufen.
void pixelTimerCallback(void* arg) {
  uint32_t freq = 0;
  if (currentSegment == SEG_Y) {
    // Lese Pixel aus aktueller Zeile (currentRow) an Position pixelCounter
    uint8_t R, G, B;
    getCanvasPixel(pixelCounter, currentRow, R, G, B);
    float Y, dummyRY, dummyBY;
    convertToSSTV(R, G, B, Y, dummyRY, dummyBY);
    freq = mapYToFrequency(Y);
  }
  else if (currentSegment == SEG_RY) {
    // Für R-Y: Mittelung zweier Zeilen (currentRowOdd und currentRowEven)
    uint8_t R1, G1, B1, R2, G2, B2;
    getCanvasPixel(pixelCounter, currentRowOdd, R1, G1, B1);
    getCanvasPixel(pixelCounter, currentRowEven, R2, G2, B2);
    float Y1, RY1, BY1, Y2, RY2, BY2;
    convertToSSTV(R1, G1, B1, Y1, RY1, BY1);
    convertToSSTV(R2, G2, B2, Y2, RY2, BY2);
    float avgRY = (RY1 + RY2) / 2.0;
    freq = mapDiffToFrequency(avgRY);
  }
  else if (currentSegment == SEG_BY) {
    // Für B-Y: Mittelung zweier Zeilen (currentRowOdd und currentRowEven)
    uint8_t R1, G1, B1, R2, G2, B2;
    getCanvasPixel(pixelCounter, currentRowOdd, R1, G1, B1);
    getCanvasPixel(pixelCounter, currentRowEven, R2, G2, B2);
    float Y1, RY1, BY1, Y2, RY2, BY2;
    convertToSSTV(R1, G1, B1, Y1, RY1, BY1);
    convertToSSTV(R2, G2, B2, Y2, RY2, BY2);
    float avgBY = (BY1 + BY2) / 2.0;
    freq = mapDiffToFrequency(avgBY);
  }
  // Setze den LEDC-Ton auf den berechneten Frequenzwert.
  ledcWriteTone(freq);
  
  pixelCounter++;
  if (pixelCounter >= imageWidth) {
    // Alle Pixel dieser Zeile wurden übertragen: Timer stoppen und Flag setzen.
    esp_timer_stop(pixelTimerHandle);
    rowFinished = true;
  }
}

// ---------------------- Funktionen für die Übertragung einzelner Scanabschnitte ----------------------

// Überträgt einen Y-Scan für die Zeile 'row' mittels Hardware-Timer.
void transmitLineY_HW(int row) {
  currentSegment = SEG_Y;
  currentRow = row;
  pixelCounter = 0;
  rowFinished = false;
  esp_timer_start_periodic(pixelTimerHandle, pixelDuration);
  while (!rowFinished) {
  }
}

// Überträgt einen R-Y-Scan für die Zeilen oddRow und evenRow mittels Hardware-Timer.
void transmitLineDiffRY_HW(int oddRow, int evenRow) {
  currentSegment = SEG_RY;
  currentRowOdd = oddRow;
  currentRowEven = evenRow;
  pixelCounter = 0;
  rowFinished = false;
  esp_timer_start_periodic(pixelTimerHandle, pixelDuration);
  while (!rowFinished) { }
}

// Überträgt einen B-Y-Scan für die Zeilen oddRow und evenRow mittels Hardware-Timer.
void transmitLineDiffBY_HW(int oddRow, int evenRow) {
  currentSegment = SEG_BY;
  currentRowOdd = oddRow;
  currentRowEven = evenRow;
  pixelCounter = 0;
  rowFinished = false;
  esp_timer_start_periodic(pixelTimerHandle, pixelDuration);
  while (!rowFinished) { }
}

// ---------------------- Testbild-Generierung und Overlay (Canvas wird direkt genutzt) ----------------------

void generateBaseImage() {
  canvas = new PSRAMCanvas16(imageWidth, imageHeight);
  if (canvas == nullptr) {
    Serial.println("Canvas couldn't be created!");
  }
  // fill canvas with background color
  canvas->fillScreen(0x29ee);
  Serial.println("Canvas created in PSRAM and prepared");
}

// ---------------------------------------------------------------------------
// Fügt Overlay-Text in den Canvas ein.
// Mit dieser Funktion kann man flexibel mehrere Strings an verschiedenen Positionen
// und in unterschiedlichen Größen (Textgröße über setTextSize) hinzufügen.
void addOverlayText(const char* text, int posX, int posY, uint8_t textSize) {
  // Optional: Setze den gewünschten GFX-Font, z. B. einen der FreeFonts, falls eingebunden.
  canvas->setFont(&FreeSansBold12pt7b);
  canvas->setTextSize(textSize);    // Skalierung des Textes (Standardfont)
  canvas->setTextColor(0xffff);
  canvas->setCursor(posX, posY);
  canvas->print(text);
}
// ---------------------- Calibration Header ----------------------
// Überträgt den Kalibrierungsheader inkl. VIS-Code (95 Dezimal) über tonePulse (hier unverändert)
void tonePulse(uint32_t frequency, uint32_t durationMicros) {
  ledcWriteTone(frequency);
  delayMicroseconds(durationMicros);
}

void transmitCalibrationHeader() {
  Serial.println("Sending SSTV header...");
  tonePulse(1900, 300000);
  tonePulse(1200, 10000);
  tonePulse(1900, 300000);
  tonePulse(1200, 30000);
  tonePulse(1100, 30000);  // Bit 0: 1
  tonePulse(1100, 30000);  // Bit 1: 1
  tonePulse(1100, 30000);  // Bit 2: 1
  tonePulse(1100, 30000);  // Bit 3: 1
  tonePulse(1100, 30000);  // Bit 4: 1
  tonePulse(1300, 30000);  // Bit 5: 0
  tonePulse(1100, 30000);  // Bit 6: 1
  tonePulse(1300, 30000);  // Parity (even)
  tonePulse(1200, 30000);
}

// ---------------------- SSTV PD120-Übertragung ----------------------
// Überträgt das komplette Bild (Zeilenpaarweise) im PD120-Modus.
// Für jedes Zeilenpaar (odd und even) werden:
//   1. Sync-Puls (20ms bei 1200Hz)
//   2. Porch (2,08ms bei 1500Hz)
//   3. Y-Scan (ungerade Zeile)
//   4. R-Y-Scan (Mittelung beider Zeilen)
//   5. B-Y-Scan (Mittelung beider Zeilen)
//   6. Y-Scan (gerade Zeile)
// übertragen.
void transmitPD120Image_HW() {
  Serial.println("Sending SSTV image data...");
  int numPairs = imageHeight / 2;
  for (int pair = 0; pair < numPairs; pair++) {
    int oddLine = pair * 2;
    int evenLine = oddLine + 1;
    
    // (1) Sync-Puls: 20 ms bei 1200 Hz
    ledcWriteTone(1200);
    uint32_t start = micros();
    while ((micros() - start) < syncPulseDuration) { }
    
    // (2) Porch: 2,08 ms bei 1500 Hz
    ledcWriteTone(1500);
    start = micros();
    while ((micros() - start) < porchDuration) { }
    
    // (3) Y-Scan (ungerade Zeile)
    transmitLineY_HW(oddLine);
    // (4) R-Y-Scan (Mittelung beider Zeilen)
    transmitLineDiffRY_HW(oddLine, evenLine);
    // (5) B-Y-Scan (Mittelung beider Zeilen)
    transmitLineDiffBY_HW(oddLine, evenLine);
    // (6) Y-Scan (gerade Zeile)
    transmitLineY_HW(evenLine);
  }
  ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
}

void drawImageFromBuffer(uint8_t* imgBuffer, int imgWidth, int imgHeight) {
  for (int y = 0; y < imgHeight; y++) {
    for (int x = 0; x < imgWidth; x++) {
      // Berechne den Index im Buffer (2 Byte pro Pixel)
      int index = (y * imgWidth + x) * 2;
      // Kombiniere die zwei Bytes zu einem 16-Bit RGB565-Wert
      uint16_t pixel = (imgBuffer[index] << 8) | imgBuffer[index + 1];
      // Zeichne den Pixel auf den Canvas an der Position (x,y)
      canvas->drawPixel(x, y, pixel);
    }
  }
}

// main function that gets accessed by main programm
void takeAndTransmitImageViaSSTV(){
  Serial.println("Takin a picture...");
  camera_fb_t *fb = NULL;

  generateBaseImage();
  uint16_t* targetBuffer = canvas->getBuffer();

  // get tmp image to avoid getting old image
  fb = esp_camera_fb_get();
  esp_camera_fb_return(fb);
  delay(500);

  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed! - using black image only");
  } else {
    Serial.println("Got image from camera...");

    int imageWidthCam = fb->width/* gewünschte Breite */;
    int imageHeightCam = fb->height/* gewünschte Höhe */;

    // RGB565-Puffer anlegen 
    uint8_t *rgb565_buffer = (uint8_t*)heap_caps_malloc(fb->width * fb->height * 2, MALLOC_CAP_SPIRAM);
    if (rgb565_buffer == NULL) {
        // Fehlerbehandlung
        Serial.println("Error creating buffer for image");
        return;
    }

    // convert jpeg image into noemal buffer
    bool result = jpg2rgb565(fb->buf, fb->len, rgb565_buffer, (jpg_scale_t)0);
    if (!result) {
        Serial.println("Error converting image into buffer!");
        free(rgb565_buffer);
    } else {
        Serial.println("Image was converted...");
    }

    int offsetY = canvas->height() - imageHeightCam;

    // Move real image onto canvas, leave room on top for Data
    for (int y = 64; y < imageHeightCam; y++) {
      for (int x = 0; x < imageWidthCam; x++) {
        int srcIndex = (y * imageWidthCam + x) * 2;
        uint16_t pixel = (((uint16_t)rgb565_buffer[srcIndex + 1]) << 8) | rgb565_buffer[srcIndex];
        int destIndex = ( (y + offsetY) * canvas->width() + x );
        targetBuffer[destIndex] = pixel;
      }
    }

    esp_camera_fb_return(fb);
    free(rgb565_buffer);
  }
  
  char hours[3]; // 2 Stellen + Nullterminator
  sprintf(hours, "%02d", gps.time.hour());
  
  char mins[3]; // 2 Stellen + Nullterminator
  sprintf(mins, "%02d", gps.time.minute());

  char secs[3]; // 2 Stellen + Nullterminator
  sprintf(secs, "%02d", gps.time.second());

  char pics[4]; // 2 Stellen + Nullterminator
  sprintf(pics, "%03d", pictureNumber);

  char alt[6]; // 5 Stellen + Nullterminator
  sprintf(alt, "%.0f", gps.altitude.meters());

  char sat[3]; // 2 Stellen + Nullterminator
  sprintf(sat, "%2d", gps.satellites.value());

  // add image overlay
  addOverlayText(CALLSIGN, 5, 40, 2);
  addOverlayText(String(String(gps.date.day()) + "." + String(gps.date.month()) + "." + String(gps.date.year()) + " - " + String(hours) + ":" + String(mins) + ":" + String(secs)).c_str(), SSTV_TOP_OVL_SHIFT, 22, 1);
  addOverlayText(String("Pic: " + String(pics)).c_str(), 545, 22, 1);
  addOverlayText(String(String(gps.location.lat(), 6) + ", " + String(gps.location.lng(), 6)).c_str(), SSTV_TOP_OVL_SHIFT, 46, 1);
  addOverlayText(String("Alt: " + String(alt) + "m").c_str(), 505, 46, 1);
  addOverlayText(String("Sats: " + String(sat)).c_str(), 545, 70, 1);
  addOverlayText(SSTV_QSL, 5, 70, 1);
  addOverlayText(String(String(temperature, 1) + "°C - " + String(humidity, 1) + "%").c_str(), 375, 70, 1);
  
  Serial.print("Starting SSTV transmission... Pic: ");
  Serial.println(pics);
  // set Freq and enbale TX
  sa818Serial.println(SA818_SSTV_CONF);
  delay(200);
  digitalWrite(SA818_PTT, LOW);
  // send SSTV with header
  transmitCalibrationHeader();
  transmitPD120Image_HW();
  // disable TX
  digitalWrite(SA818_PTT, HIGH);
  Serial.println("SSTV completed");

  pictureNumber++;
  free(targetBuffer);
  delay(5000);
}