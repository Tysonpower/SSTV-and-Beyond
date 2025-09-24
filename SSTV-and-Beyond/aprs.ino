// APRS lib to generate audio via LEDC
const int baudRate      = 1200;    // Baudrate
const unsigned long bitPeriod = 1000000 / baudRate; // ca. 833 µs pro Bit

const char *sourceCall = APRS_CALL;
const uint8_t sourceSSID = APRS_SSID;

const char *destinationCall = "APONET";   // used as Device Ident
const uint8_t destinationSSID = 0;

const char *digipeater1Call = "WIDE1";
const uint8_t digipeater1SSID = 1;

const char *digipeater2Call = "WIDE2";
const uint8_t digipeater2SSID = 1;

int sequenceNumber = 0;
float lastAlt = 0;
unsigned long lastAltTime = 0;

// NRZI-Zustand: true = Mark (1200 Hz), false = Space (2200 Hz)
bool nrziState = true;

// Puffer für den AX.25-Frame (ohne umschließende Flaggen)
#define FRAME_BUF_SIZE 512
uint8_t frameBuf[FRAME_BUF_SIZE];
size_t frameLen = 0;

int cntSinceLastLabelTX = 5;    // Labels on start and every 5 loops (around 10mins)
int cntSinceLastStatusTX = 0;   // Status ons tart and every 10 loops (around 20mins)

// ------------------------- Hilfsfunktionen -------------------------
void sendByteRaw(uint8_t byte) {
  for (int i = 0; i < 8; i++) {
    bool bit = (byte >> i) & 0x01;
    sendNRZIBit(bit);
  }
}

void waitBitPeriod() {
  uint64_t start = esp_timer_get_time();
  while ((esp_timer_get_time() - start) < bitPeriod) {
    // Busy-Wait
  }
}

void encodeAddress(const char *callsign, uint8_t ssid, bool isLast, uint8_t *dest) {
  for (int i = 0; i < 6; i++) {
    char c = callsign[i];
    if (c == '\0') c = ' '; // mit Leerzeichen auffüllen
    dest[i] = (c << 1);
  }
  // Wenn SSID 0 ist, soll kein SSID-Suffix übertragen werden.
  if (ssid == 0) {
    dest[6] = 0x00;
  } else {
    dest[6] = 0x60 | ((ssid & 0x0F) << 1) | (isLast ? 0x01 : 0x00);
  }
}

uint16_t computeFCS(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0x8408;
      else
        crc = (crc >> 1);
    }
  }
  return ~crc; // Bitweise invertieren
}

// ------------------------- NRZI-Kodierung & Bit-Stuffing -------------------------
void sendNRZIBit(bool bit) {
  if (!bit) {
    nrziState = !nrziState;
  }
  int freq = nrziState ? 1200 : 2200;
  ledcWriteTone(freq);
  waitBitPeriod();
}

void sendByteStuffed(uint8_t byte, uint8_t &onesCount) {
  for (int i = 0; i < 8; i++) {
    bool bit = (byte >> i) & 0x01;
    sendNRZIBit(bit);
    if (bit) {
      onesCount++;
      if (onesCount == 5) {
        sendNRZIBit(0);
        onesCount = 0;
      }
    } else {
      onesCount = 0;
    }
  }
}

// ------------------------- Preamble & Postamble -------------------------
void sendPreamble() {
  for (int i = 0; i < 100; i++) {
    sendByteRaw(0x7E);
  }
}

void sendPostamble() {
  for (int i = 0; i < 10; i++) {
    sendByteRaw(0x7E);
  }
}

// ------------------------- Frame stuff -------------------------

void buildFrame(AprsDatatype dataType, String payload) {
  frameLen = 0;
  uint8_t addr[7];
  
  // Destination
  encodeAddress(destinationCall, destinationSSID, false, addr);
  for (int i = 0; i < 7; i++) {
    frameBuf[frameLen++] = addr[i];
  }

  // Source (letztes Adressfeld)
  encodeAddress(sourceCall, sourceSSID, false, addr);
  for (int i = 0; i < 7; i++) {
    frameBuf[frameLen++] = addr[i];
  }
  
  // Digipeater 1
  encodeAddress(digipeater1Call, digipeater1SSID, false, addr);
  for (int i = 0; i < 7; i++) {
    frameBuf[frameLen++] = addr[i];
  }

  // Digipeater 2
  encodeAddress(digipeater2Call, digipeater2SSID, true, addr);
  for (int i = 0; i < 7; i++) {
    frameBuf[frameLen++] = addr[i];
  }

  // Steuerfeld: UI-Frame (0x03)
  frameBuf[frameLen++] = 0x03;
  
  // PID: (0xF0)
  frameBuf[frameLen++] = 0xF0;
  
  // APRS Data Type Identifier
  frameBuf[frameLen++] = (char)dataType;
  
  // Information: APRS-Text
  for (const char *p = payload.c_str(); *p != '\0'; p++) {
    frameBuf[frameLen++] = *p;
  }
  
  // FCS berechnen
  uint16_t fcs = computeFCS(frameBuf, frameLen);
  frameBuf[frameLen++] = (fcs & 0xFF);
  frameBuf[frameLen++] = ((fcs >> 8) & 0xFF);
}

// sends an APRS Frame
void aprsSendFrame(AprsDatatype dataType, String payload) {
  // create frame from data
  buildFrame(dataType, payload.c_str());

  // Erzwinge zu Beginn den Idle-Zustand (Mark, 1200 Hz)
  nrziState = true;
  ledcWriteTone(1200);
  waitBitPeriod();

  // config SA818 and start TX
  sa818Serial.println(SA818_APRS_CONF);
  delay(200);
  digitalWrite(SA818_PTT, LOW);
  
  sendPreamble();
  
  nrziState = true;
  
  uint8_t onesCount = 0;
  for (size_t i = 0; i < frameLen; i++) {
    sendByteStuffed(frameBuf[i], onesCount);
  }
  
  sendPostamble();
  
  // Setze Ausgang in Idle-Zustand (kein Signal)
  //ledcWriteTone(0);
  ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
  // Stop TX
  digitalWrite(SA818_PTT, HIGH);
  
  delay(50);
}

// main function that gets accessed by main programm
void sendAprsFrames() {
  Serial.println("Sending APRS NOW!");

  cntSinceLastLabelTX++;
  cntSinceLastStatusTX++;

  char distCall[10];
  snprintf(distCall, sizeof(distCall), "%-9s", CALLSIGN);

  // get and convert GPS Data
  char hours[3]; // 2 Stellen + Nullterminator
  sprintf(hours, "%02d", gps.time.hour());

  char mins[3]; // 2 Stellen + Nullterminator
  sprintf(mins, "%02d", gps.time.minute());

  char secs[3]; // 2 Stellen + Nullterminator
  sprintf(secs, "%02d", gps.time.second());

  char altitude[7];
  sprintf(altitude, "%06.0f", gps.altitude.feet());
  
  char course[4];
  sprintf(course, "%03.0f", gps.course.deg());
  
  char speed[5];
  sprintf(speed, "%03.0f", gps.speed.knots());

  char seqno[4];
  sprintf(seqno, "%03d", sequenceNumber);

  // build and send Position Frame
  String aprsData = String(String(hours) + String(mins) + String(secs) + "z" + formatLatitude(gps.location.lat()) + APRS_SYM_TABLE + formatLongitude(gps.location.lng()) + APRS_SYM_CODE + course + "/" + speed + "/A=" + altitude + APRS_MSG);
  Serial.println(aprsData.c_str());

  // only send APRS position if valid gps location younger than 30 secs
  if(gps.location.isValid() && gps.location.age() < 30000){
    Serial.println("Sending APRS Position");
    aprsSendFrame(AprsDatatype::posWithTime, aprsData.c_str());
  }

  delay(500);

  // send Status
  if(cntSinceLastStatusTX >= 10) {
    cntSinceLastStatusTX = 0;
    Serial.println("Sending APRS Status");
    aprsSendFrame(AprsDatatype::status , APRS_STATUS);
  }

  delay(500);

  // build and send Telemetry
  Serial.println("Sending APRS Telemetry");
  
  if(lastAlt != 0) {
    vRate = (gps.altitude.meters() - lastAlt) / ((millis() - lastAltTime) / 1000.0);
  } else {
    vRate = 0;
  }
  lastAlt = gps.altitude.meters();
  lastAltTime = millis();

  aprsSendFrame(AprsDatatype::telemetry , "#" + String(seqno) + "," + String(temperature, 2) + "," + String(humidity, 2) + "," + String(vRate, 1) + ","+ String(gps.satellites.value()) +"," + String(gps.altitude.meters(),1) +",11000000");
  sequenceNumber++;

  delay(500);

  // build and send Telemetry labels
  if(cntSinceLastLabelTX >= 5) {
    cntSinceLastLabelTX = 0;

    Serial.println("Sending APRS Telemetry Labels");
    aprsSendFrame(AprsDatatype::telemetryLabel , String(String(distCall) + ":" + APRS_TLM_LABELS).c_str());
  }
  

  //delay(500);

  // build and send Message
  //Serial.println("Sending APRS Message");
  //aprsSendFrame(AprsDatatype::message , String(String(distCall) + ":" + APRS_TEST_MESSAGE).c_str());

}