// user constants
#define CALLSIGN "DL0KA-11"
#define SSTV_QSL "PLS QSL via https://g-fliegt.de"
#define SSTV_BOOTDELAY 120
#define APRS_CALL "DL0KA"
#define APRS_SSID 11
#define APRS_STATUS "Distrikt G Ballon - https://g-fliegt.de/rapport"
#define APRS_MSG "https://app.aprs.one/c/DL0KA-11"
#define APRS_SYM_TABLE "/"
#define APRS_SYM_CODE "O"
#define APRS_TLM_LABELS "PARM.ATemp,AHum,vRate,Sats,Alt,Camera,SSTV"
#define APRS_TEST_MESSAGE "This is a short test message"

// Setup for GPS
#define GPS_RX 2
#define GPS_TX 14  // only dummy, no TX to GPS
#define GPS_BAUD 115200

// Pins for SA818
#define SA818_RX 4
#define SA818_TX 14
#define SA818_PTT 12
#define SA818_MIC 13

// DHT22 config
#define DHTPIN 15
#define DHTTYPE DHT22

// Camera pins & settings
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27

#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

// 4 for flash led or 33 for normal led
#define LED_GPIO_NUM   33

// Settings
#define SA818_SSTV_CONF "AT+DMOSETGROUP=0,144.5000,144.5000,0000,0,0000"
#define SA818_APRS_CONF "AT+DMOSETGROUP=0,144.8000,144.8000,0000,0,0000"

#define SSTV_TOP_OVL_SHIFT 265


// other stuff

enum OperationMode {IDLE, SSTV, APRS, MORSE};

// APRS Data Type Identifier
enum class AprsDatatype : char
{
  posWithoutTime = '!',
  posWithTime = '/',
  status = '>',
  telemetry = 'T',
  telemetryLabel = ':',
  message = ':'
};

//write a tone by frequency
void ledcWriteTone(uint32_t frequency) {
  ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, frequency);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 2048);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

// only send tone, no frequency change
void ledcSendTone(unsigned long duration_ms) {
  // Erzeuge Morse-Ton: Setze LEDC-Duty (hier 128) für die Dauer des Tons
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 2048);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
  delay(duration_ms);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

// Funktion zur Formatierung des Breitengrads (latitude)
// Format: ddmm.mm + N oder S
String formatLatitude(double lat) {
  char direction = (lat >= 0) ? 'N' : 'S';
  lat = fabs(lat);
  int degrees = int(lat);
  double minutes = (lat - degrees) * 60;
  char buf[10];
  // %02d: 2-stellige Gradzahl, %05.2f: Minuten mit 2 Dezimalstellen (insgesamt 5 Zeichen, inkl. Punkt)
  sprintf(buf, "%02d%05.2f%c", degrees, minutes, direction);
  return String(buf);
}

// Funktion zur Formatierung des Längengrads (longitude)
// Format: dddmm.mm + E oder W
String formatLongitude(double lon) {
  char direction = (lon >= 0) ? 'E' : 'W';
  lon = fabs(lon);
  int degrees = int(lon);
  double minutes = (lon - degrees) * 60;
  char buf[11];
  // %03d: 3-stellige Gradzahl, %05.2f: Minuten mit 2 Dezimalstellen
  sprintf(buf, "%03d%05.2f%c", degrees, minutes, direction);
  return String(buf);
}