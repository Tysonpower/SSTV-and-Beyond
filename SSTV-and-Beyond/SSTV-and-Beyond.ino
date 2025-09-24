// SSTV & Beyond Firmware
// 
// Created by Manuel DO5TY - tynet.eu
//
//////////////////////////////////////////////////////////////////////
#include <Adafruit_GFX.h>
#include "Fonts/FreeSansBold12pt7b.h"
#include <TinyGPS++.h>
#include <driver/ledc.h>
#include "esp_timer.h"   // ESP32 API V3 Timer
#include "esp_camera.h"
#include "DHT.h"
#include <shared.h>

TinyGPSPlus gps;
HardwareSerial sa818Serial(1);
HardwareSerial gpsSerial(2);

DHT dht(DHTPIN, DHTTYPE);

unsigned long lastSecondInterval = 0;

OperationMode opMode = OperationMode::IDLE;

esp_timer_handle_t pixelTimerHandle = NULL;

float temperature = 0;
float humidity = 0;

float vRate = 0;

void read_dht(){
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("Failed to read from DHT sensor!"));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("SSTV & Beyond Firmware");
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, SA818_TX);

  delay(500);
  Serial.println("Startup...");

  // config PTT pin
  pinMode(SA818_PTT, OUTPUT);
  digitalWrite(SA818_PTT, HIGH);

  // initial config SA818
  sa818Serial.begin(9600, SERIAL_8N1, SA818_RX, SA818_TX);
  delay(100);
  sa818Serial.println("AT+VERSION");
  delay(500);
  while (sa818Serial.available()) { Serial.println(sa818Serial.readStringUntil('\n')); }
  sa818Serial.println("AT+DMOCONNECT");
  delay(500);
  while (sa818Serial.available()) { Serial.println(sa818Serial.readStringUntil('\n')); }
  sa818Serial.println("AT+SETFILTER=1,1,1");
  delay(500);
  while (sa818Serial.available()) { Serial.println(sa818Serial.readStringUntil('\n')); }
  sa818Serial.println("AT+DMOSETGROUP=0,144.5000,144.5000,0000,0,0000");
  delay(500);
  while (sa818Serial.available()) { Serial.println(sa818Serial.readStringUntil('\n')); }

  // LEDC setup: CH 0, 5000 Hz, 8-Bit resolution.
  ledc_timer_config_t ledc_timer;
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_timer.duty_resolution = LEDC_TIMER_12_BIT;
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer.freq_hz = 2200;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer.deconfigure = false;

  ledc_timer_config(&ledc_timer);

  // LEDC-channel config
  ledc_channel_config_t ledc_channel;
  ledc_channel.channel = LEDC_CHANNEL_0;
  ledc_channel.duty = 2048;
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.gpio_num = SA818_MIC;
  ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_channel.hpoint = 0;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel_config(&ledc_channel);

  ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);

  // ESP-Timer: create periodic timer for later
  esp_timer_create_args_t timer_args = {
    .callback = &pixelTimerCallback,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "pixel_timer"
  };
  esp_timer_create(&timer_args, &pixelTimerHandle);

  setupCamera();

  dht.begin();

  Serial.println("Setup finished");
}

void loop() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  if ((millis() - lastSecondInterval) > 5000 ) {
    lastSecondInterval = millis();

    // print position if we got a valid one
    if (gps.location.isUpdated()) {
      Serial.print("Pos: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(", ");
      Serial.print(gps.location.lng(), 6);
      Serial.print(" Sats: ");
      Serial.print(gps.satellites.value());
      Serial.print(" Hight (Meter): ");
      Serial.print(gps.altitude.meters());
      Serial.print(" Date: ");
      Serial.print(gps.date.day());
      Serial.print("/");
      Serial.print(gps.date.month());
      Serial.print("/");
      Serial.print(gps.date.year());
      Serial.print(" Time: ");
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.println(gps.time.second());
      Serial.println();

    } else {
      Serial.println("No GPS Position...");
    }

    // check what to do next
    if(opMode == OperationMode::IDLE){
      opMode = OperationMode::MORSE;
    } else if(opMode == OperationMode::MORSE){ 
      Serial.print("Sending Morse: ");
      Serial.println(CALLSIGN);
      sendMorse(CALLSIGN, 15);

      Serial.println("Moving to APRS");
      opMode = OperationMode::APRS;
    } else if(opMode == OperationMode::APRS){

      read_dht();
      Serial.println("DHT data: " + String(temperature, 1) + "°C - " + String(humidity, 1) + "%");
      
      // Send APRS Data
      sendAprsFrames();

      Serial.println("Moving to SSTV");
      opMode = OperationMode::SSTV;
    } else if(opMode == OperationMode::SSTV){
      read_dht();
      Serial.println("DHT data: " + String(temperature, 1) + "°C - " + String(humidity, 1) + "%");

      // send SSTV PD120 image - not during Boot Delay
      if(millis() >  SSTV_BOOTDELAY * 1000) {
        Serial.println("Real SSTV");
        takeAndTransmitImageViaSSTV();
      } else {
        // delay 1 min until next loop
        Serial.println("Going to sleep instead of SSTV");
        delay(60 * 1000);
      }

      Serial.println("Moving to IDLE");
      opMode = OperationMode::IDLE;
    }
  }
  
  // delay 100ms to slow the loop, in idle it will just wait for gps data
  delay(100);
}
