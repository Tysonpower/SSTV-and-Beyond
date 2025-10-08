# SSTV-and-Beyond
SSTV &amp; Beyond firmware for ESP32-CAM based transmitters - PD120, APRS, CW

This Arduino based Firmware can run on a ESP32-CAM module (with OV2640 sensor) that is connected to a SA818 2m Band transmitter, a DHT22 temperature and humidity sensor as well as a RS232 GPS module (BN220 for example).

The first idea and test of this concept was done at a hamradio field week in march of 2025 of the IG Hamspirit e.V. - i then further developed the hardware and rewrote the complete software stack as well as implemented PD120 for better image quality. Main application for this transmitter are high altitude balloon flights.

There were two successfull tests of the current hardware and software during two balloon launches, more details and results can be found under the following links (german - use translater if needed):

[Flight Bornheim - 20.09.2025](https://g-fliegt.de/news/start-bornheim-2025-erfolgreich)

[Flight Baunatal - 30.08.2025](https://g-fliegt.de/news/baunatal-start-2025)

### Features

* use GPS for Position, Altitude and Speed
* use DHT22 to get temperature and humidity
* Capture image from Camera
* Overlay image with callsign and telemetry
* convert image including overlay to SSTV
* transmit SSTV via SA818 on 144,500mhz
* transmit Callsign in CW on 144,500mhz
* transmit APRS Position and Telemetry on 144,800mhz

### Hardware

I made a simple PCB to connect everything together, but you can also just use wires or perf baord like prototype PCBs.

Backside:
![PXL_20250707_154536705](https://github.com/user-attachments/assets/17b106ea-920c-4f6b-9019-376e4b7ef75b)

Frontside:
![PXL_20250707_154520141](https://github.com/user-attachments/assets/18700a5b-00e9-4af9-8612-14ff2c70ab27)

Schematic:
<img width="1160" height="559" alt="image" src="https://github.com/user-attachments/assets/c73fbea3-d27b-4c8c-98ba-8343ce101c28" />
