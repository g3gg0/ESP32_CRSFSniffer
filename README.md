# CRSFSniffer

This project is about analyzing the functionality of RC control links on the 868 Mhz band. The code is designed for use with a Heltec LoRa ESP32 with SX127**6** (e.g. https://www.aliexpress.com/item/33018609928.html)

## preamble

**PROJECT IS WIP, RESULTS MAY VARY, USE AT YOUR OWN RISK**

This project is solely for white-hat scientific/educational hacking. By no means do we want to piss someone off.

## getting started with Arduino IDE

The code is written to be flashed with the [Arduino IDE](https://www.arduino.cc/en/software) onto the LoRa ESP32.

First, install the IDE.

Then you need to add support for ESP32 based boards. To do so, got into `File` -> `Preferences`. In `Additional Boards Manager URLs` add the URL `https://dl.espressif.com/dl/package_esp32_index.json`. If you already have some other URL listed there, separate the URLs with a comma.

Now go to `Tools` -> `Board: XYZ` -> `Board Manager`. Search for `esp32` and install the package.

Go to `Tools` -> `Board: XYZ` -> `ESP32 Arduino`, scroll down the list and select `Heltec WiFi LoRa 32(V2)`.

Now you need to adjust one header file.

Go to `C:\Users\<USERNAME>\AppData\Local\Arduino15\packages\esp32\hardware\esp32\<VERSION>\cores\esp32` and open the file `esp32-hal.h` in an editor of your choice.
Add the line `#define CONFIG_DISABLE_HAL_LOCKS 1` to where the other `#define`s are located.

Now you are ready to flash the code.

## technical documentation

For now there is no technical documentation. Will get updated soon.

## development progress documentation

Here some videos showing progress:

### First steps
[![Video](http://img.youtube.com/vi/WCkuwO97zfM/0.jpg)](http://www.youtube.com/watch?v=WCkuwO97zfM "Video")

### Display channel values
[![Video](http://img.youtube.com/vi/pvUvxsUui7U/0.jpg)](http://www.youtube.com/watch?v=pvUvxsUui7U "Video")

### Add hopping support
[![Video](http://img.youtube.com/vi/Tf6I0-eUbNs/0.jpg)](http://www.youtube.com/watch?v=Tf6I0-eUbNs "Video")

### Scan hopping sequence
[![Video](http://img.youtube.com/vi/1E_6FAqVi6U/0.jpg)](http://www.youtube.com/watch?v=1E_6FAqVi6U "Video")

### Further analysis and plots
[![Video](http://img.youtube.com/vi/GCU4Pnw3n-c/0.jpg)](http://www.youtube.com/watch?v=GCU4Pnw3n-c "Video")
