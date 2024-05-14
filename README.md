# energystation
## Wireless electricity usage monitor

- Most mains electricity meters have a small LED that flashes each time 1Wh of evergy has been used.
- Count and store the number of flashes to simulate meter reading.
- Time the interval between successive flashes to calculate instantaneous power.
- Make this information available via Bluetooth to a phone app.

## Build instructions

1. Create an empty directory <br>`mkdir energystation`
2. Enter the directory <br>`cd energystation`
3. Initialise **west** project <br>`west init -m https://github.com/gadgetproject/energystation`
4. Fetch subordinate repos <br>`west update`
5. Fetch ESP32 HAL pre-compiled binaries <br>`west blobs fetch hal_espressif`
6. Build the software <br>`west build -b esp32_devkitc_wroom/esp32/procpu gp/soft`
7. Flash the software <br>`west flash`
8. See the software running <br>`west espressif monitor`
