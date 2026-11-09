# Provisioning secrets

Wi-Fi credentials and ThingSpeak keys live in NVS, never in source code or the
spec. Channel IDs are not secret and default to 1415485 (duck door) and 1160824
(pump) in the firmware.

1. Copy `secrets.csv.example` to `secrets.csv` and fill in the values. Keep
   `secrets.csv` out of version control. The namespace must stay `duck`.
2. Build the NVS image (size matches the 0x6000 `nvs` partition):

   ```
   python $IDF_PATH/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py \
       generate secrets.csv secrets.bin 0x6000
   ```
3. Flash it to the `nvs` partition at 0x9000:

   ```
   esptool.py -p /dev/ttyUSB0 write_flash 0x9000 secrets.bin
   ```

This replaces the whole NVS partition, so the saved door and lift positions
reset to UNKNOWN. That is fine at provisioning time.
