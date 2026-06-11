# MetaSense Dyno

PlatformIO-based firmware and web UI for the MetaSense dyno controller.

## Build

```powershell
platformio run -e esp32s3-USB
```

## Upload firmware

```powershell
platformio run -e esp32s3-USB -t upload
```

## Upload web files

```powershell
platformio run -e esp32s3-USB -t uploadfs
```

## OTA upload

```powershell
platformio run -e esp32s3-ota -t upload
platformio run -e esp32s3-ota -t uploadfs
```

## Notes

- Use `esp32s3-USB` for local USB flashing.
- Use `esp32s3-ota` for network uploads to `dyno-controller.local`.
- The web UI is served from LittleFS.