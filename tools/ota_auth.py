"""PlatformIO extra script: pass the OTA password to espota from include/WifiSecrets.h."""
import os
import re

Import("env")  # noqa: F821 - provided by PlatformIO

secrets_path = os.path.join(env.subst("$PROJECT_DIR"), "include", "WifiSecrets.h")
match = None
if os.path.exists(secrets_path):
    with open(secrets_path, encoding="utf-8") as f:
        match = re.search(r'#define\s+METASENSE_OTA_PASSWORD\s+"([^"]*)"', f.read())

if match:
    env.Append(UPLOADERFLAGS=["--auth=" + match.group(1)])
else:
    print("WARNING: METASENSE_OTA_PASSWORD not found in include/WifiSecrets.h; OTA upload will not authenticate.")
