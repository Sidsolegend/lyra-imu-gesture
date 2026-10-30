#!/usr/bin/env python3
"""Update the sealed LYRA ring's firmware over Bluetooth.

  pip install bleak
  Arduino IDE: Sketch > Export Compiled Binary  (gives lyra_ring.ino.bin)
  python3 ota_ring.py path/to/lyra_ring.ino.bin

Protocol (lib/lyra_ota.h): write "BEGIN <size>" to OTA_CTRL, then the file in order to OTA_DATA
(write-with-response, so every chunk is acknowledged before the next), then "END". The ring
checks the image, answers "OTA OK" on its TX notifications and reboots into it. Keep the ring
charging or above 30 % battery, and within a metre of the computer.
"""
import asyncio, sys, os
from bleak import BleakScanner, BleakClient

SVC = "6e400201-b5a3-f393-e0a9-e50e24dcca9e"
TX = "6e400203-b5a3-f393-e0a9-e50e24dcca9e"
CTRL = "6e400204-b5a3-f393-e0a9-e50e24dcca9e"
DATA = "6e400205-b5a3-f393-e0a9-e50e24dcca9e"


async def main(path):
    image = open(path, "rb").read()
    if not image or image[0] != 0xE9:
        sys.exit("not an ESP32 app image (.bin must start with 0xE9)")
    print("looking for LYRA-RING ...")
    dev = await BleakScanner.find_device_by_filter(
        lambda d, ad: SVC in [u.lower() for u in ad.service_uuids], timeout=15)
    if not dev:
        sys.exit("ring not found: wake it (tap) and keep it close")
    lines = asyncio.Queue()
    async with BleakClient(dev) as c:
        await c.start_notify(TX, lambda _, d: lines.put_nowait(d.decode(errors="ignore").strip()))
        chunk = max(20, min(509, c.mtu_size - 3))

        async def wait_for(*prefixes, timeout=20):
            while True:
                line = await asyncio.wait_for(lines.get(), timeout)
                if line.startswith("OTA"):
                    print("ring:", line)
                if line.startswith("OTA ERR"):
                    sys.exit(line)
                if any(line.startswith(p) for p in prefixes):
                    return line

        await c.write_gatt_char(CTRL, f"BEGIN {len(image)}".encode(), response=True)
        await wait_for("OTA READY")
        for i in range(0, len(image), chunk):
            await c.write_gatt_char(DATA, image[i:i + chunk], response=True)
            if (i // chunk) % 100 == 0:
                print(f"\r{i * 100 // len(image):3d} %  ({i}/{len(image)} bytes, {chunk} B chunks)", end="", flush=True)
        print("\r100 %")
        await c.write_gatt_char(CTRL, b"END", response=True)
        await wait_for("OTA OK", timeout=30)
    print("done: the ring reboots into the new firmware")


if __name__ == "__main__":
    if len(sys.argv) != 2 or not os.path.isfile(sys.argv[1]):
        sys.exit(__doc__)
    asyncio.run(main(sys.argv[1]))
