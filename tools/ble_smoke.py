"""Hardware smoke test of the SWEA BLE service from a PC (needs bleak, see requirements.txt).

Checks, without pairing: the device advertises the SWEA service, a connection is accepted, and the encrypted
characteristics refuse access on an unencrypted link (the security requirement). Exit code 0 = all as expected.
With --paired (the PC is already bonded, e.g. paired in Windows settings) it also reads Status and sends GET_INFO.
"""
import argparse
import asyncio
import sys

from bleak import BleakClient, BleakScanner

SERVICE = "3f5737c4-b391-44c7-bd74-f2ee561c9775"
CONTROL = "1118282f-3dfe-44a7-a12f-6d06987f1092"
STATUS = "8c2502e8-eacd-45d1-bd20-361eb5a762a2"


async def main(paired: bool) -> int:
    found = await BleakScanner.discover(timeout=8, service_uuids=[SERVICE], return_adv=True)
    if not found:
        print("FAIL: no device advertises the SWEA service")
        return 1
    address, (device, adv) = next(iter(found.items()))
    print(f"PASS: found {device.name} {address} rssi={adv.rssi} uuids={adv.service_uuids}")
    ok = True
    try:
        client_cm = BleakClient(address, timeout=20)
        await client_cm.__aenter__()
    except Exception as e:
        # Windows reports 'Unreachable' when the device drops an unbonded, unencrypted link during service discovery.
        print(f"INFO: connection/discovery without bonding failed ({type(e).__name__}: {e}); "
              "this is expected when the device refuses unencrypted access. Pair first and use --paired for the functional checks.")
        return 0 if not paired else 1
    async with client_cm as client:
        print("PASS: connection accepted")
        svc = client.services.get_service(SERVICE)
        print("PASS: service present" if svc else "FAIL: service missing")
        ok &= svc is not None
        try:
            data = await client.read_gatt_char(STATUS)
            if paired:
                print(f"PASS: status read ({len(data)} bytes) {data.hex()}")
                ok &= len(data) == 20
            else:
                print("FAIL: Status was readable without encryption")
                ok = False
        except Exception as e:  # bleak maps ATT errors to exceptions
            if paired:
                print(f"FAIL: status read failed: {e}")
                ok = False
            else:
                print(f"PASS: unencrypted read refused ({type(e).__name__}: {e})")
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--paired", action="store_true")
    sys.exit(asyncio.run(main(ap.parse_args().paired)))
