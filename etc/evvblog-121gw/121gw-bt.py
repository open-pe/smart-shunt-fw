"""

121GW EEV Blog DMM
featuring 24bit AFE HY3131 pdf:https://www.hycontek.com/wp-content/uploads/DS-HY3131_EN.pdf
- linearity: ±(0.01%+2Counts)
- temp drift (typ): 2.5ppm/°C
- https://www.digikey.de/en/product-highlight/d/digilent/dmm-shield

"""
import asyncio
import sys

import bleak


async def main():
    def _discon(client):
        print('disconnected!')
        sys.exit(0)

    cl = bleak.BleakClient('BF8C3232-8906-0D0C-F368-3CE91D98394E', disconnected_callback=_discon, timeout=4)

    await cl.connect()
    print('connected!')

    def _on_data(char, data:bytearray):
        if data[0] == 0x17:
            v = int.from_bytes(data[6:8], byteorder='big')
            print(v)

    await cl.start_notify('e7add780-b042-4876-aae1-112855353cc1', _on_data)
    await cl.write_gatt_char('e7add780-b042-4876-aae1-112855353cc1', bytes([0o0300]), False)

    while 1:
        await asyncio.sleep(1)

    async with bleak.BleakScanner() as sc:
        print(await sc.discover())


asyncio.run(main())

# https://github.com/chlordk/121gwcli
# https://gitlab.com/eevblog/app-121gw
# https://github.com/zonque/121gw-qt5
