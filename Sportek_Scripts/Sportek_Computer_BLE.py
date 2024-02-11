import asyncio
from bleak import BleakScanner
from bleak import BleakClient
from bleak import BleakGATTCharacteristic
test_UUID = "19B10019-E8F2-537E-4F6C-D104768A1214"
def test_callback(sender: BleakGATTCharacteristic, data: bytearray):
    print(f"{sender}: {data}")
async def scan():
    scanner = BleakScanner()
    found_device = await scanner.find_device_by_name("Artemis")
    if (found_device == None):
        print("Aint it");
    if found_device != None:
        async with BleakClient(found_device) as client:
            print(client.address)
            await client.start_notify(test_UUID, test_callback)
            while True:
                await asyncio.sleep(0.001)
    

asyncio.run(scan())
