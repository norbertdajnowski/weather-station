import logging
import asyncio
import platform
import sys
from time import sleep
import urllib
import urllib.request

from bleak import BleakClient
from bleak import _logger as logger


CHARACTERISTIC_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

baseURL = 'https://api.thingspeak.com/update?api_key=N4DJDAJ1LN59PKQJ&field1='

async def thingspeakWrite():

    with urllib.request.urlopen(baseURL + temp + "&field2=" + humidity + "&field3=" + pressure + "&field4=" + CO + "&field5=" + light + "&field6=" + Alcohol) as f:
        f.read()
        f.close()
    
def notification_handler(sender, data):

    global humidity
    global temp
    global pressure
    global light
    global CO
    global Alcohol
    global CO2

    try:
        text = str(data)
        text = text.rstrip(")'")
        text = text.lstrip("bytearray(b'")
        if "Humidity" in text:
            text = text.rstrip("%")
            humidity = text.lstrip("Humidity: ")
            
        elif "Temp" in text:
            text = text.rstrip("C")
            temp = text.lstrip("Temp: ")

        elif "Pressure" in text:
            text = text.rstrip("Pa")
            pressure = text.lstrip("Pressure: ")

        elif "Light" in text:
            text = text.rstrip("V")
            light = text.lstrip("Light: ")

        elif "CO1" in text:
            text = text.rstrip("PPM")
            CO = text.lstrip("CO1: ")

        elif "Alcohol" in text:
            text = text.rstrip("PPM")
            Alcohol = text.lstrip("Alcohol: ")
            
    finally:    
        print(text)


async def run(address):
    
    async with BleakClient(address) as client:
        while True:
            x = await client.is_connected()
            try:
                await client.start_notify(CHARACTERISTIC_UUID, notification_handler)    
                await asyncio.sleep(15)
                await thingspeakWrite()
            finally:
                await client.stop_notify(CHARACTERISTIC_UUID)
        

if __name__ == "__main__":
    import os

    os.environ["PYTHONASYNCIODEBUG"] = str(1)
    address = (
        "64:69:4E:80:2A:46" 
        )
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(address))
    

