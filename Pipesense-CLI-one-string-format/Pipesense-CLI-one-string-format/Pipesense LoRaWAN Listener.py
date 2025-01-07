import asyncio
import websockets
import json
from datetime import datetime, timedelta, timezone

async def listen():
    uri = "wss://iotnet.teracom.dk/app?token=vnoVqgAAABFpb3RuZXQudGVyYWNvbS5ka6fsjk89IcmqQwPO2-qRMbs="
    last_uplink_time = None  # Initialize last uplink time
    async with websockets.connect(uri) as websocket:
        while True:
            message = await websocket.recv()
            data = json.loads(message)
            
            # Extracting the required fields
            time = data.get("time")
            cmd = data.get("cmd")
            fcnt = data.get("fcnt")
            freq = data.get("freq")
            port = data.get("port")
            dr = data.get("dr")
            eui = data.get("EUI")
            rssi = data.get("rssi")
            data_field = data.get("data")

            # Safely handle the time field with millisecond precision and timezone-aware datetime
            if time is not None:
                # Convert milliseconds to seconds and use timezone-aware UTC datetime
                dt_object = datetime.fromtimestamp(time / 1000, tz=timezone.utc)
                formatted_time_with_ms = dt_object.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

                # Calculate time since last uplink if last_uplink_time is available
                if last_uplink_time:
                    time_since_last_uplink = dt_object - last_uplink_time
                    print(f"Time since last uplink: {time_since_last_uplink.total_seconds()} seconds")
                else:
                    print("This is the first uplink received.")

                # Update the last uplink time
                last_uplink_time = dt_object

                # Print uplink details
                print(f"time: {formatted_time_with_ms}, cmd: {cmd}, fcnt: {fcnt}, freq: {freq}, port: {port}, dr: {dr}, EUI: {eui}, rssi: {rssi}, data: {data_field}")
            else:
                formatted_time_with_ms = "N/A"
                print(f"cmd: {cmd}, fcnt: {fcnt}, freq: {freq}, port: {port}, dr: {dr}, EUI: {eui}, rssi: {rssi}, data: {data_field}")

if __name__ == "__main__":
    # For Python 3.10+ compatibility with event loop
    try:
        asyncio.run(listen())
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(listen())
