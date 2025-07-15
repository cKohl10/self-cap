import foxglove
import serial
import time
import argparse
from stream_classes import SerialReader, Camera, PDTracker
import numpy as np
from foxglove.schemas import Timestamp

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Stream proximity sensor data')
    parser.add_argument('-s', '--sensors', type=int, default=3, help='Number of sensors (default: 3)')
    parser.add_argument('-c', '--cam', type=int, default=0, help='Camera index (default: 0)')
    parser.add_argument('-w', '--write', type=str, default=None, help='MCAP save location')
    args = parser.parse_args()
    
    num_sensors = args.sensors
    server = foxglove.start_server()
    if args.write is not None:
        writer = foxglove.open_mcap(args.write, allow_overwrite=True)
        description = input("Enter a description for the data: ")
        foxglove.log("/info",{
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "description": description,
        })


    ser = serial.Serial('/dev/ttyACM0', 9600)
    serial_reader = SerialReader(ser, num_sensors)

    cam = Camera(args.cam)
    tracker = PDTracker(K=[0.4, 0.005], X0=np.zeros(3), size=3, diff_gains=[1, 1, 1], name="FA")


    try:
        print("Streaming data...")
        while True:
            ser_data = serial_reader.read_serial()
            img_msg = cam.get_frame()
            X = tracker.update(ser_data)
            diff = tracker.get_diff()

            serial_reader.log_data(ser_data)
            tracker.log_data(X, diff)
            if img_msg is not None:
                cam.log_data(img_msg)

    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
        cam.close()
        ser.close()
        print("Cleanup complete.")