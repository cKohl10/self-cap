import foxglove
import serial
import time
from foxglove import Channel
import numpy as np
from stream_classes import SerialReader, PDTracker

if __name__ == "__main__":

    server = foxglove.start_server()
    ser = serial.Serial('/dev/ttyACM0', 9600)

    ser_channel = Channel(
        "/sensor_raw",
        message_encoding="json",
    )

    pd_channel = Channel(
        "/pd_tracker",
        message_encoding="json",
    )

    FA_tracker = PDTracker(K=[0.2, 0.001], X0=np.zeros(3), size=3)
    SA_tracker = PDTracker(K=[0.02, 0.001], X0=np.zeros(3), size=3)
    serial_reader = SerialReader(ser, 3)
    FA_diff_gain = 1
    SA_diff_gain = 10


    print("Streaming data...")

    while True:
        data = serial_reader.read_serial()
        FA_tracker.update(data)
        SA_tracker.update(data)
        FA_diff = np.abs(FA_tracker.get_diff())*FA_diff_gain
        SA_diff = np.abs(SA_tracker.get_diff())*SA_diff_gain
        serial_reader.log_data(data)

        pd_channel.log({
            "FA0": int(FA_tracker.X[0]),
            "FA1": int(FA_tracker.X[1]),
            "FA2": int(FA_tracker.X[2]),
            "SA0": int(SA_tracker.X[0]),
            "SA1": int(SA_tracker.X[1]),
            "SA2": int(SA_tracker.X[2]),
            "FA0_diff": float(FA_diff[0]),
            "FA1_diff": float(FA_diff[1]),
            "FA2_diff": float(FA_diff[2]),
            "SA0_diff": float(SA_diff[0]),
            "SA1_diff": float(SA_diff[1]),
            "SA2_diff": float(SA_diff[2]),
        })