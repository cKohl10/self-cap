import foxglove
import serial
import time
from foxglove import Channel
import numpy as np

class PD_tracker:
    def __init__(self, K, X0, ser, size):
        self.Kp = K[0]
        self.Kd = K[1]
        self.X0 = X0
        self.X = X0
        self.last_X = X0
        self.last_ref = np.zeros(size)
        self.last_err = 0
        self.last_time = time.time()
        self.ser = ser
        self.size = size

    def update(self, ref):
        current_time = time.time()
        dt = current_time - self.last_time
        err = ref - self.X
        derr = err - self.last_err
        self.last_err = err

        self.X += self.Kp * err + self.Kd * derr / dt
        self.last_time = current_time
        return self.X
        
    def read_serial(self):
        data = self.ser.readline()
        self.ser.reset_input_buffer()  # Flush the input buffer after reading
        # Decode bytes to string and strip whitespace
        data_str = data.decode('utf-8').strip()
        # Split by comma and convert to integers
        try:
            read_numbers = np.array([int(x.strip()) for x in data_str.split(',') if x.strip()])
            if len(read_numbers) >= self.size:
                return read_numbers[:self.size]
        except ValueError:
            # Return zeros if conversion fails
            pass
        return self.last_ref

    def get_diff(self):
        diff = self.X - self.last_X
        self.last_X = self.X.copy()
        return diff

if __name__ == "__main__":

    server = foxglove.start_server()
    ser = serial.Serial('/dev/ttyACM0', 9600)

    ser_channel = Channel(
        "/sensor",
        message_encoding="json",
    )

    pd_channel = Channel(
        "/pd_tracker",
        message_encoding="json",
    )

    pd_tracker = PD_tracker(K=[0.1, 0.001], X0=np.zeros(3), ser=ser, size=3)
    diff_gain = 10


    print("Streaming data...")

    while True:
        data = pd_tracker.read_serial()
        pd_tracker.update(data)
        diff = np.abs(pd_tracker.get_diff())*diff_gain
        ser_channel.log({
            "sensor0": int(data[0]),
            "sensor1": int(data[1]),
            "sensor2": int(data[2]),
            "sensor0_diff": float(diff[0]),
            "sensor1_diff": float(diff[1]),
            "sensor2_diff": float(diff[2]),
        })

        pd_channel.log({
            "pd0": int(pd_tracker.X[0]),
            "pd1": int(pd_tracker.X[1]),
            "pd2": int(pd_tracker.X[2]),
        })