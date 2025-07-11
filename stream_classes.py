import cv2
import time
from foxglove import Channel
from foxglove.channels import RawImageChannel
from foxglove.schemas import RawImage, Timestamp
import numpy as np

class Camera:
    def __init__(self, cam):
        self.cam = cv2.VideoCapture(cam)
        self.name = "cam"+str(cam)
        self.channel = RawImageChannel("/"+self.name)
        # Get the default frame width and height
        self.frame_width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def get_frame(self):
        ret, frame = self.cam.read()
        if not ret:
            return None
        img_msg = RawImage(
            timestamp=Timestamp(
                sec=int(time.time()),
                nsec=int((time.time() - int(time.time())) * 1e9),
            ),
            frame_id=self.name,
            width=self.frame_width,
            height=self.frame_height,
            encoding="bgr8",
            step=self.frame_width*3,
            data=frame.tobytes(),
        )
        return img_msg

    def log_data(self, data):
        self.channel.log(data)

    def close(self):
        self.cam.release()
        cv2.destroyAllWindows()


class PDTracker:
    def __init__(self, K, X0, size, diff_gains, name="tracker"):
        self.Kp = K[0]
        self.Kd = K[1]
        self.X0 = X0
        self.X = X0
        self.last_X = X0
        self.last_ref = np.zeros(size)
        self.last_err = 0
        self.last_time = time.time()
        self.size = size
        self.diff_gains = diff_gains
        self.name = name

        self.channel = Channel(
            "/pd_tracker",
            message_encoding="json",
        )

    def update(self, ref):
        current_time = time.time()
        dt = current_time - self.last_time
        err = ref - self.X
        derr = err - self.last_err
        self.last_err = err

        self.X += self.Kp * err + self.Kd * derr / dt
        self.last_time = current_time
        return self.X

    def get_diff(self):
        diff = self.X - self.last_X
        self.last_X = self.X.copy()
        adjusted_diff = np.abs(diff) * self.diff_gains
        return adjusted_diff

    def log_data(self, X, diff):
        log_dict = {}
        for i in range(self.size):
            log_dict[f"{self.name}{i}"] = float(X[i])
            log_dict[f"{self.name}{i}_diff"] = float(diff[i])
        self.channel.log(log_dict)

class SerialReader:
    def __init__(self, ser, size):
        self.ser = ser
        self.size = size
        self.last_data = np.zeros(size)
        self.channel = Channel(
            "/sensor_raw",
            message_encoding="json",
        )
        
    def read_serial(self):
        data = self.ser.readline()
        self.ser.reset_input_buffer()  # Flush the input buffer after reading
        # Decode bytes to string and strip whitespace
        data_str = data.decode('utf-8').strip()
        # Split by comma and convert to integers
        try:
            read_numbers = np.array([int(x.strip()) for x in data_str.split(',') if x.strip()])
            if len(read_numbers) >= self.size:
                self.last_data = read_numbers[:self.size]
        except ValueError:
            # Return zeros if conversion fails
            pass
        return self.last_data

    def log_data(self, data):
        log_dict = {}
        for i in range(self.size):
            log_dict[f"sensor{i}"] = int(data[i])
        self.channel.log(log_dict)