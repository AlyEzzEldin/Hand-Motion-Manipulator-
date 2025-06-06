import cv2
import mediapipe as mp
import math
import serial

class VirtualHandControlledManipulator:
    def __init__(self,
                 slider_range=(0, 180),  # Hand X-axis movement
                 pan_range=(0, 180),     # Hand Y-axis movement
                 tilt_range=(0, 180),    # Hand depth movement (forward/back)
                 gripper_range=(0, 90),
                 gripper_threshold=0.12,  # Fist detection threshold
                 serial_port='COM3',      # Arduino serial port
                 baudrate=115200):        # Serial baudrate
        # MediaPipe initialization
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            max_num_hands=1
        )
        self.mp_drawing = mp.solutions.drawing_utils

        # Serial connection (non-blocking writes)
        try:
            self.ser = serial.Serial(serial_port, baudrate, timeout=0, write_timeout=0)
            print(f"[INFO] Connected to Arduino on {serial_port} at {baudrate}bps")
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open serial port {serial_port}: {e}")

        # Webcam initialization
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open webcam")

        # Configuration parameters
        self.slider_range = slider_range
        self.pan_range = pan_range
        self.tilt_range = tilt_range
        self.gripper_range = gripper_range
        self.gripper_threshold = gripper_threshold

        # State tracking
        self.current_slider = slider_range[0]
        self.current_pan = pan_range[0]
        self.current_tilt = tilt_range[0]
        self.current_gripper = gripper_range[0]
        self.current_handedness = None
        self.previous_gripper_state = None

    def map_value(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def send_serial_commands(self):
        # Batch and send all current values once per frame
        cmd_str = f"S{self.current_slider} P{self.current_pan} T{self.current_tilt} G{self.current_gripper}\n"
        # Print to console for verification
        print(f"[SERIAL OUT] {cmd_str.strip()}")
        try:
            bytes_written = self.ser.write(cmd_str.encode('utf-8'))
            print(f"[SERIAL] Wrote {bytes_written} bytes")
            # Optionally read any response from Arduino
            resp = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if resp:
                print(f"[SERIAL IN] {resp}")
        except serial.SerialException as e:
            print(f"[ERROR] Serial communication failed: {e}")

    def add_visual_feedback(self, image):
        h, w, _ = image.shape
        if hasattr(self, 'wrist'):
            x = int(self.wrist.x * w)
            cv2.line(image, (x, 0), (x, h), (255, 255, 0), 1)
            y = int(self.wrist.y * h)
            cv2.line(image, (0, y), (w, y), (0, 255, 255), 1)

        state_text = (
            f"{self.current_handedness or 'No'} Hand | "
            f"Slider:{self.current_slider} "
            f"Pan:{self.current_pan} "
            f"Tilt:{self.current_tilt} "
            f"Gripper:{self.current_gripper}"
        )
        cv2.putText(image, state_text, (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return image

    def process_frame(self, image):
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = self.hands.process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        self.current_handedness = None

        if results.multi_hand_landmarks:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks,
                                                 results.multi_handedness):
                self.current_handedness = handedness.classification[0].label
                self.mp_drawing.draw_landmarks(
                    image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                self._process_hand_movement(hand_landmarks)
            # send all commands once after processing hand
            self.send_serial_commands()

        return self.add_visual_feedback(image)

    def _process_hand_movement(self, landmarks):
        self.wrist = landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        self.index_tip = landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        thumb_tip = landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]

        # 1) Slider via wrist X-axis
        self.current_slider = math.floor(self.map_value(self.wrist.x, 0, 1, *self.slider_range))

        # 2) Pan via wrist Y-axis
        self.current_pan = math.floor(self.map_value(self.wrist.y, 0, 1, *self.pan_range))

        # 3) Tilt via depth (index_tip z-axis)
        depth = -self.index_tip.z
        depth = max(0, min(depth, 0.2))
        tilt_norm = depth / 0.2
        self.current_tilt = math.floor(self.map_value(tilt_norm, 0, 1, *self.tilt_range))

        # 4) Gripper via thumb-index distance
        dist = math.hypot(thumb_tip.x - self.index_tip.x,
                          thumb_tip.y - self.index_tip.y)
        is_open = dist > self.gripper_threshold
        state = "OPEN" if is_open else "CLOSED"
        if state != self.previous_gripper_state:
            print(f"{self.current_handedness} hand {state.lower()}")
            self.previous_gripper_state = state
        self.current_gripper = self.gripper_range[0] if is_open else self.gripper_range[1]

    def release_resources(self):
        self.cap.release()
        cv2.destroyAllWindows()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        print("Resources released")

    def run(self):
        try:
            while self.cap.isOpened():
                success, img = self.cap.read()
                if not success:
                    continue
                out = self.process_frame(img)
                cv2.imshow('Hand Control', out)
                if cv2.waitKey(5) & 0xFF == ord('q'):
                    break
        finally:
            self.release_resources()

if __name__ == "__main__":
    VirtualHandControlledManipulator(serial_port='COM20', baudrate=115200).run()
