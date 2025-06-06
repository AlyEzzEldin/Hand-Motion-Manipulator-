# Hand-Motion-Manipulator 🤖🖐️  
_Control a robotic arm with your bare hands using a webcam, MediaPipe, and Arduino._

## 🚀 Overview
HandyBot is a gesture-controlled system that lets you operate a robotic arm without touching any controls. Just move your hand in front of a webcam — and the arm follows your motion! It's powered by Python, MediaPipe for hand tracking, and Arduino for servo control.

## 🎯 Features
- ✅ Real-time hand gesture detection using MediaPipe
- ✅ Controls 4 robotic arm axes: 
  - **Slider** (X-axis hand movement)
  - **Pan** (Y-axis hand movement)
  - **Tilt** (hand depth/forward-back motion)
  - **Gripper** (based on thumb–index finger distance)
- ✅ Smooth and responsive serial communication with Arduino
- ✅ Visual feedback overlay on webcam feed
- ✅ Easy-to-customize ranges and thresholds

## 🛠️ How It Works
- Your hand is tracked in real-time using a standard webcam.
- The wrist and fingertips are analyzed to extract direction and depth.
- Movements are mapped to servo angles.
- Commands are sent over serial to an Arduino controlling servos.

## 📦 Requirements

### Hardware
- Arduino Uno/Nano/Mega
- 4x Servo motors
- USB cable for serial communication
- Webcam (built-in or external)

### Software
- Python 3.7+
- OpenCV
- MediaPipe
- PySerial

Install the dependencies:
```bash
pip install opencv-python mediapipe pyserial
