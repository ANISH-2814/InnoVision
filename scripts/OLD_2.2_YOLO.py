import cv2
from ultralytics import YOLO
import RPi.GPIO as GPIO
import time

# --- Pin setup ---
TRIG = 36  # Physical pin 36 for TRIG
ECHO = 38  # Physical pin 38 for ECHO
MOTOR = 37 # Physical pin 37 for Motor control

GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(MOTOR, GPIO.OUT)
GPIO.output(TRIG, False)
GPIO.output(MOTOR, False)  # Start with motor off
time.sleep(2)  # Let sensor settle

def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    pulse_start = time.time()
    pulse_end = time.time()

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound factor (cm)
    return round(distance, 2)

# --- YOLOv8 setup ---
model = YOLO("best.pt")  # No .to('cuda'), as Pi does not support CUDA

print("Model info:")
print(f"Model path: best.pt")
print(f"Classes: {model.names}")
print(f"Number of classes: {len(model.names)}")
print("\nRunning on device: CPU (Raspberry Pi)")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

try:
    while True:
        ret, frame = cap.read()
        if not ret: break

        # --- Measure distance ---
        distance = get_distance()

        if distance > 7:
            GPIO.output(MOTOR, True)  # Motor ON (conveyor running)
            # Print and overlay distance
            print(f"Distance: {distance} cm")
            dist_text = f"Distance: {distance:.2f} cm"
            cv2.putText(frame, dist_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("YOLOv8 Webcam", frame)
        else:
            GPIO.output(MOTOR, False) # Motor OFF (conveyor stopped)
            # Run YOLOv8 inference
            results = model.predict(frame, imgsz=320, device='cpu', classes=[0,1], conf=0.4, verbose=False)
            conf_damaged = 0.0  # class 0
            conf_correct = 0.0  # class 1

            for result in results:
                for box in result.boxes:
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    if cls_id == 0:  # 'd' = damaged
                        conf_damaged = max(conf_damaged, conf)
                        color = (0, 0, 255)  # Red
                        label = f"{model.names[0]} {conf:.2f}"
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    elif cls_id == 1:  # 'c' = correct
                        conf_correct = max(conf_correct, conf)
                        color = (0, 255, 0)  # Green
                        label = f"{model.names[1]} {conf:.2f}"
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Determine gear status and print only when conveyor is stopped
            if conf_damaged > conf_correct:
                print("Gear is damaged")
                status_text = "Gear: Damaged"
            elif conf_correct > conf_damaged:
                print("Gear is correct")
                status_text = "Gear: Correct"
            else:
                print("Gear is unknown")
                status_text = "Gear: Unknown"

            # Overlay detection results on frame (no distance overlay)
            info_text = f"D: {conf_damaged:.2f} | C: {conf_correct:.2f}"
            cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, status_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("YOLOv8 Webcam", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
