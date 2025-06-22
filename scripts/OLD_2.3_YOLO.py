import cv2
from ultralytics import YOLO
import RPi.GPIO as GPIO
import time

# --- Pin setup ---
TRIG = 36  # Physical pin 36 for TRIG
ECHO = 38  # Physical pin 38 for ECHO
MOTOR = 37  # Physical pin 37 for Motor control
SERVO = 32  # Physical pin 32 for Servo control

GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(MOTOR, GPIO.OUT)
GPIO.setup(SERVO, GPIO.OUT)
GPIO.output(TRIG, False)
GPIO.output(MOTOR, True)
time.sleep(2)

# --- Servo setup ---
servo_pwm = GPIO.PWM(SERVO, 50)
servo_pwm.start(0)

def set_servo_angle(angle):
    duty = 2.5 + (angle / 18.0)
    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    servo_pwm.ChangeDutyCycle(0)

def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    pulse_start = time.time()
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    pulse_end = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    distance = (pulse_end - pulse_start) * 17150
    return round(distance, 2)

# --- YOLOv8 setup ---
model = YOLO("best.pt")
print("Model info:")
print(f"Classes: {model.names}\n")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

WAIT_FOR_GEAR = 0
WAIT_FOR_STABILIZE = 1
ANALYZE_GEAR = 2
EJECT_GEAR = 3

state = WAIT_FOR_GEAR
decision_buffer = []
current_servo_angle = 20
set_servo_angle(20)
eject_start_time = None
stabilize_start_time = None
area_cleared = False

try:
    while True:
        ret, frame = cap.read()
        if not ret: break

        distance = get_distance()

        if state == WAIT_FOR_GEAR:
            GPIO.output(MOTOR, True)
            cv2.putText(frame, "Conveyor: RUNNING", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Distance: {distance}cm", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            if distance <= 7:
                GPIO.output(MOTOR, False)
                state = WAIT_FOR_STABILIZE
                stabilize_start_time = time.time()
                print("\nGear detected. Waiting 3 seconds to stabilize...")

        elif state == WAIT_FOR_STABILIZE:
            cv2.putText(frame, "Stabilizing gear...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            elapsed = time.time() - stabilize_start_time if stabilize_start_time else 0
            if elapsed >= 3:
                state = ANALYZE_GEAR
                decision_buffer.clear()
                print("Starting analysis...")

        elif state == ANALYZE_GEAR:
            damaged_count = 0
            correct_count = 0
            results = model.predict(frame, imgsz=320, device='cpu',
                                   classes=[0,1], conf=0.4, verbose=False)
            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    if cls_id == 0:
                        damaged_count += 1
                        color = (0, 0, 255)
                    elif cls_id == 1:
                        correct_count += 1
                        color = (0, 255, 0)
                    else:
                        color = (255, 255, 0)
                    label = f"{model.names[cls_id]} {conf:.2f}"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            decision_buffer.append((damaged_count, correct_count))
            print(f"Sample {len(decision_buffer)}: D={damaged_count}, C={correct_count}")

            progress = len(decision_buffer)/10
            cv2.putText(frame, f"Analyzing: {int(progress*100)}%", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.rectangle(frame, (10, 40), (210, 50), (100, 100, 100), 1)
            cv2.rectangle(frame, (10, 40), (10 + int(200*progress), 50),
                          (0, 255, 0), -1)

            if len(decision_buffer) >= 10:
                total_damaged = sum(d[0] for d in decision_buffer)
                total_correct = sum(d[1] for d in decision_buffer)
                if total_damaged > total_correct:
                    if current_servo_angle != 0:
                        set_servo_angle(0)
                        current_servo_angle = 0
                    final_status = "DAMAGED"
                    color = (0, 0, 255)
                elif total_correct > total_damaged:
                    if current_servo_angle != 20:
                        set_servo_angle(20)
                        current_servo_angle = 20
                    final_status = "CORRECT"
                    color = (0, 255, 0)
                else:
                    if current_servo_angle != 0:
                        set_servo_angle(0)
                        current_servo_angle = 0
                    final_status = "UNKNOWN"
                    color = (255, 255, 0)
                print(f"\nFinal Decision: {final_status}")
                print("------------------------------")
                cv2.putText(frame, f"FINAL: {final_status}", (10, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.imshow("YOLOv8 Webcam", frame)
                cv2.waitKey(2000)
                GPIO.output(MOTOR, True)
                eject_start_time = time.time()
                area_cleared = False
                state = EJECT_GEAR

        elif state == EJECT_GEAR:
            elapsed = time.time() - eject_start_time if eject_start_time else 0
            if elapsed < 5:
                cv2.putText(frame, "Ejecting gear...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            else:
                if distance > 7 and not area_cleared:
                    area_cleared = True
                if not area_cleared:
                    cv2.putText(frame, "Ejecting gear...", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                else:
                    # Show distance and running status while waiting for new gear
                    cv2.putText(frame, "Conveyor: RUNNING", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f"Distance: {distance}cm", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                # Wait for new gear to arrive
                if area_cleared and distance <= 7:
                    GPIO.output(MOTOR, False)
                    state = WAIT_FOR_STABILIZE
                    stabilize_start_time = time.time()
                    print("\nGear detected. Waiting 3 seconds to stabilize...")

        cv2.imshow("YOLOv8 Webcam", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    servo_pwm.stop()
    GPIO.cleanup()
