import threading
import serial
import time
import cv2
import math
import numpy as np
from filterpy.kalman import KalmanFilter
from ultralytics import YOLO

# Shared variables
angle_lock = threading.Lock()
shared_angle = 0
object_detected = threading.Event()     
terminate_program = threading.Event()
last_direction = None
vacuum_state = threading.Event()
reverse_state = threading.Event()  # New event to handle reverse stateQ

# Improved Arduino control function
def arduino_control():
    global shared_angle, last_direction
    try:
        ser_movement = serial.Serial('COM7', 115200, timeout=1)
        time.sleep(2)  # Wait for the serial connection to initialize
    except serial.SerialException as e:
        print(f"Error opening serial port for movement control: {e}")
        terminate_program.set()
        return
    
    last_command_time = 0

    def send_movement_command(command):
        nonlocal last_command_time
        current_time = time.time()
        if current_time - last_command_time > 0.5:  # 0.5 seconds debounce time
            ser_movement.write((command + '\n').encode())
            print(f"Sent movement command: {command}")
            try:
                response = ser_movement.readline().decode().strip()
                print("Movement Arduino Response:", response)
            except serial.SerialException as e:
                print(f"Error reading from serial port: {e}")
            last_command_time = current_time

    def send_stop_command():
        for _ in range(5):  # Retry sending the stop command up to 5 times
            ser_movement.write("MOVE 0 0\n".encode())
            try:
                response = ser_movement.readline().decode().strip()
                print("Movement Arduino Response:", response)
                if response == "OK":
                    break
            except serial.SerialException as e:
                print(f"Error reading from serial port: {e}")
            time.sleep(0.2)  # Small delay before retrying

    try:
        while not terminate_program.is_set():
            with angle_lock:
                angle = shared_angle
            if reverse_state.is_set():
                send_movement_command("MOVE 33.5 -52") # Reverse movement
            elif object_detected.is_set():
                vacuum_state.set()
                if angle > 0:
                    send_movement_command("MOVE -38 40")  # Move right
                    last_direction = "right"
                elif angle < 0:
                    send_movement_command("MOVE -35 45")  # Move left
                    last_direction = "left"
                else:
                    send_movement_command("MOVE -49.75 50")  # Stop
                    last_direction = None
            else:
                vacuum_state.clear()
                if last_direction == "right":
                    send_movement_command("MOVE -38 40")
                    last_direction = None  # Continue moving right
                elif last_direction == "left":
                    send_movement_command("MOVE -35 45")
                    last_direction = None  # Continue moving left
                else:
                    send_movement_command("MOVE 0 0")
                    last_direction = None  # Stop

            time.sleep(0.2)  # Add a small delay to avoid multiple rapid commands

    except KeyboardInterrupt:
        print("Program terminated")

    finally:
        # Send stop command to ensure the BOT stops
        send_stop_command()
        ser_movement.close()

# Vacuum control function
def vacuum_control():
    try:
        ser_vacuum = serial.Serial('COM6', 9600, timeout=1)
        time.sleep(2)  # Wait for the serial connection to initialize
    except serial.SerialException as e:
        print(f"Error opening serial port for vacuum control: {e}")
        terminate_program.set()
        return
    
    def send_vacuum_command(command):
        ser_vacuum.write((command + '\n').encode())
        print(f"Sent vacuum command: {command}")
        try:
            response = ser_vacuum.readline().decode().strip()
            print("Vacuum Arduino Response:", response)
        except serial.SerialException as e:
            print(f"Error reading from vacuum serial port: {e}")

    try:
        while not terminate_program.is_set():
            if vacuum_state.is_set():
                send_vacuum_command('1')
            else:
                time.sleep(2)
                if not vacuum_state.is_set():
                    send_vacuum_command('0')
            time.sleep(0.5)  # Check the vacuum state every 0.5 seconds

    except KeyboardInterrupt:
        print("Vacuum control terminated")

    finally:
        send_vacuum_command('0')  # Ensure vacuum is turned off
        ser_vacuum.close()

# YOLO object detection function
def yolo_detection():
    global shared_angle
    # Initialize the YOLO model
    model = YOLO("finetune_best2.pt").to('cuda')

    # Open the webcam
    cap = cv2.VideoCapture(0)

    # Set the desired frame width and height (adjust as needed)
    frame_width = 640
    frame_height = 480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    # Field of view (in degrees) of the camera
    camera_fov = 108.0

    # Calculate the sensor width based on the frame width and FOV
    sensor_width = 4.8  # Assume a common sensor width in mm for a webcam (e.g., 1/3 inch sensor)

    # Calculate the focal length using FOV
    camera_fov_rad = math.radians(camera_fov)
    focal_length = (sensor_width / 2) / math.tan(camera_fov_rad / 2)
    focal_length = focal_length * (frame_width / sensor_width)

    # Initialize the Kalman filter
    kf = KalmanFilter(dim_x=4, dim_z=2)
    kf.F = np.array([[1, 0, 1, 0],
                     [0, 1, 0, 1],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])  # State transition matrix
    kf.H = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0]])  # Measurement function
    kf.R *= 10  # Measurement uncertainty
    kf.P *= 10  # Covariance matrix
    kf.Q *= 0.01  # Process uncertainty

    # Function to calculate distance from known width and bounding box dimension
    def calculate_distance(known_width, bbox_dimension):
        return (known_width * focal_length) / bbox_dimension

    # Timer for object detection
    detection_timer = time.time()

    # Loop through the video frames
    while cap.isOpened() and not terminate_program.is_set():
        # Read a frame from the video
        success, frame = cap.read()

        if success:
            # Run YOLOv8 inference on the frame
            results = model(frame, conf=0.65)

            # Ensure results are in expected format
            if isinstance(results, list):
                closest_distance = float('inf')
                closest_object_detected = False
                closest_center_x = 0
                closest_angle = 0
                closest_bbox = None

                for result in results:
                    # Loop through all detections in result.boxes
                    for bbox in result.boxes.xyxy:
                        # Get the bounding box coordinates
                        x1, y1, x2, y2 = bbox.cpu().numpy()
                        
                        # Calculate the center of the bounding box
                        center_x = (x1 + x2) / 2
                        center_y = (y1 + y2) / 2
        
                        # Calculate the dimensions of the bounding box
                        bbox_width_px = x2 - x1
                        bbox_height_px = y2 - y1
                        
                        # Choose the larger dimension (width or height)
                        chosen_dimension_px = max(bbox_width_px, bbox_height_px)
                        
                        # Limit the chosen dimension to ensure it's within the expected range (0cm to frame_width)
                        chosen_dimension_px = min(chosen_dimension_px, frame_width)
                        
                        # Calculate the perceived distance (assuming a known width of 10 cm)
                        known_width = 10.0  # Width of the object in cm
                        perceived_distance_cm = calculate_distance(known_width, chosen_dimension_px)
                        cv2.line(frame, (int((x1 + x2) / 2), int(y1)), (int((x1 + x2) / 2), int(y1 - 50)), (255, 0, 0), 2)
                        cv2.putText(frame, f"Distance: {perceived_distance_cm:.2f} cm", (int((x1 + x2) / 2), int(y1 - 60)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                        # Track closest object based on distance
                        if perceived_distance_cm < closest_distance or not closest_object_detected:
                            closest_distance = perceived_distance_cm
                            closest_object_detected = True
                            closest_center_x = center_x  # Store the center_x of the closest object
                            closest_bbox = bbox  # Store the bbox of the closest object

                            # Calculate angle to rotate camera
                            object_offset_x = closest_center_x - frame_width / 2
                            closest_angle = int((object_offset_x / frame_width) * 180)

                        # Draw a green line and display the distance
                        cv2.line(frame, (int(center_x), int(y1)), (int(center_x), int(y1 - 50)), (255, 0, 0), 2)
                        cv2.putText(frame, f"Distance: {perceived_distance_cm:.2f} cm", (int(center_x), int(y1 - 60)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                # After processing all detections, draw "obj" next to closest object information
                if closest_object_detected:
                    x1, y1, x2, y2 = closest_bbox.cpu().numpy()
                    cv2.putText(frame, "obj", (int((x1 + x2) / 2), int(y2 + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    
                    # Draw angle of rotation next to "obj"
                    cv2.putText(frame, f"Rotate: {closest_angle} degrees", (int((x1 + x2) / 2) - 80, int(y2 + 40)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                    # Kalman filter update
                    kf.predict()
                    kf.update([closest_center_x, center_y])
                    
                    # Get the predicted state
                    predicted_x, predicted_y = kf.x[0], kf.x[1]
                    
                    # Draw the predicted position
                    cv2.circle(frame, (int(predicted_x), int(predicted_y)), 5, (0, 255, 0), -1)

                    # Update shared angle and reset detection timer
                    with angle_lock:
                        shared_angle = closest_angle
                    detection_timer = time.time()
                    object_detected.set()

                    # Start the vacuum if the object is within 50 cm
                    if closest_distance <= 50:
                        vacuum_state.set()
                        print("Vacuum ON")
                    else:
                        vacuum_state.clear()
                        print("Vacuum OFF")
                else:
                    vacuum_state.clear()
                    print("Vacuum OFF")

            # Check if object detection timer has exceeded 2 seconds
            if time.time() - detection_timer > 2:
                object_detected.clear()
                vacuum_state.clear()
                print("Vacuum OFF")

            # Display the frame
            annotated_frame = results[0].plot()
            cv2.imshow("YOLO Inference", annotated_frame)

            # Check for 'q' key press to terminate
            if cv2.waitKey(1) & 0xFF == ord("Q"):
                terminate_program.set()
                break
            # Check for 'r' key press to reverse
            if cv2.waitKey(1) & 0xFF == ord("r"):
                if reverse_state.is_set():
                    reverse_state.clear()
                else:
                    reverse_state.set()
        else:
            break

    # Release the video capture object and close the display window
    cap.release()
    cv2.destroyAllWindows()

# Create threads for Arduino control, vacuum control, and YOLO detection
arduino_thread = threading.Thread(target=arduino_control)
vacuum_thread = threading.Thread(target=vacuum_control)
yolo_thread = threading.Thread(target=yolo_detection)

# Start the threads
arduino_thread.start()
vacuum_thread.start()
yolo_thread.start()

# Wait for all threads to finish
arduino_thread.join()
vacuum_thread.join()
yolo_thread.join()
