import sensor, image, time
from pid import PID
from pyb import Servo, UART

uart = UART(1, 9600)  # Define UART1
uart.init(9600, bits=8, parity=None, stop=1)  # Initialize with given parameters

i = 0

def receiveSignal():
    global uart
    while uart.any():
        Signal = uart.readchar()
        if Signal == 6:
            return 9
    return None

pan_servo = Servo(1)
tilt_servo = Servo(2)

red_threshold = (65, 99, 8, 41, -1, 38)

redPanList = [80, 220, 220, 80]
redTiltList = [60, 60, 170, 170]

pan_pid = PID(p=0.1, d=0.01, i=0.05, imax=90)  # Adjust PID parameters
tilt_pid = PID(p=0.1, d=0.01, i=0.05, imax=90)  # Adjust PID parameters

sensor.reset()  # Initialize the camera sensor
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_pixformat(sensor.RGB565)  # Use RGB565
sensor.set_framesize(sensor.QVGA)  # Use QVGA resolution
sensor.set_vflip(False)
sensor.skip_frames(10)  # Let the new settings take effect
sensor.set_auto_whitebal(False)  # Turn off auto white balance
clock = time.clock()  # Track the frame rate

def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob[2] * blob[3] > max_size:
            max_blob = blob
            max_size = blob[2] * blob[3]
    return max_blob

def limit_angle(pan_angle, tilt_angle):
    if pan_angle > 15:
        pan_angle = 15
    if pan_angle < -15:
        pan_angle = -15
    if tilt_angle > 15:
        tilt_angle = 15
    if tilt_angle < -15:
        tilt_angle = -15
    return pan_angle, tilt_angle

rx, ry = None, None  # Initialize coordinates of the red laser point

def servo_pid_control(target_x, target_y):
    global rx, ry
    start_time = time.ticks_ms()  # Record the start time
    while True:
        pan_error = target_x - rx
        tilt_error = target_y - ry

        print("pan_error: ", pan_error)
        print("tilt_error: ", tilt_error)

        # Dead Zone
        if abs(pan_error) > 0.005 or abs(tilt_error) > 0.005:
            pan_output = pan_pid.get_pid(pan_error, 1)
            tilt_output = tilt_pid.get_pid(tilt_error, 1)
            print("pan_output", pan_output)
            print("tilt_output", tilt_output)

            pan_angle = int(pan_servo.angle() - pan_output)
            tilt_angle = int(tilt_servo.angle() + tilt_output)

            # Limit angle
            x_angle, y_angle = limit_angle(pan_angle, tilt_angle)
            pan_servo.angle(x_angle)
            tilt_servo.angle(y_angle)
            time.sleep_ms(100)
            print("Servo angles: pan = {}, tilt = {}".format(x_angle, y_angle))
            print("over")

        # Check if already aligned with the target
        if abs(pan_error) <= 0.005 and abs(tilt_error) <= 0.005:
            break

        # Check for timeout (3 seconds)
        if time.ticks_diff(time.ticks_ms(), start_time) > 3000:
            print(":(")
            return False

    return True

while True:
    clock.tick()  # Track time between snapshots
    img = sensor.snapshot()  # Capture an image

    red_blobs = img.find_blobs([red_threshold])
    if red_blobs:
        red_blob = find_max(red_blobs)
        if red_blob:
            rx = int(red_blob.cx())
            ry = int(red_blob.cy())
            print("Detected red laser at: ", rx, ry)

            # Sequentially align with the preset red laser point positions
            if receiveSignal() == 9:
                if i < len(redPanList):
                    target_x = redPanList[i]
                    target_y = redTiltList[i]
                    if not servo_pid_control(target_x, target_y):
                        break  # Exit the main loop if timeout occurs
                    i += 1
                    time.sleep(1000)  # Pause for 1 second to ensure alignment
                else:
                    print("Completed one full cycle.")
                    i = 0  # Reset the cycle if needed
                    # break  # Uncomment this if you want to stop after one cycle
