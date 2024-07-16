import sensor, image, time
from pid import PID
from pyb import Servo, UART

uart = UART(1, 9600)  # Define UART1
uart.init(9600, bits=8, parity=None, stop=1)  # Initialize with given parameters

i = 0
j = 0

pan_servo = Servo(1)
tilt_servo = Servo(2)

gray_thresholds = [(207, 255)]  # Define gray thresholds for blob detection

redPanList = []
redTiltList = []
rectanglePanList = []
rectangleTiltList = []

pan_pid = PID(p=0.07, d=0.015, i=0.05, imax=90)  # Adjust PID parameters
tilt_pid = PID(p=0.07, d=0.015, i=0.05, imax=90)  # Adjust PID parameters

rx, ry = None, None  # Initialize coordinates of the red laser point

sensor.reset()  # Initialize the camera sensor
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_pixformat(sensor.GRAYSCALE)  # Use grayscale mode
sensor.set_framesize(sensor.QVGA)  # Use QVGA resolution
sensor.set_vflip(False)
sensor.skip_frames(10)  # Let the new settings take effect
sensor.set_auto_whitebal(False)  # Turn off auto white balance
clock = time.clock()  # Track the frame rate


def record(rx, ry):
    global j
    if j < 4:
        print("Record " + str(j))
        redPanList.append(rx)
        redTiltList.append(ry)  # Append to redTiltList here if necessary
        j += 1
    elif j == 4:
        print("The coordinates of the record are:")
        for i in range(4):  # Iterate over the range of recorded points
            if i < len(redPanList) and i < len(redTiltList):
                print("redPanList: {}, redTiltList: {}".format(redPanList[i], redTiltList[i]))


def check_rectangle():
    global img, rectanglePanList, rectangleTiltList
    for r in img.find_rects(threshold=10000):
        # Draw rectangle outline
        img.draw_rectangle(r.rect(), color=(255, 0, 0))

        # Get and draw rectangle corners
        corners = r.corners()
        corners = sorted(corners, key=lambda c: (c[1], c[0]))  # Sort by y coordinate, then by x coordinate

        # Left top and right top
        if corners[0][0] > corners[1][0]:
            corners[0], corners[1] = corners[1], corners[0]

        # Left bottom and right bottom
        if corners[2][0] > corners[3][0]:
            corners[2], corners[3] = corners[3], corners[2]

        for g in range(4):
            rectanglePanList.append(corners[g][0])
        for h in range(4):
            rectangleTiltList.append(corners[h][1])
            print("rectanglePanList: {}, rectangleTiltList: {}".format(rectanglePanList[h], rectangleTiltList[h]))


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


def update_laser_position():
    global rx, ry
    img = sensor.snapshot().lens_corr(strength=1.8, zoom=1.0)
    blobs = img.find_blobs(gray_thresholds)
    if blobs:
        blob = find_max(blobs)
        if blob:
            rx = int(blob.cx())
            ry = int(blob.cy())
            print("Updated laser position: ", rx, ry)
            return True
    print("No blobs detected.")
    return False


def servo_pid_control(target_x, target_y, target_index):
    global rx, ry
    start_time = time.ticks_ms()  # Record start time
    print("Moving to target:", target_index)
    uart.write("Moving to target: {}\n".format(target_index))

    while True:
        if not update_laser_position():
            if time.ticks_diff(time.ticks_ms(), start_time) > 9000:
                print("Failed to reach the target within 3 seconds.")
                uart.write("Failed to reach the target within 3 seconds.\n")
                return False
            continue

        pan_error = target_x - rx
        tilt_error = target_y - ry

        print("pan_error: ", pan_error)
        print("tilt_error: ", tilt_error)

        # Deadzone
        if abs(pan_error) <= 2 and abs(tilt_error) <= 2:
            print("Reached within deadzone.")
            uart.write("Reached within deadzone.\n")
            return True

        pan_output = pan_pid.get_pid(pan_error, 1)
        tilt_output = tilt_pid.get_pid(tilt_error, 1)
        print("pan_output", pan_output)
        print("tilt_output", tilt_output)

        pan_angle = int(pan_servo.angle() - pan_output)
        tilt_angle = int(tilt_servo.angle() + tilt_output)

        # Limit angles
        x_angle, y_angle = limit_angle(pan_angle, tilt_angle)
        pan_servo.angle(x_angle)
        tilt_servo.angle(y_angle)
        time.sleep_ms(100)
        print("Servo angles: pan = {}, tilt = {}".format(x_angle, y_angle))
        print("\n ")

        # Timeout check (3 seconds)
        if time.ticks_diff(time.ticks_ms(), start_time) > 3000:
            print("Failed to reach the target within 3 seconds.")
            uart.write("Failed to reach the target within 3 seconds.\n")
            return False

    return True


while True:
    clock.tick()  # Track time between snapshots
    img = sensor.snapshot().lens_corr(strength=1.8, zoom=1.0)  # Capture an image
    uart.any()
    Signal = uart.readchar()

    if Signal == 6:
        record(rx, ry)

    if Signal == 8:
        check_rectangle()

    blobs = img.find_blobs(gray_thresholds)
    if blobs:
        blob = find_max(blobs)
        if blob:
            rx = int(blob.cx())
            ry = int(blob.cy())
            # Sequentially align with the preset points
            if Signal == 7:
                for i in range(len(redPanList)):
                    target_x = redPanList[i]
                    target_y = redTiltList[i]
                    if not servo_pid_control(target_x, target_y, i):
                        break  # Exit the main loop if timeout occurs
                    print("loop {} is over".format(i))
                # back to original position
                target_x = redPanList[0]
                target_y = redTiltList[0]
                if not servo_pid_control(target_x, target_y, 0):
                    break  # Exit the main loop if timeout occurs
                print("Mission accomplished")


            if Signal == 9:
                for i in range(len(rectanglePanList)):
                    target_x = rectanglePanList[i]
                    target_y = rectangleTiltList[i]
                    if not servo_pid_control(target_x, target_y, i):
                        break  # Exit the main loop if timeout occurs
                    print("loop {} is over".format(i))
                # back to orri
                target_x = rectanglePanList[0]
                target_y = rectangleTiltList[0]
                if not servo_pid_control(target_x, target_y, 0):
                    break  # Exit the main loop if timeout occurs
                print("Mission accomplished")
