import sensor, image, time
from pid import PID
from pyb import Servo, UART

uart = UART(1, 9600)  # Define UART1
uart.init(9600, bits=8, parity=None, stop=1)  # Initialize with given parameters

i = 0
j = 0

pan_servo = Servo(1)
tilt_servo = Servo(2)

red_threshold = (61, 100, 8, 22, -2, 12)

redPanList = []
redTiltList = []

pan_pid = PID(p=0.042, d=0.001, i=0.03, imax=90)  # Adjust PID parameters
tilt_pid = PID(p=0.042, d=0.001, i=0.03, imax=90)  # Adjust PID parameters

rx, ry = None, None  # Initialize coordinates of the red laser point

sensor.reset()  # Initialize the camera sensor
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_pixformat(sensor.RGB565)  # Use RGB565
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
        print("The coordinates of the record are:\n")
        for i in range(4):  # Iterate over the range of recorded points
            if i < len(redPanList) and i < len(redTiltList):
                print("redPanList: {}, redTiltList: {}".format(redPanList[i], redTiltList[i]))


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
    red_blobs = img.find_blobs([red_threshold])
    if red_blobs:
        red_blob = find_max(red_blobs)
        if red_blob:
            rx = int(red_blob.cx())
            ry = int(red_blob.cy())
            print("Updated red laser position: ", rx, ry)
            return True
    print("No red blobs detected.")
    return False


def servo_pid_control(target_x, target_y, target_index):
    global rx, ry
    start_time = time.ticks_ms()  # 记录开始时间
    print("Moving to target:", target_index)
    uart.write("Moving to target: {}\n".format(target_index))

    while True:
        if not update_laser_position():
            if time.ticks_diff(time.ticks_ms(), start_time) > 3000:
                print("Failed to reach the target within 3 seconds.")
                uart.write("Failed to reach the target within 3 seconds.\n")
                return False
            continue

        pan_error = target_x - rx
        tilt_error = target_y - ry

        print("pan_error: ", pan_error)
        print("tilt_error: ", tilt_error)

        # 死区
        if abs(pan_error) <= 2 and abs(tilt_error) <= 2:
            print("Reached within deadzone.")
            uart.write("Reached within deadzone.\n")
            return True  # 当达到死区内时，跳出循环并返回True

        pan_output = pan_pid.get_pid(pan_error, 1)
        tilt_output = tilt_pid.get_pid(tilt_error, 1)
        print("pan_output", pan_output)
        print("tilt_output", tilt_output)

        pan_angle = int(pan_servo.angle() - pan_output)
        tilt_angle = int(tilt_servo.angle() + tilt_output)

        # 限制角度
        x_angle, y_angle = limit_angle(pan_angle, tilt_angle)
        pan_servo.angle(x_angle)
        tilt_servo.angle(y_angle)
        time.sleep_ms(100)
        print("Servo angles: pan = {}, tilt = {}".format(x_angle, y_angle))
        print("\n ")

        # 检查超时 (3秒)
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

    red_blobs = img.find_blobs([red_threshold])
    if red_blobs:
        red_blob = find_max(red_blobs)
        if red_blob:
            rx = int(red_blob.cx())
            ry = int(red_blob.cy())
            #            print("Detected red laser at: ", rx, ry)

            # Sequentially align with the preset red laser point positions
            if Signal == 9:
                #                while i < len(redPanList):
                for i in range(len(redPanList)):
                    #                 if i < len(redPanList):
                    target_x = redPanList[i]
                    target_y = redTiltList[i]
                    if not servo_pid_control(target_x, target_y, i):
                        break  # Exit the main loop if timeout occurs
                    print("loop {} is over".format(i))
                    # time.sleep(100)  # Pause for 1 second to ensure alignment

                # back to orri
                target_x = redPanList[0]
                target_y = redTiltList[0]
                if not servo_pid_control(target_x, target_y, 0):
                    break  # Exit the main loop if timeout occurs
                print("Mission accomplished")

                # else:
                #     print("Completed one full cycle.")
                #     i = 0  # Reset the cycle if needed
                #     # break  # Uncomment this if you want to stop after one cycle
#    else:
#        print("No red blobs detected.")

