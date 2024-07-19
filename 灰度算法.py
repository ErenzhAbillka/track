import sensor, image, time
from pid import PID
from pyb import Servo, UART

uart = UART(1, 9600)  # Define UART1
uart.init(9600, bits=8, parity=None, stop=1)  # Initialize with given parameters

i = 0
j = 0

pan_servo = Servo(1)
tilt_servo = Servo(2)

red_thresholds = [(61, 100, 8, 22, -2, 12),
                  (0, 100, 12, 34, -28, 36)]

redPanList = []
redTiltList = []
rectanglePanList = []
rectangleTiltList = []

pan_pid = PID(p=0.028, d=0.012, i=0.2, imax=90)  # Adjust PID parameters
tilt_pid = PID(p=0.028, d=0.012, i=0.2, imax=90)  # Adjust PID parameters

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

# 定义一个全局变量来跟踪暂停状态
paused = False

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
        # theta_values = [0x2C, 4, redPanList[0], redTiltList[0], redPanList[1], redTiltList[1],
        #                          redPanList[2], redTiltList[2], redPanList[3], redTiltList[3],0x5B]
        # data = bytes(theta_values)
        # uart.write(data)
        j += 1
    elif j >= 5:
        print(":(")


def check_rectangle():
    global img, rectanglePanList, rectangleTiltList
    for r in img.find_rects(threshold=10000):
        # 绘制矩形轮廓
        img.draw_rectangle(r.rect(), color=(255, 0, 0))

        # 获取并绘制矩形角点
        corners = r.corners()
        corners = sorted(corners, key=lambda c: (c[1], c[0]))  # 按 y 坐标排序，如果 y 坐标相同按 x 坐标排序

        # 左上角和右上角
        if corners[0][0] > corners[1][0]:
            corners[0], corners[1] = corners[1], corners[0]

        # 左下角和右下角
        if corners[2][0] > corners[3][0]:
            corners[2], corners[3] = corners[3], corners[2]

        for g in range(4):
            rectanglePanList.append(corners[g][0])
        for h in range(4):
            rectangleTiltList.append(corners[h][1])
            print("rectanglePanList: {}, rectangleTiltList: {}".format(rectanglePanList[h], rectangleTiltList[h]))
        # theta_values = [0x2C, 4, rectanglePanList[0], rectangleTiltList[0], rectanglePanList[1], rectangleTiltList[1],
        #                          rectanglePanList[2], rectangleTiltList[2], rectanglePanList[3], rectangleTiltList[3],0x5B]
        # data = bytes(theta_values)
        # uart.write(data)


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
    red_blobs = img.find_blobs(red_thresholds)
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
                # uart.write(14)
                return False
            continue

        pan_error = target_x - rx
        tilt_error = target_y - ry

        print("pan_error: ", pan_error)
        print("tilt_error: ", tilt_error)

        # 死区
        if abs(pan_error) <= 5 and abs(tilt_error) <= 5:
            print("Reached within deadzone.")
            # uart.write(13)
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
            # uart.write(14)
            return False

    return True


while True:
    clock.tick()  # Track time between snapshots
    img = sensor.snapshot().lens_corr(strength=1.8, zoom=1.0)  # Capture an image
#    img_cropped = img.crop((75, 30, 260, 200))

    uart.any()
    Signal = uart.readchar()

    if Signal == 6:
        record(rx, ry)

    if Signal == 8:
        check_rectangle()

    # 如果接收到字符 5，切换暂停状态
    if Signal == 5:
        paused = not paused
        if paused:
            print("Paused")
            # uart.write("Paused\n")
        else:
            print("Resumed")
            # uart.write("Resumed\n")

    # 仅在未暂停状态下运行舵机控制代码
    if not paused:
        red_blobs = img.find_blobs(red_thresholds)
        if red_blobs:
            red_blob = find_max(red_blobs)
            if red_blob:
                rx = int(red_blob.cx())
                ry = int(red_blob.cy())
                # print("Detected red laser at: ", rx, ry)

                # Sequentially align with the preset red laser point positions
                if Signal == 7:
                    for i in range(len(redPanList)):
                        target_x = redPanList[i]
                        target_y = redTiltList[i]
                        if not servo_pid_control(target_x, target_y, i):
                            break  # Exit the main loop if timeout occurs
                        print("loop {} is over".format(i))
                    # back to orri
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

    #    else:
    #        print("No red blobs detected.")
