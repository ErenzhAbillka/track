import sensor, image, time
from pid import PID
from pyb import Servo, UART

uart = UART(1, 9600)  # 定义串口3变量
uart.init(9600, bits=8, parity=None, stop=1)  # init with given parameters

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

pan_pid = PID(p=0.1, d=0.01, i=0.05, imax=90)  # 调整PID参数
tilt_pid = PID(p=0.1, d=0.01, i=0.05, imax=90)  # 调整PID参数

sensor.reset()  # 初始化摄像头传感器
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_pixformat(sensor.RGB565)  # 使用RGB565
sensor.set_framesize(sensor.QVGA)  # 使用QVGA分辨率
sensor.set_vflip(False)
sensor.skip_frames(10)  # 让新设置生效
sensor.set_auto_whitebal(False)  # 关闭自动白平衡
clock = time.clock()  # 追踪帧率


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


rx, ry = None, None  # 初始化红色激光点的坐标


def servo_pid_control(target_x, target_y):
    global rx, ry
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

            # 限幅
            x_angle, y_angle = limit_angle(pan_angle, tilt_angle)
            pan_servo.angle(x_angle)
            tilt_servo.angle(y_angle)
            time.sleep_ms(100)
            print("Servo angles: pan = {}, tilt = {}".format(x_angle, y_angle))
            print("over")

        # 检查是否已经对准目标
        if abs(pan_error) <= 0.005 and abs(tilt_error) <= 0.005:
            break


while True:
    clock.tick()  # 记录快照之间的时间
    img = sensor.snapshot()  # 拍照并返回图像

    red_blobs = img.find_blobs([red_threshold])
    if red_blobs:
        red_blob = find_max(red_blobs)
        if red_blob:
            rx = int(red_blob.cx())
            ry = int(red_blob.cy())
            print("Detected red laser at: ", rx, ry)

            # 依次对准四个预设的红色激光点位置
            if receiveSignal() == 9:
                if i < len(redPanList):
                    pan_error = 0
                    tilt_error = 0
                    target_x = redPanList[i]
                    target_y = redTiltList[i]
                    servo_pid_control(target_x, target_y)
                    i += 1
                    time.sleep(1)  # 等待一段时间以确保对准
                else:
                    print("Completed one full cycle.")
                    break  # 完成一圈后停止