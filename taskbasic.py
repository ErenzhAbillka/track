import sensor, image, time
from pid import PID
from pyb import Servo, UART

pan_servo = Servo(1)
tilt_servo = Servo(2)

uart = UART(1, 9600)  # 定义串口3变量
uart.init(9600, bits=8, parity=None, stop=1)  # init with given parameters

red_threshold = (65, 99, 8, 41, -1, 38)
redPanList = []
redTiltList = []
count = 0

pan_pid = PID(p=0.042, d=0.001, i=0.03, imax=90)  # 脱机运行或者禁用图像传输，使用这个PID
tilt_pid = PID(p=0.042, d=0.001, i=0.03, imax=90)  # 脱机运行或者禁用图像传输，使用这个PID
# pan_pid = PID(p=0.1, i=0, imax=90) # 在线调试使用这个PID
# tilt_pid = PID(p=0.1, i=0, imax=90) # 在线调试使用这个PID

sensor.reset()  # 初始化摄像头传感器
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_pixformat(sensor.RGB565)  # 使用RGB565
sensor.set_framesize(sensor.QVGA)  # 使用QVGA分辨率
sensor.set_vflip(False)
sensor.skip_frames(10)  # 让新设置生效
sensor.set_auto_whitebal(False)  # 关闭自动白平衡
clock = time.clock()  # 追踪帧率


def receiveSignal():
    global uart, redPanList, redTiltList, count
    while uart.any():
        Signal = uart.readchar()
        if Signal == 6:
            if count < 4:
                redPanList.append(rx)
                redTiltList.append(ry)
                count += 1
                if count == 4:
                    for i in range(4):
                        print(redPanList[i], redTiltList[i])
        if Signal == 7:
            servoPIDControl()


def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob[2] * blob[3] > max_size:
            max_blob = blob
            max_size = blob[2] * blob[3]
    return max_blob


def limmitAngle(panAngle, tiltAngle):
    if panAngle > 15:
        panAngle = 15
    if panAngle < -15:
        panAngle = -15
    if tiltAngle > 15:
        tiltAngle = 15
    if tiltAngle < -15:
        tiltAngle = -15
    return panAngle, tiltAngle


rx, ry = None, None  # 初始化红色激光点的坐标


def servoPIDControl():
    for i in range(4):
        cx = redPanList[i]
        cy = redTiltList[i]
        pan_error = cx - rx
        tilt_error = cy - ry

        print("pan_error: ", pan_error)

        # Dead Zone
        if abs(pan_error) > 0.005:
            pan_output = pan_pid.get_pid(pan_error, 1)
            tilt_output = tilt_pid.get_pid(tilt_error, 1)
            print("pan_output", pan_output)

            panAngle = int(pan_servo.angle() - pan_output)
            tiltAngle = int(tilt_servo.angle() + tilt_output)

            # 限幅
            x_angle, y_angle = limmitAngle(panAngle, tiltAngle)
            pan_servo.angle(x_angle)
            tilt_servo.angle(y_angle)
        time.sleep(0.5)


while True:
    clock.tick()  # 记录快照之间的时间
    img = sensor.snapshot()  # 拍照并返回图像

    redBlobs = img.find_blobs([red_threshold])
    if redBlobs:
        redBlob = find_max(redBlobs)
        if redBlob:
            rx = int(redBlob[0] + redBlob[2] / 2)
            ry = int(redBlob[1] + redBlob[3] / 2)

    receiveSignal()