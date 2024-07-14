import sensor, image, time
from pyb import Servo, UART

pan_servo  = Servo(1)
tilt_servo = Servo(2)

uart = UART(1, 9600)  # 定义串口3变量
uart.init(9600, bits=8, parity=None, stop=1)  # init with given parameters

red_threshold = (66, 100, 6, 127, -128, 127)  # 红色激光阈值
redPanList  = []
redTiltList = []

sensor.reset() # 初始化摄像头传感器
sensor.set_contrast(3) # 相机对比度
sensor.set_gainceiling(16) # 设置相机图像增益上限
sensor.set_pixformat(sensor.RGB565) # 使用RGB565
sensor.set_framesize(sensor.QVGA) # 使用QVGA分辨率
sensor.set_vflip(False) # 垂直翻转模式
sensor.skip_frames(10) # 让新设置生效
sensor.set_auto_whitebal(False) # 关闭自动白平衡
clock = time.clock() # 追踪帧率

def receiveSignal():
    global uart, redPanList, redTiltList
    count = 0
    while uart.any():
        Signal = uart.readchar()
        if Signal == 114:
            if count < 5:
                redPanList.append(redx)
                redTiltList.append(redy)
                count = count + 1
                print(":)" + count)
            if count > 5:
                print(":(")
        if Signal == 514:
            traceRectangles(0)

def servoControl(targetx, targety):
    if targetx < redx:
        panAngle = int(pan_servo.angle() - 1)
        limmitAngle(panAngle)
        pan_servo.angle(panAngle)
    if targetx > redx:
        panAngle = int(pan_servo.angle() + 1)
        limmitAngle(panAngle)
        pan_servo.angle(panAngle)
    if targety < redy:
        tiltAngle = int(tilt_servo.angle() - 1)
        limmitAngle(tiltAngle)
        pan_servo.angle(tiltAngle)
    if targety > redy:
        tiltAngle = int(tilt_servo.angle() + 1)
        limmitAngle(tiltAngle)
        pan_servo.angle(tiltAngle)

def limmitAngle(Angle):
    if Angle > 15:
        limmited = 15
    if Angle < -15:
        limmited = -15
    return limmited

def traceRectangles(state):
    for i in range(0, 5):
        if (state == i):
            tiltAngle(redPanList[i], redTiltList[i])
            time.sleep(0.5)

def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob[2] * blob[3] > max_size:
            max_blob = blob
            max_size = blob[2] * blob[3]
    return max_blob

while True:
    clock.tick() # 记录快照之间的时间
    img = sensor.snapshot() # 拍照并返回图像
    receiveSignal()

    redBlobs = img.find_blobs([red_threshold])
    if redBlobs:
        # 如果找到了目标颜色
        max_b = find_max(redBlobs)
        # Draw a rect around the blob.
        img.draw_rectangle(max_b[0:4])  # rect
        #img.draw_rectangle(max_green_blob.rect())  # rect
        # 用矩形标记出目标颜色区域
        img.draw_cross(max_b[5], max_b[6])  # cx, cy
        # 在目标颜色区域的中心画十字形标记
        redx = max_b[5]
        redy = max_b[6]
    #____END____