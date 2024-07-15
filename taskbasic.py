import sensor, image, time
from pyb import Servo, UART

pan_servo = Servo(1)
tilt_servo = Servo(2)

uart = UART(1, 9600)  # 定义串口3变量
uart.init(9600, bits=8, parity=None, stop=1)  # init with given parameters

red_threshold = (66, 100, 6, 127, -128, 127)  # 红色激光阈值
redPanList = []
redTiltList = []

count = 0
leftUpX   , leftUpY    = 0, 0
rightUpX  , rightUpY   = 0, 0
rightDownX, rightDownY = 0, 0
leftDownX , leftDownY  = 0, 0

sensor.reset()  # 初始化摄像头传感器
sensor.set_contrast(3)  # 相机对比度
sensor.set_gainceiling(16)  # 设置相机图像增益上限
sensor.set_pixformat(sensor.RGB565)  # 使用RGB565
sensor.set_framesize(sensor.QVGA)  # 使用QVGA分辨率
sensor.set_vflip(False)  # 垂直翻转模式
sensor.skip_frames(10)  # 让新设置生效
sensor.set_auto_whitebal(False)  # 关闭自动白平衡
clock = time.clock()  # 追踪帧率


def receiveSignal():
    global uart, redPanList, redTiltList
    while uart.any():
        Signal = uart.readchar()
        if Signal == 5:
            pan_servo.angle(0)
            tilt_servo.angle(0)

        if Signal == 6:
            if count < 4:
                redPanList.append(redx)
                redTiltList.append(redy)
                count += 1
                if count >= 4:
                    print(":(")

        if Signal == 8:
            traceRectangles(0)

        if Signal == 9:
            check_rectangle()


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
    if state <= 3:
        for i in range(0, 3):
            if (state == i):
                servoControl(redPanList[i], redTiltList[i])
                time.sleep(0.7)
        state = 4
    if state == 4:
        servoControl(redPanList[0], redTiltList[0])
        state = 5
    if state == 5:
        pan_servo.angle(0)
        tilt_servo.angle(0)

def check_rectangle():
    global img, leftUpX, leftUpY, rightUpX, rightUpY, rightDownX, rightDownY, leftDownX, leftDownY
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

       labels = ['zx1', 'zy2', 'zx2', 'zy1']  # 顺时针顺序：左上、右上、右下、左下
       for i, corner in enumerate(corners):
           img.draw_circle(corner[0], corner[1], 5, color=(0, 255, 0))
           print(f"{labels[i]}: ({corner[0]}, {corner[1]})")

       leftUpX   , leftUpY    = corners[0][0], corners[0][1]
       rightUpX  , rightUpY   = corners[1][0], corners[1][1]
       rightDownX, rightDownY = corners[3][0], corners[3][1]
       leftDownX , leftDownY  = corners[2][0], corners[2][1]

def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob[2] * blob[3] > max_size:
            max_blob = blob
            max_size = blob[2] * blob[3]
    return max_blob


while True:
    clock.tick()  # 记录快照之间的时间
    img = sensor.snapshot()  # 拍照并返回图像
    receiveSignal()

    redBlobs = img.find_blobs([red_threshold])
    if redBlobs:
        # 如果找到了目标颜色
        max_b = find_max(redBlobs)
        # Draw a rect around the blob.
        img.draw_rectangle(max_b[0:4])  # rect
        # img.draw_rectangle(max_green_blob.rect())  # rect
        # 用矩形标记出目标颜色区域
        img.draw_cross(max_b[5], max_b[6])  # cx, cy
        # 在目标颜色区域的中心画十字形标记
        redx = max_b[5]
        redy = max_b[6]
    # ____END____