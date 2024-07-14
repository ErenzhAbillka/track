import sensor, image, time
from pid import PID
from pyb import Servo

pan_servo = Servo(1)
tilt_servo = Servo(2)

# pan_servo.calibration(500,2500,500)
# tilt_servo.calibration(500,2500,500)

grn_threshold = (91, 100, -36, 1, -3, 30)

pan_pid = PID(p=0.037, d=0.015, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
tilt_pid = PID(p=0.035, d=0.013, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
# pan_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID
# tilt_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID

sensor.reset() # Initialize the camera sensor.
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QVGA) # use QQVGA for speed.
sensor.set_vflip(False)
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
clock = time.clock() # Tracks FPS.
# face_cascade = image.HaarCascade("frontalface", stages=25)

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

while True:
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    blobs = img.find_blobs([grn_threshold])
    if blobs:
        blob = find_max(blobs)
        if blob:
            cx = int(blob[0] + blob[2] / 2)
            cy = int(blob[1] + blob[3] / 2)
            pan_error = cx - img.width() / 2
            tilt_error = cy - img.height() / 2 + 5

            print("pan_error: ", pan_error)

            img.draw_rectangle(blob.rect()) # rect
            img.draw_cross(cx, cy) # cx, cy

            pan_output = pan_pid.get_pid(pan_error, 1)
            tilt_output = tilt_pid.get_pid(tilt_error, 1)
            print("pan_output", pan_output)

            panAngle  = int(pan_servo.angle() - pan_output)
            tiltAngle = int(tilt_servo.angle() + tilt_output)

            # 限幅
            x_angle, y_angle = limmitAngle(panAngle, tiltAngle)
            pan_servo.angle(x_angle)
            tilt_servo.angle(y_angle)
