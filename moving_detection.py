import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
clock = time.clock()
img2 = sensor.snapshot()

while(True):
    img1 = sensor.snapshot()
    dis = img1.find_displacement(img2)
    print("displacement:")
    print(dis.x_translation())
    img2 = img1.copy()
