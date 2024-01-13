from fpioa_manager import fm
from machine import UART
from Maix import GPIO
import sensor, image, time, math

#センサーの設定
sensor.reset(freq = 24000000, set_regs = True, dual_buff = True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(1)
sensor.set_hmirror(1)
sensor.set_windowing((320, 224))

sensor.set_auto_gain(False, gain_db = 0)
sensor.set_auto_whitebal(False, rgb_gain_db = (30, 20, 40))
sensor.set_auto_exposure(False)
sensor.set_contrast(2)
sensor.set_saturation(1)
sensor.run(1)

camera_gain = sensor.get_gain_db()
sensor.set_auto_gain(False, camera_gain)

#定数定義
UART_SPEED = 115200
ANGLE_CONVERSION = 3.55555
ANGLE_Y_CONVERSION = 2.66666

#UART設定
fm.register(10, fm.fpioa.UART1_TX, force = True)
fm.register(11, fm.fpioa.UART1_RX, force = True)

uart = UART(UART.UART1, UART_SPEED, 8, None, 1, timeout = 1000, read_buf_len = 4096)

#LED
fm.register(9, fm.fpioa.GPIO0)
fm.register(17, fm.fpioa.GPIO1)
led_g=GPIO(GPIO.GPIO0, GPIO.OUT)
led_r=GPIO(GPIO.GPIO1, GPIO.OUT)

#各閾値
ball_thresholds = [(0, 100, 27, 58, 28, 80)]
y_goal_thresholds = [(0, 100, -20, 21, 30, 76)]
b_goal_thresholds = [(0, 100, 26, 78, -98, -51)]

color_tracking_roi = [0, 40, 320, 184]

while True:
    img = sensor.snapshot() #映像の取得

    #ボールを見つける
    ball_rectarray = []
    ball_x = 0
    ball_y = 0

    for blob in img.find_blobs(ball_thresholds, roi = color_tracking_roi, pixel_threshold = 100, area_threshold = 10, merge = False):
        ball_rectarray.append(list(blob.rect()))     #見つかった閾値内のオブジェクトをリストに格納

    try:
        ball_maxrect = max(ball_rectarray, key = lambda x: x[1])    #配列の中から一番画面の下にあるものを選定
        ball_x = ball_maxrect[0] + (ball_maxrect[2] / 2)  #中心のx座標の算出
        ball_y = ball_maxrect[1] + (ball_maxrect[3] / 2)  #中心のy座標の算出
        img.draw_circle(int(ball_x), int(ball_y), int((ball_maxrect[2] / 2 + ball_maxrect[3] / 2) / 2))
        img.draw_string(ball_maxrect[0], ball_maxrect[1] - 12, "ball")

    except ValueError as err:   #オブジェクトがひとつも見つからなかった場合の例外処理
        pass

    is_goal_front = 0

    #黄色ゴールを見つける
    y_goal_rectarray = []
    y_goal_x = 0
    y_goal_y = 0
    b_goal_width = 0
    y_goal_hight = 0

    for blob in img.find_blobs(y_goal_thresholds, roi = color_tracking_roi, pixel_threshold = 100, area_threshold = 100, merge = False):
        y_goal_rectarray.append(list(blob.rect()))     #見つかった閾値内のオブジェクトをリストに格納

    try:
        y_goal_maxrect = max(y_goal_rectarray, key = lambda x: x[2] * x[3])    #配列の中から面積の一番大きい物を選定
        y_goal_x = y_goal_maxrect[0] + (y_goal_maxrect[2] / 2)  #中心のx座標の算出
        y_goal_y = y_goal_maxrect[1] + (y_goal_maxrect[3] / 2)  #中心のy座標の算出
        y_goal_width = y_goal_maxrect[2]
        y_goal_hight = y_goal_maxrect[3]
        img.draw_rectangle(y_goal_maxrect)     #オブジェクトを囲う四角形の描画
        img.draw_string(y_goal_maxrect[0], y_goal_maxrect[1] - 12, "yellow goal")

    except ValueError as err:   #オブジェクトがひとつも見つからなかった場合の例外処理
        pass

    #青色ゴールを見つける
    b_goal_rectarray = []
    b_goal_x = 0
    b_goal_y = 0
    b_goal_width = 0
    b_goal_hight = 0

    for blob in img.find_blobs(b_goal_thresholds, roi = color_tracking_roi, pixel_threshold = 100, area_threshold = 100, merge = False):
        b_goal_rectarray.append(list(blob.rect()))     #見つかった閾値内のオブジェクトをリストに格納

    try:
        b_goal_maxrect = max(b_goal_rectarray, key = lambda x: x[2] * x[3])    #配列の中から面積の一番大きい物を選定
        b_goal_x = b_goal_maxrect[0] + (b_goal_maxrect[2] / 2)  #中心のx座標の算出
        b_goal_y = b_goal_maxrect[1] + (b_goal_maxrect[3] / 2)  #中心のy座標の算出
        b_goal_width = b_goal_maxrect[2]
        b_goal_hight = b_goal_maxrect[3]
        img.draw_rectangle(b_goal_maxrect)     #オブジェクトを囲う四角形の描画
        img.draw_string(b_goal_maxrect[0], b_goal_maxrect[1] - 12, "blue goal")

    except ValueError as err:   #オブジェクトがひとつも見つからなかった場合の例外処理
        pass

    #取得した値を変換
    ball_dir = int(ball_x / ANGLE_CONVERSION)
    ball_y_dir = int((ball_y / ANGLE_Y_CONVERSION) - 15)
    ball_y_1 = int(50 * math.log10(150*math.atan(math.radians(ball_y_dir))))
    if ball_y_1 <= 0:
        ball_y_1 = 0

    ball_cos = math.cos(math.radians(ball_dir - 45))
    ball_dis = int(ball_y_1 * ball_cos)
    if ball_dis <= 0:
        ball_dis = 0

    y_goal_dir = int(y_goal_x / ANGLE_CONVERSION)
    y_goal_hight = int(y_goal_hight);

    b_goal_dir = int(b_goal_x / ANGLE_CONVERSION)
    b_goal_hight = int(b_goal_hight);

    if(y_goal_hight > b_goal_hight):
        is_y_goal = 1
        goal_dir = y_goal_dir
        goal_size = y_goal_hight
        if(y_goal_x + (y_goal_width / 2) < 175 or y_goal_x - (y_goal_width / 2) > 145):
            is_goal_front = 0
        else:
            is_goal_front = 1
    else:
        is_y_goal = 0
        goal_dir = b_goal_dir
        goal_size = b_goal_hight
        if(b_goal_x + (b_goal_width / 2) < 175 or b_goal_x - (b_goal_width / 2) > 145):
            is_goal_front = 0
        else:
            is_goal_front = 1

    bool_data = (is_goal_front << 1) | is_y_goal

    #uart
    send_data = bytearray([0xFF, ball_dir, ball_dis, goal_dir, goal_size, bool_data, 0xAA])
    uart.write(send_data)
