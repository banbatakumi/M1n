from fpioa_manager import fm
from machine import UART
from Maix import GPIO
import sensor, image, time, math

sensor.reset()  #センサーの設定
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(1)
sensor.set_hmirror(1)

sensor.set_auto_gain(False, gain_db=25)
sensor.set_auto_whitebal(False, rgb_gain_db=(30, 20, 40))
sensor.set_auto_exposure(False)
sensor.skip_frames(25)

sensor.run(1)

#定数定義
UART_SPEED = 115200
ANGLE_CONVERSION = 3.55555

#UART設定
fm.register(10, fm.fpioa.UART1_TX, force=True)
fm.register(11, fm.fpioa.UART1_RX, force=True)

uart = UART(UART.UART1, UART_SPEED, 8, None, 1, timeout=1000, read_buf_len=4096)

#LED
fm.register(9,fm.fpioa.GPIO0)
fm.register(17,fm.fpioa.GPIO1)
led_g=GPIO(GPIO.GPIO0,GPIO.OUT)
led_r=GPIO(GPIO.GPIO1,GPIO.OUT)

for num in range(10):
    led_g.value(1)
    time.sleep(0.05)
    led_g.value(0)
    time.sleep(0.05)
for num in range(2):
    led_r.value(1)
    time.sleep(0.1)
    led_r.value(0)
    time.sleep(0.1)

#各閾値設定
ball_thresholds = [[0] * 6]
y_goal_thresholds = [[0] * 6]
b_goal_thresholds = [[0] * 6]

while True:
    img = sensor.snapshot() #映像の取得

    #ボールを見つける
    ball_maxrect = 0
    ball_rectarray = []
    ball_x = 0
    ball_y = 0

    for blob in img.find_blobs(ball_thresholds, pixel_threshold = 100, area_threshold = 20, merge = False):
        if(blob[1] + (blob[3] / 2) > 30):
            ball_rectarray.append(list(blob.rect()))     #見つかった閾値内のオブジェクトをリストに格納

    try:
        ball_maxrect = max(ball_rectarray, key = lambda x: x[1])    #配列の中から一番画面の下にあるものを選定
        ball_x = ball_maxrect[0] + (ball_maxrect[2] / 2)  #中心のx座標の算出
        ball_y = ball_maxrect[1] + (ball_maxrect[3] / 2)  #中心のy座標の算出
        img.draw_circle(int(ball_x), int(ball_y), int((ball_maxrect[2] / 2 + ball_maxrect[3] / 2) / 2))
        img.draw_string(ball_maxrect[0], ball_maxrect[1] - 12, "ball")

    except ValueError as err:   #オブジェクトがひとつも見つからなかった場合の例外処理
        pass

    #黄色ゴールを見つける
    y_goal_maxrect = 0
    y_goal_rectarray = []
    y_goal_x = 0
    y_goal_y = 0
    y_goal_size = 0

    for blob in img.find_blobs(y_goal_thresholds, pixel_threshold = 100, area_threshold = 100, merge = True, margin = 10):
        if(blob[1] + (blob[3] / 2) > 30):
            y_goal_rectarray.append(list(blob.rect()))     #見つかった閾値内のオブジェクトをリストに格納

    try:
        y_goal_maxrect = max(y_goal_rectarray, key = lambda x: x[2] * x[3])    #配列の中から面積の一番大きい物を選定
        y_goal_x = y_goal_maxrect[0] + (y_goal_maxrect[2] / 2)  #中心のx座標の算出
        y_goal_y = y_goal_maxrect[1] + (y_goal_maxrect[3] / 2)  #中心のy座標の算出
        y_goal_size = y_goal_maxrect[3]
        img.draw_rectangle(y_goal_maxrect)     #オブジェクトを囲う四角形の描画
        img.draw_cross(int(y_goal_x), int(y_goal_y))
        img.draw_string(y_goal_maxrect[0], y_goal_maxrect[1] - 12, "yellow goal")

    except ValueError as err:   #オブジェクトがひとつも見つからなかった場合の例外処理
        pass

    #青色ゴールを見つける
    b_goal_maxrect = 0
    b_goal_rectarray = []
    b_goal_x = 0
    b_goal_y = 0
    b_goal_size = 0

    for blob in img.find_blobs(b_goal_thresholds, pixel_threshold = 100, area_threshold = 100, merge = True, margin = 10):
       if(blob[1] + (blob[3] / 2) > 30):
            b_goal_rectarray.append(list(blob.rect()))     #見つかった閾値内のオブジェクトをリストに格納

    try:
        b_goal_maxrect = max(b_goal_rectarray, key = lambda x: x[2] * x[3])    #配列の中から面積の一番大きい物を選定
        b_goal_x = b_goal_maxrect[0] + (b_goal_maxrect[2] / 2)  #中心のx座標の算出
        b_goal_y = b_goal_maxrect[1] + (b_goal_maxrect[3] / 2)  #中心のy座標の算出
        b_goal_size = b_goal_maxrect[3]
        img.draw_rectangle(b_goal_maxrect)     #オブジェクトを囲う四角形の描画
        img.draw_cross(int(b_goal_x), int(b_goal_y))
        img.draw_string(b_goal_maxrect[0], b_goal_maxrect[1] - 12, "blue goal")

    except ValueError as err:   #オブジェクトがひとつも見つからなかった場合の例外処理
        pass

    #取得した値を変換
    ball_dir = int(ball_x / ANGLE_CONVERSION)
    ball_dis_diff_1 = ball_y * math.sin(math.radians((ball_dir - 45) / 2)) * 2
    ball_dis_diff = ball_dis_diff_1 * math.sin(math.radians((ball_dir - 45) / 2))
    ball_dis = int(ball_y - ball_dis_diff)

    y_goal_dir = int(y_goal_x / ANGLE_CONVERSION)
    y_goal_size = int(y_goal_size);

    b_goal_dir = int(b_goal_x / ANGLE_CONVERSION)
    b_goal_size = int(b_goal_size);

    #uart
    if(uart.any()):
        led_g.value(1)
        if(uart.readchar() == 0xFF):
            recv_byte = [0] * 13
            for i in range(13):
                recv_byte[i] = uart.readchar()

            if(recv_byte[12] == 0xAA):
                ball_a_min = recv_byte[0] - 127
                ball_a_max = recv_byte[1] - 127
                ball_b_min = recv_byte[2] - 127
                ball_b_max = recv_byte[3] - 127
                y_goal_a_min = recv_byte[4] - 127
                y_goal_a_max = recv_byte[5] - 127
                y_goal_b_min = recv_byte[6] - 127
                y_goal_b_max = recv_byte[7] - 127
                b_goal_a_min = recv_byte[8] - 127
                b_goal_a_max = recv_byte[9] - 127
                b_goal_b_min = recv_byte[10] - 127
                b_goal_b_max = recv_byte[11] - 127

                ball_thresholds = [(0, 100, ball_a_min, ball_a_max, ball_b_min, ball_b_max)]
                y_goal_thresholds = [(0, 100, y_goal_a_min, y_goal_a_max, y_goal_b_min, y_goal_b_max)]
                b_goal_thresholds = [(0, 100, b_goal_a_min, b_goal_a_max, b_goal_b_min, b_goal_b_max)]
        led_g.value(0)
    else:
        send_data = bytearray([0xFF, ball_dir, ball_dis, y_goal_dir, y_goal_size, b_goal_dir, b_goal_size, 0xAA])
        uart.write(send_data)

uart.deinit()
del uart
