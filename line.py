import sensor, image

#略　カメラ初期化処理

sensor.reset()  #センサーの設定
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(1)
sensor.set_vflip(1)
sensor.set_hmirror(1)

sensor.set_brightness(3)
sensor.set_auto_whitebal(1)
sensor.set_auto_exposure(0)
sensor.set_auto_gain(0)
sensor.run(1)


while True:
    img = sensor.snapshot()
    img2 = img.copy();                                   #ライン認識用の画像としてコピー
    img2 = img2.draw_rectangle(0,0,160,80,fill = True)   #ライン認識用の画像の上1/3をマスク

    linep = img2.get_regression([(0, 23, -19, 33, -45, 17)], pixels_threshold = 100,robust=True)  #最小二乗法で認識した線分の線形近似を取得



    print(linep)

    if linep:
        print(linep.line())                     #読み取ったラインのパラメータをシリアルモニタで確認
        img.draw_line(linep.line())             #読み取ったラインをシリアルモニタに表意ⓙ
        cx = int((linep.x1()+linep.x2())/2 - 80)   #ラインの横方向のセンター値
        cy = int((linep.y1()+linep.y2())/2 - 60)   #ラインの縦方向のセンター値
