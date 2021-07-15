# これはロボットキットTakiRoboF1のプログラムです。

プログラムはテンプレートに名前をつけて保存してから使ってください。
以下はこのライブラリに含まれる関数の説明です。
1. xは任意の名前とします。
1. robot x(float median_x, float median_y, float Scale);
    - これはプログラムの中に必ず記述する必要があります。引数(()のなかに書くもの)はこのライブラリに含まれるcalibrationという名前のプログラムを実行することで得ることができます。キャリブレーションの方法に関しては取扱説明書を参照してください。
    - 一応、キャリブレーションを行わずに動かすこともできます。その場合はrobot xだけを記述してください。この場合はコンパスが予想外の動きをする可能性もあるのでキャリブレーションすることをおすすめします。
1. x.getUSS();
    - 超音波センサの値を返します。
1. x.motor(spd1,spd2,spd3);
    - モーターの制御を行います。spd1,spd2,spd3はそれぞれモーター1,2,3のスピードを表しています。-255〜255までの値が有効です。マイナスは反対方向に回ります。
1. x.getLine(int num);
    - ラインセンサの値を取得します。取得したいラインセンサの番号を入力してください。ラインセンサはロボットの前から時計回りに1~4までの番号が割り振られています。
1. x.getIr(int num);
    - 赤外線センサの値を返します。取得したい赤外線センサの番号を入力してください。赤外線センサはロボットの前から時計回りに1~4までの番号が割り振られています。
1. x.irUpdate();
    - 赤外線センサの値を更新します。ループ毎に一度読み込んでください。
1. x.lineUpdate();
    - ラインセンサの値を更新します。ループ毎に一度読み込んでください。
1. x.getStartingAzimuth();
    - 電源スイッチを押した時点での機体の方位を返します。
1. x.getAzimuth();
    - 現在の方位を出力します。
1. x.init();
    - ピンやi2c、シリアルポートなどの初期化を行います。必ずsetup()内に記述してください。


