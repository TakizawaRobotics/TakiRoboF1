# これはロボットキットTakiRoboF1のプログラムです。

プログラムはテンプレートに名前をつけて保存してから使ってください。
以下はこのライブラリに含まれる関数の説明です。
1. xは任意の名前とします。スケッチ例ではrobotとしています。
1. スケッチ例templateの3行目のプログラム(takiroboF1 robot(x,y,s);)について
    - これはプログラムの中に必ず記述する必要があります。引数(()のなかに書くもの)には以下の４つの書き方があります。
        1. `takiroboF1 x;` <a id="1"></a>
            - これを記述した場合、電源を入れる毎にキャリブレーションを行う必要があります。
            - キャリブレーション方法については[キャリブレーションの方法について](#caliblation2)を参照してください。
        1. `int hoge[3]={0,0,0};`  
           `takiroboF1 x(hoge);`  
            - これを記述した場合、電源を入れる毎にキャリブレーションを行う必要があります。
            - キャリブレーション方法については[キャリブレーションの方法について](#caliblation2)を参照してください。
            - [1](#1)との違いは外部入出力ピンについての記述を行っている点である。詳細は [外部入出力ピンについて](#outorinPin)を参照してください。
        1. `takiroboF1 x(float MEDIAN_x, float MEDIAN_y, float SCALE);` <a id="3"></a>
            - これを記述した場合、電源を入れる毎にキャリブレーションを行う必要はありません。
            - キャリブレーション方法については[キャリブレーションの方法について](#caliblation1)を参照してください。
        1. `int hoge[3]={0,0,0};`  
           `takiroboF1 x(float MEDIAN_x, float MEDIAN_y, float SCALE,hoge);`  
            - これを記述した場合、電源を入れる毎にキャリブレーションを行う必要はありません。
            - キャリブレーション方法については[キャリブレーションの方法について](#caliblation1)を参照してください。
            - [3](#3)との違いは外部入出力ピンについての記述を行っている点である。詳細は [外部入出力ピンについて](#outorinPin)を参照してください。
1. x.getUSS();
    - 超音波センサの値を返します。
1. x.motor(spd1,spd2,spd3);
    - モーターの制御を行います。spd1,spd2,spd3はそれぞれモーター1,2,3のスピードを表しています。-255〜255までの値が有効です。マイナスは反対方向に回ります。
1. x.getLine(int num);
    - ラインセンサの値を取得します。取得したいラインセンサの番号を入力してください。ラインセンサはロボットの前から時計回りに1~4までの番号が割り振られています。
1. x.getIr(int num);
    - 赤外線センサの値を返します。取得したい赤外線センサの番号を入力してください。赤外線センサはロボットの前から時計回りに1~4までの番号が割り振られています。
1. x.irUpdate();
    - 赤外線センサの値を更新します。赤外線センサを使用する場合は、ループ毎に一度読み込んでください。
1. x.lineUpdate();
    - ラインセンサの値を更新します。ラインセンサを使用する場合は、ループ毎に一度読み込んでください。
1. x.getStartingAzimuth();<a id="getStartingAzimuth"></a>
    - 初めにプログラム開始用スイッチをONにした時の機体が向いていた方位を返します。
1. x.getAzimuth();
    - 現在の方位を出力します。出力値は-180~180となっています。この関数を使用するときはキャリブレーションを行ってください。
1. x.init();
    - ピンやi2c、シリアルポートなどの初期化を行います。必ずsetup()内に記述してください。
1. 外部入出力ピンについて　<a id="outorinPin"></a>
    - TakiRobo F1には3つの外部入出力ピンが用意されています。場所は基板の右側です。前から順番に1番ピン、2番ピン、3番ピンとなっています。ピンそれぞれの情報については下記の表に示します。
      ||1番ピン|2番ピン|3番ピン|
      |:--:|:--:|:--:|:--:|
      |analogRead|○|×|×|
      |analogWrite|×|×|×|
      |digitalRead|○|○|○|
      |digitalWrite|○|○|○|
      3番ピンはLED2(左側のLED)と共用になっています。そのため3番ピンを使う場合、インジケーター用のLEDは動かなくなります。1,2番ピンを優先して使うことを推奨します。
    - 1~3番ピンの入出力設定について
        - 1~3番ピンを使用するには入出力の設定を行う必要があります。設定を行うにはプログラム内の  
          `takiroboF1 x(float MEDIAN_x, float MEDIAN_y, float SCALE);`  
          を  
          `int hoge[3]={0,0,0};`  
          `takiroboF1 x(float MEDIAN_x, float MEDIAN_y, float SCALE,hoge);`  
          に変更します。なお、引数名hogeは自由に変更可能です。  
          `{0,0,0}`はそれぞれ`{1番ピン,2番ピン,3番ピン}`を表しており、ここに指定された数値を記述することで設定できます。指定された数値は以下の表に示しています。
          |0|1|2|3|
          |:--:|:--:|:--:|:--:|
          |使用しない|digitalRead|digitalWrite|analogRead|  
          例えば`int hoge[3]={2,1,0}`とした場合、1番ピンはdigitalWrite、2番ピンdigitalRead、3番ピンは使用しないという設定になります。
    - 入出力読み書き用関数
        1. x.getAnalogPin();
            - 1番ピンのアナログ値を取得します。1番ピンをアナログピンに設定していない場合は0を返します。１番ピンのみ使用可能です。
        1. x.getDigitalPin(int pin);
            - 1~3番ピンのデジタル値を取得します。引数には取得したいピンの番号を記述してください。各ピンの設定をデジタル読み込みに設定していない場合、0を返します。
        1. x.setDigitalPin(int pin, bool value);
            - 1~3番ピンのデジタル値を出力します。引数には出力したいピンの番号を記述してください。また、valueにはHIGH,LOWのどちらかを入力してください。各ピンの設定をデジタル書き込みに設定していない場合、出力されません。
1. キャリブレーションの方法について 
    - キャリブレーションは機体を様々な方向に回転させることで完了できます。
    - キャリブレーション値の設定方法は二つあります。 
        1. プログラムにキャリブレーション値を書き込む(電源を入れる毎にキャリブレーションを行う必要がない)場合 <a id="caliblation1"></a>
            - スケッチ例のcaliblationという名前のプログラムを書き込みます。
            - 書き込みが終われば実行します。実行する際は、プログラム開始用スイッチをOFFにしてください。
            - シリアルモニタを開いた後、５秒経過するとキャリブレーションが実行されます。
            - __キャリブレーションが終了しました。__ と表示された後に出力される()で囲まれた値がキャリブレーション値です。この行をコピーし、スケッチ例templateの3行目に記述されているtakiroboF1 robot(x,y,s)の()に貼り付けます。
            - takiroboF1の動かす場所を大きく変更する場合または、コンパスセンサからの値に異常が見られる場合は、キャリブレーション値を再度取得してください。

        1. 電源を入れる毎にキャリブレーションを行う場合<a id="caliblation2"></a>
            - 実行時、電源投入前からプログラム開始用スイッチをONにしてください。(この時プログラム開始用スイッチはONになっていますが、[機体の方位の取得](#getStartingAzimuth)は行いません)
            - 電源投入後、５秒経過するとキャリブレーションが実行されます。
            - LED2が点滅すると、キャリブレーションは終了です。プログラム開始用スイッチをOFFにしてください。
            - 外部入出力の3番ピンを使用している場合、この方法でのキャリブレーションは実行できません。別のキャリブレーション方法を使用してください。
            - キャリブレーションが必要ない(プログラムの中でコンパスセンサを使用しない)場合は、キャリブレーションをスキップすることが出来ます。その場合は電源投入前からプログラム開始用スイッチをOFFにしてください。(非推奨)
1. 機体のスイッチについて
    - 主電源(スライドスイッチ)は左側がOFF、右側がONです。
    - プログラム開始用スイッチ(プッシュスイッチ)は上がるとOFF、下がるとONです。
    - プログラム開始用スイッチをOFFにするとプログラムは一時停止します。ONにするとプログラムは再開されます。


