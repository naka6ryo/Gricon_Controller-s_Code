//初期設定
  //MPU6050 Madgwick filter Test
  #include <Arduino_LSM9DS1.h>
  #include <MadgwickAHRS.h>
  #include <stdlib.h>
  #include <stdio.h>
  #define ANALOG_PIN 7
  Madgwick MadgwickFilter;

  int in = 5;//ゲームからの入力用変数
  int in0 = 5;

  float a = 0.1;//水平時のヨー補正係数
  float p = 0.75;
  float r = 0.75;
  float yaw_y = 0.75;
  float G = 0.8;//Madwickフィルターのゲイン
  float T = 0.01;//積分時間
  float c = 0.1;//加速度補正係数
  float d = 0.1;//速度補正係数

  //磁気オフセット
  float xmcy = 9.996;
  float ymcy = -52.420841;
  float zmcy = 32.338142;

  //加速度オフセット
  float xacy = -0.02;
  float yacy = 0.01;
  float zacy = 0;

  //ジャイロオフセット（初期値取得プログラムで計測）
  float xgcy = 0;
  float ygcy = 0;
  float zgcy = 0;

  //初期値取得プログラム用変数
  int l = 0;

  //加速度
  float xa = 0;
  float ya = 0;
  float za = 0;
  //ひとつ前の加速度
  float xa0 = 0;
  float ya0 = 0;
  float za0 = 0;
  //グローバル座標変換用の加速度
  float xa1 = 0;
  float ya1 = 0;
  float za1 = 1;

  float xa10 = 0;
  float ya10 = 0;
  float za10 = 1;

  //ジャイロ
  float xg = 0;
  float yg = 0;
  float zg = 0;

  //地磁気
  float xm = 0;
  float ym = 0;
  float zm = 0;

  //ヨー・ピッチ・ロー
  float pitch = 0;
  float roll = 0;
  float yaw = 0;

  float pitch0 = 0;
  float roll0 = 0;
  float yaw_g0 = 0;

  //ヨー角関連
  float yaw_sim = 0;//ヨー角の初期値計算用・合計
  float yaw_s = 0;//ヨー角初期値
  float yaw_sm = 0;//ヨー角初期値・地磁気
  float yaw_g = 0;//ヨー角
  float yaw_m = 0;//ヨー角・地磁気

  //ジャイロ補正用合計
  float xg_sim = 0;
  float yg_sim = 0;
  float zg_sim = 0;

  //速度
  float xv = 0;
  float yv = 0;
  float zv = 0;
  //ひとつ前の速度
  float xv0 = 0;
  float yv0 = 0;
  float zv0 = 0;

  //移動距離
  float x_d = 0;
  float y_d = 0;
  float z_d = 0;
  //座標
  float x = 0;
  float y = 0;
  float z = 0;

  const int motorPin = 9; //モータ用のピン設定

  int bend; //曲げセンサの値
  int bend0;

  //振動検出用
  int baibx;
  int baiby;
  int baibz;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  //モータ用のピン設定
  pinMode(motorPin, OUTPUT);
  while (!Serial1)
    ;
   //Serial.println("\nStarted\n");

  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  /*Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz  Acceleration in G's X\tY\tZ");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println("Hz  Gyroscope in degrees/second X\tY\tZ");

  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println("uT  Magnetic Field in uT X\tY\tZ");*/

  delay(500);

  MadgwickFilter.begin(100);  //100Hz
  MadgwickFilter.setGain(G);
  delay(100);
}
void loop() {
  //初期値取得
    if (l == 0) {
      digitalWrite(LEDG, HIGH);
      for (int i = 0; i < 100; i++) {
        if (i % 10 < 5) {
          digitalWrite(LEDR, LOW);
        } else {
          digitalWrite(LEDR, HIGH);
        }

        IMU.readAcceleration(xa, ya, za);
        IMU.readGyroscope(xg, yg, zg);
        IMU.readMagneticField(xm, ym, zm);

        xm = xm + xmcy;
        ym = ym + ymcy;
        zm = zm + zmcy;

        xg = xg + xgcy;
        yg = yg + ygcy;
        zg = zg + zgcy;

        MadgwickFilter.update(xg, yg, zg, xa, ya, za, xm, ym, zm);
        delay(10);
      }
      
      for (int i = 0; i < 500; i++) {
        if (i % 100 < 50) {
          digitalWrite(LEDR, LOW);
        } else {
          digitalWrite(LEDR, HIGH);
        }

        IMU.readAcceleration(xa, ya, za);
        IMU.readGyroscope(xg, yg, zg);
        IMU.readMagneticField(xm, ym, zm);

        xm = xm + xmcy;
        ym = ym + ymcy;
        zm = zm + zmcy;

        xg = xg + xgcy;
        yg = yg + ygcy;
        zg = zg + zgcy;

        MadgwickFilter.update(xg, yg, zg, xa, ya, za, xm, ym, zm);

        yaw_sim = yaw_sim + MadgwickFilter.getYaw();

        xg_sim = xg_sim + xg;
        yg_sim = yg_sim + yg;
        zg_sim = zg_sim + zg;

        delay(10);
      }

      //yaw_s = - atan2(ym,xm) * 57.324;
      yaw_s = yaw_sim / 500;
      yaw_sm = atan2(ym,xm)*57.324;
      xgcy = xg_sim / 500;
      ygcy = yg_sim / 500;
      zgcy = zg_sim / 500;

      l = l + 1;
      bend0 = analogRead(ANALOG_PIN);

      for (int i = 0; i < 10; i++) {
        digitalWrite(LEDG, HIGH);
        digitalWrite(motorPin, HIGH);
        delay(50);
        digitalWrite(LEDG, LOW);
        digitalWrite(motorPin, LOW);
        delay(50);
      }
    }
  //ゲーム入力の応答
    in0 = in;
    if(Serial1.available() > 0){
      in = Serial1.read();
      if (in != '1' && in != '2' && in != '3' && in != '4' && in != '5') in = in0; 
    }

    //モータの挙動
      if (in == '5'){
        digitalWrite(motorPin, LOW);
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, LOW);
      } 
      else if (in == '1') {
        digitalWrite(LEDG, HIGH);
        if (l % 20 < 3) {
          digitalWrite(motorPin, HIGH);
          digitalWrite(LEDR, LOW);
        }else{
          digitalWrite(motorPin, LOW);
          digitalWrite(LEDR, HIGH);
        }
      }
      else if (in == '2') {
        digitalWrite(LEDG, HIGH);
        if(l % 10 < 2) {
          digitalWrite(motorPin, HIGH);
          digitalWrite(LEDR, LOW);
        }else{
          digitalWrite(motorPin, LOW);
          digitalWrite(LEDR, HIGH);
        }
      }
      else if (in == '3') {
        digitalWrite(LEDG, HIGH);
        if(l % 5 < 2) {
          digitalWrite(motorPin, HIGH);
          digitalWrite(LEDR, LOW);
        }else{
          digitalWrite(motorPin, LOW);
          digitalWrite(LEDR, HIGH);
        }
      }
      else if (in == '4') {
        digitalWrite(LEDG, HIGH);
        if(l % 10 < 9) {
          digitalWrite(motorPin, HIGH);
          digitalWrite(LEDR, LOW);
        }else{
          digitalWrite(motorPin, LOW);
          digitalWrite(LEDR, HIGH);
        }
      }
  //曲げセンサの値取得
    bend = analogRead(ANALOG_PIN) - bend0;
    if(bend < 8) bend = 0;
  //１つ前の情報の記憶
    xa0 = xa;
    ya0 = ya;
    za0 = za;
    xa10 = xa1;
    ya10 = ya1;
    za10 = za1;
    xv0 = xv;
    yv0 = yv;
    zv0 = zv;
    pitch0 = pitch;
    roll0 = roll;
    yaw_g0 = yaw_g;

  // 測定値を持ってくる
    IMU.readAcceleration(xa, ya, za);
    IMU.readGyroscope(xg, yg, zg);
    IMU.readMagneticField(xm, ym, zm);

  //オフセット補正
    xm = xm + xmcy;
    ym = ym + ymcy;
    zm = zm + zmcy;

    xg = -xg + xgcy;
    yg = -yg + ygcy;
    zg = zg + zgcy - 0.0015;

    xa = xa + xacy;
    ya = ya + yacy;
    za = za + zacy;
  
  //角度計算
    //フィルタを通す
    MadgwickFilter.update(xg, yg, zg, xa, ya, za, xm ,ym, zm);

    //角度を得る
    // pitch = -1*MadgwickFilter.getPitch();
    // roll = MadgwickFilter.getRoll();
    // yaw_g = -4*(MadgwickFilter.getYaw() - yaw_s);

    pitch = -1*MadgwickFilter.getPitch()*p + (1-p)*pitch;
    roll = MadgwickFilter.getRoll()*r + (1-r)*roll;
    yaw_g = -1*(MadgwickFilter.getYaw() - yaw_s)*yaw_y + (1-yaw_y)*yaw_g0;

    //if(fabsf(pitch) < 0.5) pitch = 0;
    //if(fabsf(roll) < 0.5) roll = 0;

    //地磁気による角度を得る
    yaw_m = -1*(atan2(ym,xm)*57.324 - yaw_sm);

    // //地磁気角度の補正
    // if(yaw_m < -180){
    //   yaw_m = 360 + yaw_m;
    // }
    // if(yaw_m > 180){
    //   yaw_m = yaw_m - 360;
    // }

    // //ヨー角の地磁気を用いた水平時の補正  
    // if(pow(pitch,2)< 1 && pow(roll,2) < 3){
    //   yaw_s = yaw_s + yaw_m - yaw_g;
    //   yaw_g = yaw_m;
    // }else if(pow(pitch,2)< 10 && pow(roll,2) < 10){
    //   yaw_s = yaw_s + a*(yaw_m - yaw_g);
    // }

    // //ピッチが９０度付近の動作の補正
    // if(fabsf(pitch)> 80 ){
    //   pitch = (180 - fabsf(pitch))*(pitch/(fabsf(pitch)));
    //   roll = (180 - fabsf(roll))*(roll/(fabsf(roll)));
    // }

  //位置情報計算
    xa = c * xa + (1-c)*xa0;
    ya = c * ya + (1-c)*ya0;
    za = c * za + (1-c)*za0;

    // //角速度をなくす
    // // if(fabsf(xg) > 150){
    // //   ya = ya - 0.0001*xg;
    // //   za = za - 0.0001*xg;
    // // }
    // // if(fabsf(yg) > 150){
    // //    xa = xa - 0.0001*yg;
    // //    za = za - 0.0001*yg;
    // // }

    //加速度のグローバル座標への変換（三角関数編）
    za1 = xa*sin(pitch*0.017) + ya*sin(roll*0.017) + za*cos(roll*0.017)*cos(pitch*0.017);
    xa1 = (xa*cos(pitch*0.017) + za*cos((pitch + 90)*0.017)*cos(roll*0.017))*cos(yaw_g*0.017) + (ya*cos(roll*0.017) + za*cos((roll + 90)*0.017)*cos(pitch*0.017))*sin(yaw_g*0.017);
    ya1 = (xa*cos(pitch*0.017) + za*cos((pitch + 90)*0.017)*cos(roll*0.017))*sin(yaw_g*0.017) + (ya*cos(roll*0.017) + za*cos((roll + 90)*0.017)*cos(pitch*0.017))*cos(yaw_g*0.017);
  
    //加速度のフィルター
    xa1 = c * xa1 + (1-c)*xa10;
    ya1 = c * ya1 + (1-c)*ya10;
    za1 = c * za1 + (1-c)*za10;

    if (fabsf(xa1) > 0.5) baibx = 1;
    else if (fabsf(xa1) < 0.5) baibx = 0;

    if (fabsf(ya1) > 0.5) baiby = 1;
    else if (fabsf(ya1) < 0.5) baibx = 0;

    if (fabsf(za1 - za10)  < 0.04) baibz = 0;
    else if (fabsf(za1 - za10) > 0.04){
      baibz = 0;
      // yaw_s = yaw_g / 4;
    };

    // //速度の計算
    // xv = xv + 0.5*T*9.80665*100*(xa1 + xa10);
    // yv = yv + 0.5*T*9.80665*100*(ya1 + ya10);
    // zv = zv + 0.5*T*9.80665*100*(za1 + za10 - 2);

    // //減速の処理
    // if ((xv - xv0)*xv < 0){
    //   xv = xv0;
    //  //Serial.print("HIIIIIIII");
    // } 
    // if ((yv - yv0)*yv < 0){
    //   yv = yv0;
    //   //Serial.print("HIIIIIIII");
    // } 
    // if ((zv - zv0)*zv < 0){
    //   zv = zv0;
    //   //Serial.print("HIIIIIIII");
    // }
  
    // //静止時の処理
    // if (fabsf(xa1 - xa10) < 0.002 && fabsf(xa1) < 0.455) {
    //   xv = 0;
    //   //xv0 = 0;
    //   //Serial.print("HIIIIIIII");
    // }
    // if (fabsf(ya1 - ya10) < 0.002 && fabsf(ya1) < 0.455) {
    //   yv = 0;
    //   //yv0 = 0;
    //   //Serial.print("HIIIIIIII");
    // }
    // if (fabsf(za1 - za10) < 0.002 && fabsf(za1 - 1) < 0.1) {
    //   zv = 0;
    //   //zv0 = 0;
    //   //Serial.print("HIIIIIIII");
    // }

    // // if(fabsf(roll - roll0) > 0.1){
    // //   yv = 0;
    // //   zv = 0;
    // // }
    // // if(fabsf(pitch - pitch0) > 0.1){
    // //   xv = 0;
    // //   zv = 0;
    // // }

    // //速度のフィルター
    // xv = d*xv + (1 - d)*xv0;
    // yv = d*yv + (1 - d)*yv0;
    // zv = d*zv + (1 - d)*zv0;
  
    // //移動距離の計算
    // x_d = 0.5*T*(xv + xv0);
    // y_d = 0.5*T*(yv + yv0);
    // z_d = 0.5*T*(zv + zv0);
  

    // //座標の計算＆フィルター
    // x = x + x_d;
    // y = y + y_d;
    // z = z + z_d;


  //値出力
    Serial1.print(pitch);
    Serial1.print(",");
    Serial1.print(roll);
    Serial1.print(",");
    Serial1.print(yaw_g);
    // Serial1.print(",");
    // Serial1.print(x);
    // Serial1.print(",");
    // Serial1.print(y);
    // Serial1.print(",");
    // Serial1.print(z);
    Serial1.print(",");
    Serial1.print(bend);
    Serial1.print(",");
    Serial1.print(baibz);
    Serial1.println("");

    // Serial.print(-pitch);
    // Serial.print(",");
    // Serial.print(roll);
    // Serial.print(",");
    // Serial.print(yaw_g);
    // Serial.print(",");
    // // Serial.print(x);
    // // Serial.print(",");
    // // Serial.print(y);
    // // Serial.print(",");
    // //Serial.print(in);
    // //Serial.print(",");
    // Serial.print(bend);
    // Serial.print(",");
    // Serial.print(baibz);
    // Serial.println("");
  l = l + 1;//ループカウント用
  if (l > 1000) l = 1;

  delay(10);
}