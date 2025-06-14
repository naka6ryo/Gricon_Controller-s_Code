// ===============================
// ライブラリ・定数定義
// ===============================
#include <Arduino_LSM9DS1.h>  // IMU用ライブラリ
#include <MadgwickAHRS.h>     // 姿勢推定フィルタ

#define ANALOG_PIN 7          // 曲げセンサアナログ入力
#define motorPin 9            // モーター制御用デジタル出力ピン

Madgwick MadgwickFilter;     // Madgwickフィルターのインスタンス

// ===============================
// 制御・状態変数
// ===============================
int in = 5, in0 = 5;          // シリアル通信による外部入力
int l = 0;                    // ループカウンタ
int bend0 = 0, bend_past = 0, bend = 0;      // 曲げセンサ値（初期基準と過去値と現在値）

// ===============================
// 補正係数（センサノイズ除去・フィルタ調整）
// ===============================
float a = 0.1;                // 地磁気補正係数
float p = 0.5, r = 0.5;     // ピッチ・ロール角の平滑化係数
float yaw_y = 0.5;           // ヨー角の平滑化係数
float b = 0.5;                //曲げセンサの平滑化係数
float G = 0.7;                // Madgwickフィルターゲイン
float T = 0.01;               // 積分周期（秒）
float c = 0.1, d = 0.1;       // 加速度・速度のフィルタ係数

// ===============================
// センサオフセット値（補正用）
// ===============================
// 地磁気補正用（新しいオフセット＋スケール）
float mag_offset[3] = { -2.4902345, 44.781494, -3.540039 };
float mag_scale[3]  = { 1.0323736, 0.9979383, 0.9715412 };
float xacy = -0.02, yacy = 0.01, zacy = 0;                // 加速度オフセット
float xgcy = 0, ygcy = 0, zgcy = 0;                       // ジャイロオフセット

// ===============================
// 各種センサデータ
// ===============================
float xa, ya, za;              // 加速度
float xa0, ya0, za0;           // 前回加速度
float xa1 = 0, ya1 = 0, za1 = 1, xa10 = 0, ya10 = 0, za10 = 1; // グローバル加速度とその前回値
float xg, yg, zg;              // ジャイロ
float xm, ym, zm;              // 地磁気

// ===============================
// 姿勢角（ピッチ・ロール・ヨー）
// ===============================
float pitch, roll, yaw;        // 現在の角度
float pitch0, roll0, yaw_g0;   // 前回の角度

// ===============================
// ヨー角および初期補正用値
// ===============================
float yaw_sim = 0, yaw_s = 0, yaw_sm = 0;
float yaw_g = 0, yaw_m = 0;

// ===============================
// ジャイロ初期値補正積算
// ===============================
float xg_sim = 0, yg_sim = 0, zg_sim = 0;

// ===============================
// 速度・位置
// ===============================
float xv = 0, yv = 0, zv = 0, xv0 = 0, yv0 = 0, zv0 = 0;
float x_d = 0, y_d = 0, z_d = 0, x = 0, y = 0, z = 0;

// ===============================
// 振動検出用
// ===============================
int baibx, baiby, baibz;

// ===============================
// IMU初期化処理
// ===============================
void initializeIMU() {
  if (!IMU.begin()) {
    while (1);
  }
  MadgwickFilter.begin(100);
  MadgwickFilter.setGain(G);
}


// ===============================
// センサ初期キャリブレーション
// ===============================
void calibrateSensors() {
  digitalWrite(LEDG, HIGH);

  for (int i = 0; i < 100; i++) {
    digitalWrite(LEDR, i % 10 < 5 ? LOW : HIGH);
    readIMUSensors(true);
    delay(10);
  }

  for (int i = 0; i < 500; i++) {
    digitalWrite(LEDR, i % 100 < 50 ? LOW : HIGH);
    readIMUSensors(true);
    yaw_sim += MadgwickFilter.getYaw();
    xg_sim += xg; yg_sim += yg; zg_sim += zg;
    delay(10);
  }

  yaw_s = yaw_sim / 500;
  yaw_sm = atan2(ym, xm) * 57.324;
  xgcy = xg_sim / 500;
  ygcy = yg_sim / 500;
  zgcy = zg_sim / 500;
  bend0 = analogRead(ANALOG_PIN);

  for (int i = 0; i < 10; i++) {
    digitalWrite(LEDG, HIGH); digitalWrite(motorPin, HIGH); delay(50);
    digitalWrite(LEDG, LOW); digitalWrite(motorPin, LOW); delay(50);
  }
}

// ===============================
// IMU読み取り関数（補正付き）
// ===============================
void readIMUSensors(bool applyOffset) {
  IMU.readAcceleration(xa, ya, za);
  IMU.readGyroscope(xg, yg, zg);
  IMU.readMagneticField(xm, ym, zm);

  // 地磁気オフセット・スケール補正
  xm = (xm - mag_offset[0]) * mag_scale[0];
  ym = (ym - mag_offset[1]) * mag_scale[1];
  zm = (zm - mag_offset[2]) * mag_scale[2];
  
  if (applyOffset) {
    xg += xgcy; yg += ygcy; zg += zgcy;
  } else {
    xg = -xg + xgcy;
    yg = -yg + ygcy;
    zg = zg + zgcy - 0.0015;
    xa += xacy; ya += yacy; za += zacy;
  }

  MadgwickFilter.update(xg, yg, zg, xa, ya, za, xm, ym, zm);
}

// ===============================
// モータ制御（シリアル入力に応じて）
// ===============================
void updateMotorState() {
  in0 = in;
  if (Serial1.available() > 0) {
    in = Serial1.read();
    if (in < '1' || in > '5') in = in0;
  }else{
    in = in0;
  }

  int flashRate = 0;
  float flashRate_deno = 0;
  if (in == '1') {flashRate = 20; flashRate_deno = 0.4;} 
  else if (in == '2') {flashRate = 40; flashRate_deno = 0.2;}
  else if (in == '3') {flashRate = 80; flashRate_deno = 0.125;}
  else if (in == '4') {flashRate = 100; flashRate_deno = 0.9;}
  else{
    digitalWrite(motorPin, LOW); 
    digitalWrite(LEDR, HIGH); 
    digitalWrite(LEDG, LOW); 
    flashRate = 0;
    //return;
  }

  if (flashRate > 0) {
    digitalWrite(LEDG, HIGH);
    bool motorOn = (l % flashRate < flashRate * flashRate_deno);
    digitalWrite(motorPin, motorOn ? HIGH : LOW);
    digitalWrite(LEDR, motorOn ? LOW : HIGH);
  }
}

// ===============================
// 姿勢角の更新（Madgwick出力）
// ===============================
void updateOrientation() {
  pitch = -1 * MadgwickFilter.getPitch() * p + (1 - p) * pitch;
  roll = MadgwickFilter.getRoll() * r + (1 - r) * roll;
  yaw_g = -1 * (MadgwickFilter.getYaw() - yaw_s) * yaw_y + (1 - yaw_y) * yaw_g0;
  yaw_m = -1 * (atan2(ym, xm) * 57.324 - yaw_sm);
}

// ===============================
// グローバル座標への加速度変換
// ===============================
float computeGlobalAccelX() {
  return (xa * cos(pitch * DEG_TO_RAD) + za * cos((pitch + 90) * DEG_TO_RAD) * cos(roll * DEG_TO_RAD)) * cos(yaw_g * DEG_TO_RAD) +
         (ya * cos(roll * DEG_TO_RAD) + za * cos((roll + 90) * DEG_TO_RAD) * cos(pitch * DEG_TO_RAD)) * sin(yaw_g * DEG_TO_RAD);
}
float computeGlobalAccelY() {
  return (xa * cos(pitch * DEG_TO_RAD) + za * cos((pitch + 90) * DEG_TO_RAD) * cos(roll * DEG_TO_RAD)) * sin(yaw_g * DEG_TO_RAD) +
         (ya * cos(roll * DEG_TO_RAD) + za * cos((roll + 90) * DEG_TO_RAD) * cos(pitch * DEG_TO_RAD)) * cos(yaw_g * DEG_TO_RAD);
}
float computeGlobalAccelZ() {
  return xa * sin(pitch * DEG_TO_RAD) + ya * sin(roll * DEG_TO_RAD) + za * cos(roll * DEG_TO_RAD) * cos(pitch * DEG_TO_RAD);
}

// ===============================
// 加速度の更新と振動検出
// ===============================
void updateAcceleration() {
  xa = c * xa + (1 - c) * xa0;
  ya = c * ya + (1 - c) * ya0;
  za = c * za + (1 - c) * za0;

  xa1 = computeGlobalAccelX();
  ya1 = computeGlobalAccelY();
  za1 = computeGlobalAccelZ();

  xa1 = c * xa1 + (1 - c) * xa10;
  ya1 = c * ya1 + (1 - c) * ya10;
  za1 = c * za1 + (1 - c) * za10;

  baibx = fabsf(xa1) > 0.5;
  baiby = fabsf(ya1) > 0.5;
  baibz = fabsf(za1 - za10) > 0.04 ? 0 : 0;
}

// ===============================
// データ送信（バイト列形式）
// ===============================
void outputDataAsBytes() {
  int16_t angle_pitch = (int16_t)(pitch * 10.0);  // 小数第1位まで保持
  int16_t angle_roll  = (int16_t)(roll * 10.0);
  int16_t angle_yaw   = (int16_t)(yaw_g * 10.0);
  // 出力用bendにクランプを適用
  uint8_t bend_to_send = constrain((int)bend, 0, 20); 


  uint8_t buffer[sizeof(int16_t) * 3 + sizeof(uint8_t)];

  memcpy(buffer,                &angle_pitch, sizeof(int16_t));
  memcpy(buffer + sizeof(int16_t), &angle_roll,  sizeof(int16_t));
  memcpy(buffer + 2 * sizeof(int16_t), &angle_yaw,   sizeof(int16_t));
  memcpy(buffer + 3 * sizeof(int16_t), &bend_to_send,  sizeof(uint8_t));

  Serial1.write('S'); // ヘッダー
  Serial1.write(buffer, sizeof(buffer));
}


// ===============================
// 初期設定処理
// ===============================
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(motorPin, OUTPUT);
  initializeIMU();
  delay(100);
}

// ===============================
// メインループ処理
// ===============================
void loop() {
  if (l == 0) calibrateSensors();
  
  bend_past = bend;
  bend = analogRead(ANALOG_PIN) - bend0;
  bend = b*bend + (1 - b)*bend_past;

  xa0 = xa; ya0 = ya; za0 = za;
  xa10 = xa1; ya10 = ya1; za10 = za1;
  xv0 = xv; yv0 = yv; zv0 = zv;
  pitch0 = pitch; roll0 = roll; yaw_g0 = yaw_g;

  readIMUSensors(false);
  updateMotorState();
  updateOrientation();
  updateAcceleration();
  outputDataAsBytes();

  l++;
  if (l > 4000) l = 1;
  delay(1);
}
