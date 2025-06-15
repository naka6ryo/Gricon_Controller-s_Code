// ===============================
// 必要なライブラリの読み込み
// ===============================
#include <Arduino_LSM9DS1.h>   // LSM9DS1 IMU用ライブラリ
#include <MadgwickAHRS.h>      // 姿勢推定フィルタ（Madgwick法）

// ===============================
// ピン定義
// ===============================
#define ANALOG_PIN 7           // 曲げセンサ接続アナログピン
#define motorPin 9             // モーター出力ピン

Madgwick MadgwickFilter;      // Madgwickフィルタのインスタンス作成

// ===============================
// 状態変数・初期化用
// ===============================
int in = 5, in0 = 5;           // シリアル入力値（前回値と現在値）
int l = 0;                     // メインループカウンタ
float bend0 = 0, bend_past = 0, bend = 0; // 曲げセンサ値（初期基準・過去・現在）

// ===============================
// フィルタリング・補正係数
// ===============================
float a = 0.1;                 // 地磁気補正係数
float p = 0.5, r = 0.5;        // ピッチ・ロール平滑化係数
float yaw_y = 0.5;             // ヨー角平滑化係数
float b = 0.1;                 // 曲げセンサ平滑化係数
float G = 0.7;                 // Madgwickフィルタゲイン
float T = 0.01;                // サンプリング周期（秒）
float c = 0.1, d = 0.1;        // 加速度・速度のフィルタ係数

// ===============================
// センサ補正用のオフセット・スケール
// ===============================
float mag_offset[3] = { -2.4902345, 44.781494, -3.540039 }; // 地磁気オフセット
float mag_scale[3]  = { 1.0323736, 0.9979383, 0.9715412 };  // 地磁気スケール補正
float acc_offset[3] = { -0.02, 0.01, 0.0 };                 // 加速度オフセット
float gyro_offset[3] = { 0.0, 0.0, 0.0 };                   // ジャイロオフセット（初期補正で決定）

// ===============================
// IMUデータ格納用の配列
// ===============================
float acc[3] = {0}, gyro[3] = {0}, mag[3] = {0};             // センサ値
float acc_prev[3] = {0}, acc_global[3] = {0,0,1}, acc_global_prev[3] = {0,0,1};
float vel[3] = {0}, vel_prev[3] = {0};                       // 速度（各軸）
float pos[3] = {0};                                          // 位置（各軸）

// ===============================
// 姿勢角・ヨー角補正用
// ===============================
float pitch = 0, roll = 0, yaw = 0;
float pitch0 = 0, roll0 = 0, yaw_g0 = 0;
float yaw_sim = 0, yaw_s = 0, yaw_sm = 0;
float yaw_g = 0, yaw_m = 0;
float gyro_sim[3] = {0};                                     // 初期ジャイロ値積算用

// ===============================
// 振動検出用フラグ
// ===============================
int baibx, baiby, baibz;

// ===============================
// IMUの初期化関数
// ===============================
void initializeIMU() {
  if (!IMU.begin()) {
    while (1);  // 初期化失敗時に停止
  }
  MadgwickFilter.begin(100);     // フィルタ更新周波数設定（Hz）
  MadgwickFilter.setGain(G);     // ゲイン設定
}

// ===============================
// 初期キャリブレーション関数
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
    for (int j = 0; j < 3; j++) gyro_sim[j] += gyro[j];
    delay(10);
  }

  yaw_s = yaw_sim / 500.0;
  yaw_sm = atan2(mag[1], mag[0]) * 57.324;
  for (int i = 0; i < 3; i++) gyro_offset[i] = gyro_sim[i] / 500.0;
  bend0 = analogRead(ANALOG_PIN);

  for (int i = 0; i < 10; i++) {
    digitalWrite(LEDG, HIGH); digitalWrite(motorPin, HIGH); delay(50);
    digitalWrite(LEDG, LOW);  digitalWrite(motorPin, LOW);  delay(50);
  }
}

// ===============================
// IMU読み取り・補正
// ===============================
void readIMUSensors(bool applyOffset) {
  IMU.readAcceleration(acc[0], acc[1], acc[2]);
  IMU.readGyroscope(gyro[0], gyro[1], gyro[2]);
  IMU.readMagneticField(mag[0], mag[1], mag[2]);

  // 地磁気のオフセット・スケーリング補正
  for (int i = 0; i < 3; i++) {
    mag[i] = (mag[i] - mag_offset[i]) * mag_scale[i];
  }

  // ★ 地磁気ベクトルの正規化を追加（安全チェック付き）★
  float mag_norm = sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
  if (mag_norm > 0.01) {
    for (int i = 0; i < 3; i++) mag[i] /= mag_norm;
  }

  // ジャイロ・加速度のオフセット補正
  if (applyOffset) {
    for (int i = 0; i < 3; i++) gyro[i] += gyro_offset[i];
  } else {
    gyro[0] = -gyro[0] + gyro_offset[0];
    gyro[1] = -gyro[1] + gyro_offset[1];
    gyro[2] =  gyro[2] + gyro_offset[2] - 0.0015;
    for (int i = 0; i < 3; i++) acc[i] += acc_offset[i];
  }

  // Madgwickフィルター更新
  MadgwickFilter.update(
    gyro[0], gyro[1], gyro[2],
    acc[0], acc[1], acc[2],
    mag[0], mag[1], mag[2]
  );
}

// ===============================
// ピッチに応じたゲインの変化
// ===============================
float computeDynamicGain(float pitch_deg) {
  // ゲインの範囲（最小〜最大）
  float beta_min = 0.001;
  float beta_max = 0.7;

  // ピッチ角を0〜90°に正規化（対称に扱う）
  float abs_pitch = fabs(pitch_deg);
  float normalized = constrain(abs_pitch / 90.0, 0.0, 1.0);

  // 累乗型の急激な減衰
  float decay = pow((1.0 - normalized), 8);  // (1 - x)^8 で急減衰
  float beta = beta_min + (beta_max - beta_min) * decay;

  return constrain(beta, beta_min, beta_max);
}



// ===============================
// 姿勢角（ピッチ・ロール・ヨー）更新
// ===============================
void updateOrientation() {
  pitch = -1 * MadgwickFilter.getPitch() * p + (1 - p) * pitch;
  roll = MadgwickFilter.getRoll() * r + (1 - r) * roll;
  yaw_g = -1 * (MadgwickFilter.getYaw() - yaw_s) * yaw_y + (1 - yaw_y) * yaw_g0;
  yaw_m = -1 * (atan2(mag[1], mag[0]) * 57.324 - yaw_sm);

  // ピッチに応じて動的ゲイン設定 
  float dynamicGain = computeDynamicGain(pitch);
  MadgwickFilter.setGain(dynamicGain);
}

// ===============================
// 加速度 → グローバル座標変換
// ===============================
float computeGlobalAccelX() {
  return (acc[0] * cos(pitch * DEG_TO_RAD) + acc[2] * cos((pitch + 90) * DEG_TO_RAD) * cos(roll * DEG_TO_RAD)) * cos(yaw_g * DEG_TO_RAD) +
         (acc[1] * cos(roll * DEG_TO_RAD) + acc[2] * cos((roll + 90) * DEG_TO_RAD) * cos(pitch * DEG_TO_RAD)) * sin(yaw_g * DEG_TO_RAD);
}
float computeGlobalAccelY() {
  return (acc[0] * cos(pitch * DEG_TO_RAD) + acc[2] * cos((pitch + 90) * DEG_TO_RAD) * cos(roll * DEG_TO_RAD)) * sin(yaw_g * DEG_TO_RAD) +
         (acc[1] * cos(roll * DEG_TO_RAD) + acc[2] * cos((roll + 90) * DEG_TO_RAD) * cos(pitch * DEG_TO_RAD)) * cos(yaw_g * DEG_TO_RAD);
}
float computeGlobalAccelZ() {
  return acc[0] * sin(pitch * DEG_TO_RAD) + acc[1] * sin(roll * DEG_TO_RAD) + acc[2] * cos(roll * DEG_TO_RAD) * cos(pitch * DEG_TO_RAD);
}

// ===============================
// 加速度の更新と振動検出
// ===============================
void updateAcceleration() {
  for (int i = 0; i < 3; i++) acc[i] = c * acc[i] + (1 - c) * acc_prev[i];

  acc_global[0] = computeGlobalAccelX();
  acc_global[1] = computeGlobalAccelY();
  acc_global[2] = computeGlobalAccelZ();

  for (int i = 0; i < 3; i++) acc_global[i] = c * acc_global[i] + (1 - c) * acc_global_prev[i];

  baibx = fabsf(acc_global[0]) > 0.5;
  baiby = fabsf(acc_global[1]) > 0.5;
  baibz = fabsf(acc_global[2] - acc_global_prev[2]) > 0.04 ? 0 : 0;
}

// ===============================
// 速度更新（簡易積分）
// ===============================
void updateVelocity() {
  for (int i = 0; i < 3; i++) {
    vel[i] = (1 - d) * vel_prev[i] + d * (acc_global[i] + acc_global_prev[i]) * T / 2.0;
  }
}

// ===============================
// 位置更新（簡易積分）
// ===============================
void updatePosition() {
  for (int i = 0; i < 3; i++) {
    pos[i] += (vel[i] + vel_prev[i]) * T / 2.0;
  }
}

// ===============================
// モータ制御処理（シリアル入力に基づく）
// ===============================
void updateMotorState() {
  in0 = in;
  if (Serial1.available() > 0) {
    in = Serial1.read();
    if (in < '1' || in > '5') in = in0;
  } else {
    in = in0;
  }

  int flashRate = 0;
  float flashRate_deno = 0;
  if (in == '1') {flashRate = 20; flashRate_deno = 0.4;} 
  else if (in == '2') {flashRate = 40; flashRate_deno = 0.2;}
  else if (in == '3') {flashRate = 80; flashRate_deno = 0.125;}
  else if (in == '4') {flashRate = 100; flashRate_deno = 0.9;}
  else {
    digitalWrite(motorPin, LOW); 
    digitalWrite(LEDR, HIGH); 
    digitalWrite(LEDG, LOW); 
    return;
  }

  digitalWrite(LEDG, HIGH);
  bool motorOn = (l % flashRate < flashRate * flashRate_deno);
  digitalWrite(motorPin, motorOn ? HIGH : LOW);
  digitalWrite(LEDR, motorOn ? LOW : HIGH);
}

// ===============================
// データ送信（シリアル通信）
// ===============================
void outputDataAsBytes() {
  int16_t angle_pitch = (int16_t)(pitch * 10.0);
  int16_t angle_roll  = (int16_t)(roll * 10.0);
  int16_t angle_yaw   = (int16_t)(yaw_g * 10.0);
  int16_t bend_to_send = (int16_t)(bend * 10.0);  

  uint8_t buffer[sizeof(int16_t) * 4];
  memcpy(buffer, &angle_pitch, sizeof(int16_t));
  memcpy(buffer + sizeof(int16_t), &angle_roll, sizeof(int16_t));
  memcpy(buffer + 2 * sizeof(int16_t), &angle_yaw, sizeof(int16_t));
  memcpy(buffer + 3 * sizeof(int16_t), &bend_to_send, sizeof(int16_t));

  Serial1.write('S');  // ヘッダ送信
  Serial1.write(buffer, sizeof(buffer));
}

// ===============================
// 初期化処理
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
  // 最初のループのみ初期キャリブレーションを実行
  if (l == 0) calibrateSensors();

  // 曲げセンサの値を読み取り、初期値を引いて平滑化
  bend_past = bend;
  bend = analogRead(ANALOG_PIN) - bend0;
  bend = b * bend + (1 - b) * bend_past;

  // 直前の加速度・グローバル加速度・速度を保存（次の更新で使用）
  for (int i = 0; i < 3; i++) {
    acc_prev[i] = acc[i];
    acc_global_prev[i] = acc_global[i];
    vel_prev[i] = vel[i];
  }

  // 直前の姿勢角を保存（平滑化に使用）
  pitch0 = pitch; roll0 = roll; yaw_g0 = yaw_g;

  // IMU（加速度・ジャイロ・地磁気）のデータを読み取って補正
  readIMUSensors(false);

  // シリアル入力値に応じてモータをON/OFF制御
  updateMotorState();

  // Madgwickフィルタにより姿勢角（ピッチ・ロール・ヨー）を更新
  updateOrientation();

  // 加速度値をグローバル座標系に変換し、ノイズ除去・振動判定
  updateAcceleration();

  // 加速度をもとに速度を更新（簡易積分）
  updateVelocity();

  // 速度をもとに位置を更新（簡易積分）
  updatePosition();

  // ピッチ・ロール・ヨー角、曲げ量をシリアル通信で送信
  outputDataAsBytes();

  // ループカウンタ更新（4000を超えたら1に戻す）
  l++;
  if (l > 4000) l = 1;

  // 1ms待機（ループ周期制御）
  delay(1);
}
