// ===============================
// 必要なライブラリの読み込み
// ===============================
#include <Arduino_LSM9DS1.h> // LSM9DS1 IMU用ライブラリ
// MadgwickAHRS.h を排除し、自前クォータニオン姿勢推定(Mahony風)を実装

// ===============================
// ピン定義
// ===============================
#define ANALOG_PIN 7 // 曲げセンサ接続アナログピン
#define motorPin 9   // モーター出力ピン

// 自前姿勢推定用クォータニオン (q0 + q1*i + q2*j + q3*k)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Mahony風フィードバックゲイン
float Kp = 2.0f;                                                  // 比例ゲイン（大きいほど加速度/地磁気へ早く収束）
float Ki = 0.0f;                                                  // 積分ゲイン（必要ならドリフト抑制用に有効化）
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // 積分項

unsigned long lastMicros = 0; // Δt計算用

// ===============================
// 状態変数・初期化用
// ===============================
int in = 5, in0 = 5;                      // シリアル入力値（前回値と現在値）
int l = 0;                                // メインループカウンタ
float bend0 = 0, bend_past = 0, bend = 0; // 曲げセンサ値（初期基準・過去・現在）

// ===============================
// フィルタリング・補正係数
// ===============================
float a = 0.1;          // 地磁気補正係数
float p = 0.5, r = 0.5; // ピッチ・ロール平滑化係数
float yaw_y = 0.3;      // ヨー角平滑化係数
float b = 0.1;          // 曲げセンサ平滑化係数
float T = 0.01;         // サンプリング周期（秒）（実際は毎ループで更新）
float c = 0.1, d = 0.1; // 加速度・速度のフィルタ係数

// ===============================
// 地磁気フェード設定（ピッチ角依存）
// ピッチ角の絶対値 |pitch| が start 以下では地磁気100%、end 以上では0%。
// その間は線形にフェードアウトします。
// 値は度数法[deg]。
// 例: 45degから減衰を開始し、75degで完全に無効化。
const float MAG_FADE_START_DEG = 90.0f;
const float MAG_FADE_END_DEG = 90.0f;

// ===============================
// センサ補正用のオフセット・スケール
// ===============================
// 地磁気オフセットベクトル（mag_offset）
float mag_offset[3] = {-3.7991, 46.8373, -13.9604};

// 楕円体補正行列（mag_correction）※3x3対称行列
float mag_correction[3][3] = {
    {0.0538, 0.0020, 0.0027},
    {0.0020, 0.0401, -0.0003},
    {0.0027, -0.0003, 0.0456}};

float acc_offset[3] = {-0.02, 0.01, 0.0}; // 加速度オフセット
float gyro_offset[3] = {0.0, 0.0, 0.0};   // ジャイロオフセット（初期補正で決定）

// ===============================
// IMUデータ格納用の配列
// ===============================
float acc[3] = {0}, gyro[3] = {0}, mag[3] = {0}; // センサ値
float acc_prev[3] = {0}, acc_global[3] = {0, 0, 1}, acc_global_prev[3] = {0, 0, 1};
float vel[3] = {0}, vel_prev[3] = {0}; // 速度（各軸）
float pos[3] = {0};                    // 位置（各軸）

// ===============================
// 姿勢角・ヨー角補正用
// ===============================
float pitch = 0, roll = 0, yaw = 0;
float pitch0 = 0, roll0 = 0, yaw_g0 = 0;
float yaw_sim = 0, yaw_s = 0, yaw_sm = 0;
float yaw_m = 0;         // 出力用ヨー角（初期オフセット基準）
float gyro_sim[3] = {0}; // 初期ジャイロ値積算用

// ===============================
// 振動検出用フラグ
// ===============================
int baibx, baiby, baibz;

// ===============================
// IMUの初期化関数
// ===============================
void initializeIMU()
{
  if (!IMU.begin())
  {
    while (1)
      ; // 初期化失敗時に停止
  }
  lastMicros = micros();
}

// ===============================
// 初期キャリブレーション関数
// ===============================
void calibrateSensors()
{
  digitalWrite(LEDG, HIGH);
  for (int i = 0; i < 300; i++)
  {
    digitalWrite(LEDR, i % 10 < 5 ? LOW : HIGH);
    readIMUSensors(true);
    delay(10);
  }

  // クォータニオン初期安定化（約15秒）
  for (int i = 0; i < 1500; i++)
  {
    digitalWrite(LEDR, i % 100 < 50 ? LOW : HIGH);
    readIMUSensors(true);
    // 仮のΔt（10ms固定）で姿勢更新
    // NOTE: キャリブレーション中は時間精度より安定化優先のため固定値
    float dt = 0.01f;
    // 姿勢更新
    // （地磁気を含む更新関数は後方で定義）
    // updateQuaternion 定義後にコンパイラが解決できるようプロトタイプを前方宣言してもよいが
    // ここでは後方に実装されるため関数プロトタイプ宣言を追加する。
    // → ファイル先頭付近にプロトタイプを追加済みではないので後方に定義を移動し再度読み直し。
    // 暫定: ここでは一旦後で置換されるダミー呼び出し(後で本体実装)。
    // 実装後: updateQuaternion(dt);
    // ---- 呼び出し ----
    // 本体はファイル後半に追加
    extern void updateQuaternion(float dt);
    updateQuaternion(dt);
    for (int j = 0; j < 3; j++)
      gyro_sim[j] += gyro[j];
    delay(10);
  }

  for (int i = 0; i < 3; i++)
    gyro_offset[i] = gyro_sim[i] / 500.0;

  // 現在のクォータニオンから初期ヨー角オフセット算出
  float yaw_raw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;
  yaw_sm = yaw_raw; // 初期オフセット
  yaw_m = 0.0f;
  bend0 = analogRead(ANALOG_PIN);

  for (int i = 0; i < 10; i++)
  {
    digitalWrite(LEDG, HIGH);
    digitalWrite(motorPin, HIGH);
    delay(50);
    digitalWrite(LEDG, LOW);
    digitalWrite(motorPin, LOW);
    delay(50);
  }
}

// ===============================
// IMU読み取り・補正
// ===============================
void readIMUSensors(bool applyOffset)
{
  IMU.readAcceleration(acc[0], acc[1], acc[2]);
  IMU.readGyroscope(gyro[0], gyro[1], gyro[2]);
  IMU.readMagneticField(mag[0], mag[1], mag[2]);

  // 一時変数に地磁気読み取り値を保存
  float mag_raw[3];
  for (int i = 0; i < 3; i++)
  {
    mag_raw[i] = mag[i] - mag_offset[i];
  }

  // 楕円体補正（A行列適用：mag = A × (raw - offset)）
  for (int i = 0; i < 3; i++)
  {
    mag[i] = 0;
    for (int j = 0; j < 3; j++)
    {
      mag[i] += mag_correction[i][j] * mag_raw[j];
    }
  }

  // ★ 地磁気ベクトルの正規化を追加（安全チェック付き）★
  float mag_norm = sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
  if (mag_norm > 0.01)
  {
    for (int i = 0; i < 3; i++)
      mag[i] /= mag_norm;
  }

  // ジャイロ・加速度のオフセット補正
  if (applyOffset)
  {
    for (int i = 0; i < 3; i++)
      gyro[i] += gyro_offset[i];
  }
  else
  {
    gyro[0] = -gyro[0] + gyro_offset[0];
    gyro[1] = -gyro[1] + gyro_offset[1];
    gyro[2] = gyro[2] + gyro_offset[2] - 0.0015;
    for (int i = 0; i < 3; i++)
      acc[i] += acc_offset[i];
  }

  // フィルタ更新はループ側で updateQuaternion() を呼ぶ
}

// ===============================
// 姿勢角（ピッチ・ロール・ヨー）更新
// ===============================
void updateOrientation()
{
  // クォータニオン -> オイラー角 (Tait-Bryan ZYX: yaw(z), pitch(y), roll(x))
  float roll_raw = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
  float pitch_raw = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / PI;
  float yaw_raw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;

  // 初期オフセット補正
  float yaw_now = yaw_raw - yaw_sm;
  if (yaw_now > 180.0f)
    yaw_now -= 360.0f;
  if (yaw_now < -180.0f)
    yaw_now += 360.0f;

  // 平滑化
  pitch = pitch_raw * p + (1 - p) * pitch;
  roll = roll_raw * r + (1 - r) * roll;
  yaw_m = yaw_now * yaw_y + (1 - yaw_y) * yaw_g0; // yaw_g0 は前回 yaw_m を保持
}

// ===============================
// 加速度 → グローバル座標変換
// ===============================
// クォータニオンによるベクトル回転 ( acc_body -> acc_global )
void rotateAccelToGlobal(float ax, float ay, float az, float &gx, float &gy, float &gz)
{
  // q * v * q_conj （vは純虚部クォータニオン(0,ax,ay,az)）
  float qx = q1, qy = q2, qz = q3, qw = q0;
  // 1) q * v
  float vw = -(qx * ax + qy * ay + qz * az);
  float vx = qw * ax + (qy * az - qz * ay);
  float vy = qw * ay + (qz * ax - qx * az);
  float vz = qw * az + (qx * ay - qy * ax);
  // 2) (q * v) * q_conj
  gx = -vw * qx + vx * qw - vy * qz + vz * qy;
  gy = -vw * qy + vy * qw - vz * qx + vx * qz;
  gz = -vw * qz + vz * qw - vx * qy + vy * qx;
}

// ===============================
// 加速度の更新と振動検出
// ===============================
void updateAcceleration()
{
  for (int i = 0; i < 3; i++)
    acc[i] = c * acc[i] + (1 - c) * acc_prev[i];

  // クォータニオンで回転（重力は未除去。必要なら gz -= 1.0f などで補正）
  rotateAccelToGlobal(acc[0], acc[1], acc[2], acc_global[0], acc_global[1], acc_global[2]);

  for (int i = 0; i < 3; i++)
    acc_global[i] = c * acc_global[i] + (1 - c) * acc_global_prev[i];

  baibx = fabsf(acc_global[0]) > 0.5;
  baiby = fabsf(acc_global[1]) > 0.5;
  baibz = fabsf(acc_global[2] - acc_global_prev[2]) > 0.04 ? 0 : 0;
}

// ===============================
// 速度更新（簡易積分）
// ===============================
void updateVelocity()
{
  for (int i = 0; i < 3; i++)
  {
    vel[i] = (1 - d) * vel_prev[i] + d * (acc_global[i] + acc_global_prev[i]) * T / 2.0;
  }
}

// ===============================
// 位置更新（簡易積分）
// ===============================
void updatePosition()
{
  for (int i = 0; i < 3; i++)
  {
    pos[i] += (vel[i] + vel_prev[i]) * T / 2.0;
  }
}

// ===============================
// モータ制御処理（シリアル入力に基づく）
// ===============================
void updateMotorState()
{
  in0 = in;
  if (Serial1.available() > 0)
  {
    in = Serial1.read();

    // '6'受信時: 現在のヨー角を基準0に再設定（yaw_sm を更新）。
    // モードは変えないため、in は直前の値に戻す。
    if (in == '6')
    {
      float yaw_raw_now = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;
      yaw_sm = yaw_raw_now; // 以後 yaw_now = yaw_raw - yaw_sm = 0 で基準化
      in = in0;             // モーターモードは維持
    }

    if (in < '1' || in > '5')
      in = in0;
  }
  else
  {
    in = in0;
  }

  int flashRate = 0;
  float flashRate_deno = 0;
  if (in == '1')
  {
    flashRate = 20;
    flashRate_deno = 0.4;
  }
  else if (in == '2')
  {
    flashRate = 40;
    flashRate_deno = 0.2;
  }
  else if (in == '3')
  {
    flashRate = 80;
    flashRate_deno = 0.125;
  }
  else if (in == '4')
  {
    flashRate = 100;
    flashRate_deno = 0.9;
  }
  else
  {
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
void outputDataAsBytes()
{
  int16_t angle_pitch = (int16_t)(pitch * 10.0);
  int16_t angle_roll = (int16_t)(roll * 10.0);
  int16_t angle_yaw = (int16_t)(yaw_m * 10.0);
  int16_t bend_to_send = (int16_t)(bend * 10.0);

  uint8_t buffer[sizeof(int16_t) * 4];
  memcpy(buffer, &angle_pitch, sizeof(int16_t));
  memcpy(buffer + sizeof(int16_t), &angle_roll, sizeof(int16_t));
  memcpy(buffer + 2 * sizeof(int16_t), &angle_yaw, sizeof(int16_t));
  memcpy(buffer + 3 * sizeof(int16_t), &bend_to_send, sizeof(int16_t));

  Serial1.write('S'); // ヘッダ送信
  Serial1.write(buffer, sizeof(buffer));
}

// ===============================
// 初期化処理
// ===============================
void setup()
{
  Serial.begin(9600);
  Serial1.begin(115200);
  pinMode(motorPin, OUTPUT);
  initializeIMU();
  delay(100);
}

// ===============================
// メインループ処理
// ===============================
void loop()
{
  // 最初のループのみ初期キャリブレーションを実行
  if (l == 0)
    calibrateSensors();

  // 曲げセンサの値を読み取り、初期値を引いて平滑化
  bend_past = bend;
  bend = analogRead(ANALOG_PIN) - bend0;
  bend = b * bend + (1 - b) * bend_past;

  // 直前の加速度・グローバル加速度・速度を保存（次の更新で使用）
  for (int i = 0; i < 3; i++)
  {
    acc_prev[i] = acc[i];
    acc_global_prev[i] = acc_global[i];
    vel_prev[i] = vel[i];
  }

  // 直前の姿勢角を保存（平滑化に使用）
  pitch0 = pitch;
  roll0 = roll;
  yaw_g0 = yaw_m;

  // IMU（加速度・ジャイロ・地磁気）のデータを読み取って補正
  readIMUSensors(false);

  // Δt計算
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastMicros) * 1e-6f;
  lastMicros = nowMicros;
  if (dt <= 0.0f || dt > 0.1f)
    dt = T; // 異常値ガード
  T = dt;   // 他処理用に更新

  // 姿勢クォータニオン更新
  updateQuaternion(dt);

  // シリアル入力値に応じてモータをON/OFF制御
  updateMotorState();

  // クォータニオンから姿勢角へ
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
  if (l > 4000)
    l = 1;

  // 1ms待機（ループ周期制御）
  delay(1);
}

// ===============================
// クォータニオン更新関数 (Mahony風アルゴリズム)
// 入力: グローバル変数 gyro(rad/s), acc(g), mag(正規化済) / dt(s)
// ===============================
void updateQuaternion(float dt)
{
  // 調整ガイド:
  // 1) Kp: 大きくすると静止時の収束早いが動的環境で振動増える。1.0～5.0で調整。
  // 2) Ki: ジャイロバイアスが残る場合のみ 0.001〜0.05 程度で徐々に上げる。
  // 3) 地磁気がノイジーな環境（磁気干渉）では halfex/halfey/halfez の磁気項を無効化する実装分岐を追加推奨。
  // 4) 重力除去: グローバル加速度で動きのみ使いたい場合は updateAcceleration 内で acc_global[2] -= 1.0f などを検討。
  // 5) ドリフト評価: 十数秒静止させ yaw/pitch/roll の変化量をログし Kp/Ki を決める。
  // 6) 最適化: sqrtf 頻出箇所を近似 1/sqrt に変えてもOK (ARM CMSIS の arm_sqrt_f32 等)。
  float gx = gyro[0] * DEG_TO_RAD; // もともとdeg/sならRAD変換 (LSM9DS1はdps)
  float gy = gyro[1] * DEG_TO_RAD;
  float gz = gyro[2] * DEG_TO_RAD;

  // 加速度正規化
  float norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  if (norm > 0.0f)
  {
    float inv = 1.0f / norm;
    float ax = acc[0] * inv;
    float ay = acc[1] * inv;
    float az = acc[2] * inv;

    // 地磁気正規化（既に正規化済みだが二重防御）
    float mn = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
    float mxn = mag[0], myn = mag[1], mzn = mag[2];
    if (mn > 0.01f)
    {
      float invm = 1.0f / mn;
      mxn *= invm;
      myn *= invm;
      mzn *= invm;
    }

    // 参照重力（クォータニオンから期待される重力方向）
    float halfvx = q1 * q3 - q0 * q2;        // 0.5 * vx 実装簡略化 (Mahony実装流用形)
    float halfvy = q0 * q1 + q2 * q3;        // 0.5 * vy
    float halfvz = q0 * q0 - 0.5f + q3 * q3; // 0.5 * vz

    // 地磁気参照（Mahony拡張）
    float hx = 2.0f * (mxn * (0.5f - q2 * q2 - q3 * q3) + myn * (q1 * q2 - q0 * q3) + mzn * (q1 * q3 + q0 * q2));
    float hy = 2.0f * (mxn * (q1 * q2 + q0 * q3) + myn * (0.5f - q1 * q1 - q3 * q3) + mzn * (q2 * q3 - q0 * q1));
    float bx = sqrtf(hx * hx + hy * hy);
    float bz = 2.0f * (mxn * (q1 * q3 - q0 * q2) + myn * (q2 * q3 + q0 * q1) + mzn * (0.5f - q1 * q1 - q2 * q2));

    float halfwx = bx * (0.5f - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2); // 0.5 * wx
    float halfwy = bx * (q1 * q2 - q0 * q3) + bz * (q0 * q1 + q2 * q3);        // 0.5 * wy
    float halfwz = bx * (q0 * q2 + q1 * q3) + bz * (0.5f - q1 * q1 - q2 * q2); // 0.5 * wz

    // 誤差 = (観測ベクトル×参照ベクトル) の形を Mahony 公式ベースで簡略
    // 地磁気項はピッチ角が大きいほど無効化（ウェイトを0へ）
    // 現在のクォータニオンからピッチ角を直接算出（度）
    float sp = 2.0f * (q0 * q2 - q3 * q1);
    if (sp > 1.0f)
      sp = 1.0f;
    else if (sp < -1.0f)
      sp = -1.0f;
    float pitch_deg_now = asinf(sp) * 180.0f / PI;
    float pitch_abs = fabsf(pitch_deg_now);
    float mag_w = 1.0f;
    if (MAG_FADE_END_DEG <= MAG_FADE_START_DEG)
    {
      // フェード区間が無効な場合は閾値でステップ切り替え
      mag_w = (pitch_abs <= MAG_FADE_START_DEG) ? 1.0f : 0.0f;
    }
    else if (pitch_abs <= MAG_FADE_START_DEG)
    {
      mag_w = 1.0f;
    }
    else if (pitch_abs >= MAG_FADE_END_DEG)
    {
      mag_w = 0.0f;
    }
    else
    {
      float t = (pitch_abs - MAG_FADE_START_DEG) / (MAG_FADE_END_DEG - MAG_FADE_START_DEG);
      mag_w = 1.0f - t; // 線形フェード
    }

    float halfex = (ay * halfvz - az * halfvy) + mag_w * (myn * halfwz - mzn * halfwy);
    float halfey = (az * halfvx - ax * halfvz) + mag_w * (mzn * halfwx - mxn * halfwz);
    float halfez = (ax * halfvy - ay * halfvx) + mag_w * (mxn * halfwy - myn * halfwx);

    // 積分項
    if (Ki > 0.0f)
    {
      integralFBx += Ki * halfex * dt;
      integralFBy += Ki * halfey * dt;
      integralFBz += Ki * halfez * dt;
    }
    else
    {
      integralFBx = integralFBy = integralFBz = 0.0f;
    }

    // 補正付きジャイロ
    gx += Kp * halfex + integralFBx;
    gy += Kp * halfey + integralFBy;
    gz += Kp * halfez + integralFBz;
  }

  // クォータニオン微分 q_dot = 0.5 * q ⊗ ω
  float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // 積分
  q0 += qDot0 * dt;
  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;

  // 正規化
  float nq = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (nq > 0.0f)
  {
    float inv = 1.0f / nq;
    q0 *= inv;
    q1 *= inv;
    q2 *= inv;
    q3 *= inv;
  }
}
