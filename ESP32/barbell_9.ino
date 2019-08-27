//#include <MPU6050.h>
#include <Wire.h>
#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b

double offsetX = 0, offsetY = 0, offsetZ = 0;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float interval, preInterval;
float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;

//LEDとBuzzer用の変数の宣言
const byte LedPin = 4;      // LEDはデジタルピン4に接続
const byte BuzzerPin = 23;  // Buzzerはデジタルピン23に接続

void culculation();
void writeMPU6050();
void readMPU6050();


//追加内容(変数類)
//選んだ重さ(0~11の12段階)
byte selectedWeight   = 11;

//トレーニングの種類の数
#define TRAINING_COUNT              3     
//各トレーニングの持ち上げる高さ[m]
const double height[TRAINING_COUNT] = {0.80, 0.55, 0.30};//実際は{0.80, 0.65, 0.50}程度？
#define STATE_DEADLIFT              0
#define STATE_SQUAT                 1
#define STATE_BENCHPRESS            2

//重さ(適正速度)の種類の数
#define WEIGHT_COUNT                12    
//適正速度[m/s]([トレーニング数][重さの数(軽い順)][上げと下げ])(とりあえずトレーニングで変更はしていない)
const double rightSpeed[TRAINING_COUNT][WEIGHT_COUNT][2] = {{{0.80, 1.60}, {0.75, 1.50}, {0.70, 1.40}, {0.65, 1.30}, {0.60, 1.20}, {0.55, 1.10}, {0.50, 1.00}, {0.45, 0.90}, {0.40, 0.80}, {0.35, 0.70}, {0.30, 0.60}, {0.25, 0.50}},
                                                            {{0.80, 1.60}, {0.75, 1.50}, {0.70, 1.40}, {0.65, 1.30}, {0.60, 1.20}, {0.55, 1.10}, {0.50, 1.00}, {0.45, 0.90}, {0.40, 0.80}, {0.35, 0.70}, {0.30, 0.60}, {0.25, 0.50}},
                                                            {{0.80, 1.60}, {0.75, 1.50}, {0.70, 1.40}, {0.65, 1.30}, {0.60, 1.20}, {0.55, 1.10}, {0.50, 1.00}, {0.45, 0.90}, {0.40, 0.80}, {0.35, 0.70}, {0.30, 0.60}, {0.25, 0.50}}};
                             
#define CALIBRATION_COUNT           3000  //キャリブレーションのループ回数
unsigned int LiftUpCount = 0;             //持ち上げた回数
double offsetG           = 0;             //重力加速度の平均
double acc_sum           = 0;             //三軸の加速度の合成
double firstX            = 0;             //起動時の加速度
double firstZ            = 0;             //起動時の加速度
double realSpeed         = 0;             //速度[m/s]
double realDisplacement  = 0;             //変位[m]

//追加内容(関数とそこで使用する変数等)
//変位等リセット
void offsetData();

//アラートとカウント
void liftUp();
#define STATE_UP                    0
#define STATE_DOWN                  1
#define STATE_ERROR_SLOW            0
#define STATE_ERROR_FAST            1
#define STATE_ERROR_SLOPE           2
#define SLOW_ALEAT_BLINK_TIME       200   //遅すぎ時のアラームの速さ[ms]
#define FAST_ALEAT_BLINK_TIME       100   //速すぎ時のアラームの速さ[ms]
#define ACC_THRESHOLD               0.15  //持ち上げ下げ反転の閾値[G]
#define SLOPE_ACC_THRESHOLD         0.20  //x軸y軸傾きのアラームが出る閾値[G]
#define COUNT_WAIT_TIME             500   //持ち上げてから反応しない時間[ms]
#define DETECT_WAIT_COUNT           3     //トレーニングを判別するのに何回はカウントしないか
#define MOTION_HEIGHT_THRESHOLD     0.07  //各トレーニング方法で設定した高さとの違いを、どこまでそのトレーニングとして認めるか[m]
#define ALERT_THRESHOLD             0.10  //遅すぎ早すぎの閾値[m/s]
#define ERROR_CONTINUE_ALERT_TIME   1000  //遅すぎ早すぎが続きアラームが鳴るまでの時間[ms]
#define G_FILTER_COEFFICIENT        0.999 //重力加速度をとるフィルターに使う(0～1の範囲)
#define FILTER_COEFFICIENT          0.995 //速度をとるフィルターに使う(0～1の範囲)
#define RESET_SPEED_COEFFICIENT     0.02  //速度が何以下だったら静止状態とし、リセットするか[m/s]
#define RESET_WAITING_TIME          10000 //何秒間速度が上の値以下だったらリセットするか[ms]
#define STOP_ALERT_WAITING_TIME     1000  //何秒間速度が上の値以下だったらアラームをストップするか[ms]
byte errorState = 0;
byte minDifferenceTraining = 0;
byte training = TRAINING_COUNT, lastDetectedTraining = TRAINING_COUNT; //検出したトレーニング
boolean updownState = 0, last_updownState = 0;
unsigned int count = 0, motionDetectCount = 0;
double speed_sum = 0, errorSpeed = 0;
double max_height = 0, min_height = 0, last_min_height = 0;
double displacement = 0;
double heightDifference[TRAINING_COUNT] = {};

unsigned long countedTime       = 0;      //カウントした時間
unsigned long errorDetectedTime = 0;      //遅すぎ早すぎを検出した時間
unsigned long stoppedTime       = 0;      //静止状態に入った時間

#define printWaitTime 10
unsigned long printedTime = 0;

void setup(){

  //LEDとBuzzer用の出力の設定
  pinMode(LedPin, OUTPUT);     // LedPinを出力に設定
  pinMode(BuzzerPin, OUTPUT);  // BuzzerPinを出力に設定
  
  const byte SDA = 25;
  const byte SCL = 26;
  Wire.begin(SDA, SCL);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Serial.begin(115200);
  delay(100);
  
  //正常に接続されているかの確認
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68){
    Serial.println("\nWHO_AM_I error.");
    while(true);
  }

  //設定を書き込む
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);   // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // gyro range: ±500dps
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro

  //キャリブレーション
  Serial.print("Calculate Calibration");
  for(int i=0; i<CALIBRATION_COUNT; i++){
    
    int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);
  
    raw_acc_x  = Wire.read() << 8 | Wire.read();
    raw_acc_y  = Wire.read() << 8 | Wire.read();
    raw_acc_z  = Wire.read() << 8 | Wire.read();
    raw_t      = Wire.read() << 8 | Wire.read();
    raw_gyro_x = Wire.read() << 8 | Wire.read();
    raw_gyro_y = Wire.read() << 8 | Wire.read();
    raw_gyro_z = Wire.read() << 8 | Wire.read();
    acc_x = ((float)raw_acc_x) / 16384.0;
    acc_y = ((float)raw_acc_y) / 16384.0;
    acc_z = ((float)raw_acc_z) / 16384.0;
    dpsX = ((float)raw_gyro_x) / 65.5;
    dpsY = ((float)raw_gyro_y) / 65.5;
    dpsZ = ((float)raw_gyro_z) / 65.5;
    offsetX += dpsX;
    offsetY += dpsY;
    offsetZ += dpsZ;
    offsetG += sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
    firstX  += acc_x;
    firstZ  += acc_z;
    if(i % 1000 == 0){
      Serial.print(".");
    }
  }
  offsetX /= CALIBRATION_COUNT;
  offsetY /= CALIBRATION_COUNT;
  offsetZ /= CALIBRATION_COUNT;
  offsetG /= CALIBRATION_COUNT;
  firstX  /= CALIBRATION_COUNT;
  firstZ  /= CALIBRATION_COUNT;
  calculation();
  offsetData();
  
  Serial.println();
  Serial.print("offsetX : ");
  Serial.println(offsetX);
  Serial.print("offsetY : ");
  Serial.println(offsetY);
  Serial.print("offsetZ : ");
  Serial.println(offsetZ);
  Serial.print("offsetG : ");
  Serial.println(offsetG);

  //選んだ重さが不正でないか
  if(selectedWeight < 0){
    selectedWeight = 0;
  }else if(WEIGHT_COUNT-1 < selectedWeight){
    selectedWeight = WEIGHT_COUNT-1;
  }
  
  // LEDとBuzzerの動作
//  for(int LB=0; LB<5; LB++){
//    digitalWrite(LedPin, HIGH);     // LEDをオン
//    delay(250);                     // 1/4秒待つ
//    digitalWrite(LedPin, LOW);      // LEDをオフ
//    delay(250);                     // 1/4秒待つ
//    digitalWrite(BuzzerPin, HIGH);  // Buzzerをオン
//    delay(500);                     // 1/2秒待つ
//    digitalWrite(BuzzerPin, LOW);   // Buzzerをオフ
//    delay(500);                     // 1/2秒待つ
//  }
}

void loop() {

  calculation();
  liftUp();
  
  LiftUpCount = count;

  if(printWaitTime < millis()-printedTime){
//  switch(updownState){
//  case STATE_UP:
//    Serial.println("UP");
//    break;
//  case STATE_DOWN:
//    Serial.println("DOWN");
//    break;
//  }
//
    
//    Serial.print(speed_sum);
//    Serial.print(",");
//    Serial.print(errorSpeed);
//    Serial.print(",");
//    Serial.print(realSpeed);

//    Serial.print("  x : ");
//    Serial.print(angleX);
//    Serial.print("  y : ");
//    Serial.print(angleY);
//    Serial.print("  z: ");
//    Serial.print(angleZ);

//    Serial.print("  x : ");
//    Serial.print(acc_x);
//    Serial.print("  y : ");
//    Serial.print(acc_y);
//    Serial.print("  z: ");
//    Serial.print(acc_z);

    Serial.print("  count : ");
    Serial.print(count);
    Serial.print("  speed : ");
    Serial.print(realSpeed);
    Serial.print("  height : ");
    Serial.print(realDisplacement);
    Serial.print("  max height : ");
    Serial.print(max_height);
    Serial.print("  training : ");
    switch(training){
    case STATE_DEADLIFT:
      Serial.print("DEADLIFT");
      break;
    case STATE_SQUAT:
      Serial.print("SQUAT");
      break;
    case STATE_BENCHPRESS:
      Serial.print("BENCHPRESS");
      break;
    case TRAINING_COUNT:
      Serial.print("NOT FOUND");
      break;
    }
    if(training < TRAINING_COUNT){  //トレーニングが検出できているとき
      Serial.print("  right speed : ");
      Serial.print(rightSpeed[training][selectedWeight][updownState]);
    }
    Serial.println();
    printedTime = millis();
  }
}

void liftUp(){
  //持ち上げた回数のカウント
  if(acc_sum > offsetG+ACC_THRESHOLD){        //下ろしたとき
    updownState = STATE_DOWN;
  }else if(acc_sum < offsetG-ACC_THRESHOLD){  //持ち上げたとき
    updownState = STATE_UP;
  }
  if(updownState == STATE_UP){                //持ち上げているとき
    if(max_height < realDisplacement){        //高さの最大値更新
      max_height = realDisplacement;
    }
  }else if(updownState == STATE_DOWN){        //下ろしているとき
    if(displacement < min_height){            //高さの最小値(本来は0だが、高さのずれを記録)更新
      min_height = displacement;
    }
  }
  
  if(updownState!=last_updownState && updownState==STATE_UP && COUNT_WAIT_TIME<millis()-countedTime){ //持ち上げ始めたとき
    countedTime = millis();
    //トレーニング方法の判定
    minDifferenceTraining = 0;
    for(int i=0; i<TRAINING_COUNT; i++){  //各トレーニングごとに規定値からの差を計算
      heightDifference[i] = fabs(max_height-height[i]);
      if(heightDifference[i] < heightDifference[minDifferenceTraining]){  //一番差が小さいトレーニングを記録
        minDifferenceTraining = i;
      }
    }
    if(heightDifference[minDifferenceTraining] < MOTION_HEIGHT_THRESHOLD){  //一番差が小さいトレーニングの規定値からの差が一定値以内(トレーニングが検出できた)とき
      if(lastDetectedTraining == minDifferenceTraining){  //前回検出したトレーニングと同じなら
        motionDetectCount++;  //とりあえずカウント
        if(DETECT_WAIT_COUNT <= motionDetectCount+2){  //一定回数同じトレーニングをカウントしたら
          training = minDifferenceTraining;
          count = motionDetectCount+2;
        }
      }else if(TRAINING_COUNT <= training){
        motionDetectCount = 0;
      }
      lastDetectedTraining = minDifferenceTraining;
    }else if(TRAINING_COUNT <= training){
      motionDetectCount = 0;
      training = TRAINING_COUNT;              //どのトレーニングでもない
      lastDetectedTraining = TRAINING_COUNT;  //どのトレーニングでもない
    }
    offsetData();
  }
  if(fabs(realSpeed) < RESET_SPEED_COEFFICIENT){   //速度が一定以下の時(静止状態)
    if(RESET_WAITING_TIME < millis()-stoppedTime){ //その状態が一定時間以上続いたとき
      count = 0;
      training = TRAINING_COUNT;              //どのトレーニングでもない
      lastDetectedTraining = TRAINING_COUNT;  //どのトレーニングでもない
    }
  }else{
    stoppedTime = millis();
  }
  last_updownState = updownState;   //記録
  
  //アラート
  if(training<TRAINING_COUNT && millis()-stoppedTime<STOP_ALERT_WAITING_TIME){      //トレーニングが判定できていて、静止状態でもないとき
    double speedDifference = realSpeed - rightSpeed[training][selectedWeight][updownState];
    if(SLOPE_ACC_THRESHOLD<fabs(firstX-acc_x) || SLOPE_ACC_THRESHOLD<fabs(firstZ-acc_z)){     //一定以上x軸, y軸に傾いている
      errorState = STATE_ERROR_SLOPE;
    }else if(ALERT_THRESHOLD < speedDifference){  //速度が適正値より一定以上早い
      errorState = STATE_ERROR_FAST;
    }else if(speedDifference < -ALERT_THRESHOLD){ //速度が適正値より一定以上遅い
      errorState = STATE_ERROR_SLOW;
    }else{
      errorDetectedTime = millis();
    }
    if(ERROR_CONTINUE_ALERT_TIME < millis()-errorDetectedTime){ //一定時間以上不正値が続いたとき
      switch(errorState){
      case STATE_ERROR_SLOPE:
        digitalWrite(BuzzerPin, HIGH);
        Serial.println("SLOPE");
        break;
      case STATE_ERROR_FAST:
        digitalWrite(BuzzerPin, ((millis()-(ERROR_CONTINUE_ALERT_TIME+errorDetectedTime))/FAST_ALEAT_BLINK_TIME)%2);
        Serial.println("FASTER");
        break;
      case STATE_ERROR_SLOW:
        digitalWrite(BuzzerPin, ((millis()-(ERROR_CONTINUE_ALERT_TIME+errorDetectedTime))/SLOW_ALEAT_BLINK_TIME)%2);
        Serial.println("SLOWER");
        break;
      }
    }else{
      digitalWrite(BuzzerPin, LOW);
    }
  }else{
    digitalWrite(BuzzerPin, LOW);
  }
}

void offsetData(){
  //最低値更新と変数リセット
  last_min_height = min_height;
  min_height = max_height + last_min_height;
  max_height = 0;
}

//角度, 速度, 変位を計算
void calculation(){

  int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
  
  //レジスタアドレス0x3Bから、計14バイト分のデータを出力するようMPU6050へ指示
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  //出力されたデータを読み込み、ビットシフト演算
  raw_acc_x  = Wire.read() << 8 | Wire.read();
  raw_acc_y  = Wire.read() << 8 | Wire.read();
  raw_acc_z  = Wire.read() << 8 | Wire.read();
  raw_t      = Wire.read() << 8 | Wire.read();
  raw_gyro_x = Wire.read() << 8 | Wire.read();
  raw_gyro_y = Wire.read() << 8 | Wire.read();
  raw_gyro_z = Wire.read() << 8 | Wire.read();
  
  //単位Gへ変換
  acc_x = ((float)raw_acc_x) / 16384.0;
  acc_y = ((float)raw_acc_y) / 16384.0;
  acc_z = ((float)raw_acc_z) / 16384.0;

  //3軸の加速度を合成
  acc_sum = sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);

  //加速度センサーから角度を算出
  acc_angle_y = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
  acc_angle_x = atan2(acc_y, acc_z + abs(acc_x)) * 360 / 2.0 / PI;

  dpsX = ((float)raw_gyro_x) / 65.5; // LSB sensitivity: 65.5 LSB/dps @ ±500dps
  dpsY = ((float)raw_gyro_y) / 65.5;
  dpsZ = ((float)raw_gyro_z) / 65.5;
  
  //前回計算した時から今までの経過時間を算出
  interval = millis() - preInterval;
  preInterval = millis();
  
  //数値積分
  gyro_angle_x += (dpsX - offsetX) * (interval * 0.001);
  gyro_angle_y += (dpsY - offsetY) * (interval * 0.001);
  gyro_angle_z += (dpsZ - offsetZ) * (interval * 0.001);

  offsetG = G_FILTER_COEFFICIENT*offsetG + (1-G_FILTER_COEFFICIENT)*acc_sum;
  speed_sum    += 9.8 * (acc_sum - offsetG) * (interval * 0.001);

  //ずれを引いて純粋な速度及び変位を求める(前半はバンドパスフィルタ)
  errorSpeed = FILTER_COEFFICIENT*errorSpeed + (1-FILTER_COEFFICIENT)*speed_sum;
  realSpeed  = speed_sum - errorSpeed;
  displacement += realSpeed * (interval * 0.001);
  realDisplacement = displacement - last_min_height;
  
  //相補フィルター
  angleX = (0.996 * gyro_angle_x) + (0.004 * acc_angle_x);
  angleY = (0.996 * gyro_angle_y) + (0.004 * acc_angle_y);
  angleZ = gyro_angle_z;
  gyro_angle_x = angleX;
  gyro_angle_y = angleY;
  gyro_angle_z = angleZ;
}

//I2C書き込み
void writeMPU6050(byte reg, byte Data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(Data);
  Wire.endTransmission();
}

//I2C読み込み
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU6050_ADDR, 1/*length*/); 
  byte Data =  Wire.read();
  return Data;
}
