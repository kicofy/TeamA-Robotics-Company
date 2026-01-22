/*
 * 角度闭环控制（编码器反馈）
 * 目标：串口输入 "to+角度" 或 "to-角度"，电机自动调整到目标角度
 * 要求：误差在 ±2 度以内停止，若未到达会持续调整
 *
 * 接线说明：
 * 驱动器（A路）控制：
 * - Feather A0 → 驱动器「A路电机 PWM 输入1」
 * - Feather A1 → 驱动器「A路电机 PWM 输入2」
 * - Feather 3.3V → 驱动器「逻辑电源输入正极」
 * - Feather GND → 驱动器「电源负极」（共地）
 *
 * 编码器（3530B）：
 * - 蓝（编码器电源+）→ Feather 3.3V
 * - 绿（编码器电源−）→ Feather GND
 * - 黄（A相）→ Feather D5
 * - 白（B相）→ Feather D6
 *
 * 3530B 编码器参数（摘要）：
 * - AB 双相增量式磁性霍尔编码器
 * - 基础脉冲：16 PPR
 * - 输出脉冲数 = 16 PPR × 减速比
 * - 开漏输出，需要上拉电阻（建议 10kΩ 上拉到 3.3V）
 */

// 电机驱动器PWM引脚定义
const int MOTOR_PWM1 = A0;  // 驱动器「A路电机 PWM 输入1」
const int MOTOR_PWM2 = A1;  // 驱动器「A路电机 PWM 输入2」

// 编码器引脚定义（按你提供的接线）
const int ENCODER_A = D5;   // D5 - 编码器A相（黄线）
const int ENCODER_B = D6;   // D6 - 编码器B相（白线）

// 编码器计数
volatile long encoderCount = 0;
volatile int lastEncoded = 0;
volatile long encoderZero = 0;  // 初始零点

// 编码器参数
const float basePPR = 16.0; // 3530B 基础 PPR
float gearRatio = 1.0;      // 减速比（请按实际填写）

// 控制参数
const float angleTolerance = 2.0; // 允许误差（度）
const int minSpeed = 60;          // 最小PWM，避免卡死
const int maxSpeed = 150;         // 最大PWM
const float kp = 2.0;             // 比例系数（度 -> PWM）

// 目标角度
float targetAngleDeg = 0.0;
bool hasTarget = false;

// 电机状态
int motorSpeed = 0;
bool motorDirection = true;

// 控制周期
unsigned long lastControlTime = 0;
const unsigned long controlIntervalMs = 20;

float pulsesPerRevolution() {
  return basePPR * gearRatio;
}

// 读取编码器当前状态（2位）
int readEncoder() {
  return (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
}

// 编码器中断服务函数
void updateEncoder() {
  int encoded = readEncoder();
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderCount++;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderCount--;
  }

  lastEncoded = encoded;
}

long readEncoderCountSafe() {
  noInterrupts();
  long count = encoderCount;
  interrupts();
  return count;
}

float getAngleDegrees(long count) {
  return (count - encoderZero) * (360.0 / pulsesPerRevolution());
}

void applyMotorSpeed() {
  if (motorDirection) {
    analogWrite(MOTOR_PWM1, motorSpeed);
    analogWrite(MOTOR_PWM2, 0);
  } else {
    analogWrite(MOTOR_PWM1, 0);
    analogWrite(MOTOR_PWM2, motorSpeed);
  }
}

void stopMotor() {
  motorSpeed = 0;
  analogWrite(MOTOR_PWM1, 0);
  analogWrite(MOTOR_PWM2, 0);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  pinMode(MOTOR_PWM1, OUTPUT);
  pinMode(MOTOR_PWM2, OUTPUT);
  analogWrite(MOTOR_PWM1, 0);
  analogWrite(MOTOR_PWM2, 0);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  lastEncoded = readEncoder();
  encoderZero = encoderCount;

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoder, CHANGE);

  Serial.println("角度闭环控制已启动");
  Serial.println("命令：");
  Serial.println("  to+90 / to-45 / to90  -> 设定目标角度（相对零点）");
  Serial.println("  zero                  -> 设定当前位置为零点");
  Serial.println("  ratio100              -> 设置减速比（例如 100）");
  Serial.println("  status                -> 输出当前状态");
  Serial.println();
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("to")) {
      String value = cmd.substring(2);
      if (value.length() > 0) {
        targetAngleDeg = value.toFloat();
        hasTarget = true;
        Serial.print("目标角度设为: ");
        Serial.print(targetAngleDeg, 2);
        Serial.println(" deg");
      } else {
        Serial.println("错误：请输入 to+角度，例如 to+90");
      }
    } else if (cmd == "zero") {
      encoderZero = readEncoderCountSafe();
      targetAngleDeg = 0.0;
      hasTarget = false;
      stopMotor();
      Serial.println("已设为零点");
    } else if (cmd.startsWith("ratio")) {
      String value = cmd.substring(5);
      float newRatio = value.toFloat();
      if (newRatio > 0.0) {
        gearRatio = newRatio;
        Serial.print("减速比设为: ");
        Serial.println(gearRatio, 2);
      } else {
        Serial.println("错误：减速比必须大于0");
      }
    } else if (cmd == "status") {
      long count = readEncoderCountSafe();
      Serial.print("计数: ");
      Serial.print(count);
      Serial.print(" | 当前角度: ");
      Serial.print(getAngleDegrees(count), 2);
      Serial.print(" deg | 目标角度: ");
      Serial.print(targetAngleDeg, 2);
      Serial.print(" deg | 误差: ");
      Serial.print(targetAngleDeg - getAngleDegrees(count), 2);
      Serial.println(" deg");
    } else if (cmd.length() > 0) {
      Serial.print("未知命令: ");
      Serial.println(cmd);
    }
  }

  unsigned long now = millis();
  if (now - lastControlTime >= controlIntervalMs) {
    lastControlTime = now;

    if (hasTarget) {
      long count = readEncoderCountSafe();
      float currentAngle = getAngleDegrees(count);
      float error = targetAngleDeg - currentAngle;
      float absError = abs(error);

      if (absError <= angleTolerance) {
        stopMotor();
      } else {
        motorDirection = (error >= 0.0);
        int speed = (int)(kp * absError);
        if (speed < minSpeed) speed = minSpeed;
        if (speed > maxSpeed) speed = maxSpeed;
        motorSpeed = speed;
        applyMotorSpeed();
      }
    }
  }
}
