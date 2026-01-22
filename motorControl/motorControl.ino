/*
 * 电机驱动器控制与编码器脉冲频率读取
 * 使用 Feather RP2040
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
 * - 黄（A相）→ Feather D6
 * - 白（B相）→ Feather D5
 * 
 * 重要：编码器是"开漏输出(open-drain/open-collector)"
 * - 当前使用内部上拉(INPUT_PULLUP)进行测试
 * - 如需更稳定，建议使用外部10kΩ上拉电阻：
 *   * D6通过10kΩ电阻连接到3.3V
 *   * D5通过10kΩ电阻连接到3.3V
 */

// 电机驱动器PWM引脚定义
const int MOTOR_PWM1 = A0;  // 驱动器「A路电机 PWM 输入1」
const int MOTOR_PWM2 = A1;  // 驱动器「A路电机 PWM 输入2」

// 编码器引脚定义
const int ENCODER_A = D6;    // D6 - 编码器A相（黄线）
const int ENCODER_B = D5;    // D5 - 编码器B相（白线）

// 编码器相关变量
volatile long encoderCount = 0;        // 编码器脉冲计数（带符号，可检测方向）
volatile int lastEncoded = 0;          // 上次编码器状态
unsigned long lastTime = 0;            // 上次计算频率的时间
unsigned long frequencyInterval = 100; // 频率计算间隔（毫秒）

// PWM值范围：0-255
int motorSpeed = 0;  // 电机速度（0-255）
bool motorDirection = true;  // true为正转，false为反转

// 正弦波速度控制相关变量
bool sineWaveMode = true;    // 正弦波模式开关
int maxSpeed = 200;          // 正弦波最大速度（0-255）
float sinePeriod = 5000.0;   // 正弦波周期（毫秒），默认5秒一个周期
unsigned long sineStartTime = 0;  // 正弦波开始时间

// 编码器诊断相关变量
bool encoderDiagnosticMode = false;  // 编码器诊断模式开关
unsigned long lastEncoderDiagnosticTime = 0;

void setup() {
  // 初始化串口通信
  Serial.begin(115200);
  while (!Serial) {
    ; // 等待串口连接
  }
  
  Serial.println("电机驱动器控制与编码器频率读取系统");
  Serial.println("======================================");
  
  // 配置电机驱动器PWM引脚
  pinMode(MOTOR_PWM1, OUTPUT);
  pinMode(MOTOR_PWM2, OUTPUT);
  analogWrite(MOTOR_PWM1, 0);
  analogWrite(MOTOR_PWM2, 0);
  
  // 配置编码器引脚（输入，带内部上拉）
  // 重要：编码器是"开漏/集电极开路输出(open-drain/open-collector)"
  // A/B相必须上拉到3.3V才能正常工作
  // 
  // 上拉方式：
  // 1. 最稳定：外接10kΩ上拉电阻到3.3V（推荐用于生产环境）
  // 2. 测试用：使用INPUT_PULLUP内部上拉（当前方式，适合测试）
  //
  // 注意：如果编码器A相和B相始终是HIGH且无变化，可能是：
  // 1. 编码器未连接或未工作（未连接时上拉会让引脚保持HIGH）
  // 2. 编码器接线错误
  // 3. 内部上拉可能不够强，建议使用外部10kΩ上拉电阻
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  // 读取初始编码器状态
  lastEncoded = readEncoder();
  
  // 检查编码器引脚初始状态
  int aInit = digitalRead(ENCODER_A);
  int bInit = digitalRead(ENCODER_B);
  Serial.print("编码器初始状态 - A相(D6): ");
  Serial.print(aInit ? "HIGH" : "LOW");
  Serial.print(", B相(D5): ");
  Serial.println(bInit ? "HIGH" : "LOW");
  
  if (aInit == HIGH && bInit == HIGH) {
    Serial.println("警告：编码器A相和B相都是HIGH，请检查：");
    Serial.println("  1. 编码器是否已正确连接到D5和D6？");
    Serial.println("  2. 编码器电源（蓝线→3.3V，绿线→GND）是否已连接？");
    Serial.println("  3. 编码器是否在正常工作？");
    Serial.println();
    Serial.println("重要提示：");
    Serial.println("  编码器是开漏输出，当前使用内部上拉(INPUT_PULLUP)");
    Serial.println("  如果信号不稳定或无变化，建议使用外部10kΩ上拉电阻：");
    Serial.println("  - D5和D6分别通过10kΩ电阻连接到3.3V");
    Serial.println("  - 这样可以提供更稳定可靠的上拉");
  }
  
  // 配置编码器中断（在RP2040上，多个引脚可以触发中断）
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoder, CHANGE);
  
  // 初始化定时器
  lastTime = millis();
  sineStartTime = millis();
  
  Serial.println("系统初始化完成");
  Serial.println();
  Serial.println("=== 引脚配置 ===");
  Serial.print("电机PWM1引脚 (A0): ");
  Serial.println(MOTOR_PWM1);
  Serial.print("电机PWM2引脚 (A1): ");
  Serial.println(MOTOR_PWM2);
  Serial.print("编码器A相引脚 (D6): ");
  Serial.println(ENCODER_A);
  Serial.print("编码器B相引脚 (D5): ");
  Serial.println(ENCODER_B);
  Serial.println();
  Serial.println("输入命令：");
  Serial.println("  's' + 数字(0-255) - 设置固定速度，如 s100");
  Serial.println("  'sine' - 启用正弦波速度模式");
  Serial.println("  'fixed' - 启用固定速度模式");
  Serial.println("  'max' + 数字(0-255) - 设置正弦波最大速度，如 max200");
  Serial.println("  'period' + 数字(毫秒) - 设置正弦波周期，如 period5000");
  Serial.println("  'f' - 正转");
  Serial.println("  'r' - 反转");
  Serial.println("  'stop' - 停止电机");
  Serial.println("  'test' - 测试电机（低速运行2秒）");
  Serial.println("  'q' - 显示状态信息");
  Serial.println("  'd' - 显示PWM输出诊断信息");
  Serial.println("  'e' - 显示编码器诊断信息（实时监控）");
  Serial.println();
  Serial.println("提示：默认启用正弦波速度模式！");
  Serial.println();
}

void loop() {
  // 处理串口命令
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      handleCommand(command);
    }
  }
  
  // 正弦波速度模式更新
  if (sineWaveMode) {
    updateSineWaveSpeed();
  }
  
  // 编码器诊断模式
  if (encoderDiagnosticMode) {
    unsigned long currentTime = millis();
    if (currentTime - lastEncoderDiagnosticTime >= 100) {  // 每100ms显示一次
      showEncoderDiagnostic();
      lastEncoderDiagnosticTime = currentTime;
    }
  }
  
  // 定期计算并输出编码器脉冲频率
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= frequencyInterval) {
    calculateAndDisplayFrequency();
    lastTime = currentTime;
  }
}

// 读取编码器当前状态（4位二进制）
int readEncoder() {
  return (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
}

// 编码器中断服务函数
void updateEncoder() {
  int encoded = readEncoder();
  int sum = (lastEncoded << 2) | encoded;
  
  // 根据编码器状态变化判断方向并更新计数
  // 正交编码器的状态转换：00->01->11->10->00 (正转)
  //                     00->10->11->01->00 (反转)
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderCount++;  // 正转
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderCount--;  // 反转
  }
  
  lastEncoded = encoded;
}

// 显示编码器诊断信息
void showEncoderDiagnostic() {
  static int lastAState = -1;
  static int lastBState = -1;
  static unsigned long stateChangeCount = 0;
  static unsigned long lastChangeTime = 0;
  
  int aState = digitalRead(ENCODER_A);
  int bState = digitalRead(ENCODER_B);
  int encoderState = readEncoder();
  
  // 检测状态变化
  if (aState != lastAState || bState != lastBState) {
    stateChangeCount++;
    lastChangeTime = millis();
  }
  lastAState = aState;
  lastBState = bState;
  
  Serial.print("[编码器诊断] A相(D6):");
  Serial.print(aState ? "HIGH" : "LOW");
  Serial.print(" | B相(D5):");
  Serial.print(bState ? "HIGH" : "LOW");
  Serial.print(" | 状态值:");
  Serial.print(encoderState);
  Serial.print(" (二进制:");
  Serial.print(encoderState, BIN);
  Serial.print(") | 计数:");
  Serial.print(encoderCount);
  Serial.print(" | 状态变化次数:");
  Serial.print(stateChangeCount);
  
  if (stateChangeCount == 0) {
    Serial.print(" [未检测到变化 - 可能未连接或编码器未工作]");
  }
  Serial.println();
}

// 计算并显示编码器脉冲频率
void calculateAndDisplayFrequency() {
  static long lastCount = 0;
  long currentCount = encoderCount;
  long pulseCount = currentCount - lastCount;
  
  // 计算频率（脉冲/秒）
  // frequencyInterval是毫秒，需要转换为秒
  float frequency = (pulseCount * 1000.0) / frequencyInterval;
  
  // 计算转速（如果知道编码器分辨率）
  // 例如：如果编码器是1000 PPR（每转脉冲数），则转速 = 频率 / 1000 * 60 (RPM)
  // 这里假设为示例，实际需要根据您的编码器规格调整
  // float rpm = abs(frequency) / 1000.0 * 60.0;  // 假设1000 PPR
  
  // 输出信息
  Serial.print("编码器计数: ");
  Serial.print(currentCount);
  Serial.print(" | 脉冲频率: ");
  Serial.print(abs(frequency), 2);
  Serial.print(" Hz");
  Serial.print(" | 方向: ");
  Serial.print(frequency >= 0 ? "正转" : "反转");
  Serial.print(" | 电机速度: ");
  Serial.print(motorSpeed);
  Serial.print(" | 模式: ");
  Serial.print(sineWaveMode ? "正弦波" : "固定");
  Serial.print(" | 方向: ");
  Serial.print(motorDirection ? "正转" : "反转");
  
  // 如果编码器计数一直是0，显示编码器引脚状态用于诊断
  static unsigned long lastDiagnosticTime = 0;
  static long lastDiagnosticCount = 0;
  static int lastAState = -1;
  static int lastBState = -1;
  static bool hasSeenChange = false;
  
  // 检测是否有状态变化
  int currentAState = digitalRead(ENCODER_A);
  int currentBState = digitalRead(ENCODER_B);
  if (currentAState != lastAState || currentBState != lastBState) {
    hasSeenChange = true;
  }
  lastAState = currentAState;
  lastBState = currentBState;
  
  if (currentCount == 0 && lastDiagnosticCount == 0 && (millis() - lastDiagnosticTime > 2000)) {
    Serial.print(" [警告:编码器无信号] A相:");
    Serial.print(currentAState ? "H" : "L");
    Serial.print(" B相:");
    Serial.print(currentBState ? "H" : "L");
    if (!hasSeenChange) {
      Serial.print(" [引脚状态无变化 - 请检查接线]");
    } else if (currentAState == HIGH && currentBState == HIGH) {
      Serial.print(" [可能未连接 - 发送'e'命令查看详细诊断]");
    }
    lastDiagnosticTime = millis();
  }
  lastDiagnosticCount = currentCount;
  
  Serial.println();
  
  lastCount = currentCount;
}

// 设置电机速度（0-255）
void setMotorSpeed(int speed) {
  motorSpeed = constrain(speed, 0, 255);
  applyMotorSpeed();
}

// 应用当前速度到电机（内部函数）
void applyMotorSpeed() {
  if (motorDirection) {
    // 正转：PWM1输出速度，PWM2输出0
    analogWrite(MOTOR_PWM1, motorSpeed);
    analogWrite(MOTOR_PWM2, 0);
  } else {
    // 反转：PWM1输出0，PWM2输出速度
    analogWrite(MOTOR_PWM1, 0);
    analogWrite(MOTOR_PWM2, motorSpeed);
  }
}

// 更新正弦波速度
void updateSineWaveSpeed() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - sineStartTime;
  
  // 计算正弦波角度（0到2π）
  float angle = (float(elapsedTime % int(sinePeriod)) / sinePeriod) * 2.0 * PI;
  
  // 计算正弦值（-1到1），然后映射到0到maxSpeed
  float sineValue = sin(angle);
  
  // 将正弦值从[-1, 1]映射到[0, maxSpeed]
  // (sineValue + 1) / 2 将范围映射到[0, 1]
  // 然后乘以maxSpeed得到[0, maxSpeed]
  int newSpeed = int((sineValue + 1.0) / 2.0 * maxSpeed);
  
  // 更新速度（只在速度改变时应用，避免频繁调用analogWrite）
  if (abs(newSpeed - motorSpeed) > 0) {
    motorSpeed = newSpeed;
    applyMotorSpeed();
  }
}

// 设置电机方向
void setMotorDirection(bool forward) {
  motorDirection = forward;
  setMotorSpeed(motorSpeed);  // 重新设置速度以应用新方向
}

// 停止电机
void stopMotor() {
  motorSpeed = 0;
  sineWaveMode = false;  // 停止时禁用正弦波模式
  analogWrite(MOTOR_PWM1, 0);
  analogWrite(MOTOR_PWM2, 0);
  Serial.println("电机已停止");
}

// 处理串口命令
void handleCommand(String cmd) {
  if (cmd.startsWith("s") && cmd.length() > 1) {
    // 设置速度命令：s + 数字
    int speed = cmd.substring(1).toInt();
    if (speed >= 0 && speed <= 255) {
      setMotorSpeed(speed);
      Serial.print("电机速度设置为: ");
      Serial.println(speed);
    } else {
      Serial.println("错误：速度值必须在0-255之间");
    }
  } else if (cmd == "f" || cmd == "F") {
    // 正转
    setMotorDirection(true);
    Serial.println("电机方向：正转");
  } else if (cmd == "r" || cmd == "R") {
    // 反转
    setMotorDirection(false);
    Serial.println("电机方向：反转");
  } else if (cmd == "stop" || cmd == "STOP" || (cmd == "s" && cmd.length() == 1)) {
    // 停止
    stopMotor();
  } else if (cmd == "q" || cmd == "Q") {
    // 显示状态
    Serial.println("========== 系统状态 ==========");
    Serial.print("速度模式: ");
    Serial.println(sineWaveMode ? "正弦波模式" : "固定速度模式");
    Serial.print("电机速度: ");
    Serial.println(motorSpeed);
    if (sineWaveMode) {
      Serial.print("正弦波最大速度: ");
      Serial.println(maxSpeed);
      Serial.print("正弦波周期: ");
      Serial.print(sinePeriod / 1000.0);
      Serial.println(" 秒");
    }
    Serial.print("电机方向: ");
    Serial.println(motorDirection ? "正转" : "反转");
    Serial.print("编码器计数: ");
    Serial.println(encoderCount);
    Serial.println("==============================");
  } else if (cmd == "sine" || cmd == "SINE") {
    // 启用正弦波模式
    sineWaveMode = true;
    sineStartTime = millis();
    Serial.println("已启用正弦波速度模式");
    Serial.print("最大速度: ");
    Serial.print(maxSpeed);
    Serial.print(", 周期: ");
    Serial.print(sinePeriod / 1000.0);
    Serial.println(" 秒");
  } else if (cmd == "fixed" || cmd == "FIXED") {
    // 启用固定速度模式
    sineWaveMode = false;
    Serial.println("已启用固定速度模式");
  } else if (cmd.startsWith("max") && cmd.length() > 3) {
    // 设置正弦波最大速度
    int newMax = cmd.substring(3).toInt();
    if (newMax >= 0 && newMax <= 255) {
      maxSpeed = newMax;
      Serial.print("正弦波最大速度设置为: ");
      Serial.println(maxSpeed);
    } else {
      Serial.println("错误：最大速度值必须在0-255之间");
    }
  } else if (cmd.startsWith("period") && cmd.length() > 6) {
    // 设置正弦波周期
    float newPeriod = cmd.substring(6).toFloat();
    if (newPeriod > 0) {
      sinePeriod = newPeriod;
      Serial.print("正弦波周期设置为: ");
      Serial.print(sinePeriod / 1000.0);
      Serial.println(" 秒");
    } else {
      Serial.println("错误：周期必须大于0");
    }
  } else if (cmd == "test" || cmd == "TEST") {
    // 测试电机
    Serial.println("开始测试电机（低速运行2秒）...");
    bool oldMode = sineWaveMode;
    sineWaveMode = false;  // 临时禁用正弦波模式
    setMotorDirection(true);
    setMotorSpeed(100);  // 中等速度测试
    delay(2000);
    stopMotor();
    sineWaveMode = oldMode;  // 恢复原模式
    Serial.println("测试完成");
  } else if (cmd == "d" || cmd == "D") {
    // 诊断信息
    Serial.println("========== PWM诊断信息 ==========");
    Serial.print("PWM1引脚 (A0) 当前值: ");
    Serial.print(analogRead(MOTOR_PWM1));
    Serial.print(" (数字读取，可能不准确)");
    Serial.println();
    Serial.print("PWM2引脚 (A1) 当前值: ");
    Serial.print(analogRead(MOTOR_PWM2));
    Serial.print(" (数字读取，可能不准确)");
    Serial.println();
    Serial.print("编码器A相 (D6): ");
    Serial.println(digitalRead(ENCODER_A) ? "HIGH" : "LOW");
    Serial.print("编码器B相 (D5): ");
    Serial.println(digitalRead(ENCODER_B) ? "HIGH" : "LOW");
    Serial.print("编码器计数: ");
    Serial.println(encoderCount);
    Serial.println("===================================");
  } else if (cmd == "e" || cmd == "E") {
    // 编码器诊断模式
    encoderDiagnosticMode = !encoderDiagnosticMode;
    if (encoderDiagnosticMode) {
      Serial.println("编码器诊断模式已启用（每100ms显示一次状态）");
      Serial.println("发送 'e' 再次可关闭诊断模式");
      lastEncoderDiagnosticTime = millis();
    } else {
      Serial.println("编码器诊断模式已关闭");
    }
  } else {
    Serial.print("未知命令: ");
    Serial.println(cmd);
    Serial.println("可用命令: s[0-255], f, r, stop, test, q, d, e");
  }
}
