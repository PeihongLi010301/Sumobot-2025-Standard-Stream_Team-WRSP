/* Sumobot 2025 – Standard Stream
 * 代码作者: Peihong Li (李培泓), Zhengyang Yu (余政阳)
 *
 * 简介:
 *  - 三向超声 (前/左/右) + 双IR边缘检测 的对抗机器人
 *  - 以状态机 (SEARCHING / ATTACKING / AVOIDING_EDGE) 管理策略，边缘回避优先级最高
 *  - 进攻采用“50ms步长”的持续逼抢，每步都复核边缘/目标，安全且响应快
 *  - 关键改进：将PID输出由“PWM幅值”改为“固定功率下的转向时长(ms)” 结合一阶滤波、积分限幅与最大输出限制，显著提升低速段纠偏稳定性
 *
 * Overview:
 *  - Sumo robot using 3 HC-SR04 sonars (front/left/right) + dual IR line sensors.
 *  - Finite-state machine: SEARCHING / ATTACKING / AVOIDING_EDGE (edge-avoid has highest priority).
 *  - Continuous attack in 50ms steps to re-check edge/opponent frequently (safe & responsive).
 *  - Key improvement: time-based PID turning (ms) with filtering, anti-windup and output cap,
 *    much more stable than PWM-difference at low speed.
 *
 * 硬件连接 Pin Mapping:
 *  - L298N: ENA=D10(右), ENB=D11(左), R_FWD=D9, R_BWD=D8, L_BWD=D7, L_FWD=D6
 *  - 超声波: Front TRIG=D3/ECHO=D2, Left TRIG=D5/ECHO=D4, Right TRIG=A0/ECHO=A1
 *  - IR边缘: 左 A2 (白边时高), 右 A3 (白边时低)
 *
 * 关键参数 Key Params:
 *  - EDGE_DETECT_CONFIRM_COUNT=2：IR需连续命中2次才判边缘，抑制抖动
 *  - ATTACK_STEP=50ms, MAX_ATTACK_TIME=1200ms：逼抢步长 最长时长
 *  - PID: kp=4.0, ki=0.8, kd=1.5；积分限幅±50；MAX_CONTROLLER_OUTPUT=500ms
 *  - DEGREES_TO_MS=4.4：测得的机器人以满功率（PWM=255）转动 1° 所需的时间 ≈ 4.4 毫秒
 *  - 90°定时转向 delay(400)：实测所得
 *
 * 调参建议 Tuning Tips:
 *  - IR阈值：EDGE_THRESHOLD_LEFT/RIGHT 依据赛道白边反射特性微调
 *  - 攻击节奏：ATTACK_STEP 越小越灵敏但CPU 循环频率太高，频繁 delay + 传感器读取，会浪费时间在I/O上，可能还没前进出明显位移就被检测打断，逼抢动作变得断断续续，故选用50ms
 *  - PID：先调 kp 至能稳定纠偏，再加 kd 抑制抖动，最后加小幅 ki 修正系统性偏差
 *  - 最大输出：MAX_CONTROLLER_OUTPUT 控制单次转向上限，防呆/防暴走
 *
 * 安全策略 / Safety:
 *  - detect_edge() 结果一旦成立，立即抢占，执行 avoid_edge()（停→退→90°转）
 *  - 所有长动作均设置上限（如攻击总时长），防止卡死
 *
 * 已知限制 / Known Limits:
 *  - 超声测距存在盲区（<2cm无法探测）与偶发timeout，已通过滤波/上限处理提高鲁棒性
 *  - 90°转向基于定时，非闭环角度；如需精确角度，需改用带编码器的电机（本赛禁用）
 */

// -------------------- Motor control --------------------
#define ENA 10 // right PWM (L298N ENB/ENA命名随接线)
#define ENB 11 // left  PWM
#define MOTOR_RIGHT_FORWARD 9
#define MOTOR_RIGHT_BACKWARD 8
#define MOTOR_LEFT_BACKWARD 7
#define MOTOR_LEFT_FORWARD 6

// -------------------- IR sensors (edge detection) --------------------
#define IR_SENSOR_LEFT A2   // 左IR：白边时读数高（~920），阈值设高些
#define IR_SENSOR_RIGHT A3  // 右IR：白边时读数低（~30），阈值设低些

// -------------------- Ultrasonic (HC-SR04) --------------------
#define TRIG_PIN_LEFT 5
#define ECHO_PIN_LEFT 4
#define TRIG_PIN_FRONT 3 
#define ECHO_PIN_FRONT 2
#define TRIG_PIN_RIGHT A0
#define ECHO_PIN_RIGHT A1

// Speed of sound cm/us (未直接使用，保留为可能的精确定义)
#define SPEED_OF_SOUND 0.034

// Sonar clamp range
#define MAX_DISTANCE 90   // 超出视为无敌人
#define MIN_DISTANCE 2    // 超声波传感器自身的探测下限

// ---------- Edge Detection Constants ----------
#define EDGE_THRESHOLD_LEFT 700   // 左IR高于此认为白边
#define EDGE_THRESHOLD_RIGHT 50   // 右IR低于此认为白边
#define WHITE_SURFACE 1
#define BLACK_SURFACE 0

// ---------- Edge detection debounce ----------
bool left_edge_detected = false;
bool right_edge_detected = false;
int edge_detect_count = 0;                  // 连续确认计数
const int EDGE_DETECT_CONFIRM_COUNT = 2;    // 去抖：至少命中2次才判定

// ---------- PID (time-based output in ms) ----------
float kp = 4.0;
float ki = 0.8;
float kd = 1.5;

float prev_error = 0;
float error = 0;
float integral = 0;                 // 抗积分饱和在控制函数中
float controller_output = 0;        // 输出：转向时间 (ms)
float MAX_CONTROLLER_OUTPUT = 500;  // 单次最大转向时长 (ms)

int flag = 0;                       // 1:right, 2:left（用于选择误差来源）
float filtered_error = 0;           // 一阶滤波误差
float filtered_dir_error = 0;

// Rotation constant (if need angle-based timing)
#define DEGREES_TO_MS 4.4

// ---------- FSM ----------
enum RobotState { SEARCHING, ATTACKING, AVOIDING_EDGE };
RobotState currentState = SEARCHING;

void setup() {
  Serial.begin(9600);

  // Motors
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);

  // IR
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);

  // Sonars
  pinMode(TRIG_PIN_FRONT, OUTPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);
  pinMode(TRIG_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);

  delay(5000); // 开赛前预留校准/放置时间
  Serial.println("Sumobot initialized!");
}

void loop() {
  // 1) 边缘优先：一旦探测到边缘，立刻进入回避流程
  if (detect_edge()) {
    currentState = AVOIDING_EDGE;
    avoid_edge();
    integral = 0;     // 切状态时重置积分，防止残留
    return;
  }

  // 2) 正常对抗逻辑
  currentState = SEARCHING;
  String direction = detect_robot_dir();  // 基于三向最小距离选择方向

  if (direction == "front") {
    currentState = ATTACKING;
    attack_continuously();  // 50ms 步长的持续逼抢（内含边缘/目标复核）
    integral = 0;           // 直行后复位积分
  } else if (direction == "right") {
    flag = 1;
    turn_right_time(get_dir_error(flag));  // 时间式PID：输出为delay(ms)
  } else if (direction == "left"){
    flag = 2;
    turn_left_time(get_dir_error(flag));
  } else {
    rotate();  // 未锁定目标：原地对转进行检索
  }
}

// 持续逼抢：每50ms复核一次边缘与目标，以安全与响应速度为平衡
void attack_continuously() {
  int attack_duration = 0;
  const int MAX_ATTACK_TIME = 1200; // 单次逼抢上限
  const int ATTACK_STEP = 50;       // 50ms步长

  while (attack_duration < MAX_ATTACK_TIME) {
    if (detect_edge()) {
      stop_motors();
      move_backward();
      delay(10);
      return;
    }

    String current_direction = detect_robot_dir();
    if (current_direction != "front") {
      stop_motors(); // 目标偏移，立即停止本次逼抢
      return;
    }

    move_forward();
    delay(ATTACK_STEP);
    attack_duration += ATTACK_STEP;
  }
  return;
}

// 原地对转（右轮前进，左轮后退）
void rotate() {
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  analogWrite(ENB, 255);

  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  analogWrite(ENA, 255);
}

// ---- 边缘检测（带去抖）----
bool detect_edge() {
  int left_sensor = analogRead(IR_SENSOR_LEFT);
  int right_sensor = analogRead(IR_SENSOR_RIGHT);

  // 判定：左高于阈值/右低于阈值即视为白边
  left_edge_detected = (left_sensor > EDGE_THRESHOLD_LEFT);
  right_edge_detected = (right_sensor < EDGE_THRESHOLD_RIGHT);

  if (left_edge_detected || right_edge_detected) {
    edge_detect_count++;     // 连续命中次数+1
  } else {
    edge_detect_count = 0;   // 只要一次不命中就清零
  }
  return edge_detect_count >= EDGE_DETECT_CONFIRM_COUNT;
}

// ---- 回避流程：停→退→按侧转90°（定时），最后清状态 ----
void avoid_edge() {
  stop_motors();
  delay(10);

  move_backward();
  delay(300);

  stop_motors();
  delay(10);

  if (left_edge_detected && !right_edge_detected) {
    turn_right_90();
  } else if (right_edge_detected && !left_edge_detected) {
    turn_left_90();
  } else if (left_edge_detected && right_edge_detected) {
    // 双侧同时触边：加大后退并右转（可改为随机）
    move_backward();
    delay(300);
    turn_right_90();
  }

  // 清空缓存，避免连锁触发
  left_edge_detected = false;
  right_edge_detected = false;
  edge_detect_count = 0;

  stop_motors();
  delay(10);
}

// ---- 基于三向距离的对手方位判定（取最小者）----
String detect_robot_dir() {
  float front_dist = front_distance();
  float left_dist = left_distance();
  float right_dist = right_distance();

  if ((front_dist < left_dist) && (front_dist < right_dist)) {
    return "front";
  } else if ((left_dist < front_dist) && (left_dist < right_dist)) {
    return "left";
  } else if ((right_dist < front_dist) && (right_dist < left_dist)) {
    return "right";
  } else {
    return "None";
  }
}

// -------------------- Motion primitives --------------------
void move_forward() {
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

void move_backward() {
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
}

void stop_motors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

// 90°定时转向：400ms为实测经验值
void turn_left_90() {
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  delay(400);
  stop_motors();
}

void turn_right_90() {
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
  delay(400);
  stop_motors();
}

// ---- 方向误差：使用 |F-R| 或 |F-L|，并做轻滤波 ----
float get_dir_error(int flag) {
  float raw_error;
  if (flag == 1) {
    raw_error = abs(front_distance() - right_distance());
  } else if (flag == 2) {
    raw_error = abs(front_distance() - left_distance());
  } else {
    raw_error = 0;
  }
  // 轻滤波，抑制超声瞬时噪声
  filtered_dir_error = 0.8 * filtered_dir_error + 0.2 * raw_error;

  Serial.print("Error:");
  Serial.println(filtered_dir_error);
  return filtered_dir_error;
}

// ---- 时间式PID控制器：输出为“转向时长 (ms)” ----
float time_controller(float error) {
  // 一阶低通滤波，稳定误差
  filtered_error = 0.9 * filtered_error + 0.1 * error;

  // 小误差死区：避免无意义微动
  if (filtered_error < 1.5) {
    return 0;
  }

  integral += filtered_error;
  // 抗积分饱和：防止长时间偏差导致过大超调
  integral = constrain(integral, -50, 50);

  float derivative = filtered_error - prev_error;
  controller_output = kp * filtered_error + ki * integral + kd * derivative;

  // 限幅：确保单次转向时长 <= 500ms
  controller_output = constrain(controller_output, 0, MAX_CONTROLLER_OUTPUT);

  prev_error = filtered_error;

  Serial.println("controller_output (ms):");
  Serial.println(controller_output);
  return controller_output; // ms
}

// ---- 右转（时间式）：全功率 + 定时，避免低速死区 ----
void turn_right_time(float error) {
  float turn_time = time_controller(error);
  if (turn_time > 0) {
    // 左轮前进、右轮后退 -> 右转
    digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    analogWrite(ENB, 255);

    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
    analogWrite(ENA, 255);

    delay(turn_time);
    stop_motors();
    delay(1); // 机械稳定
  }
}

// ---- 左转（时间式）----
void turn_left_time(float error) {
  float turn_time = time_controller(error);
  if (turn_time > 0) {
    // 右轮前进、左轮后退 -> 左转
    digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
    analogWrite(ENA, 255);

    digitalWrite(MOTOR_LEFT_FORWARD, LOW);
    digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
    analogWrite(ENB, 255);

    delay(turn_time);
    stop_motors();
    delay(1);
  }
}

// ---- 距离读数（带超时与上下限裁剪）----
float front_distance() { return get_distance(TRIG_PIN_FRONT, ECHO_PIN_FRONT) - 2; } // -2 作微校正
float left_distance()  { return get_distance(TRIG_PIN_LEFT,  ECHO_PIN_LEFT); }
float right_distance() { return get_distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT); }

float get_distance(int TRIG_PIN, int ECHO_PIN) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 超时：30ms内未返回视为失败 -> 返回MAX_DISTANCE
  float duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return MAX_DISTANCE;

  float distance = duration / 58.0; // HC-SR04 常用换算
  if (distance > MAX_DISTANCE) return MAX_DISTANCE;
  if (distance < MIN_DISTANCE)  return MIN_DISTANCE;
  return distance;
}
