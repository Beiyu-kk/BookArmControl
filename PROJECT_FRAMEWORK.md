# Book Arm Fusion Project Framework

本文档用于说明 `Book_Arm_Fusion` 固件的整体框架、主要模块、启动流程、运行逻辑和命令流转方式。适合后续调试、扩展功能、排查机械臂、外置夹爪与 ST3215 杆电机协同问题时参考。

## 1. 项目定位

`Book_Arm_Fusion` 是运行在 ESP32 上的机械臂融合控制固件。它把 RoArm-M2 机械臂控制、外置 Gripper-B 夹爪控制、串接在夹爪后的 ST3215 杆电机控制、串口 JSON 控制、Web 控制、ESP-NOW 控制、LittleFS 任务文件和 OLED 状态显示整合到同一个工程中。

项目的核心思想是：

```text
外部输入 JSON 命令
-> ESP32 统一解析命令类型 T
-> 分发给机械臂、外置夹爪、外置杆电机、文件系统、Wi-Fi、ESP-NOW 等模块
-> 通过共享总线仲裁器避免机械臂、夹爪和杆电机抢占 TTL 舵机总线
```

## 2. 顶层文件结构

| 文件 | 作用 |
|---|---|
| `Book_Arm_Fusion.ino` | Arduino 主入口，负责 `setup()` 初始化和 `loop()` 主循环 |
| `RoArm-M2_config.h` | 全局配置、引脚定义、舵机 ID、机械臂几何参数、运行状态变量 |
| `RoArm-M2_module.h` | 机械臂核心控制：反馈读取、角度换算、关节控制、逆解、连续运动、扭矩控制 |
| `External_gripper.h` | 外置 Gripper-B 控制：开合、角度、反馈、扭矩锁、联动命令 |
| `External_rod.h` | 外置 ST3215 杆电机控制：伸出/收回、角度、连续电机模式、反馈、扭矩锁、三执行器联动 |
| `servo_bus_arbiter.h` | 共享 TTL 舵机总线仲裁，避免机械臂、夹爪和杆电机同时发包 |
| `json_cmd.h` | JSON 命令编号定义，例如 `T=105`、`T=133`、`T=140`、`T=160` |
| `uart_ctrl.h` | JSON 命令分发器，按 `T` 字段调用对应功能 |
| `RoArm-M2_advance.h` | 任务系统和高级动作，支持 mission 文件播放 |
| `files_ctrl.h` | LittleFS 文件创建、读取、追加、删除等操作 |
| `wifi_ctrl.h` | Wi-Fi AP/STA/AP+STA 配置与状态反馈 |
| `http_server.h` | HTTP/Web Server 入口，把 Web 请求转成 JSON 命令 |
| `esp_now_ctrl.h` | ESP-NOW 初始化、发送、接收和队列化处理 |
| `switch_module.h` | 12V 开关、电机/气动模块、灯光 PWM 控制 |
| `oled_ctrl.h` | OLED 初始化和状态显示 |
| `m2_web_page.h` | Web 控制页面内容 |
| `platformio.ini` | PlatformIO 编译、烧录、依赖和串口配置 |
| `README.md` | 用户使用说明、编译烧录和常用测试命令 |

## 3. 硬件通信关系

本项目中有两层主要通信：

```text
电脑 / 上位机
  |
  | USB 串口，115200
  v
ESP32
  |
  | Serial1，TTL UART，总线舵机协议，1000000
  v
RoArm-M2 舵机总线 + 外置 Gripper-B + 外置 ST3215 杆电机
```

说明：

- 电脑到 ESP32 使用 USB 串口，发送 JSON 文本命令。
- ESP32 到机械臂舵机使用 TTL UART 总线，不是 CAN，也不是 RS485。
- 外置 Gripper-B 和 ST3215 杆电机默认串接在同一条 TTL 舵机总线上。
- 因为机械臂、外置夹爪和杆电机共总线，所有总线读写必须经过 `servo_bus_arbiter.h`。

## 4. 启动流程

主入口在 `Book_Arm_Fusion.ino` 的 `setup()`。

启动顺序大致如下：

```text
1. Serial.begin(115200)
2. Wire.begin(S_SDA, S_SCL)
3. 初始化 OLED
4. 初始化 LittleFS
5. 初始化 12V 开关 / 灯光控制引脚
6. 初始化共享总线互斥锁
7. 初始化机械臂 TTL 舵机总线 Serial1
8. 初始化外置 Gripper-B
9. 初始化外置 ST3215 杆电机
10. 检查机械臂舵机状态
11. 机械臂移动到初始姿态
12. 重置 PID 和扭矩限制
12. 初始化 Wi-Fi
13. 初始化 HTTP/Web Server
14. 初始化 ESP-NOW
15. 读取本机 MAC 地址
16. 创建并播放 boot mission
17. 设置末端手腕/原夹爪扭矩
```

启动后，固件进入 `loop()` 主循环。

## 5. 主循环逻辑

主循环负责不断处理外部输入和周期性任务。

简化逻辑如下：

```text
loop()
  -> serialCtrl()                 处理 USB 串口 JSON 命令
  -> server.handleClient()         处理 HTTP/Web 请求
  -> espNowHandlePendingCommand()  处理 ESP-NOW 队列命令
  -> constantHandle()              处理连续运动控制
  -> RoArmM2_getPosByServoFeedback() 周期性读取机械臂反馈
  -> ESP-NOW flow leader 同步发送
  -> InfoPrint=2 时持续输出反馈
  -> 处理 runNewJsonCmd 标记命令
```

主循环不是一个实时运动规划器，而是一个高频轮询式调度器。它不断检查是否有新命令、是否需要连续运动、是否可以读取反馈。

## 6. JSON 命令流转

项目中多个入口最终都汇入同一个 JSON 命令分发器。

```text
USB 串口
HTTP / Web
ESP-NOW
mission 文件
  |
  v
jsonCmdReceive
  |
  v
jsonCmdReceiveHandler()
  |
  v
switch(T)
  |
  +-- 机械臂控制
  +-- 外置夹爪控制
  +-- 机械臂 + 夹爪联动
  +-- 文件系统 / mission
  +-- Wi-Fi / ESP-NOW
  +-- 12V 开关 / 灯光
```

关键文件：

- `json_cmd.h`：定义每个 `T` 命令编号。
- `uart_ctrl.h`：实现 `jsonCmdReceiveHandler()`，按 `T` 分发。

例如：

```json
{"T":105}
```

会进入机械臂反馈读取逻辑。

```json
{"T":133}
```

会进入外置夹爪反馈读取逻辑。

```json
{"T":140,"b":0,"s":20,"e":90,"h":180,"g":90,"spd":8,"acc":8,"gspd":80,"gacc":10,"torque":600}
```

会进入机械臂和外置夹爪联动逻辑。

## 7. 机械臂控制模块

机械臂核心逻辑在 `RoArm-M2_module.h`。

主要职责：

- 初始化 `SCServo` 舵机库和 `Serial1`。
- 读取舵机反馈，包括位置、速度、负载、电压、电流、温度。
- 把关节角度转换成舵机位置。
- 把舵机反馈位置转换成关节角度。
- 控制单个关节或多个关节同步运动。
- 根据末端坐标做简单逆运动学计算。
- 支持连续运动控制。
- 控制舵机扭矩锁和扭矩限制。

机械臂关节命名：

| 字段 | 含义 | 舵机 |
|---|---|---|
| `b` | base，底座关节 | `BASE_SERVO_ID` |
| `s` | shoulder，肩关节 | `SHOULDER_DRIVING_SERVO_ID` + `SHOULDER_DRIVEN_SERVO_ID` |
| `e` | elbow，肘关节 | `ELBOW_SERVO_ID` |
| `h` | hand，末端手腕/原夹爪关节 | `GRIPPER_SERVO_ID` |

角度控制大致流程：

```text
JSON 角度 b/s/e/h
-> 角度转弧度
-> 弧度转舵机位置
-> 填充 goalPos
-> SyncWritePosEx 同步下发给总线舵机
```

## 8. 外置夹爪模块

外置夹爪逻辑在 `External_gripper.h`。

主要职责：

- 初始化外置夹爪总线对象。
- 读取夹爪反馈。
- 控制夹爪打开、闭合、移动到指定角度。
- 控制夹爪扭矩锁。
- 记录夹爪目标扭矩请求值。
- 提供 `T=140` 联动入口。

常用命令：

| 命令 | 作用 |
|---|---|
| `T=130` | 打开外置夹爪 |
| `T=131` | 闭合外置夹爪 |
| `T=132` | 外置夹爪移动到指定角度 |
| `T=133` | 读取外置夹爪反馈 |
| `T=134` | 外置夹爪扭矩锁控制 |
| `T=140` | 机械臂和外置夹爪联动 |

## 9. 共享总线仲裁

机械臂舵机和外置 Gripper-B 默认共用同一条 TTL 舵机总线。如果两个模块同时发包，会导致反馈失败、包冲突、动作丢失，严重时可能造成危险动作。

为了解决这个问题，项目新增 `servo_bus_arbiter.h`。

它提供：

- `SharedBus_init()`：初始化 FreeRTOS 递归互斥锁。
- `SharedBus_take()`：申请总线使用权。
- `SharedBus_release()`：释放总线，并设置安静窗口。
- `SharedBus_pauseConstantMotion()`：临时暂停连续运动。
- `SharedBus_canRunConstantMotion()`：判断当前是否允许连续运动。

使用原则：

```text
任何机械臂或夹爪的总线读写
-> 先申请 SharedBus 锁
-> 完成 SCServo 读写
-> 释放 SharedBus 锁
```

这样可以保证同一时刻只有一个模块访问 TTL 舵机总线。

## 10. 扭矩保护逻辑

项目曾经存在一个危险路径：反馈临时失败时会执行广播关扭矩。

现在的设计是：

- 总线忙不等于舵机故障。
- 反馈短暂失败不会自动关闭机械臂扭矩。
- `T=140` 联动结束后会主动保持机械臂舵机扭矩锁。
- 只有显式急停、显式扭矩控制命令和启动校准流程才会关闭扭矩。

显式关扭矩命令：

```json
{"T":210,"cmd":0}
```

急停命令：

```json
{"T":0}
```

## 11. T=140 联动命令流程

`T=140` 是本项目的融合控制命令。

示例：

```json
{"T":140,"b":0,"s":20,"e":90,"h":180,"g":90,"spd":8,"acc":8,"gspd":80,"gacc":10,"torque":600}
```

执行流程：

```text
收到 T=140
-> 暂停机械臂连续控制
-> 解析机械臂 b/s/e/h/spd/acc
-> 下发机械臂关节目标
-> 短暂延时
-> 解析夹爪 g/gspd/gacc/torque
-> 下发外置夹爪目标
-> 设置总线安静窗口
-> 强制保持机械臂扭矩锁
```

字段说明：

| 字段 | 含义 |
|---|---|
| `T` | 命令编号，`140` 表示联动 |
| `b` | 底座关节目标角度，单位度 |
| `s` | 肩关节目标角度，单位度 |
| `e` | 肘关节目标角度，单位度 |
| `h` | 末端手腕/原夹爪关节目标角度，单位度 |
| `g` | 外置夹爪目标角度，单位度 |
| `spd` | 机械臂运动速度 |
| `acc` | 机械臂运动加速度 |
| `gspd` | 外置夹爪运动速度 |
| `gacc` | 外置夹爪运动加速度 |
| `torque` | 外置夹爪目标扭矩请求值 |

## 12. Web、ESP-NOW 和 Mission

### Web

`http_server.h` 接收 Web 页面发来的 JSON 字符串，然后反序列化到 `jsonCmdReceive`，最终调用 `jsonCmdReceiveHandler()`。

### ESP-NOW

`esp_now_ctrl.h` 负责 ESP-NOW 通信。

当前设计中，ESP-NOW 接收回调不会直接控制舵机，而是把消息放进队列。主循环再调用 `espNowHandlePendingCommand()` 统一处理。这样可以避免在回调上下文里直接访问舵机总线。

### Mission 文件

`RoArm-M2_advance.h` 和 `files_ctrl.h` 支持把 JSON 命令保存到 LittleFS 文件中，然后按顺序播放。

典型流程：

```text
创建 mission
-> 追加 JSON 步骤
-> 播放 mission
-> 每一步反序列化成 jsonCmdReceive
-> 调用 jsonCmdReceiveHandler()
```

## 13. 扩展新命令的方法

新增一个 JSON 命令通常需要三步：

1. 在 `json_cmd.h` 中定义新的命令编号。

```cpp
#define CMD_MY_NEW_COMMAND 700
```

2. 在 `uart_ctrl.h` 的 `jsonCmdReceiveHandler()` 中增加 `case`。

```cpp
case CMD_MY_NEW_COMMAND:
  myNewFunction(jsonCmdReceive["value"]);
  break;
```

3. 在合适的模块文件中实现 `myNewFunction()`。

如果新功能需要访问机械臂或外置夹爪 TTL 总线，必须经过 `SharedBus_take()` / `SharedBus_release()`，或者复用已有的封装函数。

## 14. 调试建议

- 先测试 `T=105` 和 `T=133`，确认机械臂和夹爪反馈正常。
- 第一次运动使用低速参数，例如 `spd=5~10`、`acc=5~10`。
- 调姿态时每次只改一个关节参数。
- 出现异常时立即发送 `{"T":0}`。
- 如果出现反馈失败，先检查电源、共地、舵机 ID、TTL 接线和总线负载。
- 如果出现掉扭矩，优先检查电源电流和机械姿态是否顶死；当前固件不会因为反馈短暂失败主动关闭全体扭矩。

## 15. 总结

本项目的主线可以概括为：

```text
输入层：串口 / Web / ESP-NOW / mission
  -> 命令层：JSON + T 命令编号
  -> 分发层：jsonCmdReceiveHandler()
  -> 功能层：机械臂 / 外置夹爪 / Wi-Fi / 文件系统 / 12V 开关
  -> 硬件层：ESP32 GPIO / TTL 舵机总线 / OLED / LittleFS
```

其中最重要的安全边界是：

- 所有机械臂和夹爪总线访问必须通过共享总线仲裁。
- 反馈失败不能直接等价为舵机故障。
- 只有显式安全命令才允许关闭机械臂扭矩。
