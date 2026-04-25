# Book Arm Fusion

这是运行在 ESP32 上的三执行器融合固件：RoArm-M2 机械臂、外置 Gripper-B 夹爪、以及串接在夹爪后面的 ST3215 杆电机。项目保留串口 JSON、HTTP、ESP-NOW、LittleFS 任务文件和 OLED 状态显示。

## 当前安全策略

这版固件已经关闭危险的自动动作：

- 上电后不会自动执行机械臂回初始位。
- 上电后不会自动播放 LittleFS 里的 `boot` 任务。
- `RoArmM2_moveInit()` 不再自动释放肩部从动舵机力矩。
- 反馈读取失败不会触发自动断力矩。

只有显式发送下面这些命令才会释放力矩或急停：

```json
{"T":0}
{"T":210,"cmd":0}
{"T":134,"cmd":0}
{"T":154,"cmd":0}
```

对应配置在 `RoArm-M2_config.h`：

```cpp
#define ARM_AUTO_MOVE_INIT_ON_BOOT 0
#define ARM_DISABLE_AUTO_TORQUE_RELEASE 1
#define RUN_BOOT_MISSION_ON_STARTUP 0
```

## 硬件连接

- ESP32 到机械臂、夹爪、杆电机使用同一条 TTL 总线。
- 默认舵机总线引脚：`GPIO18(RX)` / `GPIO19(TX)`。
- 舵机总线波特率：`1000000`。
- USB 串口波特率：`115200`。
- 机械臂舵机 ID：`11-15`。
- 外置夹爪默认 ID：`1`。
- 外置杆电机默认 ID：`2`。
- ST3215 出厂默认 ID 通常是 `1`，接到同一条总线前，必须先单独把杆电机改成 `2`。
- ESP32、机械臂电源、夹爪电源、杆电机电源必须共地。
- 舵机不能只靠 ESP32 USB 供电，必须打开外部舵机电源。

## 完整启动流程

### 1. 进入项目目录

```bash
cd E:\master_degree\project\10.图书馆机器人\esp32\Book_Arm_Fusion
```

### 2. 确认串口

```bash
pio device list
```

如果看到 `COM8`，可以直接使用默认配置。否则把下面命令里的 `COM8` 换成实际端口。

### 3. 编译

```bash
pio run
```

### 4. 烧录

默认使用 `platformio.ini` 里的 `upload_port = COM8`：

```bash
pio run -t upload
```

如果端口不是 `COM8`：

```bash
pio run -t upload --upload-port COMx
```

如果提示端口被占用，先关闭串口监视器、Arduino 串口助手、VSCode monitor 等所有占用该串口的软件，再重新烧录。

### 5. 打开串口监视器

```bash
pio device monitor -p COM8 -b 115200
```

如果端口不是 `COM8`，替换为实际端口。

### 6. 上电后先观察启动信息

正常情况下会看到类似：

```text
Book Arm Fusion started.
Skip boot-time moveInit for safety; no automatic torque release.
Skip boot mission playback for safety.
```

这表示程序已经启动，并且没有自动回初始位、没有自动播放 boot 任务。

### 7. 先读反馈，不要立刻发动作命令

机械臂反馈：

```json
{"T":105}
```

夹爪反馈：

```json
{"T":133}
```

杆电机反馈：

```json
{"T":153}
```

只有确认反馈正常后，再进行低速动作测试。

## 低速动作测试顺序

### 1. 测试夹爪

```json
{"T":132,"angle":90,"spd":30,"acc":5,"torque":300}
```

```json
{"T":132,"angle":130,"spd":30,"acc":5,"torque":300}
```

### 2. 测试杆电机

```json
{"T":152,"angle":20,"spd":30,"acc":5,"torque":300}
```

```json
{"T":152,"angle":0,"spd":30,"acc":5,"torque":300}
```

### 3. 测试机械臂底座

```json
{"T":121,"joint":1,"angle":5,"spd":5,"acc":5}
```

```json
{"T":121,"joint":1,"angle":0,"spd":5,"acc":5}
```

### 4. 三者联动小测试

```json
{"T":160,"b":0,"s":0,"e":90,"h":180,"g":90,"r":20,"spd":5,"acc":5,"gspd":30,"gacc":5,"gtorque":300,"rspd":30,"racc":5,"rtorque":300}
{"T":160,"b":0,"s":-40,"e":90,"h":180,"g":160,"r":80,"spd":5,"acc":5,"gspd":30,"gacc":5,"gtorque":300,"rspd":30,"racc":5,"rtorque":300}
```

## 常用命令说明

急停：

```json
{"T":0}
```

机械臂所有总线舵机扭矩：

```json
{"T":210,"cmd":0}
{"T":210,"cmd":1}
```

夹爪扭矩：

```json
{"T":134,"cmd":0}
{"T":134,"cmd":1}
```

杆电机扭矩：

```json
{"T":154,"cmd":0}
{"T":154,"cmd":1}
```

机械臂、夹爪、杆电机三者联动：

```json
{"T":160,"b":0,"s":0,"e":90,"h":180,"g":90,"r":20,"spd":5,"acc":5,"gspd":30,"gacc":5,"gtorque":300,"rspd":30,"racc":5,"rtorque":300}
```

参数含义：

- `b`：底座角度，单位度。
- `s`：肩关节角度，单位度。
- `e`：肘关节角度，单位度。
- `h`：机械臂末端手腕/原末端舵机角度，单位度。
- `g`：外置夹爪角度，单位度。
- `r`：外置杆电机角度，单位度。
- `spd` / `acc`：机械臂速度和加速度。
- `gspd` / `gacc` / `gtorque`：夹爪速度、加速度、扭矩参数。
- `rspd` / `racc` / `rtorque`：杆电机速度、加速度、扭矩参数。

## ST3215 注意事项

- 同一条 TTL 总线上每个 ID 必须唯一。
- 本工程把杆电机位置模式按 `0-360` 度映射到 `0-4095`。
- `T=157` 会把杆电机切回位置舵机模式。
- `T=158` 会把杆电机切到连续电机模式。
- 模式写入会保存到舵机内部，不要在高频循环中反复发送。
- 连续电机模式下，先用小速度测试，停止命令是：

```json
{"T":159,"speed":0,"acc":10}
```

## 故障排查

如果反馈全是 failed：

- 确认外部舵机电源已打开。
- 确认 ESP32 和所有舵机电源共地。
- 确认当前电脑串口连接的是烧录了本工程的那块 ESP32。
- 确认 TTL 总线接到 `GPIO18/GPIO19`。
- 确认夹爪 ID 是 `1`，杆电机 ID 是 `2`。

如果烧录失败并提示端口忙：

- 关闭所有串口监视器。
- 重新插拔 ESP32。
- 再执行 `pio run -t upload`。
