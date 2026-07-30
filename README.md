# NUEDC 控制程序 (2026)

基于 **MSPM0G3507** 主控 + **LibXR** 框架的竞赛小车控制项目。

---

## 快速开始

### 环境要求

| 工具 | 说明 |
|------|------|
| **ARM GCC** | `arm-none-eabi-g++` (v15.2.1+) |
| **CMake** | v3.20+ |
| **Ninja** | 构建系统 |
| **J-Link / Ozone** | 调试 & 烧录 (可选) |

### 构建

```bash
# 首次配置
cmake -B build -G Ninja

# 编译
cmake --build build
```

产物：

| 文件 | 说明 |
|------|------|
| `build/ti_mspm0_libxr_dev.elf` | ELF 调试文件 |
| `build/ti_mspm0_libxr_dev.hex` | Intel HEX (烧录) |
| `build/ti_mspm0_libxr_dev.bin` | 裸二进制 |

### 烧录

**方式一：Ozone (推荐)**

打开 `ozone.jdebug`，点击 **Download & Debug** 即可烧录并进入调试。

**方式二：J-Link 命令行**

```bash
JLinkExe -device MSPM0G3507 -if SWD -speed 4000 -autoconnect 1 \
  -CommanderScript flash.jlink
```

> 也可直接用 J-Flash 加载 `.hex` 烧录。

**方式三：第三方调试器**

如需使用 DAP-Link / PyOCD / OpenOCD，需自行配置目标脚本。

---

## 上电启动

烧录完成后，给板上电：

1. **OLED 屏幕** 应立刻点亮，显示当前姿态角（ROLL / PITCH / YAW）
2. **串口终端** 输出提示符：

```
ramfs:/$ 
```

> 如果没看到输出，按一下复位键（RESET），检查串口接线。

---

## 硬件资源

### 引脚分配

| 功能 | 外设 | 引脚 |
|------|------|------|
| 串口终端 | UART | TX=PA10, RX=PA11 |
| OLED 显示屏 | I2C1 | SDA, SCL (ssd1306, 0x3C) |
| 6 轴 IMU | I2C0 | SDA, SCL (ICM42688) |
| 电机 PWM | TimerA | 详见 sysconfig |
| 编码器输入 | GPIO | A/B 相信号 |
| 灰度传感器 (×8) | GPIO | 8 路数字输入 |
| 按键 (×4) | GPIO | KEY1~KEY4 |
| LED (×2) | GPIO | LED1, LED2 |

### 芯片资源占用

| 资源 | 总量 | 已用 | 占用率 |
|------|------|------|--------|
| Flash | 124 KB | ~97 KB | 77% |
| RAM   | 32 KB  | ~10 KB | 31% |

---

## 串口终端

### 连接参数

| 参数 | 值 |
|------|-----|
| 波特率 | **9600** |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验   | 无 |

接线：

```
板子 PA10 (TX)  ←→  USB 串口模块 RX
板子 PA11 (RX)  ←→  USB 串口模块 TX
板子 GND        ←→  USB 串口模块 GND
```

### 终端软件

| 软件 | 平台 | 用法 |
|------|------|------|
| **PuTTY** | Windows | Serial → COM口, 9600 |
| **MobaXterm** | Windows | Session → Serial |
| **screen** | Linux/macOS | `screen /dev/ttyUSB0 9600` |
| **minicom** | Linux | `minicom -b 9600 -D /dev/ttyUSB0` |
| **VSCode Serial Monitor** | 全平台 | 安装扩展，选 COM 口 |

> 终端软件需设为 **CR/LF** 模式。

### 内建命令

#### `ls` — 列出当前目录

```
ramfs:/$ ls
d diag
d drive
d chassis
```

`d`=目录, `f`=文件, `x`=可执行

#### `cd <目录>` — 切换目录

```
ramfs:/$ cd diag
ramfs:diag$
```

#### `Tab` — 自动补全

```
ramfs:/$ d<Tab>
ramfs:/$ diag/
```

#### `↑` / `↓` — 命令历史

### 模块命令

#### `diag` — 系统诊断快照

```
ramfs:/$ diag
diag seq=42 t=1234567 imu=1 who=0xFC attitude=1 yaw_mdeg=-1234 display=1 line=0xFF pos=45 lost=0
```

| 字段 | 说明 |
|------|------|
| `seq` | 诊断序列号，每帧递增 |
| `t` | 时间戳 (ms) |
| `imu` | IMU 在线状态 |
| `who` | IMU 芯片 ID |
| `attitude` | 姿态解算有效标志 |
| `yaw_mdeg` | 偏航角 (毫度) |
| `display` | OLED 在线状态 |
| `line` | 灰度传感器通道位掩码 |
| `pos` | 赛道线位置 |
| `lost` | 线丢失计数 |

#### `drive` — 循迹控制

循迹（Line Following）通过 `LineTracker` + `SimpleDicision` 模块实现。

**启动循迹：**

```
ramfs:/$ drive start
```

或等价的：

```
ramfs:/$ drive line
ramfs:/$ drive run
```

**停止：**

```
ramfs:/$ drive stop
```

**查看状态：**

```
ramfs:/$ drive status
drive state=line segment=A->B lap=0/1 dist=12.3/982.0mm wheel=65.0mm cpr=1560.0 line=18.0+/-2.0mm left=123 right=125
```

| 字段 | 说明 |
|------|------|
| `state` | 当前状态：stopped / line / finished |
| `segment` | 当前边段（A→B / B→C / C→D / D→A） |
| `lap` | 已完成圈数 / 目标圈数 |
| `dist` | 当前边已走距离 / 目标距离 (mm) |
| `wheel` | 车轮直径 (mm) |
| `cpr` | 编码器每转脉冲数 |
| `line` | 赛道宽度 + 容差 (mm) |
| `left` / `right` | 左右编码器本次增量 |

> ⚠️ **自动启动：** `SimpleDicision` 现在会遵循 `xrobot.yaml` 中的 `auto_start`。
> 无人值守调试时建议保持 `auto_start: false`；如果改为 `true`，请确保轮子悬空或拆卸，防止意外移动。

#### `chassis` — 底盘控制

```
ramfs:/$ chassis
```

输入后查看子命令用法。

---

## 项目结构

```
├── CMakeLists.txt          # 顶层构建
├── cmake/LibXR.CMake       # LibXR 配置
├── Modules/                # 应用模块
│   ├── Scheduler/          # 主调度器（终端、诊断、显示）
│   ├── SSD1306/            # OLED 驱动
│   ├── DisplaySurface/     # 显示帧路由
│   ├── ICM42688/           # 6 轴 IMU 驱动
│   ├── MadgwickAHRS/       # 姿态解算
│   ├── GreySensor/         # 灰度传感器
│   ├── DRV8870Motor/       # 电机驱动
│   ├── DifferentialChassis/# 差速底盘
│   ├── LineTracker/        # 循迹
│   ├── SimpleDicision/     # 决策控制
│   ├── BitsButtonXR/       # 按键
│   └── SimpleMotor/        # 简单电机 PWM
├── User/
│   ├── main.c              # 入口
│   ├── app_main.cpp        # 初始化所有外设 & 模块
│   └── xrobot_main.hpp     # 模块装配
├── libxr/                  # LibXR 框架源码
├── sysconfig/              # SysConfig 生成文件
├── mspm0-sdk/              # TI MSPM0 SDK
└── build/                  # 构建输出
```

## Debug Notes

无人值守调试时请务必确保轮子悬空或拆卸，防止意外移动造成伤害。
