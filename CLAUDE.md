# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Embedded firmware for a RoboMaster infantry robot (bottom/chassis board). Target MCU: **STM32F407IG** running FreeRTOS. Built with the arm-none-eabi-gcc toolchain.

## Build & Flash Commands

```bash
# Compile (parallel, adjust -j to CPU core count)
make -j12

# Compile and flash via CMSIS-DAP / DAPLink (requires OpenOCD)
make -j12 && make download_dap

# Compile and flash via JLink
make -j12 && make download_jlink

# CMake + Ninja (alternative, used by CI)
cd build && cmake -G Ninja .. && ninja
```

Flash targets use `openocd_dap.cfg` or `openocd_jlink.cfg` in the repo root.  
Output binary: `build/basic_framework.elf` / `.bin`.  
There are no unit tests; validation is done on hardware.

## Architecture: Three-Layer Design

The codebase is strictly split into three layers. **Never include upper-layer headers from lower layers.**

```
application/   ← app layer: robot logic, pub-sub consumers/producers
modules/       ← module layer: hardware drivers + algorithms
bsp/           ← BSP layer: peripheral abstractions over STM32 HAL
```

### BSP (`bsp/`)
Wraps STM32 HAL peripherals (CAN, USART, SPI, I2C, USB, PWM, ADC, GPIO, DWT, flash). Each peripheral exposes an instance-based API:
```c
XXXInstance *XXXRegister(XXX_Init_Config_s *conf);
```
The caller passes a config struct including a **callback function pointer** so the BSP can invoke module code from interrupt context without including module headers (inverted dependency).

### Module (`modules/`)
Hardware-agnostic drivers and algorithms built on top of BSP instances:
- **Motors**: `modules/motor/` — DJI (M2006/M3508/GM6020), HT04, LK, servo, stepper. All motor logic (PID, power limiting) runs in `motor_task.c` at 1 kHz via `MotorControlTask()`. Closed-loop type is configured with bitmask flags (`SPEED_LOOP | ANGLE_LOOP`, etc.) in `motor_def.h`.
- **IMU / INS**: `modules/imu/ins_task.c` — BMI088 + IST8310 fusion via Quaternion EKF, runs at 1 kHz in `StartINSTASK`.
- **Vision / PC comm**: `modules/master_machine/master_process.c` — receives `Vision_Recv_s` (fire mode, target state, pitch/yaw) from Jetson Orin Nano. Transport selected at compile time in `robot_def.h`:
  - `VISION_USE_VCP` — USB CDC (currently active)
  - `VISION_USE_UART` — USART1 (hardware pin labeled USART2, 4-pin)
- **Pub-sub**: `modules/message_center/` — lightweight topic-based message passing between apps. Apps call `PubRegister` / `SubRegister` by topic name string, then `PubPushMessage` / `SubGetMessage`.
- **Daemon**: `modules/daemon/` — watchdog for modules; triggers callbacks + buzzer on offline/timeout events.
- **Referee system**: `modules/referee/` — decodes referee system UART frames; provides heat, bullet speed, enemy color to apps.

### Application (`application/`)
Four apps, all running in the FreeRTOS `StartROBOTTASK` at 200 Hz (5 ms period) via `RobotTask()`:

| App | File | Role |
|-----|------|------|
| `cmd` | `application/cmd/robot_cmd.c` | Reads RC + vision, publishes `Chassis_Ctrl_Cmd_s`, `Gimbal_Ctrl_Cmd_s`, `Shoot_Ctrl_Cmd_s` |
| `chassis` | `application/chassis/chassis.c` | Subscribes to cmd, runs Mecanum kinematics, drives 4× wheel motors |
| `gimbal` | `application/gimbal/gimbal.c` | Subscribes to cmd, controls GM6020 yaw/pitch |
| `shoot` | `application/shoot/shoot.c` | Subscribes to cmd, controls friction wheels + loader |

## Key Configuration File

**`application/robot_def.h`** — the single source of truth for robot-specific parameters:
- Board role: `ONE_BOARD` / `CHASSIS_BOARD` / `GIMBAL_BOARD` (only one may be defined; compile error otherwise)
- Vision transport: `VISION_USE_VCP` or `VISION_USE_UART`
- Mechanical constants: encoder zero positions, wheel dimensions, gimbal angle limits, bullet speed limits
- All control structs shared between apps (`Chassis_Ctrl_Cmd_s`, `Gimbal_Ctrl_Cmd_s`, `Shoot_Ctrl_Cmd_s`, and their upload counterparts)

Change `robot_def.h`, then recompile — no other files need touching for hardware reconfiguration.

## FreeRTOS Task Summary

| Task | Period | Priority | Purpose |
|------|--------|----------|---------|
| `StartINSTASK` | 1 ms | AboveNormal | BMI088 read + quaternion EKF + `VisionSend()` |
| `StartMOTORTASK` | 1 ms | Normal | PID update for all registered motors |
| `StartROBOTTASK` | 5 ms | Normal | `cmd` → `chassis`/`gimbal`/`shoot` logic |
| `StartDAEMONTASK` | 10 ms | Normal | Watchdog checks + buzzer |
| `StartUITASK` | ~1 ms | Normal | Referee system UI refresh |

## Coding Conventions

- Instance-based C style (no `class`): every module has `XXXInstance *` as its first parameter, acting as `this`.
- Struct initialization always uses `XXX_Init_Config_s` passed to `XXXRegister()`.
- `#pragma pack(1)` is used on structs that cross communication boundaries.
- Use `LOGINFO` / `LOGERROR` / `LOGWARNING` (from `bsp/log/bsp_log.h`) for debug output via Segger RTT.
- Use `DWT_GetTimeline_ms()` for timing; never use HAL_Delay inside tasks.
- Inter-app data must go through `message_center` pub-sub, not direct includes between apps.

## Adding a New Module

1. Create `modules/<name>/<name>.c` and `<name>.h` following the `XXXInstance / XXXRegister` pattern.
2. Include only BSP headers — never app headers.
3. Register a daemon instance if the module can go offline.
4. Add source file to `C_SOURCES` in `Makefile` and `CMakeLists.txt`.
5. See `.Doc/架构介绍与开发指南.md` for the full coding style guide and file tree.

## Isolated Module Testing

To test a single BSP or module without running the full robot:
1. In `Src/main.c`, replace `RoboInit()` with `BSPInit()` (include `bsp_init.h`).
2. Include only the target module header and initialize it per its `.md` doc.
3. Optionally comment out `osKernelStart()` and test in the `while(1)` loop.
