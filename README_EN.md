# YueLu2022/2023 EC basic_framework-dev

HNU RoboMaster YueLu Team 2022–2023 General-Purpose Embedded Control Framework.

<div align=center>
	<img src=".assets/yuelu_badge.png"/>
	<p>
		<img src="https://img.shields.io/badge/version-beta-blue"/>
		<img src="https://img.shields.io/badge/license-MIT-green"/>
    	<img src="https://github.com/HNUYueLuRM/basic_framework/actions/workflows/c-cpp.yml/badge.svg"/>
    </p>
   	<p>
		<img src="https://gitee.com/HNUYueLuRM/basic_framework/badge/star.svg"/>
		<img src="https://gitee.com/HNUYueLuRM/basic_framework/badge/fork.svg"/>
	</p>
    <h5><p><font face="consolas">Best RoboMaster embedded EC open-source code ever?</p></h>
</div>

> ***Maybe not the best — but definitely the most complete, most documented, and most beginner-friendly open-source EC framework available.***

***==Don't forget: the `.Doc` folder in the repository root contains detailed instructions for setting up the environment and development workflow!==***

- If GitHub is inaccessible, use the [Gitee mirror](https://gitee.com/hnuyuelurm/basic_framework)
- If Gitee content is blocked, use the [GitHub repository](https://github.com/HNUYueLuRM/basic_framework)

> An advanced C++ rewrite built on top of basic_framework — [***powerful_framework***](https://gitee.com/hnuyuelurm/powerful_framework) — is now available! It adds a new message-passing mechanism, strict cross-task data read/write protection, a modern CMake+Ninja build system for maximum compile speed, aggressive embedded-specific compiler optimizations, and extension library support for custom CMSIS-DSP and Eigen. Come try it out or contribute!

---

## Features and Showcase

### Origin

This is the general-purpose embedded control framework developed by the Electrical Engineering group of Hunan University's RoboMaster YueLu Team for the 2022–2023 season. It is suitable for use as a **robot main controller**, custom modules (IMU, ranging sensors, etc.), and supercapacitor controllers.

Looking at the current RoboMaster open-source community, most teams lack a unified, well-structured framework that follows the principles of large-scale software development. Some schools have drastically different codebases for different robot types, and some barely have any comments — relying entirely on mentorship and tribal knowledge. There are exceptions, such as DynamicX (GDUT)'s rm_control, which is a polished and sophisticated system; however, its Linux+ROS+C++ stack is too complex for beginners (though we highly recommend it for those already familiar with the basics).

### Advantages

basic_framework was created to fill this gap. It aims to be an SDK that is **easy to get started with**, **portable**, **highly reusable**, **clearly layered**, and **structurally clean** — for use within our team, other RM teams, and the broader embedded development community.

Through carefully designed BSP and module abstractions, along with mature app examples, the framework enables **rapid construction** of control software for robots of different types and configurations. **Extensibility** and **maintainability** are greatly improved over most existing open-source solutions and our own legacy code.

Alongside this framework, we have introduced a **modern development workflow** based on arm-gnu toolchain + VSCode/CLion + Ozone & SystemViewer / FreeMaster, complete with **comprehensive tutorials**. No more last-century UIs or code without autocomplete and syntax highlighting. This workflow **significantly boosts development and debugging efficiency**. Hardware module testing and full robot integration have never been **this easy**.

> Designing embedded systems with software engineering principles is a force multiplier.
>
> <p align=right>— Wozki Shuode</p>

### Demo

![](.assets/allrobot.jpg)

<center>Robot lineup powered by basic_framework</center>

Combat demonstrations:

1. 400HP dual-gimbal sentry at 30 m/s bullet speed vs. 2× 200HP infantry at 15 m/s, 120W chassis power
2. 100HP infantry, 120W power, 15 m/s bullet speed, rate-limited to 0.5 rounds/s

![sentry_infantry12](.assets/sentry_infantry12.gif)

<center>The vision recognition and prediction algorithm shown is built on rm_vision</center>

3. Engineer robot: streamlined ore pickup / ore exchange / rescue

   ![engineering](.assets/engineering.gif)

4. Balancing infantry robot

   ![balance](.assets/balance.gif)

All of the above robots are programmed using basic_framework and are available in our repository: [HNUYueLuRM](https://gitee.com/hnuyuelurm)

> For more test videos, follow our Bilibili channel: [湖南大学跃鹿战队](https://space.bilibili.com/522795884), or search "跃鹿战队" on Bilibili to watch our competition videos.

### Available Features

You can build your own modules and applications on top of these well-abstracted components.

#### BSP (Board Support Package)

| Category            | Modules                    |
| ------------------- | -------------------------- |
| Communication       | USART, SPI, I2C, CAN, USB  |
| Logging             | log, flash                 |
| Functional          | GPIO (EXTI), PWM, ADC      |
| Utility             | DWT                        |

#### Module Layer

| Category           | Modules                                                                                  |
| ------------------ | ---------------------------------------------------------------------------------------- |
| Motors             | DJI, HT04, LK (Lingkong), stepper motor, servo                                          |
| Communication      | Multi-board communication (CAN-based), Seasky protocol PC comm, referee system data/UI/multi-bot, VOFA protocol, DT7-DR16 remote controller |
| Functional Modules | Buzzer, OLED, BMI088, IST8310, supercapacitor, TFmini Plus                               |
| Application Support| Common algorithm library, watchdog thread, message center                                |

#### Application Layer

- `robot_cmd` — command publisher, the central control signal source
- `gimbal` — for infantry, hero, sentry, and drone
- `chassis` — Mecanum/omnidirectional wheel chassis
- `balance_chassis` — for balancing infantry
- `shoot` — for robots equipped with a shooting mechanism

---

## Architecture

### Software Stack

<img src=".assets/image-20230725153133419.png" alt="image-20230725153133419" style="zoom: 67%;" />

Built on top of the CubeMX-generated HAL files, with optional CMSIS-DSP and Segger RTT additions.

### Design Philosophy

1. ***First, a high-level overview of the framework's design pattern.***

   The framework is structurally divided into three layers: **bsp / module / app**. The overall design pattern is a **hierarchical composition model**: each "class" contains the lower-level "classes" it needs, and more powerful behavior is assembled from simpler primitives. At the top, **app** components are decoupled via a **pub-sub message mechanism**, preventing circular includes.

   The goal is for the BSP abstraction to make module development easy (no need to know how the hardware works under the hood), and for the module abstraction to make app development fully hardware-agnostic — ideally reaching the point where *"you can rapidly develop applications just by reading the module documentation."* The design vision for BSP and module is to become what is commonly called ***middleware***.

   **Pub-sub in action**: The app layer contains four applications — `chassis`, `gimbal`, `shoot`, and `cmd`. The `cmd` app reads from control input sources (remote controller / PC / sensors) and translates them into **concrete actions** for actuators (torque, speed, position, angle, etc.), then **publishes** this information. `chassis`, `gimbal`, and `shoot` **subscribe** to these messages and drive their sub-modules accordingly.

   **Hierarchical composition in action**: Take `chassis` as an example. It contains 4 wheel motor modules. When `chassis` receives a command to move at 1 m/s, it uses kinematic/dynamic solvers (based on the chassis type: swerve, Mecanum, omni, or balancing) to compute target values per motor, then calls the motor module interface. Each motor module has its own PID controller and sensor data (current, speed, angle) to compute the required current setpoint. If the motor uses CAN to communicate with the ESC, it uses a `CANInstance` (from `bsp_can`) to format and send the control message. The call hierarchy is: **chassis ⊃ motor ⊃ bsp_can**.

2. ***Now let's introduce each of the three layers in detail.***

   - **BSP** (Board Support Package) provides software abstractions for the microcontroller's peripherals, so that the module layer can use hardware-agnostic interfaces. BSP is tightly coupled to ST's HAL. Porting to other ST chips requires minimal BSP changes; for other MCUs, it is recommended to retain the **interface design** and re-implement the interface calls. Each peripheral header defines a **`XXXInstance`** struct containing all data needed: TX/RX buffers, length, ID (if applicable), and a parent pointer (pointing to the owning module, for callbacks). Since C has no `class`, all BSP interfaces take an additional `XXXInstance*` parameter to emulate the C++ `this` pointer.

   - **Module** layer includes real **hardware modules** that require peripheral support (e.g. motors, servos, IMU, ranging sensors), as well as **algorithm modules** implemented in software (PID, filters, state observers). It also contains unified interface modules that adapt different control input sources (remote controller, PS gamepad, video-link, PC), and a message center for inter-app data exchange.

     The module layer is also instance-based. An app contains multiple module instances and can interact with hardware in a hardware-agnostic manner — e.g. commanding a motor to spin at a given speed, closing a pneumatic valve, or sending feedback to the PC.

   - **App** is the highest layer. Multiple app tasks run in FreeRTOS; event-driven tasks can also be registered as needed. All task scheduling is managed in `app/robot_task`. The current app layer is a robot development example — with highly abstracted modules underneath, you can do virtually anything at the app level.

     The current app design supports multi-board setups via **conditional compilation**. For example, an infantry robot can place the main MCU on the gimbal while a supercapacitor board doubles as the chassis board. CAN/SPI/UART connects them, and the configuration is set via macros in **`app/robot_def.h`**. More boards can be added as needed (dual-gimbal sentry, engineer robot, etc.).

     This framework easily extends to all robot types. Our repository includes code examples for infantry, balancing infantry, sentry, hero, engineer, and aerial robots — all built following the same three-layer structure. For a new robot design, you only need to update parameters in `robot_def.h` (sensor placement, chassis wheel spacing, magazine capacity, etc.) to **deploy immediately**.

3. ***Finally, a note on how to develop within each layer.***

   For BSP and module instance design, the framework uses **object-oriented C style**, with a unified variable and function naming convention. Call hierarchy and data flow are clear throughout.

   To avoid "lower-layer code including upper-layer headers," BSP instances require modules to provide callback function pointers at registration time. These callbacks are invoked when a corresponding interrupt or event occurs, enabling "reverse calls" up the stack. This pattern can also be applied to the app layer — triggering tasks on events rather than polling on a timer — reducing CPU load.

   BSP and module instance initialization follows the pattern: **`XXXInstance* XXXRegister(XXX_Init_Config_s* conf)`** — pass in a config struct, get back an instance pointer (treat it as `this`). A watchdog thread is also provided: modules can opt in to receive LOG warnings, buzzer/LED alerts, and error/offline callbacks when anomalies occur, ensuring system robustness and safety.

   For app development, since the lower-level interfaces are already well-designed, different robots can directly **`fork`** basic_framework and develop the app layer. When BSP or module is updated, use `git cherry-pick` to pull only the relevant commits into your fork — **live updates without manual branch merges**.

---

## Execution Order and Data Flow

### Initialization

![image-20230725153635454](.assets/image-20230725153635454.png)

### Task Structure

BSP, module, and app all have corresponding RTOS tasks. The BSP provides `bsp_tools` for task creation — designed to move complex callback work out of interrupt context and into tasks, ensuring real-time responsiveness and data integrity. Some modules and apps create periodic or event-driven tasks at initialization, which are woken up or run cyclically at appropriate times.

<img src=".assets/image-20230725152433502.png" alt="image-20230725152433502" style="zoom:50%;" />

### Data Flow

![](.assets/dataflow.svg)

<center>Recommended: open the SVG in a browser for best viewing</center>

---

## Development Tools

### Toolchain

The **arm-gnu toolchain** (arm-none-eabi-xxx) is strongly recommended.

Official download: [Arm GNU Toolchain Downloads – Arm Developer](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

We recommend managing libraries and tools via **Msys2**. See the [How to Use This Framework](#how-to-use-this-framework) section for details.

> The arm-cc toolchain (Keil's default) is still supported — generate an MDK project from CubeMX, then manually add all basic_framework headers and sources. However, this is **strongly discouraged**: arm-cc only supports single-threaded compilation, offers far fewer optimization options, and is less customizable. ~~If you insist, you can use the Keil Assistant extension for VSCode.~~

### IDE

**VSCode** is used as the "IDE". CLion and Visual Studio are also supported with manual configuration. Required plugin setup is documented in [VSCode+Ozone使用方法.md](.Doc/VSCode+Ozone使用方法.md). VSCode's plugin ecosystem, language server, and autocomplete significantly boost development speed. Compilation is done via integrated tasks; the terminal can be added to VSCode for a seamless experience. Variable/register inspection is available via plugins, and `launch.json` provides extensive debug customization.

Git integration, combined with GitLens / Git Graph / Git History plugins, makes version control and collaboration straightforward. Live Share lets teammates collaborate in real time on difficult bugs. More tips are in the **How to Use This Framework** section.

> **In any case, do not use KEIL as your code editor.**

### Debugging and Profiling

- Basic debugging can be done in VSCode. The latest cortex-debug plugin supports live watch and a variable oscilloscope (visualization) with multiple GDB servers (JLink / ST-Link / OpenOCD / PyOCD). *Avoid using a serial debugger unless absolutely necessary.*
- For high-speed variable inspection, data logging, and multi-channel visualization (e.g. PID tuning, hard-to-find bugs), use **Segger Ozone**.
- **FreeMaster** is also available as a debugging alternative.
- Routine performance profiling can be done with `bsp_dwt`. For detailed task and per-function execution timing, use **Segger SystemViewer**.

---

## How to Use This Framework

### Compile and Flash

This project is based on the RoboMaster Development Board C (MCU: STM32F407IG). It uses the onboard BMI-088 IMU and targets a standard infantry robot: a 2-DOF GM6020 gimbal, an M2006-driven ammo feeder + 2×M3508 friction-wheel shooter, an MG90 servo ammo-cover, and a 4-wheel Mecanum chassis with supercapacitor controller.

1. Modify the board and robot configuration in `app/robot_def.h` according to the comments.
2. Update initialization configs in each app (e.g. motor IDs, PC communication baud rate / UART or VCP, IMU sample rate, supercap ID, etc.).
3. Follow [VSCode+Ozone使用方法.md](.Doc/VSCode+Ozone使用方法.md) to set up your build environment (***Msys2 + mingw64/ucrt64/clang64 is highly recommended***).
4. Open the project in VSCode, go to Terminal → Run Build Task to start compilation. A successful build produces output like:

![image-20230725154910307](.assets/image-20230725154910307.png)

The toolchain reports predicted RAM, CCRAM, and Flash usage, along with the final binary size and path.

5. Connect your board via a debugger, go to Terminal → Run Task, and select `download_dap` or `download_jlink` (or your custom st-link/ulink/... task) to flash. The terminal or JFlash will report erase/program/verify progress.
6. To debug, select the appropriate debug config in the left panel and press F5 (or click the green triangle). See the configuration docs for adding debugger executables to your PATH.

**For a more detailed development workflow and best practices, refer to `.Doc/VSCode+Ozone使用方法.md`**, which covers prerequisite knowledge, environment setup, toolchain internals, and usage guides.

To contribute to this repository, first read `.Doc/架构介绍与开发指南.md`, which contains the full **file tree** and coding/naming conventions. It also covers alternative toolchain and IDE options.

### Documentation

`README.md` (this file) is the project overview for developers.

The `.Doc` directory contains **8** markdown documents:

- [Bug_Report](.Doc/Bug_Report.md) — Issue submission templates. Please use these when reporting problems.
- [TODO](.Doc/TODO.md) — Planned features and maintenance notes.
- **[VSCode+Ozone使用方法](.Doc/VSCode+Ozone使用方法.md)** — **Important, read before starting.** Covers differences from KEIL workflow, toolchain basics, environment setup, VSCode editing/debugging, and Ozone oscilloscope & trace usage.
- [合理地进行PID参数整定](.Doc/合理地进行PID参数整定.md) — PID tuning guide, including empirical rules and model-based feedforward / disturbance rejection methods.
- [如何定位bug](.Doc/如何定位bug.md) — Efficient bug localization and reproduction in embedded development. Basic debugger usage tips.
- [必须做&禁止做](.Doc/必须做&禁止做.md) — DOs and DON'Ts. Title says it all.
- **[架构介绍与开发指南](.Doc/架构介绍与开发指南.md)** — **Important, read before developing.** Required reading if you plan to add new BSP/module components or assemble new apps. Contains the project's **file tree** and coding style guide.
- [让VSCode成为更称手的IDE](.Doc/让VSCode成为更称手的IDE.md) — Useful plugins, editor customization, and tips to improve development speed.

### Reading the Code

All three layers are thoroughly commented to aid reading and secondary development. Each layer has its own overview document, and every BSP/module/app has its own dedicated doc with interface descriptions and suggestions for improvement or further development.

We recommend reading the code **top-down**: app → module → BSP. We also provide tutorial videos covering each abstraction layer, overall design rationale, HAL dependencies, and miscellaneous development tips: [basic_framework Tutorial Series](https://space.bilibili.com/522795884/channel/collectiondetail)

### Running Individual BSP/Module Tests

**Every BSP and module folder contains its own documentation** with test cases and usage examples.

For hardware verification, wiring checks, or new-member onboarding:

1. Remove `RoboInit()` from `main.c`, include `bsp_init.h`, and replace it with `BSPInit()`.
2. Include the target BSP or module header and initialize it per the documentation example.
3. Optionally comment out the RTOS initialization and run the test in the `while(1)` loop in `main.c`, or use the timer tasks provided by `bsp_tim.h`.
4. Compile, flash, run, debug.

We have also designed a **hardware functional test program** controlled via serial port and remote controller, so that mechanical and vision team members can test hardware modules independently — without blocking the team when no EC engineers are available.

### VSCode Integrated Tasks

The `.vscode` folder contains pre-written tasks for: compile, flash, launch RTT terminal (LOG), and start Ozone debugging. Some features require plugin settings or adding executables to your PATH — all steps are detailed in [VSCode+Ozone使用方法.md](.Doc/VSCode+Ozone使用方法.md). `launch.json` contains the four most common debug configurations: JLink-server or OpenOCD, start or attach.

### For Advanced Users

A `Makefile.upgrade` script is provided for a more refined build experience.

Custom compiler optimizations can be added freely for higher-level tuning.

ST now maintains their HAL on GitHub — download the latest version and integrate it into the project build if needed.

For a fully open-source setup, customize OpenOCD debug and flash options using `openocd_dap.cfg` and `openocd_jlink.cfg` in the repo root.

To compile a specific version of CMSIS-DSP or CMSIS-OS from source, download from the official GitHub repos and add the build rules to the Makefile.

A `CMakeLists.txt` is included for integration with modern build systems. If you want to use CMake, you should be capable of configuring the environment yourself. See [***powerful_framework***](https://gitee.com/hnuyuelurm/powerful_framework) for reference.

If RTOS tasks run out of stack space, increase the task stack size in CubeMX. Advanced FreeRTOS features can be enabled by turning on the corresponding macros in the CubeMX config page and regenerating.

---

## Roadmap

- Items listed in `.Doc/TODO.md` for potential enhancements and optimizations.
- Consolidate all configuration into a single place for easier modification (move everything into `app/robot_def.h`?).
- Add beginner-level training tutorials for all three layers, supporting standalone module execution for easy onboarding.
- Optimize pub-sub message mechanism performance; migrate app tasks to a **state machine + event-driven** callback model where possible.
- Build a Qt or CLI-based configuration UI for robot setup (mainly `robot_def` and per-task module init configs), enabling no-code robot deployment.
- Write a ROS driver for the framework, connecting to the PC (NUC) via USB and unifying the vision/algorithm and EC development workflows.

---

## Acknowledgements

This framework's design was inspired by the EC_framework of Harbin Institute of Technology (Shenzhen)'s NanGong Xiaoyao Team and the official RoboMaster RoboRTS-firmware. Attitude estimation was improved based on the quaternion EKF implementation from Harbin Engineering University's Chuang Meng Zhi Yi Team. Referee system data parsing was ported from Shenzhen University RoboPilot's 2021 EC hero open-source code.

Special thanks to all electrical engineering group members of the 2022–2023 YueLu Team who participated in testing and developing the new framework, the mechanical group for designing the robot platforms, the vision group for joint debugging, and the operations team for filming, documentation, and promotion.
