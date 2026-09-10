> ## ⚠️ Fork for one specific hardware-modified board — do not flash on a stock controller
>
> This is a personal fork of [Koxx3/SmartESC_STM32_v2](https://github.com/Koxx3/SmartESC_STM32_v2)
> (branch `vesc_comp`), modified for a **hardware-modified** Xiaomi M365 controller.
> It is not a general-purpose build and it is **not safe on an unmodified controller.**
>
> The firmware assumes these board changes. Several of them are compiled in, not configurable:
>
> | Change | This build assumes | Stock M365 |
> |---|---|---|
> | Shunts | doubled in parallel, **1 mΩ** | 2 mΩ |
> | Battery | **15S, 63 V** full charge | 10S, 42 V |
> | MOSFETs | **IRFB4110**, 100 V | stock parts |
> | Bulk capacitor | **1000 µF, 100 V** | stock |
> | Vin regulator cap | **100 V part** | stock |
> | Battery divider | R41 **1M** (was 100k), R44 **33k** (was 6.2k) | 100k / 6.2k |
> | Phase V-sense † | R19/R21/R22 **68k** (was 22k), R6 **39k** (was 20k) | 22k / 20k |
> | Motor | Xiaomi **Scooter 4** hub motor, 10" wheel | M365 motor |
> | Display | custom **ESP32** on the half-duplex UART, M365 0x64/0x65 protocol | stock M365 display |
>
> † Phase V-sense is the one row that does not change firmware behaviour — those ADC
> channels are read by nothing. It is pin protection for the higher pack voltage.
>
> **On a stock controller every current reading is halved, so every real current
> limit doubles** — into stock FETs and a 10S pack. The battery voltage divider is
> also calibrated for the modified one, so voltage and every cutoff derived from it
> will be wrong.
>
> There is also **no motor temperature protection** in this firmware: `l_temp_motor_start`
> and `l_temp_motor_end` are stored but read by nothing, and the motor temperature
> readback is hardcoded to 0.
>
> If your controller is stock, use [the upstream project](https://github.com/Koxx3/SmartESC_STM32_v2)
> instead. No support, no warranty — this runs a vehicle, and you are responsible for
> what you flash.
>
> Every change against upstream is documented in [`docs/CHANGES.md`](docs/CHANGES.md).

# SmartESC

**SmartESC (aka SESC) is an alternative firmware for Xiaomi M365 and Ninebot G30 controller.**

![image](https://user-images.githubusercontent.com/11454444/148704200-e28ee13e-c91b-4aac-8dbf-6021095749a5.png)

# Overview

Avantage over other Xiaomi custom firmwares :
- you can put any motor since it can detect all motor parameters and optimize them
- you can use any battery, even 20s (with hardware modifications), change the voltage divider and set the new value in the controller
- you can change the shunts values and set the new value in the controller
- you can setup the controller very easily with VESCTool and make a lot of performance/stability tests
- you can use the controller with any other device, event without any display
- soon, we hope to support multiple linked controller

Cons :
- you loose the ability to monitor M365 BMS for now (work in progress), but don't worry, you still have the voltage ;)
- you loose the ability to update firmwares through bluetooth
- for now, it needs hall sensors (no sensorless mode). [work in progress]

It can interface :
- the stock M365/G30 display
- VESCTool through VESC interface on the BMS UART (full duplex UART).

Nota : this firmware is in beta. 

You'll be able to setup and control the controller/motor from VESCTool interface with a simple USB/Serial adapter.
With any small arduino, use analog acceleration/brake throttles to control any electic moving device like escooter, gokart, electric skateboard without using the stock display.


# Download 

Download the latest build for M365 : [![Package Control total downloads](https://img.shields.io/github/downloads/Koxx3/SmartESC_STM32_v2/total.svg)](https://github.com/Koxx3/SmartESC_STM32_v2/releases/latest/download/m365.bin)


# Build

Last automatic build status : [![Build on commit](https://github.com/Koxx3/SmartESC_STM32_v2/actions/workflows/build_on_commit.yml/badge.svg)](https://github.com/Koxx3/SmartESC_STM32_v2/actions/workflows/build_on_commit.yml)

If you want to build it manually, for an easier build, you need `git` and `docker`.

## Clone the project
`git clone https://github.com/Koxx3/SmartESC_STM32_v2.git`

## Build on Linux
Launch from terminal:

`chmod +x docker_build*; ./docker_build_m365.sh`

`chmod +x docker_build*; ./docker_build_g30.sh`

## Build on Windows
Double click :

`docker_build_m365.sh`

`docker_build_g30.sh`


# Programming

You need a ST-Link device to reprogram the M365/G3O controller.
It costs 3/4€ on Aliexpress.

Plug the st-link following this schematic :
![image](https://user-images.githubusercontent.com/11454444/146688635-b5a1ed07-3482-420f-b324-9e58b0a19dc9.png)

With [STM32 ST-Link Utility](https://www.st.com/en/development-tools/stsw-link004.html), disable Re&d out protection.

Menu "Target" => "Option bytes"
![image](https://user-images.githubusercontent.com/11454444/146688019-3e5122c7-f3fb-4964-a44f-684af023746e.png)


# VescTool

Use [VescTool](https://vesc-project.com/vesc_tool) to setup the motor and input properties.

Use a serial USB adapter to connect the Xiaomi controller as an USB VESC :
![image](https://user-images.githubusercontent.com/11454444/146688647-e3e4d833-7c93-4b4b-a297-cc61ba52071e.png)

Launch VESCTool and connect with COM port.
![image](https://user-images.githubusercontent.com/11454444/146687240-e393ea2e-dfd9-4fac-870e-4cf526a61187.png)

Launcher Motor setup wizzard.
![image](https://user-images.githubusercontent.com/11454444/146688494-b4a6c183-a89f-4517-af1f-61b5358aad40.png)

Enter all your settings in the different windows.

Enable the keyboard control :

![image](https://user-images.githubusercontent.com/11454444/146688470-adf8a8f7-e3b4-43f4-9038-479d3d5585c5.png)

You're ready to test your M365 controller with your keyboard !


# ESP32 test module

M365 connections :
![image](https://user-images.githubusercontent.com/11454444/146688619-c3bc8e6d-6884-4b1c-81d6-9ec456d1e41b.png)

Use ESP32 prototype board with and ESP32-devkit-c module :
![image](https://user-images.githubusercontent.com/11454444/146688428-d8978339-fab1-4a7b-a88f-305298b6b64f.png)

Use the code provided in the [serial-trottle-brake-esp32](/serial-trottle-brake-esp32) folder with Platform.io

# Error Code SESC (Smart ESC) in VESC Tool

Can read it in "VESC Terminal" (or others Serial Terminal)

- 0 = MC_NO_ERROR     (No error)
- 0= MC_NO_FAULTS     (No error)
- 1 = MC_FOC_DURATION (FOC rate to high)
- 2 = MC_OVER_VOLT    (Software over voltage)
- 4 = MC_UNDER_VOLT   (Software under voltage)
- 8 = MC_OVER_TEMP    (Software over temperature)
- 16 = MC_START_UP    (Startup failed)
- 32 = MC_SPEED_FDBK  (Speed feedback)
- 64 = MC_BREAK_IN    (Emergency input (Over current))
- 128 = MC_SW_ERROR

# Command available in VESCTool terminal

- help (see all available commands)
- foc_openloop [current] [erpm]  (Exemple : foc_openloop 10 500)
- ... (lot of other commands)
