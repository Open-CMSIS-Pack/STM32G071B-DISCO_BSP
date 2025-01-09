# STM32G071B-DISCO Discovery board

## Overview

The STM32G071B-DISCO Discovery board is a demonstration and development platform for the STMicroelectronics Arm® Cortex®-M0+ core-based STM32G071RB microcontroller and particularly the USB Type-C™ and Power Delivery controllers. The STM32G071B-DISCO Discovery board is presented in a plastic casing with all necessary interfaces for easy connection to, and interoperability with, other USB Type-C™ devices. The STM32G071B-DISCO Discovery board discovers and displays USB Type-C™ port capabilities such as data role, power role, VBUS and IBUS monitoring. The integrated ST-LINK/V2-1 provides an embedded in-circuit debugger and programmer for the STM32 MCU, allowing SWD debugging support and external GUI trace in the STM32Cube™ USB-PD monitor. All functions are accessible by a 4-way user joystick to glance through menus and information displayed by the 128 x 64 pixels OLED embedded display. An 8-pin user connector gathers all communication signals, with the possibility of additional internal GPIO extensions.

ST-LINK/V2-1 is integrated into the board, as the embedded in-circuit debugger and programmer for the STM32 MCU and USB Virtual COM port bridge.

## Getting started

- [User manual](https://www.st.com/resource/en/user_manual/um2546-stm32g071bdisco-usbc-discovery-kit-stmicroelectronics.pdf)

### ST-LINK driver installation and firmware upgrade (on Microsoft Windows)

1. Download the latest [ST-LINK driver](https://www.st.com/en/development-tools/stsw-link009.html).
2. Extract the archive and run `dpinst_amd64.exe`. Follow the displayed instructions.
3. Download the latest [ST-LINK firmware upgrade](https://www.st.com/en/development-tools/stsw-link007.html).
4. Extract the archive and run the `ST-LinkUpgrade.exe` program.
5. Connect the board to your PC using a USB cable and wait until the USB enumeration is completed.
6. In the **ST-Link Upgrade** program, press the **Device Connect** button.
7. When the ST-LINK driver is correctly installed, the current ST-LINK version is displayed.
8. Press the **Yes >>>>** button to start the firmware upgrade process.

## Technical reference

- [STM32G071RB microcontroller](https://www.st.com/en/microcontrollers-microprocessors/stm32g071rb.html)
- [STM32G071B-DISCO board](https://www.st.com/en/evaluation-tools/stm32g071b-disco.html)
- [User manual](https://www.st.com/resource/en/user_manual/um2546-stm32g071bdisco-usbc-discovery-kit-stmicroelectronics.pdf)
- [Data brief](https://www.st.com/resource/en/data_brief/stm32g071b-disco.pdf)
- [Schematic](https://www.st.com/resource/en/schematic_pack/mb1378-c02_schematic.pdf)
