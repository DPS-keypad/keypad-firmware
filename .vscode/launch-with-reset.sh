#!/bin/bash
echo "Resetting STM32 device..."
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "init; reset halt; exit"
sleep 1
code --open-url "vscode://ms-vscode.cpptools/debug/launch?name=Debug%20Keypad%20Firmware"