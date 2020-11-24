# HP-LEDPWM
WiFi Control LED bright/SCR/Color temperature by using ESP8266/ESP32

Protocol: MQTT

## Features
- Configurable device name
- Adjustable PWM Frequence
- Support Invert/Non-Invert MOS driver/SCR Edge Trigger/SCR Trigger UP/Down/DAC Output
- Two Channel (Multiple ColorTemperatue or Multi Channel)
- PWM Source Output
- Channel EN output
- Analog Input to control bright
- Start/End Range for bright contorl
- Smooth Bright / Color Temperature Change
- Auto save Bright / Color Temperature and will smooth startup
- Constant Bright Control by LDR (Photoresisotr)
- Remote adjustable ADC calibration
- Remote adjustable LDR cal
- Realtime PID status output on web
- Adjustable SCR pre-release
- SCR Zero-crossing filter to prevent mis-trigger
- Adjustable SCR startup delay to support RC network zero-crossing

## Demo MQTT Server
https://github.com/magicbear/HP-MQTTServer

## Diagram
![Diagram](https://raw.githubusercontent.com/magicbear/HP-LEDPWM/master/sch.png)

## PCB
![Diagram](https://raw.githubusercontent.com/magicbear/HP-LEDPWM/master/top.svg)

## Video Demo
https://www.bilibili.com/video/av58690546/

## Depend Arduino Libraries
- PubSubClient
- AutoPID