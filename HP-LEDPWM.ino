#ifdef ARDUINO_ARCH_ESP32
  #include <WiFi.h>
  #include <HTTPUpdate.h>
  #include <math.h>
  #include <driver/dac.h>
  #include <driver/ledc.h>
  #include <rom/rtc.h>
  #include <esp_task_wdt.h>
  #define ESPhttpUpdate httpUpdate
#elif ARDUINO_ARCH_ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266httpUpdate.h>
  extern "C"{
      #include "pwm.h"
  }
#endif
#include <PubSubClient.h>
#include "config.h"

// CONFIG: REVERSE THE OUTPUT FOR MOSFET+BJT Driver
// WORK_MODE = 2: USING SCR to OUTPUT
/*
 * MODE 0  => CH1 NON-INVENTING   CH2 INVERTING
 * MODE 1  => CH1 INVERTING       CH2 NON-INVERTING
 * MODE 2  => CH1 TRIGGER NON-INVERTING
 * MODE 3  => CH1 INVERTING       CH2 INVERTING
 */
uint8_t WORK_MODE = 1;
// CONFIG: SMOOTH PWM INTERVAL IN mS
unsigned int SMOOTH_INTERVAL = 200;
uint16_t STARTUP_SMOOTH_INTERVAL = 500;
// CONFIG: Warm / Single LED PWM Pin  (ESP-01  RXD0)
uint8_t  PWM_PIN = 3;
uint8_t  PWM_WPIN = 0;
uint8_t  PWM_SCR_TRIGGER = 0;
int16_t  PWM_SCR_DELAY = 0;
uint8_t  PWM_AUTO_FULL_POWER = 1;

#define DEBUG_LEVEL 2

#ifndef MQTT_CLASS
#define MQTT_CLASS "HP-LEDPWM"
#define _VERSION "2.30"

#ifdef ARDUINO_ARCH_ESP32
#define VERSION _VERSION"_32"
#define timer1_write(value) if (timer != NULL) { timerWrite(timer, 5000000 - value); timerAlarmEnable(timer); }
#else
#define VERSION _VERSION
  #define PWM_CHANNELS 2
  // PWM setup (choice all pins that you use PWM)
  uint32 io_info[PWM_CHANNELS][3];
  
  // PWM initial duty: all off
  uint32 pwm_duty_init[PWM_CHANNELS];
#endif

#endif

uint8_t  CFG_RESET_PIN = 0;
// Period of PWM frequency -> default of SDK: 5000 -> * 200ns ^= 1 kHz
uint16_t PWM_PERIOD = 200;
#ifdef ARDUINO_ARCH_ESP32
uint32_t PWM_FREQUENCE = 20000;
#else
uint16_t PWM_FREQUENCE = 20000;
#endif

uint8_t  PWM_SOURCE  = 0;
uint8_t  PWM_SOURCE_DUTY = 0;
uint16_t PWM_START = 0;
uint16_t PWM_END = PWM_PERIOD;

uint8_t  EN_PORT_CH1 = 0;
uint8_t  EN_PORT_CH2 = 0;

bool save_config = true;
bool ignoreOutput = false;

// Current Bright in PWM Range (正比)
int scr_current_bright = 0;
int scr_current_bright2 = 0;

uint16_t set_ct = 4000;
uint8_t  set_state = 0;
uint8_t  inTrigger = 0;
uint8_t  CH2_MODE = 0;

uint32_t zc_interval = 0;
uint32_t zc_last = 0;
uint32_t last_zc_interval = 0;

uint8_t  AUTO_SAVE_DISABLED = 0;

int retry_failed_count = 0;

char mqtt_cls[sizeof(MQTT_CLASS) + 13];
char msg_buf[160];

uint8_t BRIGHT_ADC_PIN = 0;

#ifdef ARDUINO_ARCH_ESP32
uint16_t OLD_PWM_FREQ = 0;
hw_timer_t * timer = NULL;
#endif

const hp_cfg_t def_cfg[] = {
  {31, sizeof(uint8_t), (uint8_t)10,      &set_state, true},        // BRIGHT
  {32, 0, (uint8_t)0, &dev_name, true},                    // NAME LENGTH
#ifdef ARDUINO_ARCH_ESP32
  {60, sizeof(uint32_t), (uint32_t)20000, &PWM_FREQUENCE, false},   // UINT16: PWM FREQUENCE
  {64, sizeof(uint16_t), (uint16_t)0, &OLD_PWM_FREQ, false},   // UINT16: PWM FREQUENCE
#else
  {64, sizeof(uint16_t), (uint16_t)20000, &PWM_FREQUENCE, false},   // UINT16: PWM FREQUENCE
#endif
  {66, sizeof(uint16_t), (uint16_t)0,     &PWM_START, false},       // UINT16: PWM START
  {68, sizeof(uint16_t), (uint16_t)200,   &PWM_END, false},         // UINT16: PWM END
  {70, sizeof(uint16_t), (uint16_t)4100,  &set_ct, false},          // UINT16: COLOR TEMPERATURE / BRIGHT 2
  {72, sizeof(uint16_t), (uint16_t)200,   &PWM_PERIOD, false},      // UINT16: PWM PERIOD
  {74, sizeof(uint8_t), (uint8_t)0,   &PWM_PIN, false},             // UINT8:  PWM_PIN    Single-CH / Warm LED PIN
  {75, sizeof(uint8_t), (uint8_t)0,   &PWM_WPIN, false},            // UINT8:  PWM_WPIN   White LED PIN
  {76, sizeof(uint8_t), (uint8_t)1,   &WORK_MODE, false},           // UINT8:  MODE   0 NON-INVERT  1 INVERT  2 SCR NON-INVERT  3 SCR INVERT
  {77, sizeof(uint8_t), (uint8_t)0,   &PWM_SCR_TRIGGER, false},     // UINT8:  SCR MODE ZeroDetect Pin
  {78, sizeof(int16_t), (int16_t)0,   &PWM_SCR_DELAY, false},       // UINT16: SCR MODE ZeroDetect DELAY
  {80, sizeof(uint8_t), (uint8_t)0,   &CFG_RESET_PIN, false},       // UINT8:  RESET PIN
  {81, sizeof(uint8_t), (uint8_t)0,   &EN_PORT_CH1, false},             // UINT8:  EN PIN
  {82, sizeof(uint8_t), (uint8_t)1,   &PWM_AUTO_FULL_POWER, false}, // UINT8:  AUTO FULL POWER
  {83, sizeof(uint8_t), (uint8_t)0,   &EN_PORT_CH2, false},             // UINT8:  EN PIN
  {84, sizeof(uint16_t), (uint16_t)500, &STARTUP_SMOOTH_INTERVAL, true},    // UINT16: STARTUP SMOOTH INTERVAL
  {86, sizeof(uint8_t), (uint8_t)0,   &PWM_SOURCE, false},             // UINT8:  PWM_SOURCE
  {87, sizeof(uint8_t), (uint8_t)0,   &PWM_SOURCE_DUTY, false},             // UINT8:  PWM_SOURCE
  {88, sizeof(uint8_t), (uint8_t)0,   &BRIGHT_ADC_PIN, false},             // UINT8:  BRIGHT_ADC_PIN
  {89, sizeof(uint8_t), (uint8_t)0,   &AUTO_SAVE_DISABLED, false},             // UINT8:  BRIGHT_ADC_PIN
  {90, sizeof(uint8_t), (uint8_t)0,   &CH2_MODE, false},             // UINT8:  CH2_MODE
  {96, 0, (uint8_t)0, &ssid, true},                    // STRING: SSID
  {128, 0, (uint8_t)0, &password, true},                   // STRING: WIFI PASSWORD 
  {160, 0, (uint8_t)0, &mqtt_server, true},        // STRING: MQTT SERVER
  {192, sizeof(uint16_t), (uint16_t)1234, &port, true},             // UINT16: PORT
  {NULL, 0,0, NULL, false}
};

#define STARTUP_SMOOTH_EXECUTE updatePWMValue((last_set_state - (float)(last_set_state - set_state) * (millis() - last_state_hold) / STARTUP_SMOOTH_INTERVAL), CH2_MODE == 0 ? set_ct : (last_set_ct - (float)(last_set_ct - set_ct) * (millis() - last_state_hold) / STARTUP_SMOOTH_INTERVAL));
#define isScrMode (WORK_MODE == 2 || WORK_MODE == 3 || WORK_MODE == 4)

WiFiClient espClient;
PubSubClient client(espClient);

bool inSmooth = false;
char last_state = -1;
int last_set_state = 0;
uint16_t last_set_ct = 4000;
long last_rssi = -1;
unsigned long last_send_rssi;
unsigned long last_send_meta = 0;
unsigned long last_state_hold;
bool otaMode = true;                             //OTA mode flag
bool hasPacket = false;
uint32_t boot_time;
bool bootCountReset = false;

int16_t lastAdcValue = 0;

#ifdef ARDUINO_ARCH_ESP8266
uint32_t PinToGPIOMuxFunc(uint8_t pin)
{
    // FUNC_GPIO0:  0, 2, 4, 5
    // FUNC_GPIO3: 1, 3, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
    if (pin == 0 || pin == 2 || pin == 4 || pin == 5) return FUNC_GPIO0;
    return FUNC_GPIO3;
}

uint32_t PinToGPIOMux(uint8_t pin)
{
    if (pin == 0) return PERIPHS_IO_MUX_GPIO0_U;
    if (pin == 1) return PERIPHS_IO_MUX_U0TXD_U;
    if (pin == 2) return PERIPHS_IO_MUX_GPIO2_U;
    if (pin == 3) return PERIPHS_IO_MUX_U0RXD_U;
    if (pin == 4) return PERIPHS_IO_MUX_GPIO4_U;
    if (pin == 5) return PERIPHS_IO_MUX_GPIO5_U;
    if (pin == 6) return PERIPHS_IO_MUX_SD_CLK_U;
    if (pin == 7) return PERIPHS_IO_MUX_SD_DATA0_U;
    if (pin == 8) return PERIPHS_IO_MUX_SD_DATA1_U;
    if (pin == 9) return PERIPHS_IO_MUX_SD_DATA2_U;
    if (pin == 10) return PERIPHS_IO_MUX_SD_DATA3_U;
    if (pin == 11) return PERIPHS_IO_MUX_SD_CMD_U;
    if (pin == 12) return PERIPHS_IO_MUX_MTDI_U;
    if (pin == 13) return PERIPHS_IO_MUX_MTCK_U;
    if (pin == 14) return PERIPHS_IO_MUX_MTMS_U;
    if (pin == 15) return PERIPHS_IO_MUX_MTDO_U;
}
#endif

bool isChannelInvert(uint8_t pin)
{
    if (WORK_MODE == 0)
    {
        if (pin == PWM_PIN) return false;
        if (pin == PWM_WPIN) return true;
    } else if (WORK_MODE == 1)
    {
        if (pin == PWM_PIN) return true;
        if (pin == PWM_WPIN) return false;
    } else if (WORK_MODE == 3)
    {
//        return true;
    }
    return false;
}

void analogPinInit(int32_t freq, int32_t period, uint8_t pwm_pin)
{
    bool auto_update_pwm_end = false;
#if DEBUG_LEVEL >= 3
    Serial.printf("Initalize Analog PWM Pin %d (Period = %d   Freq = %d)\n", pwm_pin, period, freq);
#endif
#ifdef ARDUINO_ARCH_ESP32
    uint8_t bits = (int)log2(period)+1;
    if (pow(2, log2(period)) == period)
    {
        bits--;
    }
    if (isScrMode)
    {
        pinMode(pwm_pin, OUTPUT);
        digitalWrite(pwm_pin, HIGH);
    } else if (pwm_pin == 101 || pwm_pin == 102) 
    {
        dac_output_enable(pwm_pin == 101 ? DAC_CHANNEL_1 : DAC_CHANNEL_2);
    } else if (pwm_pin == PWM_PIN)
    {
        if (PWM_END == PWM_PERIOD)
        {
            auto_update_pwm_end = true;
        }
        PWM_PERIOD = pow(2, bits);
        if (auto_update_pwm_end) PWM_END = PWM_PERIOD;
        ledcSetup(0, freq, bits);
        ledcWrite(0, isChannelInvert(pwm_pin) ? PWM_PERIOD : 0);
        ledcAttachPin(pwm_pin, 0);
    } else if (pwm_pin == PWM_WPIN) {
        ledcSetup(1, freq, bits);
        ledcWrite(1, isChannelInvert(pwm_pin) ? PWM_PERIOD : 0);
        ledcAttachPin(pwm_pin, 1);
    } else if (pwm_pin == PWM_SOURCE) {
        ledcSetup(2, freq, bits);
        ledcWrite(2, pow(2, bits) / 2);
        ledcAttachPin(pwm_pin, 2);
    }
#else
    if (PWM_END == PWM_PERIOD)
    {
        auto_update_pwm_end = true;
    }
    if (5000000 % freq == 0)
    {
        PWM_PERIOD = 5000000 / PWM_FREQUENCE;
    } else 
    {
        PWM_PERIOD = 320;
        freq = 5000000 / PWM_PERIOD;
    }
    if (auto_update_pwm_end) PWM_END = PWM_PERIOD;

    pinMode(pwm_pin, OUTPUT);

    if (WORK_MODE == 0 || WORK_MODE == 1)
    {
        if (pwm_pin == PWM_PIN)
        {
            io_info[0][0] = PinToGPIOMux(pwm_pin);
            io_info[0][1] = PinToGPIOMuxFunc(pwm_pin);
            io_info[0][2] = pwm_pin;
            pwm_duty_init[0] = isChannelInvert(pwm_pin) ? PWM_PERIOD : 0;
        } else if (pwm_pin == PWM_WPIN) {
            io_info[1][0] = PinToGPIOMux(pwm_pin);
            io_info[1][1] = PinToGPIOMuxFunc(pwm_pin);
            io_info[1][2] = pwm_pin;
            pwm_duty_init[1] = isChannelInvert(pwm_pin) ? PWM_PERIOD : 0;
        }
        digitalWrite(pwm_pin, isChannelInvert(pwm_pin) ? PWM_PERIOD : 0);
        // Initialize
        pwm_init(PWM_PERIOD, pwm_duty_init, PWM_WPIN == 0 ? 1 : 2, io_info);
     
        digitalWrite(pwm_pin, isChannelInvert(pwm_pin) ? PWM_PERIOD : 0);
        if (pwm_pin == PWM_PIN)
        {
            pwm_set_duty(isChannelInvert(pwm_pin) ? PWM_PERIOD : 0, 0);
        } else if (pwm_pin == PWM_WPIN) {
            pwm_set_duty(isChannelInvert(pwm_pin) ? PWM_PERIOD : 0, 1);
        }
        // Commit
        pwm_start(); 
        digitalWrite(pwm_pin, isChannelInvert(pwm_pin) ? PWM_PERIOD : 0);
    } else if (isScrMode)
    {
        digitalWrite(pwm_pin, isChannelInvert(pwm_pin) ? LOW : HIGH);
    }
#endif
}

void pwmWrite(uint8_t pwm_pin, int32_t duty)
{
    if (isScrMode) return;
#ifdef ARDUINO_ARCH_ESP32
    if (pwm_pin == 101)
    {
        dacWrite(25, duty * 255 / PWM_PERIOD);
    } else if (pwm_pin == 102)
    {
        dacWrite(26, duty * 255 / PWM_PERIOD);
    } else if (pwm_pin == PWM_PIN)
    {
        ledcWrite(0, duty);
    } else if (pwm_pin == PWM_WPIN)
    {
        ledcWrite(1, duty);
    } else if (pwm_pin == PWM_SOURCE)
    {
        ledcWrite(2, duty);
    }
#else
    if (pwm_pin == PWM_PIN)
    {
        pwm_set_duty(duty, 0);
    } else if (pwm_pin == PWM_WPIN)
    {
        pwm_set_duty(duty, 1);
    }
      pwm_start(); // commit

//    analogWrite(pwm_pin, duty);
#endif
}

void rebootSystem()
{
    ESP.restart();
//#ifdef ARDUINO_ARCH_ESP32
//
//#else
//    ESP.reset();
//#endif
}

float mapfloat(float x, long in_min, long in_max, long out_min, long out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void updatePWMValue(float set_state, float set_ct)
{
    int pwm_state = mapfloat(set_state, 0, 100, PWM_START, PWM_END);
    if (pwm_state == PWM_START) pwm_state = 0;
    if (pwm_state == PWM_END && PWM_AUTO_FULL_POWER) pwm_state = PWM_PERIOD;
    if (PWM_WPIN != 0)
    {
        float warm_power = set_state / 100.;
        float cold_power = 0;

        if (CH2_MODE == 0)
        {
            float set_power = 0.5 + (set_ct - 4100) / 3000.;
            
            warm_power = 1-set_power;
            cold_power = set_power;
            
            if (set_power > 0 && set_power < 0.5)
            {
                warm_power = 1;
                cold_power = set_power / (1-set_power);
            } else if (set_power >= 0.5 && set_power < 1)
            {
                warm_power = (1-set_power) / set_power;
                cold_power = 1;
            }
        } else 
        {
            pwm_state = PWM_END;
            cold_power = set_ct / 100.;
        }
        
    #if DEBUG_LEVEL >= 2
        if (!ignoreOutput)
        {
            if (CH2_MODE == 0)
            {
                Serial.printf("Set Power %f PWM = %d  CT: %f  COLD: %d%% %d WARM: %d%% %d\n", set_state, pwm_state, set_ct, (int)(cold_power * 100), (int)(pwm_state * cold_power), (int)(warm_power * 100), (int)(pwm_state * warm_power));
            } else 
            {
                Serial.printf("Set Power CH1 = %f PWM = %d  CH2: %f   CH1(COLD): %d%% %d   CH2(WARM): %d%% %d\n", set_state, pwm_state, set_ct, (int)(cold_power * 100), (int)(pwm_state * cold_power), (int)(warm_power * 100), (int)(pwm_state * warm_power));
            }
        }
    #endif
    
        if (EN_PORT_CH1 != 0)
        {
            if (warm_power == 0)
            {
                pinMode(EN_PORT_CH1, OUTPUT);
                digitalWrite(EN_PORT_CH1, HIGH);
            } else 
            {
                pinMode(EN_PORT_CH1, INPUT);
            }
        }
        if (EN_PORT_CH2 != 0)
        {
            if (cold_power == 0)
            {
                pinMode(EN_PORT_CH1, OUTPUT);
                digitalWrite(EN_PORT_CH1, HIGH);
            } else 
            {
                pinMode(EN_PORT_CH1, INPUT);
            }
        }

        if (!isScrMode)
        {
            pwmWrite(PWM_PIN, isChannelInvert(PWM_PIN) ? PWM_PERIOD - pwm_state * warm_power : pwm_state * warm_power);
            pwmWrite(PWM_WPIN, isChannelInvert(PWM_WPIN) ? PWM_PERIOD - pwm_state * cold_power : pwm_state * cold_power);
        } else 
        {
            scr_current_bright = pwm_state * warm_power;
            scr_current_bright2 = pwm_state * cold_power;            
        }
    } else {
        if (EN_PORT_CH1 != 0)
        {
            if (set_state == 0)
            {
                pinMode(EN_PORT_CH1, OUTPUT);
                digitalWrite(EN_PORT_CH1, HIGH);
            } else 
            {
                pinMode(EN_PORT_CH1, INPUT);
            }
        }
        
        if (!isScrMode)
        {
            pwmWrite(PWM_PIN, isChannelInvert(PWM_PIN) ? PWM_PERIOD - pwm_state : pwm_state);       
        } else 
        {
            scr_current_bright = pwm_state;
        }
        #if DEBUG_LEVEL >= 2
        if (!isScrMode && !ignoreOutput)
            Serial.printf("Set Power Power %f  => %d\n", set_state, pwm_state);
        #endif
    }
    if (PWM_SOURCE != 0)
    {
        pwmWrite(PWM_SOURCE, PWM_PERIOD * PWM_SOURCE_DUTY / 100);
    }
}

uint8_t ZC = 0;
uint8_t justZeroCrossing = 0;
uint32_t ZC_id = 0;
uint32_t lastZC_id = 0;
bool timerRunning = false;
bool scrState = 0;

inline void updateSCRState(bool state, int pin)
{
    scrState = state;
    if (state == false)
    {
#if DEBUG_LEVEL >= 5
        Serial.printf("S %ld\n", micros());
#endif
//        if (scr_current_bright != PWM_PERIOD)
        {
            digitalWrite(pin, HIGH);
        }
    } else 
    {
#if DEBUG_LEVEL >= 5
        Serial.printf("O %ld\n", micros());
#endif
        // 打开SCR
        digitalWrite(pin, LOW);
    }
}

uint32_t target_scr_timerout1 = 0;
uint32_t target_scr_timerout2 = 0;

void ICACHE_RAM_ATTR onTimerISR(){
    if (PWM_SCR_DELAY != 0)
    {
          uint16_t on_time = (float(scr_current_bright) / PWM_PERIOD * zc_interval); // SCR打开时间
          bool using_cycleTime = on_time * 2 / zc_interval > 0;
          uint32_t zc_cycle = micros() - zc_last;

          // (using_cycleTime && zc_cycle <= PWM_SCR_DELAY + 30) || (!using_cycleTime && 
          if (justZeroCrossing == 1)
          {
              justZeroCrossing = 2;
              // 新过零触发，旧触发开启中
              if (scr_current_bright != PWM_PERIOD)
              {
                  updateSCRState(false, PWM_PIN); // 过零触发，关闭SCR
                  timer1_write(5 * (zc_interval - on_time + ((int32_t)PWM_SCR_DELAY - (int32_t)zc_cycle)));
//                  Serial.printf("w %d %d\n", zc_interval - on_time + ((int32_t)(PWM_SCR_DELAY - (int32_t)zc_cycle)), on_time);
              } else {
                  timerRunning = false;
              }
          } else if (scr_current_bright != 0)
          {
              lastZC_id = ZC_id + 1;
              updateSCRState(true, PWM_PIN); // 打开SCR
              justZeroCrossing = 1;
              timer1_write(5 * (on_time));
          }
    }
    // 打开SCR
    else
    {
        if (target_scr_timerout1 != 0 && micros() >= target_scr_timerout1 && scr_current_bright != 0)
        {
            target_scr_timerout1 = 0;
            timerRunning = false;
            updateSCRState(true, PWM_PIN); // 打开SCR
        }
        if (target_scr_timerout2 != 0 && micros() >= target_scr_timerout2 && scr_current_bright2 != 0)
        {
            target_scr_timerout2 = 0;
            timerRunning = false;
            updateSCRState(true, PWM_WPIN); // 打开SCR
        }
    }
}

void ICACHE_RAM_ATTR ZC_detect()
{
    inTrigger = 1;
    if (zc_interval == 0)
    {
        if (zc_last != 0)
        {
            zc_interval = micros() - zc_last;
        } else {
            zc_last = micros();
        }
    } else if (micros() - zc_last >= 3000)  // Max 300 Hz
    {
        if (micros() - zc_last <= 12500)  // Min at 80 Hz
        {
            zc_interval = micros() - zc_last;          
        }
        zc_last = micros();
        ZC = 1;

        // DIV 16 = 80/16 = 5 Mhz = 0.2us per tick
        // 1us = 5 tick
        if (PWM_SCR_DELAY != 0)
        {
            if (scr_current_bright == PWM_PERIOD)
            {
                updateSCRState(true, PWM_PIN); // 过零触发，关闭SCR
            } else if (!timerRunning)
            {
                justZeroCrossing = 1;
                ZC_id = ZC_id + 1;
                timerRunning = true;
                timer1_write(5 * PWM_SCR_DELAY);
            }
        } else {
            updateSCRState(false, PWM_PIN); // 过零触发，关闭SCR
            if (PWM_WPIN != 0)
            {
                updateSCRState(false, PWM_WPIN); // 过零触发，关闭SCR
            }
            if (scr_current_bright != 0){
                timerRunning = true;
                long tSleepus = map(scr_current_bright, PWM_START, PWM_PERIOD, zc_interval, 0);
                target_scr_timerout1 = micros() + tSleepus;
                if (PWM_WPIN != 0){
                    long tSleepus2 = map(scr_current_bright2, PWM_START, PWM_PERIOD, zc_interval, 0);
                    target_scr_timerout2 = micros() + tSleepus2;
                }
//                long tSleepus = map(scr_current_bright, PWM_START, PWM_PERIOD, 5 * zc_interval, 0);
    #if DEBUG_LEVEL >= 5
                Serial.printf("T: %d\n", tSleepus);
    #endif
    //(zc_interval - (float(scr_current_bright) / PWM_PERIOD * zc_interval)
//                timer1_write(tSleepus);
            }
        }
    }
    inTrigger = 0;
}


void inline disableInterrupts()
{
#if DEBUG_LEVEL >= 3
  Serial.printf("Disable interrupt\n");
#endif
  timerRunning = false;
  if (PWM_SCR_TRIGGER != 0)
  {
     detachInterrupt(PWM_SCR_TRIGGER);
#if ARDUINO_ARCH_ESP8266
     timer1_detachInterrupt();
     timer1_disable();
     timer1_isr_init();
#elif ARDUINO_ARCH_ESP32
     if (timer != NULL)
     {
        timerEnd(timer);
     }
#endif
  }
}

void inline enableInterrupts()
{
#if DEBUG_LEVEL >= 3
  Serial.printf("Enable interrupt\n");
#endif
  if (PWM_SCR_TRIGGER != 0)
  {
      if (WORK_MODE == 2)
      {
          pinMode(PWM_SCR_TRIGGER, INPUT_PULLUP);
          attachInterrupt(PWM_SCR_TRIGGER, ZC_detect, RISING);       // Enable external interrupt (INT0)
      } else if (WORK_MODE == 3) {        
          pinMode(PWM_SCR_TRIGGER, INPUT_PULLUP);
          attachInterrupt(PWM_SCR_TRIGGER, ZC_detect, FALLING);       // Enable external interrupt (INT0)
      } else if (WORK_MODE == 4) {        
          pinMode(PWM_SCR_TRIGGER, INPUT_PULLUP);
          attachInterrupt(PWM_SCR_TRIGGER, ZC_detect, CHANGE);       // Enable external interrupt (INT0)
      }
//      pinMode(PWM_PIN, 
//      if (IN_STARTUP)
      {
          if (WORK_MODE == 2 || WORK_MODE == 3 || WORK_MODE == 4) {
#if ARDUINO_ARCH_ESP8266
              timer1_disable();
              timer1_attachInterrupt(onTimerISR);
              timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
#elif ARDUINO_ARCH_ESP32
              if (timer != NULL)
              {
                  timerEnd(timer);
              }
              timer = timerBegin(0, getApbFrequency() / 5000000, true); // timer_id = 0; divider=80; countUp = true;
              timerAttachInterrupt(timer, &onTimerISR, true);
              if (PWM_SCR_DELAY != 0)
              {
                  timerAlarmWrite(timer, 5000000, false); // Run by manual tick
              } else {
                  timerAlarmWrite(timer, 1250, true);  // Run by auto tick with 800 Hz
              }
              timerAlarmEnable(timer);
#endif
          }
      }
  }
}

//
//void ESP_delayMicroseconds(uint32_t us){
//  uint32_t start = micros();
//  while(micros() - start < us){ yield(); }
//}

void setup() {
  uint16_t boot_count = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.persistent( false );
  Serial.begin(115200);
  byte mac[6];
  WiFi.macAddress(mac);
  sprintf(mqtt_cls, MQTT_CLASS"-%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
#ifdef ARDUINO_ARCH_ESP8266
  rst_info *resetInfo;
  resetInfo = ESP.getResetInfoPtr();
#elif ARDUINO_ARCH_ESP32
  int resetCode = rtc_get_reset_reason(0);
#endif
  
  cfg_begin();
  bool cfgCheck = CFG_CHECK();
  if (!CFG_LOAD())
  {
      CFG_INIT(true);
  }
  if (PWM_FREQUENCE == 0 || PWM_PERIOD == 0)
  {
      Serial.printf("Invalid PWM Frequence, reset configure\n");
      CFG_INIT(true);
      CFG_LOAD();
  }
  if (!cfgCheck)
  {
#if ARDUINO_ARCH_ESP32
      uint16_t pin35_value = analogRead(35);
      // >= V1.7.1 PCB Auto Settings
      pinMode(27, INPUT_PULLUP);
      pinMode(34, INPUT_PULLUP);
      pinMode(35, INPUT_PULLUP);
      delay(100);
      // >= V1.7.1 PCB Auto Settings
      Serial.printf("AutoConf Resistor Value: %ld\n", pin35_value);
      if (2400 <= pin35_value && pin35_value <= 2750) // 4.7k/10k with 5%
      {
          Serial.printf("Auto Initalize by PCB Board - SCR Version\n");
          WORK_MODE = 4;
          PWM_PIN = 4;
          PWM_WPIN = 0;
          EN_PORT_CH1 = 0;
          EN_PORT_CH2 = 0;
          PWM_SCR_TRIGGER = 5;
      } else if (digitalRead(34) == HIGH)
      {
          Serial.printf("Auto Initalize by PCB Board - Dual CH\n");
          WORK_MODE = 0;
          PWM_PIN = 25;
          PWM_WPIN = 26;
          EN_PORT_CH1 = 32;
          EN_PORT_CH2 = 33;
      } else if (digitalRead(35) == HIGH)
      {
          Serial.printf("Auto Initalize by PCB Board - Single CH\n");
          WORK_MODE = 0;
          PWM_PIN = 25;
          PWM_WPIN = 0;
          EN_PORT_CH1 = 32;
          EN_PORT_CH2 = 0;
      } else if (digitalRead(27) == LOW)
      {
          Serial.printf("Auto Initalize by PCB Board - Dual CH V1.7\n");
          WORK_MODE = 0;
          PWM_PIN = 26;
          PWM_WPIN = 25;
      }
#endif
  }
  if (EN_PORT_CH1 != 0)
  {
      pinMode(EN_PORT_CH1, INPUT);
  }
  if (EN_PORT_CH2 != 0)
  {
      pinMode(EN_PORT_CH2, INPUT);
  }

  char *p;
  char chEEP;
  int iOffset;
  int rc;
  
  analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_PIN);
  if (PWM_WPIN != 0)
  {
      analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_WPIN);
  }
  if (PWM_SOURCE != 0)
  {
      analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_SOURCE);
  }
  ignoreOutput = true;
  updatePWMValue(0, CH2_MODE == 0 ? set_ct : 0);
  uint32_t last_state_hold = 0;
#ifdef ARDUINO_ARCH_ESP8266
  if ((resetInfo->reason == REASON_DEFAULT_RST || resetInfo->reason == REASON_EXT_SYS_RST) && strlen(ssid) != 0)
#elif ARDUINO_ARCH_ESP32
  if (PWM_FREQUENCE <= 40000)
  {
      setCpuFrequencyMhz(80);
  }
  if ((resetCode == POWERON_RESET || resetCode == RTCWDT_RTC_RESET) && strlen(ssid) != 0)
#endif
  {
      last_state_hold = millis();
      last_set_state = 0;
      if (CH2_MODE == 1) last_set_ct = 0;
      boot_count = boot_count_increase();
  }
#ifdef ARDUINO_ARCH_ESP8266
  // ESP8266 will break PWM update between WiFi connecting, so we need to make smooth before WiFi action
  if (WORK_MODE == 0 || WORK_MODE == 1)
  {
      while (last_state_hold != 0  && millis() - last_state_hold < STARTUP_SMOOTH_INTERVAL)
      {
          STARTUP_SMOOTH_EXECUTE;
          yield();
      }
  }
#endif
  
  if (CFG_RESET_PIN != 0)
  {
      pinMode(CFG_RESET_PIN, INPUT_PULLUP);
      delay(1);
      if (digitalRead(CFG_RESET_PIN) == LOW)
      {
          Serial.printf("Reset config by RESET PIN %d\n", CFG_RESET_PIN);
          CFG_INIT(false);
          CFG_LOAD();
          rebootSystem();
      }
  } else {
      if (boot_count > 5)
      {
          Serial.printf("Reset config by Reboot 5 times\n");
          CFG_INIT(false);
          CFG_LOAD();
          rebootSystem();
      }
  }

  boot_time = millis();
  if (PWM_PIN != LED_BUILTIN)
  {
      digitalWrite(LED_BUILTIN, LOW);
  }
  
//  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);

  Serial.printf("\n");
  Serial.printf("CONFIG SRC: %s\n", cfg_spiffs() ? "SPIFFS" : "EEPROM");
  Serial.printf("Boot Count: %d\n", boot_count);
#ifdef ARDUINO_ARCH_ESP8266
  Serial.printf("Flash: %d\n", ESP.getFlashChipRealSize());
  Serial.printf("Reset Reason: %d -> %s\n", resetInfo->reason, ESP.getResetReason().c_str());
#elif ARDUINO_ARCH_ESP32
  Serial.printf("Reset Reason: %d\n", resetCode);
  Serial.printf("CPU Speed: %d MHz  XTAL: %d MHz  APB: %d Hz\n", getCpuFrequencyMhz(), getXtalFrequencyMhz(), getApbFrequency());
#endif
  Serial.printf("Version: %s\n", VERSION);
  Serial.printf("Build Date: %s %s\n", __DATE__, __TIME__);
  Serial.printf("Device ID: %s\n", mqtt_cls+sizeof(MQTT_CLASS));

  startupWifiConfigure(def_cfg, msg_buf, sizeof(msg_buf), mqtt_cls);

#ifdef ARDUINO_ARCH_ESP8266
  updatePWMValue(set_state, set_ct);
#elif ARDUINO_ARCH_ESP32
  updatePWMValue(set_state, set_ct);
#endif
  
//  ignoreOutput = true;
  Serial.printf("\n");
  Serial.printf("SSID = %s  PASSWORD = %s\n", ssid, password);
  Serial.printf("PWM Frequence = %d   Range = %d - %d   Period = %d\n", PWM_FREQUENCE, PWM_START, PWM_END, PWM_PERIOD);
  if (WORK_MODE == 2 || WORK_MODE == 3 || WORK_MODE == 4)
  {
      Serial.printf("SCR PIN: %d  TRIGGER PIN: %d  MODE: %d  CFG RESET PIN: %d\n", PWM_PIN, PWM_SCR_TRIGGER, WORK_MODE, CFG_RESET_PIN);
  } else {
      Serial.printf("PWM PIN: %d  WARM PIN: %d  MODE: %d  CFG RESET PIN: %d\n", PWM_PIN, PWM_WPIN, WORK_MODE, CFG_RESET_PIN);
  }

  enableInterrupts();
  
  Serial.print("Connecting to WiFi");
  uint32_t last_print_dot = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (last_state_hold != 0 && millis() - last_state_hold < STARTUP_SMOOTH_INTERVAL)
    {
        STARTUP_SMOOTH_EXECUTE;
    } else 
    {
        last_set_state = set_state;
        last_set_ct = set_ct;
        updatePWMValue(set_state, set_ct);
    }
    if (millis() - last_print_dot >= 200)
    {
        Serial.print(".");
        last_print_dot = millis();
    }
    if (millis() >= 60000)
    {
        ESP.restart();
    }
    if (!bootCountReset && millis() - boot_time >= 5000)
    {
        bootCountReset = true;
        disableInterrupts();
        boot_count_reset();
        enableInterrupts();
    }
    yield();
  }
  while (last_state_hold != 0 && millis() - last_state_hold < STARTUP_SMOOTH_INTERVAL)
  {
      STARTUP_SMOOTH_EXECUTE;
      yield();
  }

  ignoreOutput = false;
  last_state_hold = 0;
  
  Serial.printf("\n");

  IPAddress myAddress = WiFi.localIP();
  Serial.printf("Connected to wifi. My address: ");
  Serial.print(myAddress);
  Serial.printf("\nConnecting to %s:%d ", mqtt_server, port);
////  WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 1000);
  client.setServer((const char *)mqtt_server, port);//端口号
  client.setCallback(callback); //用于接收服务器接收的数据
//  Serial.printf("Success\n");
  if (LED_BUILTIN != PWM_PIN && LED_BUILTIN != PWM_WPIN)
  {
      digitalWrite(LED_BUILTIN, HIGH);    
  }
//  IN_STARTUP = false;
//  disableInterrupts();
//  enableInterrupts();
}


void callback(char* topic, byte* payload, unsigned int length) {//用于接收数据
  int l=0;
  int p=1;
  hasPacket = true;
  disableInterrupts();
  if (strcmp(topic, "set_bright") == 0 || strcmp(topic, "set_ct_abx") == 0)
  {
    if (!inSmooth)
    {
        last_set_state = set_state;
        last_set_ct = set_ct;
        inSmooth = true;
    } else {
        last_set_state = last_set_state - (float)(last_set_state - set_state) * (millis() - last_state_hold) / SMOOTH_INTERVAL;
        last_set_ct = last_set_ct - (float)(last_set_ct - set_ct) * (millis() - last_state_hold) / SMOOTH_INTERVAL;
    }
    SMOOTH_INTERVAL = 200;
    payload[length] = 0;
    save_config = !AUTO_SAVE_DISABLED && true;
    if (strchr((char *)payload, ',') != NULL)
    {
        save_config = false;
        SMOOTH_INTERVAL = atoi(strchr((char *)payload, ',') + 1);
        *strchr((char *)payload, ',') = 0;
        Serial.printf("Set SMOOTH_INTERVAL to %d\n", SMOOTH_INTERVAL);
    }
    if (strcmp(topic, "set_bright") == 0)
    {
        set_state = atoi((char *)payload);
        if (set_state < 0) set_state = 0;
        if (set_state > 100) set_state = 100;
    } else 
    {
        set_ct = atoi((char *)payload);
        if (CH2_MODE == 0)
        {
            if (set_ct < 2600)
            {
                set_ct = 2600;
            }
            if (set_ct > 5600)
            {
                set_ct = 5600;
            }
        } else 
        {
            if (set_ct < 0) set_ct = 0;
            if (set_ct > 100) set_ct = 100;
        }
    }
    
    if (last_set_state == set_state && last_set_ct == set_ct)
    {
        inSmooth = false;
        Serial.print("Ignore: ");
        updatePWMValue(set_state, set_ct);
    } else if (!inSmooth)
    {
        Serial.print("DIRECT: ");
        updatePWMValue(set_state, set_ct);
    }
    last_state_hold = millis();
    last_state = -1;
  } else if (strcmp(topic, "ota") == 0)
  {
    WiFiClient ota_client;

    char bufferByte = payload[length];
    payload[length] = 0;
    Serial.printf("Start OTA from URL: %s\n", (char *)payload);
    t_httpUpdate_return ret = ESPhttpUpdate.update(ota_client, (char *)payload);

    payload[length] = bufferByte;

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        sprintf(msg_buf, "{\"ota\":\"%s\"}", ESPhttpUpdate.getLastErrorString().c_str());
        client.publish("status", msg_buf);
        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        sprintf(msg_buf, "{\"ota\":\"no updates\"}");
        client.publish("status", msg_buf);
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        sprintf(msg_buf, "{\"ota\":\"success\"}");
        client.publish("status", msg_buf);
        Serial.println("HTTP_UPDATE_OK");
        break;
    }
  } else if (strcmp(topic, "setName") == 0)
  {
      if (length < 32)
      {
          strncpy(dev_name, (const char *)payload, length);
          dev_name[length] = 0;
          Serial.printf("Set Device Name to %s\n", dev_name);
      }
//      Serial.printf("Write %d bytes\n", CFG_WRITE_STRING(32, (char *)payload, length));
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_pwm_freq") == 0)
  {
      char bufferByte = payload[length];
      payload[length] = 0;
      PWM_FREQUENCE = strtoul((char *)payload, NULL, 10);
      
      payload[length] = bufferByte;
      CFG_SAVE();
      analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_PIN);
      if (PWM_WPIN != 0)
      {
          analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_WPIN);
      }
      if (PWM_SOURCE != 0)
      {
          analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_SOURCE);
      }
      updatePWMValue(set_state, set_ct);
      sendMeta();
  } else if (strcmp(topic, "set_pwm_start") == 0)
  {
      char bufferByte = payload[length];
      payload[length] = 0;
      PWM_START = atoi((char *)payload);
      payload[length] = bufferByte;
      CFG_SAVE();
      updatePWMValue(set_state, set_ct);
      sendMeta();
  } else if (strcmp(topic, "set_pwm_end") == 0)
  {
      char bufferByte = payload[length];
      payload[length] = 0;
      PWM_END = atoi((char *)payload);
      payload[length] = bufferByte;
      CFG_SAVE();
      updatePWMValue(set_state, set_ct);
      sendMeta();
  } else if (strcmp(topic, "set_pwm_period") == 0)
  {
      char bufferByte = payload[length];
      payload[length] = 0;
      if (PWM_END == PWM_PERIOD)
      {
          PWM_END = atoi((char *)payload);
      }
      PWM_PERIOD = atoi((char *)payload);
      payload[length] = bufferByte;
      CFG_SAVE();
      
      analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_PIN);
      if (PWM_WPIN != 0)
      {
          analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_WPIN);
      }
      if (PWM_SOURCE != 0)
      {
          analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_SOURCE);
      }
      updatePWMValue(set_state, set_ct);
      sendMeta();
  } else if (strcmp(topic, "set_pwm_pin") == 0)
  {
      payload[length] = 0;
      pwmWrite(PWM_PIN, 0);
      PWM_PIN = atoi((char *)payload);
      analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_PIN);
      updatePWMValue(set_state, set_ct);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_pwm_wpin") == 0)
  {
      payload[length] = 0;
      if (PWM_WPIN != 0)
      {        
          pwmWrite(PWM_WPIN, 0);
      }
      PWM_WPIN = atoi((char *)payload);
      analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_WPIN);
      updatePWMValue(set_state, set_ct);
      CFG_SAVE();
      sprintf(msg_buf, "{\"bright2\":\"unset\",\"ct\":\"unset\"}");
      client.publish("status", msg_buf);
      last_state = -1;
      sendMeta();
  } else if (strcmp(topic, "set_pwm_src") == 0)
  {
      payload[length] = 0;
      if (PWM_SOURCE != 0)
      {        
          pwmWrite(PWM_SOURCE, 0);
      }
      PWM_SOURCE = atoi((char *)payload);
      analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_SOURCE);
      updatePWMValue(set_state, set_ct);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_pwm_duty") == 0)
  {
      payload[length] = 0;
      PWM_SOURCE_DUTY = atoi((char *)payload);
      analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_SOURCE);
      updatePWMValue(set_state, set_ct);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_pwm_mode") == 0)
  {
      payload[length] = 0;
      WORK_MODE = atoi((char *)payload);
      updatePWMValue(set_state, set_ct);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_pwm_trigger") == 0)
  {
      payload[length] = 0;
      PWM_SCR_TRIGGER = atoi((char *)payload);
      updatePWMValue(set_state, set_ct);
      CFG_SAVE();
      sendMeta();
      rebootSystem();
  } else if (strcmp(topic, "set_pwm_delay") == 0)
  {
      payload[length] = 0;
      PWM_SCR_DELAY = atoi((char *)payload);
      updatePWMValue(set_state, set_ct);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_config_pin") == 0)
  {
      payload[length] = 0;
      CFG_RESET_PIN = atoi((char *)payload);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_en1") == 0)
  {
      payload[length] = 0;
      EN_PORT_CH1 = atoi((char *)payload);
      updatePWMValue(set_state, set_ct);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_en2") == 0)
  {
      payload[length] = 0;
      EN_PORT_CH2 = atoi((char *)payload);
      updatePWMValue(set_state, set_ct);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_adc_pin") == 0)
  {
      payload[length] = 0;
      BRIGHT_ADC_PIN = atoi((char *)payload);
      updatePWMValue(set_state, set_ct);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_auto_full_power") == 0)
  {
      payload[length] = 0;
      PWM_AUTO_FULL_POWER = atoi((char *)payload);
      updatePWMValue(set_state, set_ct);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_autosave") == 0)
  {
      payload[length] = 0;
      AUTO_SAVE_DISABLED = atoi((char *)payload);
      updatePWMValue(set_state, set_ct);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_ch2mode") == 0)
  {
      payload[length] = 0;
      CH2_MODE = atoi((char *)payload);
      if (CH2_MODE == 1 && set_ct > 100)
      {
          set_ct = 100;
      } else if (CH2_MODE == 0)
      {
          if (set_ct < 2600)
          {
              set_ct = 2600;
          }
          if (set_ct > 5600)
          {
              set_ct = 5600;
          }
      }
      updatePWMValue(set_state, set_ct);
      CFG_SAVE();
      sprintf(msg_buf, "{\"bright2\":\"unset\",\"ct\":\"unset\"}");
      client.publish("status", msg_buf);
      last_state = -1;
      sendMeta();
  } else if (strcmp(topic, "set_startup_smooth") == 0)
  {
      payload[length] = 0;
      STARTUP_SMOOTH_INTERVAL = atoi((char *)payload);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "reset") == 0)
  {
      if (length > 0 && payload[0] == '1')
      {
          cfg_reset();
      } else
      {
          CFG_INIT(false);
      }
      Serial.printf("Reset System\n");
  } else if (strcmp(topic, "reboot") == 0)
  {
      rebootSystem();
  }
  enableInterrupts();
}

void sendMeta()
{
    last_send_meta = millis();
  // max length = 64 - 21
    char *p = msg_buf + sprintf(msg_buf, "{\"name\":\"%s\"", dev_name);
    p = p + sprintf(p, ",\"pwm_freq\":%d,\"pwm_start\":%d,\"pwm_end\":%d,\"period\":%d,\"en1\":%d,\"en2\":%d,\"adc\":%d,\"ch2mode\":%d}", PWM_FREQUENCE, PWM_START, PWM_END, PWM_PERIOD, EN_PORT_CH1, EN_PORT_CH2, BRIGHT_ADC_PIN, CH2_MODE);
    client.publish("dev", msg_buf);
    if (isScrMode)
    {
        p = msg_buf + sprintf(msg_buf, "{\"scr_delay\":%d,\"trigger_pin\":%d}", PWM_SCR_DELAY, PWM_SCR_TRIGGER);
        client.publish("dev", msg_buf);        
    }
    
    p = msg_buf + sprintf(msg_buf, "{\"pin\":%d,\"wpin\":%d,\"pwm_src\":%d, \"duty\":%d,\"mode\":%d,\"cfg_pin\":%d,\"auto_full_power\":%d,\"no_autosave\":%d,\"startup_smooth\":%d}", PWM_PIN, PWM_WPIN, PWM_SOURCE, PWM_SOURCE_DUTY, WORK_MODE, CFG_RESET_PIN, PWM_AUTO_FULL_POWER, AUTO_SAVE_DISABLED, STARTUP_SMOOTH_INTERVAL);
    client.publish("dev", msg_buf);
}

void reconnect() {//等待，直到连接上服务器
  uint32_t disconnectTime = 0;
  if (WiFi.status() != WL_CONNECTED) {
      disconnectTime = millis();
  }
  while (WiFi.status() != WL_CONNECTED) {
      if (millis() - disconnectTime >= 30000)
      {
          break;
      }
      delay(1);
  }
  if (WiFi.status() != WL_CONNECTED) {
      Serial.printf("WiFi: DISCONNECTED, RESET SYSTEM\n");
      rebootSystem();
  }
  if (!client.connected()){
      Serial.print("Connecting to MQTT server...");
#ifdef ARDUINO_ARCH_ESP32
      esp_task_wdt_init(30, true);
      esp_task_wdt_add(NULL);
#endif
  }
  while (!client.connected()) {//如果没有连接上
    Serial.printf(".");
    if (client.connect(mqtt_cls)) {//接入时的用户名，尽量取一个很不常用的用户名
      retry_failed_count = 0;
      Serial.printf(" success, login by: %s\n",mqtt_cls);
#ifdef ARDUINO_ARCH_ESP32
      esp_task_wdt_delete(NULL);
      esp_task_wdt_deinit();
#endif
      sendMeta();
      last_rssi = -1;
      last_state = -1;
    } else {
#ifdef ARDUINO_ARCH_ESP32
      esp_task_wdt_reset();
#endif
      retry_failed_count++;
      Serial.print(" failed, rc=");//连接失败
      Serial.print(client.state());//重新连接
      Serial.printf(" try again in 1 seconds\n");//延时5秒后重新连接
      delay(1000);
      if (retry_failed_count >= 10)
      {
          Serial.printf("MQTT: Reconnect Too many times, RESET SYSTEM\n");
          rebootSystem();
      } else 
      {
        Serial.print("Connecting to MQTT server");
      }
    }
  }
}

int buflen = 0;
void loop() {
   hasPacket = false;
   if (ZC)
   {
        ZC = 0;
        if (abs(zc_interval - last_zc_interval) >= 1000)
        {
#if DEBUG_LEVEL >= 3
            Serial.printf("Zero Detect Interval: %d\n", zc_interval);
#endif
            last_zc_interval = zc_interval;
        }
   }
   while (Serial.available())
   {
      char ch = Serial.read();
      if (ch == 'S')
      {
          Serial.printf("Set pin %d to OUTPUT\n", PWM_PIN);
          pinMode(PWM_PIN, OUTPUT);
          digitalWrite(PWM_PIN, HIGH);
//          updateSCRState(false);
      } else if (ch == 'O') {
          Serial.printf("Set pin %d to OUTPUT OPEN\n", PWM_PIN);
          pinMode(PWM_PIN, OUTPUT);
          digitalWrite(PWM_PIN, LOW);
//          updateSCRState(true);
      } else if (ch == 'R' && Serial.read() == 'r') {
          cfg_reset();
          ESP.restart();
      }
   }
    if (!bootCountReset && millis() - boot_time >= 5000)
    {
        bootCountReset = true;
        disableInterrupts();
        boot_count_reset();
        enableInterrupts();
    }
   reconnect();//确保连上服务器，否则一直等待。
   client.loop();//MUC接收数据的主循环函数。
   long rssi = WiFi.RSSI();
   if (SMOOTH_INTERVAL == 0)
   {
      SMOOTH_INTERVAL = 200;
   }
   if (inSmooth && last_state_hold != 0 && millis() - last_state_hold <= SMOOTH_INTERVAL && set_state != -1)
   {
      if (!isScrMode)
        Serial.print("Smooth: ");
      updatePWMValue((last_set_state - (float)(last_set_state - set_state) * (millis() - last_state_hold) / SMOOTH_INTERVAL), (last_set_ct - (float)(last_set_ct - set_ct) * (millis() - last_state_hold) / SMOOTH_INTERVAL));
   }
   if (inSmooth && millis() - last_state_hold > SMOOTH_INTERVAL && set_state != -1)
   {
      if (!isScrMode)
        Serial.print("Final: ");
      updatePWMValue(set_state, set_ct);
      inSmooth = false;
   }
   if (last_state_hold != 0 && set_state > 0 && !inSmooth && !inTrigger && millis() - last_state_hold >= 2000)
   {
      if (save_config) {// && WORK_MODE != 2 && WORK_MODE != 3){
        Serial.printf("Saving config to FLASH...");
        uint32_t t1 = micros();
        uint32_t usages = 0;
        disableInterrupts();
        CFG_SAFE_SAVE();
        enableInterrupts();
        yield();
        usages += micros() - t1;
        delay(50);
        t1 = micros();
        disableInterrupts();
        cfg_confirm();
        enableInterrupts();
        Serial.printf("Done in %ld us  save: %ld us\n", micros() - t1 + usages, usages);
      }
      last_state_hold = 0;
   }
   if (last_state != set_state || (abs(rssi - last_rssi) >= 3 && millis() - last_send_rssi >= 5000))
   {
      last_send_rssi = millis();
      last_state = set_state;
      last_rssi = rssi;
      if (PWM_WPIN != 0)
      {
          if (CH2_MODE == 0)
          {
              sprintf(msg_buf, "{\"bright\":%d,\"ct\":%d,\"rssi\":%ld,\"version\":\"%s\",\"boot\":%ld}", set_state, set_ct, rssi, VERSION, millis());
          } else 
          {
              sprintf(msg_buf, "{\"bright\":%d,\"bright2\":%d,\"rssi\":%ld,\"version\":\"%s\",\"boot\":%ld}", set_state, set_ct, rssi, VERSION, millis());
          }
      } else {
          sprintf(msg_buf, "{\"bright\":%d,\"rssi\":%ld,\"version\":\"%s\",\"boot\":%ld}", set_state, rssi, VERSION, millis());
      }
      client.publish("status", msg_buf);
   }
   if (BRIGHT_ADC_PIN != 0)
   {
      int16_t new_adc_value = analogRead(BRIGHT_ADC_PIN);
      if (abs(new_adc_value - lastAdcValue) > 4096/20)
      {
        delay(50);
        int16_t new_adc_value2 = analogRead(BRIGHT_ADC_PIN);
        if (abs(new_adc_value - new_adc_value2) < 100 && abs(new_adc_value - lastAdcValue) > 4096/20)
        {
          Serial.printf("Change value: %d\n", abs(new_adc_value - lastAdcValue));
            sprintf(msg_buf, "%d", (uint32_t)new_adc_value * 100 / 4096);
            lastAdcValue = new_adc_value;
            callback("set_bright", (byte *)msg_buf, strlen(msg_buf));
        }
      }
   }

   if (millis() - last_send_meta >= 60000)
   {
      if (!isScrMode)
          Serial.printf("PING %ld  WiFI: %d\n", millis(), WiFi.status());
      sendMeta();
   }

//   if (!hasPacket)
//   {
//      delay(20);
//   }
}
