#ifdef ARDUINO_ARCH_ESP32
  #include <WiFi.h>
  #include <HTTPUpdate.h>
  #include <math.h>
  #include <driver/dac.h>
  #include <driver/ledc.h>
  #include <rom/rtc.h>
  #include <esp_task_wdt.h>
  #include <esp32-hal-timer.c>
  #include <esp32-hal-adc.h>
  #define ESPhttpUpdate httpUpdate
#elif ARDUINO_ARCH_ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266httpUpdate.h>
  extern "C"{
      #include "pwm.h"
  }
#endif
#include <PubSubClient.h>
#include <AutoPID.h>
#include "config.h"

#if defined(ARDUINO_ESP8266_GENERIC)
  #warning "BOARD: ESP8266, please confirm using 4M with 1M SPIFFS"
#elif defined(ARDUINO_ESP8266_ESP01)
  #warning "BOARD: ESP8285, please confirm using 1M with no SPIFFS"
#endif

// CONFIG: REVERSE THE OUTPUT FOR MOSFET+BJT Driver
// config.work_mode = 2: USING SCR to OUTPUT
/*
 * MODE 0  => CH1 NON-INVENTING   CH2 INVERTING
 * MODE 1  => CH1 INVERTING       CH2 NON-INVERTING
 * MODE 2  => CH1 TRIGGER NON-INVERTING
 * MODE 3  => CH1 INVERTING       CH2 INVERTING
 */
// CONFIG: SMOOTH PWM INTERVAL IN mS
unsigned int SMOOTH_INTERVAL = 200;

typedef struct {
    uint32_t  scr1_open;
    uint32_t  scr1_close;
    uint32_t  scr2_open;
    uint32_t  scr2_close;
} scr_timing_t;

scr_timing_t executeTiming, pendingTiming;

#define DEBUG_LEVEL 2
#define DEBUG_TRIGGER_PIN 33
//#define DEBUG_SAVE_PIN 33

#ifndef MQTT_CLASS
#define MQTT_CLASS "HP-LEDPWM"
#define VERSION "2.53"

#ifdef ARDUINO_ARCH_ESP8266
  #define PWM_CHANNELS 2
  // PWM setup (choice all pins that you use PWM)
  uint32 io_info[PWM_CHANNELS][3];
  
  // PWM initial duty: all off
  uint32 pwm_duty_init[PWM_CHANNELS];
#endif

#endif

bool save_config = true;
bool ignoreOutput = false;
bool realtime_data = false;

// Current Bright in PWM Range (正比)
int scr_current_bright = 0;
int scr_current_bright2 = 0;

uint8_t  inTrigger = 0;

uint32_t zc_interval = 0;
uint32_t zc_last = 0;
uint32_t last_zc_interval = 0;

char mqtt_cls[sizeof(MQTT_CLASS) + 13];
char msg_buf[160];

#ifdef ARDUINO_ARCH_ESP32
hw_timer_t *timer = NULL;
#define timer1_write(value) if (timer != NULL) { timer->dev->load_high = 0; timer->dev->load_low = (5000000 - value); timer->dev->reload = 1; timer->dev->config.alarm_en = 1; }
// ICACHE_RAM FOR timerWrite(5000000-value); timerAlarmEnable(timer);
#endif

// NOTICE: NAME not longer then 15 bytes
typedef __attribute__((__packed__)) struct {
    uint8_t  reversion;   // Config
    uint8_t  bright;
    uint32_t pwm_freq;    // 60 0
    uint16_t pwm_start;   // 4
    uint16_t pwm_end;     // 6
    uint16_t ct_abx;      // 8
    uint16_t pwm_period;  // 10
    uint8_t  pin_ch1;     // 12
    uint8_t  pin_ch2;
    uint8_t  work_mode;
    uint8_t  pin_scr_trig;
    int16_t  scr_delay;   // 78/16
    uint8_t  pin_reset;
    uint8_t  pin_en1;
    uint8_t  pin_en2;
    uint8_t  auto_fp;
    uint16_t startup_smooth; // 84
    uint8_t  pwm_src;      // 86
    uint8_t  pwm_src_duty;
    uint8_t  pin_adc_bright;
    uint8_t  autosave;
    uint8_t  mode_ch2;  // 90
    int8_t   scr_prerelease; // 91
    int8_t   pin_sensor;
    uint16_t max_lm;
    int32_t  KP;
    int32_t  KI;
    int32_t  KD;
    uint32_t sensor_ref_r1;
    uint16_t sensor_ref;    // 2.495 multiple 1000
    uint16_t adc_volt_0db;
    uint16_t adc_volt_6db;
    uint16_t adc_volt_11db;
    float    sensor_ax; // y = ax*pow(x,b)
    float    sensor_b;
} conf_t;
//  __attribute__ ((aligned (2)))

#define KP_DIV 1000.
#define KI_DIV 1000.
#define KD_DIV 1000.


conf_t config;
#define AUTO_CONF(conf_name, default_value, init) { 32 + offsetof(conf_t, conf_name), sizeof(config.conf_name), (__typeof(config.conf_name))default_value,  &config.conf_name, init, #conf_name }
#define AUTO_CONF_INT_COMMAND(topic, cmd, field, action)  else if (strcmp(topic, cmd) == 0) { \
  payload[length] = 0; \
  config.field = atoi((char *)payload); \
  cfg_save(def_cfg); \
  sendMeta(); \
  action; \
}
#define AUTO_CONF_FLOAT_COMMAND(topic, cmd, field, action)  else if (strcmp(topic, cmd) == 0) { \
  payload[length] = 0; \
  config.field = atof((char *)payload); \
  cfg_save(def_cfg); \
  sendMeta(); \
  action; \
}

  
const hp_cfg_t old_def_cfg[] = {
  {31, sizeof(uint8_t), (uint8_t)10,      &config.bright, true, "bright"},        // BRIGHT
  {32, 0, (uint8_t)0, &dev_name, true, "name"},                    // NAME LENGTH
#ifdef ARDUINO_ARCH_ESP32
  {60, sizeof(uint32_t), (uint32_t)20000, &config.pwm_freq, false, "pwm_freq"},   // UINT32: PWM FREQUENCE
#else
  {64, sizeof(uint16_t), (uint16_t)20000, &config.pwm_freq, false},   // UINT16: PWM FREQUENCE
#endif
  {66, sizeof(uint16_t), (uint16_t)0,     &config.pwm_start, false, "pwm_start"},       // UINT16: PWM START
  {68, sizeof(uint16_t), (uint16_t)200,   &config.pwm_end, false, "pwm_end"},         // UINT16: PWM END
  {70, sizeof(uint16_t), (uint16_t)4100,  &config.ct_abx, false, "ct_abx"},          // UINT16: COLOR TEMPERATURE / BRIGHT 2
  {72, sizeof(uint16_t), (uint16_t)200,   &config.pwm_period, false, "pwm_period"},      // UINT16: PWM PERIOD
  {74, sizeof(uint8_t), (uint8_t)0,   &config.pin_ch1, false, "pin_ch1"},             // UINT8:  config.pin_ch1    Single-CH / Warm LED PIN
  {75, sizeof(uint8_t), (uint8_t)0,   &config.pin_ch2, false, "pin_ch2"},            // UINT8:  config.pin_ch2   White LED PIN
  {76, sizeof(uint8_t), (uint8_t)1,   &config.work_mode, false, "mode"},           // UINT8:  MODE   0 NON-INVERT  1 INVERT  2 SCR NON-INVERT  3 SCR INVERT
  {77, sizeof(uint8_t), (uint8_t)0,   &config.pin_scr_trig, false, "pin_scr_trig"},     // UINT8:  SCR MODE ZeroDetect Pin
  {78, sizeof(int16_t), (int16_t)0,   &config.scr_delay, false, "scr_delay"},       // UINT16: SCR MODE ZeroDetect DELAY
  {80, sizeof(uint8_t), (uint8_t)0,   &config.pin_reset, false, "cfg_reset"},       // UINT8:  RESET PIN
  {81, sizeof(uint8_t), (uint8_t)0,   &config.pin_en1, false, "pin_en1"},             // UINT8:  EN PIN
  {82, sizeof(uint8_t), (uint8_t)1,   &config.auto_fp, false, "auto_fp"}, // UINT8:  AUTO FULL POWER
  {83, sizeof(uint8_t), (uint8_t)0,   &config.pin_en2, false, "pin_en2"},             // UINT8:  EN PIN
  {84, sizeof(uint16_t), (uint16_t)500, &config.startup_smooth, true, "startup_smooth"},    // UINT16: STARTUP SMOOTH INTERVAL
  {86, sizeof(uint8_t), (uint8_t)0,   &config.pwm_src, false, "pwm_src"},             // UINT8:  config.pwm_src
  {87, sizeof(uint8_t), (uint8_t)0,   &config.pwm_src_duty, false, "pwm_src_duty"},             // UINT8:  config.pwm_src
  {88, sizeof(uint8_t), (uint8_t)0,   &config.pin_adc_bright, false, "pin_adc_bright"},             // UINT8:  config.pin_adc_bright
  {89, sizeof(uint8_t), (uint8_t)0,   &config.autosave, false, "b_autosave"},             // UINT8:  config.autosave
  {90, sizeof(uint8_t), (uint8_t)0,   &config.mode_ch2, false, "ch2_mode"},             // UINT8:  config.mode_ch2
  {91, sizeof(int8_t), (int8_t)5,     &config.scr_prerelease, false, "scr_prerelease"},             // UINT8:  SCR_PRERELEASE
  {96, 0, (uint8_t)0, &ssid, true, "ssid"},                    // STRING: SSID
  {128, 0, (uint8_t)0, &password, true, "password"},                   // STRING: WIFI PASSWORD 
  {160, 0, (uint8_t)0, &mqtt_server, true, "mqtt_srv"},        // STRING: MQTT SERVER
  {192, sizeof(uint16_t), (uint16_t)1234, &port, true, "mqtt_port"},             // UINT16: PORT
  {NULL, 0,0, NULL, false, NULL}
};

const hp_cfg_t def_cfg[] = {
    AUTO_CONF(reversion, 1, false),
    AUTO_CONF(bright, 10, true),
    AUTO_CONF(pwm_freq, 20000, false),
    AUTO_CONF(pwm_start, 0, false),
    AUTO_CONF(pwm_end, 200, false),
    AUTO_CONF(ct_abx, 4100, false),
    AUTO_CONF(pwm_period, 200, false),
    AUTO_CONF(pin_ch1, 0, false),
    AUTO_CONF(pin_ch2, 0, false),
    AUTO_CONF(work_mode, 1, false),
    AUTO_CONF(pin_scr_trig, 0, false),
    AUTO_CONF(scr_delay, 0, false),
    AUTO_CONF(pin_reset, 0, false),
    AUTO_CONF(pin_en1, 0, false),
    AUTO_CONF(pin_en2, 0, false),
    AUTO_CONF(auto_fp, 1, false),
    AUTO_CONF(startup_smooth, 500, false),
    AUTO_CONF(pwm_src, 0, false),
    AUTO_CONF(pwm_src_duty, 0, false),
    AUTO_CONF(pin_adc_bright, 0, false),
    AUTO_CONF(autosave, 0, false),
    AUTO_CONF(mode_ch2, 0, false),
    AUTO_CONF(scr_prerelease, -50, false),
    AUTO_CONF(pin_sensor, 0, false),
    AUTO_CONF(max_lm, 1000, false),
    AUTO_CONF(KP, 100, false),
    AUTO_CONF(KI, 1500, false),
    AUTO_CONF(KD, 800, false),
    AUTO_CONF(sensor_ref_r1, 10000, false),
    AUTO_CONF(sensor_ref, 2495, false),
    AUTO_CONF(adc_volt_0db, 1100, false),
    AUTO_CONF(adc_volt_6db, 2000, false),
    AUTO_CONF(adc_volt_11db, 3600, false),
    AUTO_CONF(sensor_ax, 6e+08f, false),
    AUTO_CONF(sensor_b, -1.898f, false),
    {140, 0, (uint8_t)0, &dev_name, true, "name"},                    // STRING: name
    {178, 0, (uint8_t)0, &ssid, true, "ssid"},                    // STRING: SSID
    {210, 0, (uint8_t)0, &password, true, "password"},                   // STRING: WIFI PASSWORD 
    {230, 0, (uint8_t)0, &mqtt_server, true, "mqtt_srv"},        // STRING: MQTT SERVER
    {250, sizeof(uint16_t), (uint16_t)1234, &port, true, "mqtt_port"},             // UINT16: PORT
    {NULL, 0,0, NULL, false, NULL}
};

double sensor_bright;
double target_bright;
double pidBright;
AutoPID pidControl(&sensor_bright, &target_bright, &pidBright, 0, 100, config.KP / 1000., config.KI / 1000000., config.KD / 1000000.);


#define STARTUP_SMOOTH_EXECUTE updatePWMValue((last_set_state - (float)(last_set_state - config.bright) * (millis() - last_state_hold) / config.startup_smooth), config.mode_ch2 == 0 ? config.ct_abx : (last_ct_abx - (float)(last_ct_abx - config.ct_abx) * (millis() - last_state_hold) / config.startup_smooth));
#define isScrMode (config.work_mode == 2 || config.work_mode == 3 || config.work_mode == 4)

WiFiClient espClient;
PubSubClient client(espClient);

bool inSmooth = false;
char last_state = -1;
int last_set_state = 0;
uint16_t last_ct_abx = 4000;
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
    if (config.work_mode == 0)
    {
        if (pin == config.pin_ch1) return false;
        if (pin == config.pin_ch2) return true;
    } else if (config.work_mode == 1)
    {
        if (pin == config.pin_ch1) return true;
        if (pin == config.pin_ch2) return false;
    } else if (config.work_mode == 3)
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
    } else if (pwm_pin == config.pin_ch1)
    {
        if (config.pwm_end == config.pwm_period)
        {
            auto_update_pwm_end = true;
        }
        config.pwm_period = pow(2, bits);
        if (auto_update_pwm_end) config.pwm_end = config.pwm_period;
        ledcSetup(0, freq, bits);
        ledcWrite(0, isChannelInvert(pwm_pin) ? config.pwm_period : 0);
        ledcAttachPin(pwm_pin, 0);
    } else if (pwm_pin == config.pin_ch2) {
        ledcSetup(1, freq, bits);
        ledcWrite(1, isChannelInvert(pwm_pin) ? config.pwm_period : 0);
        ledcAttachPin(pwm_pin, 1);
    } else if (pwm_pin == config.pwm_src) {
        ledcSetup(2, freq, bits);
        ledcWrite(2, pow(2, bits) / 2);
        ledcAttachPin(pwm_pin, 2);
    }
#else
    if (config.pwm_end == config.pwm_period)
    {
        auto_update_pwm_end = true;
    }
    if (5000000 % freq == 0)
    {
        config.pwm_period = 5000000 / config.pwm_freq;
    } else 
    {
        config.pwm_period = 320;
        freq = 5000000 / config.pwm_period;
    }
    if (auto_update_pwm_end) config.pwm_end = config.pwm_period;

    pinMode(pwm_pin, OUTPUT);

    if (config.work_mode == 0 || config.work_mode == 1)
    {
        if (pwm_pin == config.pin_ch1)
        {
            io_info[0][0] = PinToGPIOMux(pwm_pin);
            io_info[0][1] = PinToGPIOMuxFunc(pwm_pin);
            io_info[0][2] = pwm_pin;
            pwm_duty_init[0] = isChannelInvert(pwm_pin) ? config.pwm_period : 0;
        } else if (pwm_pin == config.pin_ch2) {
            io_info[1][0] = PinToGPIOMux(pwm_pin);
            io_info[1][1] = PinToGPIOMuxFunc(pwm_pin);
            io_info[1][2] = pwm_pin;
            pwm_duty_init[1] = isChannelInvert(pwm_pin) ? config.pwm_period : 0;
        }
        digitalWrite(pwm_pin, isChannelInvert(pwm_pin) ? config.pwm_period : 0);
        // Initialize
        pwm_init(config.pwm_period, pwm_duty_init, config.pin_ch2 == 0 ? 1 : 2, io_info);
     
        digitalWrite(pwm_pin, isChannelInvert(pwm_pin) ? config.pwm_period : 0);
        if (pwm_pin == config.pin_ch1)
        {
            pwm_set_duty(isChannelInvert(pwm_pin) ? config.pwm_period : 0, 0);
        } else if (pwm_pin == config.pin_ch2) {
            pwm_set_duty(isChannelInvert(pwm_pin) ? config.pwm_period : 0, 1);
        }
        // Commit
        pwm_start(); 
        digitalWrite(pwm_pin, isChannelInvert(pwm_pin) ? config.pwm_period : 0);
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
        dacWrite(25, duty * 255 / config.pwm_period);
    } else if (pwm_pin == 102)
    {
        dacWrite(26, duty * 255 / config.pwm_period);
    } else if (pwm_pin == config.pin_ch1)
    {
        ledcWrite(0, duty);
    } else if (pwm_pin == config.pin_ch2)
    {
        ledcWrite(1, duty);
    } else if (pwm_pin == config.pwm_src)
    {
        ledcWrite(2, duty);
    }
#else
    if (pwm_pin == config.pin_ch1)
    {
        pwm_set_duty(duty, 0);
    } else if (pwm_pin == config.pin_ch2)
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

void updatePWMValue(float set_state, float ct_abx)
{
    int pwm_state = mapfloat(set_state, 0, 100, config.pwm_start, config.pwm_end);
    if (pwm_state == config.pwm_start) pwm_state = 0;
    if (pwm_state == config.pwm_end && config.auto_fp) pwm_state = config.pwm_period;
    if (config.pin_ch2 != 0)
    {
        float warm_power = set_state / 100.;
        float cold_power = 0;

        if (config.mode_ch2 == 0)
        {
            float set_power = 0.5 + (ct_abx - 4100) / 3000.;
            
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
            pwm_state = (config.pwm_end - config.pwm_start);
            cold_power = ct_abx / 100.;
        }
        
    #if DEBUG_LEVEL >= 2
        if (!ignoreOutput && (config.pin_sensor == 0 || config.max_lm == 0))
        {
            if (config.mode_ch2 == 0)
            {
                Serial.printf("Set Power %f PWM = %d  CT: %f  COLD: %d%% %d WARM: %d%% %d\n", set_state, pwm_state, ct_abx, (int)(cold_power * 100), (int)(pwm_state * cold_power), (int)(warm_power * 100), (int)(pwm_state * warm_power));
            } else 
            {
                Serial.printf("Set Power CH1 = %f PWM = %d  CH2: %f   CH1(COLD): %d%% %d   CH2(WARM): %d%% %d\n", set_state, pwm_state, ct_abx, (int)(cold_power * 100), (int)(pwm_state * cold_power), (int)(warm_power * 100), (int)(pwm_state * warm_power));
            }
        }
    #endif
    
        if (config.pin_en1 != 0)
        {
            if (warm_power == 0)
            {
                pinMode(config.pin_en1, OUTPUT);
                digitalWrite(config.pin_en1, HIGH);
            } else 
            {
                pinMode(config.pin_en1, INPUT);
            }
        }
        if (config.pin_en2 != 0)
        {
            if (cold_power == 0)
            {
                pinMode(config.pin_en1, OUTPUT);
                digitalWrite(config.pin_en1, HIGH);
            } else 
            {
                pinMode(config.pin_en1, INPUT);
            }
        }

        if (!isScrMode)
        {
            pwmWrite(config.pin_ch1, isChannelInvert(config.pin_ch1) ? config.pwm_period - pwm_state * warm_power : pwm_state * warm_power);
            pwmWrite(config.pin_ch2, isChannelInvert(config.pin_ch2) ? config.pwm_period - pwm_state * cold_power : pwm_state * cold_power);
        } else 
        {
            //
            scr_current_bright = (config.mode_ch2 != 0 && warm_power != 0 ? config.pwm_start : 0) + pwm_state * warm_power;
            if (scr_current_bright == config.pwm_end && config.auto_fp) scr_current_bright = config.pwm_period;
            scr_current_bright2 = (config.mode_ch2 != 0 && cold_power != 0 ? config.pwm_start : 0) + pwm_state * cold_power;
            if (scr_current_bright2 == config.pwm_end && config.auto_fp) scr_current_bright2 = config.pwm_period;
        }
    } else {
        if (config.pin_en1 != 0)
        {
            if (set_state == 0)
            {
                pinMode(config.pin_en1, OUTPUT);
                digitalWrite(config.pin_en1, HIGH);
            } else 
            {
                pinMode(config.pin_en1, INPUT);
            }
        }
        
        if (!isScrMode)
        {
            pwmWrite(config.pin_ch1, isChannelInvert(config.pin_ch1) ? config.pwm_period - pwm_state : pwm_state);       
        } else 
        {
            scr_current_bright = pwm_state;
        }
        #if DEBUG_LEVEL >= 2
        if (!isScrMode && !ignoreOutput && (config.pin_sensor == 0 || config.max_lm == 0))
            Serial.printf("Set Power Power %f  => %d\n", set_state, pwm_state);
        #endif
    }
    if (config.pwm_src != 0)
    {
        pwmWrite(config.pwm_src, config.pwm_period * config.pwm_src_duty / 100);
    }
}

uint8_t ZC = 0;
uint8_t justZeroCrossing = 0;
uint32_t ZC_id = 0;
uint32_t lastZC_id = 0;
bool timerRunning = false;
//bool scrState = 0;

#define updateSCRState(state, pin) digitalWrite(pin, !state)
//inline void updateSCRState(bool state, int pin)
//{
//    scrState = state;
//    if (state == false)
//    {
//#if DEBUG_LEVEL >= 5
//        Serial.printf("S %ld\n", micros());
//#endif
//        digitalWrite(pin, HIGH);
//    } else 
//    {
//#if DEBUG_LEVEL >= 5
//        Serial.printf("O %ld\n", micros());
//#endif
//        // 打开SCR
//        digitalWrite(pin, LOW);
//    }
//}

bool ICACHE_RAM_ATTR timerMarkNextTick(uint32_t currentMicro)
{
    uint32_t nextTick = 0xffffffff;
    if (currentMicro < executeTiming.scr1_open  && executeTiming.scr1_open  - currentMicro <= nextTick)  nextTick = executeTiming.scr1_open - currentMicro;
    if (currentMicro < executeTiming.scr1_close && executeTiming.scr1_close - currentMicro <= nextTick)  nextTick = executeTiming.scr1_close - currentMicro;
    if (currentMicro < executeTiming.scr2_open  && executeTiming.scr2_open  - currentMicro <= nextTick)  nextTick = executeTiming.scr2_open - currentMicro;
    if (currentMicro < executeTiming.scr2_close && executeTiming.scr2_close - currentMicro <= nextTick)  nextTick = executeTiming.scr2_close - currentMicro;
    // Checking for overflow micros
    if (nextTick <= 5000000)  // nextTick != 0xffffffff && 
    {
        timer1_write(5 * (nextTick));
        return true;
    } else if (memcmp(&executeTiming, &pendingTiming, sizeof(scr_timing_t)) != 0) {
        memcpy(&executeTiming, &pendingTiming, sizeof(scr_timing_t));
        return timerMarkNextTick(currentMicro);
    }
    return false;
}

static uint32_t lastSCROpenTiming = 0;

void ICACHE_RAM_ATTR onTimerISR(){
    uint32_t  currentMicro = micros();
    bool matched = false;
    if (executeTiming.scr1_open != 0 && currentMicro >= executeTiming.scr1_open && scr_current_bright != 0)
    {
        matched = true;
        executeTiming.scr1_open = 0;
        lastSCROpenTiming = currentMicro;
        updateSCRState(true, config.pin_ch1); // 打开SCR
    }
    if (executeTiming.scr1_close != 0 && currentMicro >= executeTiming.scr1_close)
    {
        matched = true;
        executeTiming.scr1_close = 0;
        updateSCRState(false, config.pin_ch1); // 关闭SCR
    }
    if (executeTiming.scr2_open != 0 && currentMicro >= executeTiming.scr2_open && scr_current_bright2 != 0)
    {
        matched = true;
        executeTiming.scr2_open = 0;
        lastSCROpenTiming = currentMicro;
        updateSCRState(true, config.pin_ch2); // 打开SCR
    }
    if (executeTiming.scr2_close != 0 && currentMicro >= executeTiming.scr2_close)
    {
        matched = true;
        executeTiming.scr2_close = 0;
        updateSCRState(false, config.pin_ch2); // 打开SCR
    }
    if (!timerMarkNextTick(currentMicro) && !matched)
    {
        // 5 MHz counting, 1 Tick = 0.2us, sleep 0.1% of duty cycle ~= 10ms / 1000 = 10us
        timerRunning = false;
    }
}

long inline inline_map(long x, long in_min, long in_max, long out_min, long out_max) {
    const long dividend = out_max - out_min;
    const long divisor = in_max - in_min;
    const long delta = x - in_min;

    return (delta * dividend + (divisor / 2)) / divisor + out_min;
}

void ICACHE_RAM_ATTR calcSCRTiming(uint32_t currentMicro)
{
    // DIV 16 = 80/16 = 5 Mhz = 0.2us per tick
    // 1us = 5 tick
    if (scr_current_bright == 0 || (config.scr_delay == 0 && config.scr_prerelease == 0 && scr_current_bright != config.pwm_period))
    {
        updateSCRState(false, config.pin_ch1); // 过零触发，关闭SCR
    }
    if (scr_current_bright == 0 || (config.scr_delay == 0 && config.scr_prerelease == 0 && scr_current_bright2 != config.pwm_period && config.pin_ch2 != 0))
    {
        updateSCRState(false, config.pin_ch2); // 过零触发，关闭SCR
    }
    memset(&pendingTiming, 0, sizeof(scr_timing_t));
    if (scr_current_bright != 0)
    {
        if (scr_current_bright == config.pwm_period)
        {
            updateSCRState(true, config.pin_ch1);
        }else {
            pendingTiming.scr1_open = zc_last + (config.scr_delay != 0 ? config.scr_delay : 0) + inline_map(scr_current_bright, 0, config.pwm_period, zc_interval, 0);
            if (config.scr_prerelease > 0)
            {
                pendingTiming.scr1_close = pendingTiming.scr1_open + config.scr_prerelease * 10;
            } else if (config.scr_prerelease < 0)
            {
                pendingTiming.scr1_close = zc_interval - config.scr_prerelease*10 > (zc_last - zc_last) ?  zc_last + (config.scr_delay != 0 ? config.scr_delay : 0) + zc_interval - (zc_last - zc_last) + config.scr_prerelease*10 : 0;
                if (pendingTiming.scr1_close <= pendingTiming.scr1_open) pendingTiming.scr1_open = 0;
            }
            if (pendingTiming.scr1_close == 0 || pendingTiming.scr1_close >= zc_last + zc_interval + config.scr_delay)
            {
                pendingTiming.scr1_close = zc_last + zc_interval - (zc_last - zc_last) + config.scr_delay;
            }
        }
    }
    if (scr_current_bright2 != 0)
    {
        if (scr_current_bright2 == config.pwm_period)
        {
            updateSCRState(true, config.pin_ch2);
        }else {
            pendingTiming.scr2_open = zc_last + (config.scr_delay != 0 ? config.scr_delay : 0) + (config.pwm_start * zc_interval / config.pwm_period) + inline_map(scr_current_bright2, 0, config.pwm_period, zc_interval, 0);
            if (config.scr_prerelease > 0)
            {
                pendingTiming.scr2_close = pendingTiming.scr1_open + config.scr_prerelease * 10;
            } else if (config.scr_prerelease < 0)
            {
                pendingTiming.scr2_close = zc_interval - config.scr_prerelease*10 > (zc_last - zc_last) ?  zc_last + (config.scr_delay != 0 ? config.scr_delay : 0) + zc_interval - (zc_last - zc_last) + config.scr_prerelease*10 : 0;
                if (pendingTiming.scr2_close <= pendingTiming.scr2_open) pendingTiming.scr2_open = 0;
            } else 
            {
                pendingTiming.scr2_close = zc_last + zc_interval - (zc_last - zc_last) + config.scr_delay;
            }
        }
    }
    timerMarkNextTick(currentMicro);
}

void ICACHE_RAM_ATTR ZC_detect()
{
    uint32_t currentMicro = micros();
    if (currentMicro - lastSCROpenTiming <= 100 && (executeTiming.scr1_close == 0 || executeTiming.scr1_close - currentMicro >= 100) && (executeTiming.scr2_close == 0 || executeTiming.scr2_close - currentMicro >= 100)) // less then 20us
    {
//        Serial.printf("Surge Detected\n");
        return;
    }
#ifdef DEBUG_TRIGGER_PIN
    digitalWrite(DEBUG_TRIGGER_PIN, HIGH);
#endif
    inTrigger = 1;
    if (zc_interval == 0)
    {
        if (zc_last != 0)
        {
            zc_interval = currentMicro - zc_last;
        } else {
            zc_last = currentMicro;
        }
        inTrigger = 0;
    } else if (currentMicro - zc_last >= 3000)  // Max 300 Hz
    {
        if (currentMicro - zc_last <= 12500)  // Min at 80 Hz
        {
            zc_interval = currentMicro - zc_last;          
        }
        zc_last = currentMicro;
        ZC = 1;

        inTrigger = 0;
        calcSCRTiming(currentMicro);
    }
#ifdef DEBUG_TRIGGER_PIN
    digitalWrite(DEBUG_TRIGGER_PIN, LOW);
#endif
}


void inline disableInterrupts()
{
#if DEBUG_LEVEL >= 3
  Serial.printf("Disable interrupt\n");
#endif
  timerRunning = false;
  if (config.pin_scr_trig != 0)
  {
     detachInterrupt(config.pin_scr_trig);
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
  if (config.pin_scr_trig != 0)
  {
      if (config.work_mode == 2)
      {
          pinMode(config.pin_scr_trig, INPUT_PULLUP);
          attachInterrupt(config.pin_scr_trig, ZC_detect, RISING);       // Enable external interrupt (INT0)
      } else if (config.work_mode == 3) {        
          pinMode(config.pin_scr_trig, INPUT_PULLUP);
          attachInterrupt(config.pin_scr_trig, ZC_detect, FALLING);       // Enable external interrupt (INT0)
      } else if (config.work_mode == 4) {        
          pinMode(config.pin_scr_trig, INPUT_PULLUP);
          attachInterrupt(config.pin_scr_trig, ZC_detect, CHANGE);       // Enable external interrupt (INT0)
      }
//      pinMode(config.pin_ch1, 
      {
          if (config.work_mode == 2 || config.work_mode == 3 || config.work_mode == 4) {
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
              timerAlarmWrite(timer, 5000000, false); // Run by manual tick
//                  timerAlarmWrite(timer, 1250, true);  // Run by auto tick with 2000 Hz
              timerAlarmEnable(timer);
#endif
          }
      }
  }
}

void setup() {
  uint16_t boot_count = 0;
#ifdef DEBUG_TRIGGER_PIN
  pinMode(DEBUG_TRIGGER_PIN, OUTPUT);
  digitalWrite(DEBUG_TRIGGER_PIN, LOW);
#endif
#ifdef DEBUG_SAVE_PIN
  pinMode(DEBUG_SAVE_PIN, OUTPUT);
  digitalWrite(DEBUG_SAVE_PIN, LOW);
#endif
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
  // ESP_RST_BROWNOUT

  memset(&executeTiming, 0, sizeof(scr_timing_t));
  memset(&pendingTiming, 0, sizeof(scr_timing_t));
  cfg_begin();
  bool cfgCheck = CFG_CHECK();
  if (!CFG_LOAD())
  {
      CFG_INIT(true);
  }
  if (config.reversion != 1)
  {
      // Migrate from Old Version ** NOT FOR NVS **
      Serial.printf("Migrate from Old Config Reversion: %d\n", config.reversion);
      cfg_load(old_def_cfg);
      config.reversion = def_cfg[0].data.uint8;
      Serial.printf("Migrated config Reversion: %d\n", config.reversion);
#ifdef ARDUINO_ARCH_ESP8266
      // Migrate from old 16 bits to 32 bits
      config.pwm_freq = config.pwm_freq & 0xffff;
#endif
      CFG_SAVE();
      cfg_load(def_cfg);
      Serial.printf("Migrated config Reversion: %d\n", config.reversion);
  }
  if (config.pwm_freq == 0 || config.pwm_period == 0)
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
          config.work_mode = 4;
          config.pin_ch1 = 4;
          config.pin_ch2 = 16;
          config.mode_ch2 = 1;
          config.pin_en1 = 0;
          config.pin_en2 = 0;
          config.pin_scr_trig = 5;
          config.pin_sensor = 32;
          CFG_SAVE();
      } else if (digitalRead(34) == HIGH)
      {
          Serial.printf("Auto Initalize by PCB Board - Dual CH\n");
          config.work_mode = 0;
          config.pin_ch1 = 25;
          config.pin_ch2 = 26;
          config.pin_en1 = 32;
          config.pin_en2 = 33;
      } else if (digitalRead(35) == HIGH)
      {
          Serial.printf("Auto Initalize by PCB Board - Single CH\n");
          config.work_mode = 0;
          config.pin_ch1 = 25;
          config.pin_ch2 = 0;
          config.pin_en1 = 32;
          config.pin_en2 = 0;
      } else if (digitalRead(27) == LOW)
      {
          Serial.printf("Auto Initalize by PCB Board - Dual CH V1.7\n");
          config.work_mode = 0;
          config.pin_ch1 = 26;
          config.pin_ch2 = 25;
      }
#endif
  }
  if (config.pin_en1 != 0)
  {
      pinMode(config.pin_en1, INPUT);
  }
  if (config.pin_en2 != 0)
  {
      pinMode(config.pin_en2, INPUT);
  }

  char *p;
  char chEEP;
  int iOffset;
  int rc;
  
  analogPinInit(config.pwm_freq, config.pwm_period, config.pin_ch1);
  if (config.pin_ch2 != 0)
  {
      analogPinInit(config.pwm_freq, config.pwm_period, config.pin_ch2);
  }
  if (config.pwm_src != 0)
  {
      analogPinInit(config.pwm_freq, config.pwm_period, config.pwm_src);
  }
  ignoreOutput = true;
  updatePWMValue(0, config.mode_ch2 == 0 ? config.ct_abx : 0);
#ifdef ARDUINO_ARCH_ESP8266
  if ((resetInfo->reason == REASON_DEFAULT_RST || resetInfo->reason == REASON_EXT_SYS_RST) && strlen(ssid) != 0)
#elif ARDUINO_ARCH_ESP32
  if (config.pwm_freq <= 40000 && (config.work_mode == 0 || config.work_mode == 1))
  {
      setCpuFrequencyMhz(80);
  }
  if ((resetCode == POWERON_RESET || resetCode == RTCWDT_RTC_RESET) && strlen(ssid) != 0)
#endif
  {
      last_state_hold = millis();
      last_set_state = 0;
      if (config.mode_ch2 == 1) last_ct_abx = 0;
      boot_count = boot_count_increase();
  }
#ifdef ARDUINO_ARCH_ESP8266
  // ESP8266 will break PWM update between WiFi connecting, so we need to make smooth before WiFi action
  if (config.work_mode == 0 || config.work_mode == 1)
  {
      while (last_state_hold != 0  && millis() - last_state_hold < config.startup_smooth)
      {
          STARTUP_SMOOTH_EXECUTE;
          yield();
      }
  }
#endif
  
  if (config.pin_reset != 0)
  {
      pinMode(config.pin_reset, INPUT_PULLUP);
      delay(1);
      if (digitalRead(config.pin_reset) == LOW)
      {
          Serial.printf("Reset config by RESET PIN %d\n", config.pin_reset);
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
  if (config.pin_ch1 != LED_BUILTIN)
  {
      digitalWrite(LED_BUILTIN, LOW);
  }
  
//  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);

  Serial.printf("\n");
  Serial.printf("CONFIG SRC: %s\n", cfg_get_backend());
  Serial.printf("Boot Count: %d\n", boot_count);
#ifdef ARDUINO_ARCH_ESP8266
  Serial.printf("Flash: %d\n", ESP.getFlashChipRealSize());
  Serial.printf("Reset Reason: %d -> %s\n", resetInfo->reason, ESP.getResetReason().c_str());
#elif ARDUINO_ARCH_ESP32
  Serial.printf("Reset Reason: %d\n", resetCode);
//  Serial.printf("RTC Reset Reason: %d\n", rtc_get_reset_reason());
  Serial.printf("CPU Speed: %d MHz  XTAL: %d MHz  APB: %d Hz\n", getCpuFrequencyMhz(), getXtalFrequencyMhz(), getApbFrequency());
#endif
  Serial.printf("Version: %s\n", VERSION);
  Serial.printf("Board: %s\n", ARDUINO_BOARD);
  Serial.printf("Build Date: %s %s  CFG SIZE: %d\n", __DATE__, __TIME__, sizeof(conf_t));
  Serial.printf("Device ID: %s\n", mqtt_cls+sizeof(MQTT_CLASS));

  startupWifiConfigure(def_cfg, msg_buf, sizeof(msg_buf), mqtt_cls);

#ifdef ARDUINO_ARCH_ESP8266
  updatePWMValue(config.bright, config.ct_abx);
#elif ARDUINO_ARCH_ESP32
  updatePWMValue(config.bright, config.ct_abx);
#endif
  
//  ignoreOutput = true;
  Serial.printf("\n");
  Serial.printf("SSID = %s  PASSWORD = %s\n", ssid, password);
  Serial.printf("PWM Frequence = %d   Range = %d - %d   Period = %d\n", config.pwm_freq, config.pwm_start, config.pwm_end, config.pwm_period);
  if (config.work_mode == 2 || config.work_mode == 3 || config.work_mode == 4)
  {
      Serial.printf("SCR PIN: %d  TRIGGER PIN: %d  MODE: %d  CFG RESET PIN: %d\n", config.pin_ch1, config.pin_scr_trig, config.work_mode, config.pin_reset);
  } else {
      Serial.printf("PWM PIN: %d  WARM PIN: %d  MODE: %d  CFG RESET PIN: %d\n", config.pin_ch1, config.pin_ch2, config.work_mode, config.pin_reset);
  }

  enableInterrupts();

#ifdef ARDUINO_ARCH_ESP8266
  Serial.print("Connecting to WiFi");
  uint32_t last_print_dot = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (last_state_hold != 0 && millis() - last_state_hold < config.startup_smooth)
    {
        STARTUP_SMOOTH_EXECUTE;
    } else 
    {
        last_set_state = config.bright;
        last_ct_abx = config.ct_abx;
        updatePWMValue(config.bright, config.ct_abx);
    }
    if (millis() - last_print_dot >= 200)
    {
        Serial.print(".");
        last_print_dot = millis();
    }
    while (Serial.available())
    {
      char ch = Serial.read();
      if (ch == 'R') {
          delay(50);
          if (Serial.read() == 'r')
          {
              Serial.printf("Reset config command from serial.\n");
              cfg_reset(def_cfg);
              ESP.restart();
          }
      }
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
  Serial.printf("\n");

#endif
  while (last_state_hold != 0 && millis() - last_state_hold < config.startup_smooth)
  {
      STARTUP_SMOOTH_EXECUTE;
      yield();
  }

  ignoreOutput = false;
  last_state_hold = 0;
  
////  WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 1000);
  client.setServer((const char *)mqtt_server, port);//端口号
  client.setCallback(callback); //用于接收服务器接收的数据
//  Serial.printf("Success\n");
  if (LED_BUILTIN != config.pin_ch1 && LED_BUILTIN != config.pin_ch2)
  {
      digitalWrite(LED_BUILTIN, HIGH);    
  }

  // if luminous is more than 500 lx degrees below or above setpoint, OUTPUT will be set to min or max respectively
//  pidControl.setBangBang(15000);
  //set PID update interval to 100ms
  pidControl.setTimeStep(100);
  pidControl.setGains(config.KP / KP_DIV, config.KI / KI_DIV, config.KD / KD_DIV);
  pidControl.reset();
}

void sendMeta()
{
    last_send_meta = millis();
  // max length = 64 - 21
    char *p = msg_buf + sprintf(msg_buf, "{\"name\":\"%s\",\"board\":\"%s\"", dev_name, ARDUINO_BOARD);
    p = p + sprintf(p, ",\"pwm_freq\":%d,\"pwm_start\":%d,\"pwm_end\":%d,\"period\":%d,\"en1\":%d,\"en2\":%d,\"adc\":%d,\"ch2mode\":%d}", config.pwm_freq, config.pwm_start, config.pwm_end, config.pwm_period, config.pin_en1, config.pin_en2, config.pin_adc_bright, config.mode_ch2);
    client.publish("dev", msg_buf);
    if (isScrMode)
    {
        p = msg_buf + sprintf(msg_buf, "{\"scr_delay\":%d,\"trigger_pin\":%d,\"prerelease\":%d}", config.scr_delay, config.pin_scr_trig, config.scr_prerelease);
        client.publish("dev", msg_buf);
    }
//    if (config.pin_sensor != 0)
    {
        p = msg_buf + sprintf(msg_buf, "{\"sensor_pin\":%d,\"max_lm\":%d,\"kp\":%ld,\"ki\":%ld,\"kd\":%ld,\"ref\":\"%d,%d\",\"adcref\":\"%d,%d,%d\",\"ax\":%f,\"b\":%f}", config.pin_sensor, config.max_lm, config.KP, config.KI, config.KD, config.sensor_ref, config.sensor_ref_r1, config.adc_volt_0db, config.adc_volt_6db, config.adc_volt_11db, config.sensor_ax, config.sensor_b);
        client.publish("dev", msg_buf);
    }
    
    p = msg_buf + sprintf(msg_buf, "{\"pin\":%d,\"wpin\":%d,\"pwm_src\":%d, \"duty\":%d,\"mode\":%d,\"cfg_pin\":%d,\"auto_full_power\":%d,\"no_autosave\":%d,\"startup_smooth\":%d}", config.pin_ch1, config.pin_ch2, config.pwm_src, config.pwm_src_duty, config.work_mode, config.pin_reset, config.auto_fp, config.autosave, config.startup_smooth);
    client.publish("dev", msg_buf);
}


void callback(char* topic, byte* payload, unsigned int length) {//用于接收数据
  int l=0;
  int p=1;
  hasPacket = true;
  if (strcmp(topic, "set_bright") == 0 || strcmp(topic, "set_ct_abx") == 0)
  {
    if (!inSmooth)
    {
        last_set_state = config.bright;
        last_ct_abx = config.ct_abx;
        inSmooth = (config.pin_sensor == 0 || config.max_lm == 0);
    } else {
        last_set_state = last_set_state - (float)(last_set_state - config.bright) * (millis() - last_state_hold) / SMOOTH_INTERVAL;
        last_ct_abx = last_ct_abx - (float)(last_ct_abx - config.ct_abx) * (millis() - last_state_hold) / SMOOTH_INTERVAL;
    }
    SMOOTH_INTERVAL = 200;
    payload[length] = 0;
    save_config = !config.autosave && true;
    if (strchr((char *)payload, ',') != NULL)
    {
        save_config = false;
        SMOOTH_INTERVAL = atoi(strchr((char *)payload, ',') + 1);
        *strchr((char *)payload, ',') = 0;
        Serial.printf("Set SMOOTH_INTERVAL to %d\n", SMOOTH_INTERVAL);
    }
    if (strcmp(topic, "set_bright") == 0)
    {
        config.bright = atoi((char *)payload);
        if (config.bright < 0) config.bright = 0;
        if (config.bright > 100) config.bright = 100;
    } else 
    {
        config.ct_abx = atoi((char *)payload);
        if (config.mode_ch2 == 0)
        {
            if (config.ct_abx < 2600)
            {
                config.ct_abx = 2600;
            }
            if (config.ct_abx > 5600)
            {
                config.ct_abx = 5600;
            }
        } else 
        {
            if (config.ct_abx < 0) config.ct_abx = 0;
            if (config.ct_abx > 100) config.ct_abx = 100;
        }
    }
    
    if (last_set_state == config.bright && last_ct_abx == config.ct_abx)
    {
        inSmooth = false;
        Serial.print("Ignore: ");
        inSmooth = true;
        last_state_hold = 0;
    } else if (!inSmooth)
    {
        Serial.print("DIRECT: ");
        inSmooth = true;
        last_state_hold = 0;
    } else {
        last_state_hold = millis();
    }
    last_state = -1;
    return;
  }
  disableInterrupts();
  if (strcmp(topic, "ota") == 0)
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
        rebootSystem();
        break;

      case HTTP_UPDATE_NO_UPDATES:
        sprintf(msg_buf, "{\"ota\":\"no updates\"}");
        client.publish("status", msg_buf);
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        rebootSystem();
        break;

      case HTTP_UPDATE_OK:
        sprintf(msg_buf, "{\"ota\":\"success\"}");
        client.publish("status", msg_buf);
        Serial.println("HTTP_UPDATE_OK");
        rebootSystem();
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
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_pwm_freq") == 0)
  {
      char bufferByte = payload[length];
      payload[length] = 0;
      config.pwm_freq = strtoul((char *)payload, NULL, 10);
      
      payload[length] = bufferByte;
      CFG_SAVE();
      analogPinInit(config.pwm_freq, config.pwm_period, config.pin_ch1);
      if (config.pin_ch2 != 0)
      {
          analogPinInit(config.pwm_freq, config.pwm_period, config.pin_ch2);
      }
      if (config.pwm_src != 0)
      {
          analogPinInit(config.pwm_freq, config.pwm_period, config.pwm_src);
      }
      sendMeta();
      inSmooth = true; last_state_hold = 0;
  } else if (strcmp(topic, "set_pwm_period") == 0)
  {
      char bufferByte = payload[length];
      payload[length] = 0;
      if (config.pwm_end == config.pwm_period)
      {
          config.pwm_end = atoi((char *)payload);
      }
      config.pwm_period = atoi((char *)payload);
      payload[length] = bufferByte;
      CFG_SAVE();
      
      analogPinInit(config.pwm_freq, config.pwm_period, config.pin_ch1);
      if (config.pin_ch2 != 0)
      {
          analogPinInit(config.pwm_freq, config.pwm_period, config.pin_ch2);
      }
      if (config.pwm_src != 0)
      {
          analogPinInit(config.pwm_freq, config.pwm_period, config.pwm_src);
      }
      sendMeta();
      inSmooth = true; last_state_hold = 0;
  } else if (strcmp(topic, "set_pwm_pin") == 0)
  {
      payload[length] = 0;
      pwmWrite(config.pin_ch1, 0);
      config.pin_ch1 = atoi((char *)payload);
      analogPinInit(config.pwm_freq, config.pwm_period, config.pin_ch1);
      CFG_SAVE();
      sendMeta();
      inSmooth = true; last_state_hold = 0;
  } else if (strcmp(topic, "set_pwm_wpin") == 0)
  {
      payload[length] = 0;
      if (config.pin_ch2 != 0)
      {        
          pwmWrite(config.pin_ch2, 0);
      }
      config.pin_ch2 = atoi((char *)payload);
      analogPinInit(config.pwm_freq, config.pwm_period, config.pin_ch2);
      CFG_SAVE();
      sprintf(msg_buf, "{\"bright2\":\"unset\",\"ct\":\"unset\"}");
      client.publish("status", msg_buf);
      last_state = -1;
      sendMeta();
      inSmooth = true; last_state_hold = 0;
  } else if (strcmp(topic, "set_pwm_src") == 0)
  {
      payload[length] = 0;
      if (config.pwm_src != 0)
      {        
          pwmWrite(config.pwm_src, 0);
      }
      config.pwm_src = atoi((char *)payload);
      analogPinInit(config.pwm_freq, config.pwm_period, config.pwm_src);
      CFG_SAVE();
      sendMeta();
      inSmooth = true; last_state_hold = 0;
  } else if (strcmp(topic, "set_pwm_duty") == 0)
  {
      payload[length] = 0;
      config.pwm_src_duty = atoi((char *)payload);
      analogPinInit(config.pwm_freq, config.pwm_period, config.pwm_src);
      CFG_SAVE();
      sendMeta();
      inSmooth = true; last_state_hold = 0;
  }
  else if (strcmp(topic, "set_ch2mode") == 0)
  {
      payload[length] = 0;
      config.mode_ch2 = atoi((char *)payload);
      if (config.mode_ch2 == 1 && config.ct_abx > 100)
      {
          config.ct_abx = 100;
      } else if (config.mode_ch2 == 0)
      {
          if (config.ct_abx < 2600)
          {
              config.ct_abx = 2600;
          }
          if (config.ct_abx > 5600)
          {
              config.ct_abx = 5600;
          }
      }
      CFG_SAVE();
      sprintf(msg_buf, "{\"bright2\":\"unset\",\"ct\":\"unset\"}");
      client.publish("status", msg_buf);
      last_state = -1;
      inSmooth = true; last_state_hold = 0;
      sendMeta();
  } else if (strcmp(topic, "reset") == 0)
  {
      if (length > 0 && payload[0] == '1')
      {
          cfg_reset(def_cfg);
      } else
      {
          CFG_INIT(false);
      }
      Serial.printf("Reset System\n");
  } else if (strcmp(topic, "reboot") == 0)
  {
      rebootSystem();
  } else if (strcmp(topic, "toggle_realtime") == 0)
  {
      realtime_data = !realtime_data;
  }
  AUTO_CONF_INT_COMMAND(topic, "set_config_pin", pin_reset, )
  AUTO_CONF_INT_COMMAND(topic, "set_autosave", autosave, )
  AUTO_CONF_INT_COMMAND(topic, "set_pwm_trigger", pin_scr_trig, rebootSystem())
  AUTO_CONF_INT_COMMAND(topic, "set_pwm_start", pwm_start, inSmooth = true; last_state_hold = 0)
  AUTO_CONF_INT_COMMAND(topic, "set_pwm_end", pwm_end, inSmooth = true; last_state_hold = 0)
  AUTO_CONF_INT_COMMAND(topic, "set_pwm_mode", work_mode, inSmooth = true; last_state_hold = 0)
  AUTO_CONF_INT_COMMAND(topic, "set_pwm_delay", scr_delay, inSmooth = true; last_state_hold = 0)
  AUTO_CONF_INT_COMMAND(topic, "set_en1", pin_en1, inSmooth = true; last_state_hold = 0)
  AUTO_CONF_INT_COMMAND(topic, "set_en2", pin_en2, inSmooth = true; last_state_hold = 0)
  AUTO_CONF_INT_COMMAND(topic, "set_adc_pin", pin_adc_bright, inSmooth = true; last_state_hold = 0)
  AUTO_CONF_INT_COMMAND(topic, "set_auto_full_power", auto_fp, inSmooth = true; last_state_hold = 0)
  AUTO_CONF_INT_COMMAND(topic, "set_scr_prerelease", scr_prerelease, )
  AUTO_CONF_INT_COMMAND(topic, "set_startup_smooth", startup_smooth, )
  AUTO_CONF_INT_COMMAND(topic, "set_sensor_pin", pin_sensor, inSmooth = true; last_state_hold = 0)
  AUTO_CONF_INT_COMMAND(topic, "set_max_lm", max_lm, inSmooth = true; last_state_hold = 0)
  AUTO_CONF_INT_COMMAND(topic, "set_kp", KP, pidControl.setGains(config.KP / KP_DIV, config.KI / KI_DIV, config.KD / KD_DIV); pidControl.reset())
  AUTO_CONF_INT_COMMAND(topic, "set_ki", KI, pidControl.setGains(config.KP / KP_DIV, config.KI / KI_DIV, config.KD / KD_DIV); pidControl.reset())
  AUTO_CONF_INT_COMMAND(topic, "set_kd", KD, pidControl.setGains(config.KP / KP_DIV, config.KI / KI_DIV, config.KD / KD_DIV); pidControl.reset())
  AUTO_CONF_INT_COMMAND(topic, "set_sensor_ref", sensor_ref, )
  AUTO_CONF_INT_COMMAND(topic, "set_sensor_ref_r1", sensor_ref_r1, )
  AUTO_CONF_INT_COMMAND(topic, "set_adc_0db", adc_volt_0db, )
  AUTO_CONF_INT_COMMAND(topic, "set_adc_6db", adc_volt_6db, )
  AUTO_CONF_INT_COMMAND(topic, "set_adc_11db", adc_volt_11db, )
  AUTO_CONF_FLOAT_COMMAND(topic, "set_sensor_ax", sensor_ax, )
  AUTO_CONF_FLOAT_COMMAND(topic, "set_sensor_b", sensor_b, )
  
  enableInterrupts();
}

bool reconnect() {//等待，直到连接上服务器
  static uint32_t disconnectTime = 0;
  static wl_status_t lastWiFiStatus = WL_NO_SHIELD;
  static int retry_failed_count = 0;

  if (WiFi.status() != lastWiFiStatus)
  {
      if (WiFi.status() != WL_CONNECTED) {
          disconnectTime = millis();
      } else 
      {
          disconnectTime = 0;
          IPAddress myAddress = WiFi.localIP();
          Serial.printf("Connected to wifi, IP: ");
          Serial.print(myAddress);
          Serial.printf("\n");
      }
      lastWiFiStatus = WiFi.status();
  }
  if (WiFi.status() != WL_CONNECTED && millis() - disconnectTime >= 30000) {
      Serial.printf("WiFi: Status = %d (Disconnected), RESET SYSTEM\n", WiFi.status());
      rebootSystem();
  }
  if (WiFi.status() == WL_CONNECTED && !client.connected()){
      Serial.printf("Connecting to MQTT %s:%d\n", mqtt_server, port);
#ifdef ARDUINO_ARCH_ESP32
      esp_task_wdt_init(30, true);
      esp_task_wdt_add(NULL);
#endif
      if (client.connect(mqtt_cls)) {//接入时的用户名，尽量取一个很不常用的用户名
        retry_failed_count = 0;
        Serial.printf("Connect to MQTT success, login by: %s\n",mqtt_cls);
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
        Serial.printf("Connect failed, rc=%d\n", client.state());//连接失败
        client.disconnect();
        delay(1000);
        if (retry_failed_count >= 10)
        {
            Serial.printf("MQTT: Reconnect Too many times, RESET SYSTEM\n");
            rebootSystem();
        }
      }
  }
  return WiFi.status() == WL_CONNECTED && client.connected();
}

int uint16_cmpfunc (const void * a, const void * b) {
   return ( *(uint16_t*)a - *(uint16_t*)b );
}

#include <EEPROM.h>
int buflen = 0;
void loop() {
    uint32_t currentMillis = millis();
    static uint32_t lastSensorDetect = 0;
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
          Serial.printf("Set pin %d to OUTPUT\n", config.pin_ch1);
          pinMode(config.pin_ch1, OUTPUT);
          digitalWrite(config.pin_ch1, HIGH);
//          updateSCRState(false);
      } else if (ch == 'O') {
          Serial.printf("Set pin %d to OUTPUT OPEN\n", config.pin_ch1);
          pinMode(config.pin_ch1, OUTPUT);
          digitalWrite(config.pin_ch1, LOW);
//          updateSCRState(true);
      } else if (ch == 'R' && Serial.read() == 'r') {
          cfg_reset(def_cfg);
          ESP.restart();
      }
    }
    if (!bootCountReset && currentMillis - boot_time >= 5000)
    {
        bootCountReset = true;
        disableInterrupts();
        boot_count_reset();
        enableInterrupts();
    }
    if (SMOOTH_INTERVAL == 0)
    {
      SMOOTH_INTERVAL = 200;
    }
    if (inSmooth && last_state_hold != 0 && currentMillis - last_state_hold <= SMOOTH_INTERVAL && config.bright != -1)
    {
      if (!isScrMode)
        Serial.print("Smooth: ");
      updatePWMValue((last_set_state - (float)(last_set_state - config.bright) * (currentMillis - last_state_hold) / SMOOTH_INTERVAL), (last_ct_abx - (float)(last_ct_abx - config.ct_abx) * (currentMillis - last_state_hold) / SMOOTH_INTERVAL));
    }
    if (inSmooth && currentMillis - last_state_hold > SMOOTH_INTERVAL && config.bright != -1)
    {
      if (!isScrMode)
        Serial.print("Final: ");
      updatePWMValue(config.bright, config.ct_abx);
      inSmooth = false;
    }
    if (last_state_hold != 0 && (config.bright > 0 || (config.mode_ch2 == 1 && config.ct_abx > 0)) && !inSmooth && !inTrigger && currentMillis - last_state_hold >= 2000)
    {
      if (save_config) {
#ifdef DEBUG_SAVE_PIN
        digitalWrite(DEBUG_SAVE_PIN, HIGH);
#endif
        Serial.printf("Saving config to FLASH...");
        uint32_t t1 = micros();
        uint32_t usages = 0;
        if (cfg_get_backend_t() == STORAGE_NVS)
        {
            cfg_save(def_cfg, true);
        } else {
            disableInterrupts();
            cfg_save(def_cfg, true);
            enableInterrupts();
        }
        usages += micros() - t1;
        Serial.printf(" in %ld us\n", micros() - t1);
#ifdef DEBUG_SAVE_PIN
        digitalWrite(DEBUG_SAVE_PIN, LOW);
#endif
      }
      last_state_hold = 0;
    }
    if (config.pin_adc_bright != 0)
    {
//      analogSetAttenuation(ADC_11db);
      int16_t new_adc_value = analogRead(config.pin_adc_bright);
      if (abs(new_adc_value - lastAdcValue) > 4096/20)
      {
        delay(50);
        int16_t new_adc_value2 = analogRead(config.pin_adc_bright);
        if (abs(new_adc_value - new_adc_value2) < 100 && abs(new_adc_value - lastAdcValue) > 4096/20)
        {
          Serial.printf("Change value: %d\n", abs(new_adc_value - lastAdcValue));
            sprintf(msg_buf, "%d", (uint32_t)new_adc_value * 100 / 4096);
            lastAdcValue = new_adc_value;
            callback("set_bright", (byte *)msg_buf, strlen(msg_buf));
        }
      }
    }
    if (config.pin_sensor != 0 && config.max_lm != 0 && (currentMillis - lastSensorDetect >= 100 || currentMillis < lastSensorDetect))
    {
      // Resistor Side
      // Imax = (2.495)/100000 ~= 24.95uA
      // Rmax(sensor) = 3.3/Imax - 100000 ~= 66498
      // Vsensor = Imax*R
      // Rsensor = Vsensor / Imax
      // Rsensor ＝Vsensor / 10 / Imax
      // OpAmp Non-Inv
      // Isensor = 2.495 / (100000 + Rsensor)
      // Vsensor = Isensor * Rsense
      // Rsensor(LowSide) = (Vsensor / Gain * 100000) / (2.495 - Vsensor / Gain)
      // Rsensor(HighSide) = (2.495 * R2 - Vsensor * R2) / Vsensor

      uint16_t *adcVoltageRange;
      adcVoltageRange = &config.adc_volt_11db;
      analogSetPinAttenuation(config.pin_sensor, ADC_11db);      
      if (analogRead(config.pin_sensor) <= 1100)
      {
          adcVoltageRange = &config.adc_volt_0db;
          analogSetPinAttenuation(config.pin_sensor, ADC_0db);
      } else if (analogRead(config.pin_sensor) <= 2200)
      {
          adcVoltageRange = &config.adc_volt_6db;
          analogSetPinAttenuation(config.pin_sensor, ADC_6db);
      }
      bool calibrateMode = (*adcVoltageRange & 0x8000) == 0x8000;
      uint32_t sensor_adc = 0;
      uint16_t sensor_adcValues[32];
      for (int i = 0; i < 32; i++)
      {
          sensor_adcValues[i] = analogRead(config.pin_sensor);
      }
      qsort(sensor_adcValues, 32, sizeof(uint16_t), uint16_cmpfunc);
      for (int i = 8; i < 24; i++)
      {
          sensor_adc += sensor_adcValues[i];
      }
      sensor_adc >>= 3;
      if (calibrateMode)
      {
          Serial.printf("In Calibrate Mode, full range = %d\n", (int)((*adcVoltageRange & 0x7fff) * 8191. / sensor_adc));
          *adcVoltageRange = (*adcVoltageRange & 0x7fff) * 8191. / sensor_adc;
          if (cfg_get_backend_t() == STORAGE_NVS)
          {
              cfg_save(def_cfg, true);
          } else {
              disableInterrupts();
              cfg_save(def_cfg, true);
              enableInterrupts();
          }
          sendMeta();
      }
      float voltageRange = (*adcVoltageRange & 0x7fff) / 1000.;
      
      sensor_bright = (8191-sensor_adc * (voltageRange / 3.6)) / 8191. * 100;
      target_bright = config.bright;
      float Vsensor = (sensor_adc / 8191. * voltageRange);
      float Rsensor = (config.sensor_ref/1000. * config.sensor_ref_r1 - Vsensor * config.sensor_ref_r1) / Vsensor;
      float lumen = config.sensor_ax * pow(Rsensor, config.sensor_b);
      sensor_bright = lumen / config.max_lm * 100;
      if (sensor_bright >= 100) sensor_bright = 100;
      target_bright = config.bright;
      pidControl.run();

      if (realtime_data){
          sprintf(msg_buf, "{\"current_bright\":%f,\"action\":%f,\"lumen\":%f,\"target_lumen\":%f,\"adc_voltage\":%f,\"resistor\":%f}", sensor_bright, pidBright, lumen,config.bright/100.*config.max_lm,Vsensor, Rsensor);
          client.publish("callback", msg_buf);
      }
      
//      (vSensor/11 * 100000)/(2.495 - vSensor / 11));
      updatePWMValue(pidBright, config.mode_ch2 == 0 ? config.ct_abx : pidBright * config.ct_abx / 100.);
      lastSensorDetect = currentMillis;
    }
    if (reconnect()) {//确保连上服务器，否则一直等待。
      client.loop();//MUC接收数据的主循环函数。
      long rssi = WiFi.RSSI();
       if (last_state != config.bright || (abs(rssi - last_rssi) >= 3 && currentMillis - last_send_rssi >= 5000))
       {
          last_send_rssi = currentMillis;
          last_state = config.bright;
          last_rssi = rssi;
          if (config.pin_ch2 != 0)
          {
              if (config.mode_ch2 == 0)
              {
                  sprintf(msg_buf, "{\"bright\":%d,\"ct\":%d,\"rssi\":%ld,\"version\":\"%s\",\"boot\":%ld}", config.bright, config.ct_abx, rssi, VERSION, currentMillis);
              } else 
              {
                  sprintf(msg_buf, "{\"bright\":%d,\"bright2\":%d,\"rssi\":%ld,\"version\":\"%s\",\"boot\":%ld}", config.bright, config.ct_abx, rssi, VERSION, currentMillis);
              }
          } else {
              sprintf(msg_buf, "{\"bright\":%d,\"rssi\":%ld,\"version\":\"%s\",\"boot\":%ld}", config.bright, rssi, VERSION, currentMillis);
          }
          client.publish("status", msg_buf);
       }
       if (currentMillis - last_send_meta >= 60000)
       {
          if (!isScrMode)
              Serial.printf("PING %ld  WiFI: %d\n", currentMillis, WiFi.status());
          sendMeta();
       }
    }
}
