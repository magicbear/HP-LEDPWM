#ifdef ARDUINO_ARCH_ESP32
  #include <WiFi.h>
  #include <HTTPUpdate.h>
  #include <math.h>
  #include <driver/dac.h>
  #include <driver/ledc.h>
  #include <rom/rtc.h>
  #define ESPhttpUpdate httpUpdate
#else
  #include <ESP8266WiFi.h>
  #include <ESP8266httpUpdate.h>
  extern "C"{
      #include "pwm.h"
  }
#endif
#include <PubSubClient.h>
#include <Ticker.h>
#include "config.h"

// CONFIG: REVERSE THE OUTPUT FOR MOSFET+BJT Driver
// OUTPUT_BY_MOSFET = 2: USING SCR to OUTPUT
uint8_t OUTPUT_BY_MOSFET = 1;
// CONFIG: SMOOTH PWM INTERVAL IN mS
unsigned int SMOOTH_INTERVAL = 200;
// CONFIG: Warm / Single LED PWM Pin  (ESP-01  RXD0)
uint8_t  PWM_PIN = 3;
uint8_t  PWM_WPIN = 0;
uint8_t  PWM_SCR_TRIGGER = 0;
int16_t  PWM_SCR_DELAY = 0;
uint8_t  PWM_AUTO_FULL_POWER = 1;

#ifndef MQTT_CLASS
#define MQTT_CLASS "HP-LEDPWM"
#define _VERSION "2.14"

#ifdef ARDUINO_ARCH_ESP32
#define VERSION _VERSION"_32"
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
uint16_t PWM_FREQUENCE = 20000;

uint16_t PWM_START = 0;
uint16_t PWM_END = PWM_PERIOD;

uint8_t  EN_PORT = 0;

bool save_config = true;
bool ignoreOutput = false;

int current_bright = 0;

uint16_t set_ct = 4000;
uint8_t  set_state = 0;
uint8_t  inTrigger = 0;
uint16_t port = 1883;//服务器端口号

uint32_t zc_interval = 0;
uint32_t zc_last = 0;

int retry_failed_count = 0;

char* ssid;
char* password;
char* mqtt_server[16];//服务器的地址 
char mqtt_cls[sizeof(MQTT_CLASS) + 13];
char msg_buf[160];

const hp_cfg_t def_cfg[] = {
  {31, sizeof(uint8_t), (uint8_t)10,      &set_state, true},        // BRIGHT
  {32, sizeof(uint8_t), (uint8_t)0, NULL, true},                    // NAME LENGTH
  {64, sizeof(uint16_t), (uint16_t)20000, &PWM_FREQUENCE, false},   // UINT16: PWM FREQUENCE
  {66, sizeof(uint16_t), (uint16_t)0,     &PWM_START, false},       // UINT16: PWM START
  {68, sizeof(uint16_t), (uint16_t)200,   &PWM_END, false},         // UINT16: PWM END
  {70, sizeof(uint16_t), (uint16_t)4100,  &set_ct, false},          // UINT16: COLOR TEMPERATURE
  {72, sizeof(uint16_t), (uint16_t)200,   &PWM_PERIOD, false},      // UINT16: PWM PERIOD
  {74, sizeof(uint8_t), (uint8_t)0,   &PWM_PIN, false},             // UINT8:  PWM_PIN
  {75, sizeof(uint8_t), (uint8_t)0,   &PWM_WPIN, false},            // UINT8:  PWM_WPIN
  {76, sizeof(uint8_t), (uint8_t)1,   &OUTPUT_BY_MOSFET, false},    // UINT8:  OUTPUT_BY_MOSFET
  {77, sizeof(uint8_t), (uint8_t)0,   &PWM_SCR_TRIGGER, false},     // UINT8:  SCR MODE ZeroDetect Pin
  {78, sizeof(int16_t), (int16_t)0,   &PWM_SCR_DELAY, false},       // UINT16: SCR MODE ZeroDetect DELAY
  {80, sizeof(uint8_t), (uint8_t)0,   &CFG_RESET_PIN, false},       // UINT8:  RESET PIN
  {81, sizeof(uint8_t), (uint8_t)5,   &EN_PORT, false},             // UINT8:  EN PIN
  {82, sizeof(uint8_t), (uint8_t)1,   &PWM_AUTO_FULL_POWER, false}, // UINT8:  AUTO FULL POWER
  {96, sizeof(uint8_t), (uint8_t)0, NULL, true},                    // STRING: SSID
  {128, sizeof(uint8_t), (uint8_t)0, NULL, true},                   // STRING: WIFI PASSWORD 
  {160, sizeof(mqtt_server), (uint8_t)0, mqtt_server, true},        // STRING: MQTT SERVER
  {192, sizeof(uint16_t), (uint16_t)1883, &port, true},             // UINT16: PORT
};

WiFiClient espClient;
PubSubClient client(espClient);
Ticker myTicker; //建立一個需要定時調度的對象

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

void analogPinInit(int32_t freq, int32_t period, uint8_t pwm_pin)
{
#ifdef ARDUINO_ARCH_ESP32
    uint8_t bits = (int)log2(period)+1;
    if (pwm_pin == 101 || pwm_pin == 102) 
    {
        dac_output_enable(pwm_pin == 101 ? DAC_CHANNEL_1 : DAC_CHANNEL_2);
    } else if (pwm_pin == PWM_PIN)
    {
        ledcSetup(0, freq, bits);
        if (OUTPUT_BY_MOSFET == 0)
        {
            ledcWrite(0, 0);
        } else 
        {
            ledcWrite(0, PWM_PERIOD);
        }
        ledcAttachPin(pwm_pin, 0);
    } else {
        ledcSetup(1, freq, bits);
        if (OUTPUT_BY_MOSFET == 0)
        {
            ledcWrite(1, 0);
        } else 
        {
            ledcWrite(1, PWM_PERIOD);
        }
        ledcAttachPin(pwm_pin, 1);
    }
#else
    Serial.printf("Initalize Analog PWM Pin %d (Period = %d   Freq = %d)\n", pwm_pin, period, freq);

    if (5000000 % freq == 0)
    {
        PWM_PERIOD = 5000000 / PWM_FREQUENCE;
    } else 
    {
        PWM_PERIOD = 320;
        freq = 5000000 / PWM_PERIOD;
    }
    
    if (pwm_pin == PWM_PIN)
    {
        io_info[0][0] = PinToGPIOMux(pwm_pin);
        io_info[0][1] = PinToGPIOMuxFunc(pwm_pin);
        io_info[0][2] = pwm_pin;
        pwm_duty_init[0] = OUTPUT_BY_MOSFET ? PWM_PERIOD : 0;
    } else {
        io_info[1][0] = PinToGPIOMux(pwm_pin);
        io_info[1][1] = PinToGPIOMuxFunc(pwm_pin);
        io_info[1][2] = pwm_pin;
        pwm_duty_init[1] = OUTPUT_BY_MOSFET ? PWM_PERIOD : 0;
    }
    digitalWrite(pwm_pin, OUTPUT_BY_MOSFET ? HIGH : LOW);
    // Initialize
    pwm_init(PWM_PERIOD, pwm_duty_init, PWM_WPIN == 0 ? 1 : 2, io_info);
  
    digitalWrite(pwm_pin, OUTPUT_BY_MOSFET ? HIGH : LOW);
    if (pwm_pin == PWM_PIN)
    {
        pwm_set_duty(OUTPUT_BY_MOSFET ? PWM_PERIOD : 0, 0);
    } else {
        pwm_set_duty(OUTPUT_BY_MOSFET ? PWM_PERIOD : 0, 1);
    }
    // Commit
    pwm_start();
    digitalWrite(pwm_pin, OUTPUT_BY_MOSFET ? HIGH : LOW);
  
//    analogWriteFreq(freq);  // 10kHz
//    analogWriteRange(period);
//    pinMode(pwm_pin, OUTPUT);
#endif
}

void pwmWrite(uint8_t pwm_pin, int32_t duty)
{
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
    } else 
    {
        ledcWrite(1, duty);
    }
#else
    if (pwm_pin == PWM_PIN)
    {
        pwm_set_duty(duty, 0);
    } else {
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

void updatePWMValue(float set_state)
{
    int pwm_state = mapfloat(set_state, 0, 100, PWM_START, PWM_END);
    if (pwm_state == PWM_START) pwm_state = 0;
    if (pwm_state == PWM_END && PWM_AUTO_FULL_POWER) pwm_state = PWM_PERIOD;
    if (PWM_WPIN != 0)
    {
        float set_power = 0.5 + (set_ct - 4100) / 3000.;
        if (inSmooth)
        {
            set_power = 0.5 + ((last_set_ct - (float)(last_set_ct - set_ct) * (millis() - last_state_hold) / SMOOTH_INTERVAL) - 4100) / 3000.;
        }
    
        float warm_power = 1-set_power;
        float cold_power = set_power;
    
        if (set_power > 0 && set_power < 0.5)
        {
            warm_power = 1;
            cold_power = set_power / (1-set_power);
        } else if (set_power >= 0.5 && set_power < 1)
        {
            warm_power = (1-set_power) / set_power;
            cold_power = 1;
        }

        if (!ignoreOutput)
        {
            Serial.printf("Set Power %f PWM = %d  CT: %d WARM: %d%% %d  COLD: %d%% %d\n", set_state, pwm_state, set_ct, (int)(warm_power * 100), (int)(pwm_state * warm_power), (int)(cold_power * 100), (int)(pwm_state * cold_power));
        }
    
        if (OUTPUT_BY_MOSFET == 1){
            pwmWrite(PWM_PIN, PWM_PERIOD - pwm_state * warm_power);
            pwmWrite(PWM_WPIN, PWM_PERIOD - pwm_state * cold_power);
        } else if (OUTPUT_BY_MOSFET == 0){
            pwmWrite(PWM_PIN, pwm_state * warm_power);
            pwmWrite(PWM_WPIN, pwm_state * cold_power);
        }
    } else {
        if (EN_PORT != 0)
        {
            if (set_state == 0)
            {
                pinMode(EN_PORT, OUTPUT);
                digitalWrite(EN_PORT, HIGH);
            } else 
            {
                pinMode(EN_PORT, INPUT);
            }
        }
    
        if (OUTPUT_BY_MOSFET == 1)
        {
            pwm_state = PWM_PERIOD - pwm_state; 
            pwmWrite(PWM_PIN, pwm_state);       
        } else if (OUTPUT_BY_MOSFET == 0)
        {
            pwmWrite(PWM_PIN, pwm_state); 
        } else 
        {
            pwm_state = PWM_PERIOD - pwm_state;
            current_bright = pwm_state;
        }
        if (OUTPUT_BY_MOSFET != 2 && !ignoreOutput)
            Serial.printf("Set Power Power %f  => %d\n", set_state, pwm_state);
    }
}

//void tickerHandle()
//{
//    digitalWrite(PWM_PIN, LOW);
//    myTicker.detach();
//    inTrigger = 0;
//}

uint8_t ZC = 0;
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
        if (micros() - zc_last <= 1000000)
        {
            zc_interval = micros() - zc_last;          
        }
        zc_last = micros();
        ZC = 1;
    }
    inTrigger = 0;
}
//
//void ESP_delayMicroseconds(uint32_t us){
//  uint32_t start = micros();
//  while(micros() - start < us){ yield(); }
//}

void setup() {
  uint16_t boot_count = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  if (EN_PORT != 0)
  {
      pinMode(EN_PORT, INPUT);
  }
  WiFi.persistent( false );
  Serial.begin(115200);
  byte mac[6];
  WiFi.macAddress(mac);
  sprintf(mqtt_cls, MQTT_CLASS"-%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
#ifdef ARDUINO_ARCH_ESP8266
  rst_info *resetInfo;
  resetInfo = ESP.getResetInfoPtr();
#else
  int resetCode = rtc_get_reset_reason(0);
#endif

  cfg_begin();
  CFG_CHECK();
  CFG_LOAD();
  if (PWM_FREQUENCE == 0 || PWM_PERIOD == 0)
  {
      CFG_INIT(true);
      CFG_LOAD();
  }

  char *p;
  char chEEP;
  int iOffset;
  int rc = CFG_READ_STRING(96, msg_buf, sizeof(msg_buf));
  if (rc == -1)
  {
      CFG_INIT(true);
      rebootSystem();
  }
  ssid = msg_buf;  
  password = msg_buf + rc + 1;
  
  rc = CFG_READ_STRING(128, msg_buf + rc + 1, sizeof(msg_buf) - rc - 1);
  if (rc == -1)
  {
      CFG_INIT(true);
      rebootSystem();
  }
  
  analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_PIN);
  if (PWM_WPIN != 0)
  {
      analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_WPIN);
  }
  delay(100);
#ifdef ARDUINO_ARCH_ESP8266
  if ((resetInfo->reason == REASON_DEFAULT_RST || resetInfo->reason == REASON_EXT_SYS_RST) && strlen(ssid) != 0)
#else
  if ((resetCode == POWERON_RESET) && strlen(ssid) != 0)
#endif
  {
      ignoreOutput = true;
      uint32_t last_state_hold = millis();
      last_set_state = 0;
      SMOOTH_INTERVAL = 200;
      while (millis() - last_state_hold < SMOOTH_INTERVAL)
      {
          updatePWMValue((last_set_state - (float)(last_set_state - set_state) * (millis() - last_state_hold) / SMOOTH_INTERVAL));
      }
      ignoreOutput = false;
      boot_count = boot_count_increase();
  }
  
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
#else
  Serial.printf("Reset Reason: %d\n", resetCode);
#endif
  Serial.printf("Version: %s\n", VERSION);
  Serial.printf("Device ID: %s\n", mqtt_cls+sizeof(MQTT_CLASS));
  
  if (strlen(ssid) == 0)
  {
      WiFi.mode(WIFI_AP);
      
      uint32_t t_start = micros();
      while (micros() - t_start <= 100) yield();
      
#ifdef ARDUINO_ARCH_ESP8266
      WiFi.softAPConfig(IPAddress(192,168,32,1), IPAddress(192,168,32,1), IPAddress(255, 255, 255, 0)); // Set AP Address
#endif
      while(!WiFi.softAP(mqtt_cls)){}; // Startup AP
 
      WiFiServer telnetServer(23);
      WiFiClient telnetClient;

      telnetServer.begin(23);

      Serial.print("Please input SSID: ");
      ssid = msg_buf;
      p = msg_buf;
      chEEP = '\0';
      while (chEEP != '\n')
      {
          if (!telnetClient && (telnetClient = telnetServer.available()))
          {
//              delay(100);
              t_start = micros();
              while (micros() - t_start <= 100) yield();
              while (telnetClient.available()) telnetClient.read();
              telnetClient.printf("Please input SSID: ");
          }
          if (telnetClient)
          {
              if (telnetClient.connected() && telnetClient.available())
              {
                  chEEP = telnetClient.read();
                  if (chEEP == '\b')
                  {
                      *p--;
                      continue;
                  }
                  *p++ = chEEP == '\n' || chEEP == '\r' ? '\0' : chEEP;
              }
          }
          if (Serial.available())
          {
              chEEP = Serial.read();
              *p++ = chEEP == '\n' || chEEP == '\r' ? '\0' : chEEP;
          }
      }
      if (telnetClient)
      {
          telnetClient.print("Please input Password: ");
      } else
      {
          Serial.printf("\n");
          Serial.print("Please input Password: ");
      }
      password = p;
      chEEP = '\0';
      while (chEEP != '\n')
      {
          if (telnetClient || (telnetClient = telnetServer.available()))
          {
              if (telnetClient.connected() && telnetClient.available())
              {
                  chEEP = telnetClient.read();
                  if (chEEP == '\b')
                  {
                      *p--;
                      continue;
                  }
                  *p++ = chEEP == '\n' || chEEP == '\r' ? '\0' : chEEP;
              }
          }
          if (Serial.available())
          {
              chEEP = Serial.read();
              *p++ = chEEP == '\n' || chEEP == '\r' ? '\0' : chEEP;
          }
      }
      if (telnetClient)
      {
          telnetClient.print("Please input MQTT Server & Port: ");
      } else
      {
          Serial.printf("\n");
          Serial.print("Please input MQTT Server & Port: ");
      }
      p = (char *)mqtt_server;
      chEEP = '\0';
      bool isPort = false;
      while (chEEP != '\n')
      {
          chEEP = 0;
          if (telnetClient || (telnetClient = telnetServer.available()))
          {
              if (telnetClient.connected() && telnetClient.available())
              {
                  chEEP = telnetClient.read();   
              }
          }
          if (Serial.available())
          {
              chEEP = Serial.read();
          }
          if (chEEP != 0)
          {
              if (chEEP == '\b')
              {
                  *p--;
                  continue;
              }
              if (chEEP == '\r') continue;
              if (chEEP == ':')
              {
                  *p++ = '\0';
                  isPort = true;
                  port = 0;
              } else if  (isPort) {
                  if (chEEP != '\n') {
                      port = port * 10 + chEEP - '0';
                  }                  
              } else {
                  *p++ = chEEP == '\n' || chEEP == '\r' ? '\0' : chEEP;
              }
          }
      }
      if (telnetClient)
      {
          telnetClient.printf("Input SSID = %s  PASSWORD = %s  MQTT Server: %s:%d\n", ssid, password, mqtt_server, port);
          telnetClient.print("Confirm? [y/n]");
      } else
      {
          Serial.printf("\n");
          Serial.printf("Input SSID = %s  PASSWORD = %s  MQTT Server: %s:%d\n", ssid, password, mqtt_server, port);
          Serial.print("Confirm? [y/n]");
      }
      while (chEEP != 'y')
      {
          chEEP = 0;
          if (telnetClient || (telnetClient = telnetServer.available()))
          {
              if (telnetClient.connected() && telnetClient.available())
              {
                  chEEP = telnetClient.read();        
              }
          }
          if (Serial.available())
          {
              chEEP = Serial.read();
          }
          if (chEEP == 'n')
          {
              rebootSystem();
          }
      }
      Serial.printf("\n");
      CFG_WRITE_STRING(96, ssid, sizeof(msg_buf));
      CFG_WRITE_STRING(128, password, sizeof(msg_buf) - (password - msg_buf));
      CFG_WRITE_STRING(160, (char *)mqtt_server, sizeof(mqtt_server));
      CFG_SAVE();
      if (telnetClient)
      {
          rebootSystem();
      }
  }

  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);
  updatePWMValue(set_state);
  last_state_hold = 0;
  
  ignoreOutput = true;
  Serial.printf("\n");
  Serial.printf("SSID = %s  PASSWORD = %s\n", ssid, password);
  Serial.printf("PWM Frequence = %d   Range = %d - %d   Period = %d\n", PWM_FREQUENCE, PWM_START, PWM_END, PWM_PERIOD);
  Serial.printf("PWM PIN: %d  WARM PIN: %d  MODE: %d  CFG RESET PIN: %d\n", PWM_PIN, PWM_WPIN, OUTPUT_BY_MOSFET, CFG_RESET_PIN);
  Serial.print("Connecting to WiFi");
  uint32_t last_print_dot = millis();
  while (WiFi.status() != WL_CONNECTED) {
//    delay(100);
    updatePWMValue(set_state);
    if (millis() - last_print_dot >= 200)
    {
        Serial.print(".");
        last_print_dot = millis();
    }
    if (!bootCountReset && millis() - boot_time >= 5000)
    {
        bootCountReset = true;
        boot_count_reset();
    }
    yield();
  }
  ignoreOutput = false;
  
  Serial.printf("\n");
  
  IPAddress myAddress = WiFi.localIP();
  Serial.printf("Connected to wifi. My address: ");
  Serial.print(myAddress);
  
  if (PWM_SCR_TRIGGER != 0)
  {
      pinMode(PWM_SCR_TRIGGER, INPUT_PULLUP);
      attachInterrupt(PWM_SCR_TRIGGER, ZC_detect, RISING);       // Enable external interrupt (INT0)
  }
  
  Serial.printf("\nConnecting to %s:%d ", mqtt_server, port);
//  WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 1000);
  client.setServer((const char *)mqtt_server, port);//端口号
  client.setCallback(callback); //用于接收服务器接收的数据
  Serial.printf("Success\n");
  if (LED_BUILTIN != PWM_PIN && LED_BUILTIN != PWM_WPIN)
  {
      digitalWrite(LED_BUILTIN, HIGH);    
  }
}


void callback(char* topic, byte* payload, unsigned int length) {//用于接收数据
  int l=0;
  int p=1;
  hasPacket = true;
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
    save_config = true;
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
    }
    
    if (last_set_state == set_state && last_set_ct == set_ct)
    {
        inSmooth = false;
        Serial.print("Ignore: ");
        updatePWMValue(set_state);
    } else if (!inSmooth)
    {
        Serial.print("DIRECT: ");
        updatePWMValue(set_state);
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
      Serial.printf("Write %d bytes\n", CFG_WRITE_STRING(32, (char *)payload, length));
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
      updatePWMValue(set_state);
      sendMeta();
  } else if (strcmp(topic, "set_pwm_start") == 0)
  {
      char bufferByte = payload[length];
      payload[length] = 0;
      PWM_START = atoi((char *)payload);
      payload[length] = bufferByte;
      CFG_SAVE();
      updatePWMValue(set_state);
      sendMeta();
  } else if (strcmp(topic, "set_pwm_end") == 0)
  {
      char bufferByte = payload[length];
      payload[length] = 0;
      PWM_END = atoi((char *)payload);
      payload[length] = bufferByte;
      CFG_SAVE();
      updatePWMValue(set_state);
      sendMeta();
  } else if (strcmp(topic, "set_pwm_period") == 0)
  {
      char bufferByte = payload[length];
      payload[length] = 0;
      PWM_PERIOD = atoi((char *)payload);
      payload[length] = bufferByte;
      CFG_SAVE();
      
      analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_PIN);
      if (PWM_WPIN != 0)
      {
          analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_WPIN);
      }
      updatePWMValue(set_state);
      sendMeta();
  } else if (strcmp(topic, "set_pwm_pin") == 0)
  {
      payload[length] = 0;
      pwmWrite(PWM_PIN, 0);
      PWM_PIN = atoi((char *)payload);
      analogPinInit(PWM_FREQUENCE, PWM_PERIOD, PWM_PIN);
      updatePWMValue(set_state);
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
      updatePWMValue(set_state);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_pwm_mode") == 0)
  {
      payload[length] = 0;
      OUTPUT_BY_MOSFET = atoi((char *)payload);
      updatePWMValue(set_state);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_pwm_trigger") == 0)
  {
      payload[length] = 0;
      PWM_SCR_TRIGGER = atoi((char *)payload);
      updatePWMValue(set_state);
      CFG_SAVE();
      sendMeta();
      rebootSystem();
  } else if (strcmp(topic, "set_pwm_delay") == 0)
  {
      payload[length] = 0;
      PWM_SCR_DELAY = atoi((char *)payload);
      updatePWMValue(set_state);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_config_pin") == 0)
  {
      payload[length] = 0;
      CFG_RESET_PIN = atoi((char *)payload);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_en_pin") == 0)
  {
      payload[length] = 0;
      EN_PORT = atoi((char *)payload);
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "set_auto_full_power") == 0)
  {
      payload[length] = 0;
      PWM_AUTO_FULL_POWER = atoi((char *)payload);
      updatePWMValue(set_state);
      CFG_SAVE();
      sendMeta();      
  } else if (strcmp(topic, "reset") == 0)
  {
      if (length > 0 && payload[0] == '1')
      {
          CFG_INIT(true);  
      } else
      {
          CFG_INIT(false);
      }
      Serial.printf("Reset System\n");
  } else if (strcmp(topic, "reboot") == 0)
  {
      rebootSystem();
  }
}

void sendMeta()
{
    last_send_meta = millis();
  // max length = 64 - 21
    char *p = msg_buf + sprintf(msg_buf, "{\"name\":\"");
    p += CFG_READ_STRING(32, p, 32);
    p = p + sprintf(p, "\",\"pwm_freq\":%d,\"pwm_start\":%d,\"pwm_end\":%d,\"period\":%d}", PWM_FREQUENCE, PWM_START, PWM_END, PWM_PERIOD);
    client.publish("dev", msg_buf);
    p = msg_buf + sprintf(msg_buf, "{\"pin\":%d,\"wpin\":%d,\"mode\":%d,\"cfg_pin\":%d,\"auto_full_power\":%d}", PWM_PIN, PWM_WPIN, OUTPUT_BY_MOSFET, CFG_RESET_PIN, PWM_AUTO_FULL_POWER);
    client.publish("dev", msg_buf);
}

void reconnect() {//等待，直到连接上服务器
  if (WiFi.status() != WL_CONNECTED) {
      Serial.printf("WiFi: DISCONNECTED, RESET SYSTEM\n");
      rebootSystem();
  }
  if (!client.connected()){
      Serial.print("Connecting to MQTT server...");
  }
  while (!client.connected()) {//如果没有连接上
    if (client.connect(mqtt_cls)) {//接入时的用户名，尽量取一个很不常用的用户名
      retry_failed_count = 0;
      Serial.printf(" success, login by: %s\n",mqtt_cls);//连接失
      sendMeta();
      last_rssi = -1;
      last_state = -1;
    } else {
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
uint32_t last_zc_interval = 0;
void loop() {
   hasPacket = false;
   if (ZC)
   {
        if (OUTPUT_BY_MOSFET == 2)
        {
            int32_t delay_us = PWM_SCR_DELAY + float(current_bright) / PWM_PERIOD * zc_interval;
            while (micros() - zc_last < delay_us);
            if (current_bright != PWM_PERIOD)
            {
                digitalWrite(PWM_PIN, HIGH);
            }            
            if (zc_interval - (PWM_SCR_DELAY + float(current_bright) / PWM_PERIOD * zc_interval) - 1000 > 0)
            {
                delayMicroseconds(zc_interval - (PWM_SCR_DELAY + float(current_bright) / PWM_PERIOD * zc_interval) - 1000);
            }
            if (current_bright != 0)
            {
                digitalWrite(PWM_PIN, LOW);
            }
//            pwmWrite(PWM_PIN, PWM_PERIOD - current_bright); 
//            delayMicroseconds(1000);  // 100us to activate MOC3021
//            digitalWrite(PWM_PIN, LOW);
        } else {
          
            if (PWM_SCR_DELAY > 0)
            {
                delayMicroseconds(PWM_SCR_DELAY);
            }
            analogPinInit(1000000 / zc_interval, PWM_PERIOD, PWM_PIN);
            pwmWrite(PWM_PIN, current_bright); 
        }
        ZC = 0;
        if (abs(zc_interval - last_zc_interval) >= 2000)
        {
//            Serial.printf("Zero Detect Interval: %d\n", zc_interval);
            last_zc_interval = zc_interval;
        }
   }
    if (!bootCountReset && millis() - boot_time >= 5000)
    {
        bootCountReset = true;
        boot_count_reset();
    }
   reconnect();//确保连上服务器，否则一直等待。
   client.loop();//MUC接收数据的主循环函数。
   long rssi = WiFi.RSSI();
   if (inSmooth && last_state_hold != 0 && millis() - last_state_hold <= SMOOTH_INTERVAL && set_state != -1)
   {
      if (OUTPUT_BY_MOSFET != 2)
        Serial.print("Smooth: ");
      updatePWMValue((last_set_state - (float)(last_set_state - set_state) * (millis() - last_state_hold) / SMOOTH_INTERVAL));
   }
   if (inSmooth && millis() - last_state_hold > SMOOTH_INTERVAL && set_state != -1)
   {
      if (OUTPUT_BY_MOSFET != 2)
        Serial.print("Final: ");
      updatePWMValue(set_state);
      inSmooth = false;
   }
   if (last_state_hold != 0 && set_state > 0 && !inSmooth && !inTrigger && millis() - last_state_hold >= 2000)
   {
      if (save_config && OUTPUT_BY_MOSFET != 2){
        Serial.printf("Saving config to FLASH...");
        CFG_SAVE();
        Serial.printf("Done\n");
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
          sprintf(msg_buf, "{\"bright\":%d,\"ct\":%d,\"rssi\":%ld,\"version\":\"%s\"}", set_state, set_ct, rssi, VERSION);
      } else {
          sprintf(msg_buf, "{\"bright\":%d,\"rssi\":%ld,\"version\":\"%s\"}", set_state, rssi, VERSION);
      }
      client.publish("status", msg_buf);
   }

   if (millis() - last_send_meta >= 60000)
   {
      if (OUTPUT_BY_MOSFET != 2)
          Serial.printf("PING %ld  WiFI: %d\n", millis(), WiFi.status());
      sendMeta();
   }

//   if (!hasPacket)
//   {
//      delay(20);
//   }
}
