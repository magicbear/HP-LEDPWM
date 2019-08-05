#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <ESP8266httpUpdate.h>
#include <EEPROM.h>

#include "config.h"

extern "C"{
  #include "pwm.h"
}

// CONFIG: USING INTERRUPT PWM instand for SDK PWM
//#define USING_CUSTOM_PWM
// CONFIG: REVERSE THE OUTPUT FOR MOSFET+BJT Driver
#define OUTPUT_BY_MOSFET
// CONFIG: SMOOTH PWM INTERVAL IN mS
#define SMOOTH_INTERVAL 200
// CONFIG: TOTAL PWM channels (When using DUAL ColorTemperature strip will set to 2)
#define PWM_CHANNELS 2
// CONFIG: Warm / Single LED PWM Pin  (ESP-01  RXD0)
#define PWM_PIN 3
// CONFIG: Cold LED PWM Pin
//#define PWM_WPIN 2

#ifdef USING_CUSTOM_PWM
  // PWM setup (choice all pins that you use PWM)
  uint32 io_info[PWM_CHANNELS][3] = {
    // MUX, FUNC, PIN
    {PERIPHS_IO_MUX_U0RXD_U,  FUNC_GPIO3, 3}, // D3
    {PERIPHS_IO_MUX_GPIO2_U,  FUNC_GPIO2, 2} // D2
  };
  
  // PWM initial duty: all off
  uint32 pwm_duty_init[PWM_CHANNELS];
#endif

#ifndef MQTT_CLASS
#define MQTT_CLASS "HP-LEDPWM"
#define VERSION "1.91"
#endif

const byte BUILTIN_LED1 = 1; //GPIO0

#ifdef USING_CUSTOM_PWM
uint16_t ACCEPT_FACTOR[] = {
  1, 2, 4, 5, 8, 10, 16, 20, 25, 32, 40, 50, 64, 80, 100, 125, 160, 200, 250, 320, 400, 500, 625, 800, 1000, 1250, 1600, 2000, 2500, 3125, 4000, 5000, 6250, 8000, 10000, 12500, 15625, 20000, 25000, 31250, 40000, 50000, 62500
};
#endif
// Period of PWM frequency -> default of SDK: 5000 -> * 200ns ^= 1 kHz
uint16_t PWM_PERIOD = 250;
uint16_t PWM_FREQUENCE = 20000;

uint16_t PWM_START = 0;
uint16_t PWM_END = PWM_PERIOD;

uint16_t set_ct = 4000;
uint8_t  set_state = 0;
uint16_t port = 1883;//服务器端口号

char* ssid;
char* password;
char* mqtt_server[16];//服务器的地址 
char mqtt_cls[sizeof(MQTT_CLASS) + 13];
char msg_buf[128];

const hp_cfg_t def_cfg[] = {
  {15, sizeof(uint8_t), (uint8_t)10,      &set_state},       // BRIGHT
  {16, sizeof(uint8_t), (uint8_t)0, NULL},                   // NAME LENGTH
  {64, sizeof(uint16_t), (uint16_t)20000, &PWM_FREQUENCE},   // UINT16: PWM FREQUENCE
  {66, sizeof(uint16_t), (uint16_t)0,     &PWM_START},       // UINT16: PWM START
  {68, sizeof(uint16_t), (uint16_t)250,   &PWM_END},         // UINT16: PWM END
  {70, sizeof(uint16_t), (uint16_t)4100,  &set_ct},          // UINT16: COLOR TEMPERATURE
  {96, sizeof(uint8_t), (uint8_t)0, NULL},                   // STRING: SSID
  {128, sizeof(uint8_t), (uint8_t)0, NULL},                  // STRING: WIFI PASSWORD 
  {160, sizeof(mqtt_server), (uint8_t)0, mqtt_server},       // STRING: MQTT SERVER
  {192, sizeof(uint16_t), (uint16_t)1883, &port},            // UINT16: PORT
};

WiFiClient espClient;
PubSubClient client(espClient);

bool inSmooth = false;
char last_state = -1;
int last_set_state = 0;
long last_rssi = -1;
unsigned long last_send_rssi;
unsigned long last_state_hold;
bool otaMode = true;                             //OTA mode flag
bool hasPacket = false;

void updatePWMValue(int set_state)
{
    int pwm_state = map(set_state, 0, 100, PWM_START, PWM_END);
    if (pwm_state == PWM_PERIOD-PWM_START) pwm_state = PWM_PERIOD;
    if (pwm_state == PWM_PERIOD-PWM_END) pwm_state = 0;
#ifdef PWM_WPIN
    float set_power = 0.5 + (set_ct - 4100) / 3000.;

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

  Serial.printf("Set Power Power %d PWM = %d  CT: %d WARM: %d%% %d  COLD: %d%% %d\n", set_state, pwm_state, set_ct, (int)(warm_power * 100), (int)(pwm_state * warm_power), (int)(cold_power * 100), (int)(pwm_state * cold_power));

  
#ifdef OUTPUT_BY_MOSFET

  #ifdef USING_CUSTOM_PWM
      pwm_set_duty(PWM_PERIOD - pwm_state * warm_power, 0);
      pwm_set_duty(PWM_PERIOD - pwm_state * cold_power, 1);
      pwm_start(); // commit
  #else
      analogWrite(PWM_PIN, PWM_PERIOD - pwm_state * warm_power);
      analogWrite(PWM_WPIN, PWM_PERIOD - pwm_state * cold_power);
  #endif
#else

  #ifdef USING_CUSTOM_PWM
      pwm_set_duty(pwm_state * warm_power, 0);
      pwm_set_duty(pwm_state * cold_power, 1);
      pwm_start(); // commit
  #else
      analogWrite(PWM_PIN, pwm_state * warm_power);
      analogWrite(PWM_WPIN, pwm_state * cold_power);
  #endif
#endif

#else

#ifdef OUTPUT_BY_MOSFET
  pwm_state = PWM_PERIOD - pwm_state;
#endif

  Serial.printf("Set Power Power %d  => %d\n", set_state, pwm_state);
  #ifdef USING_CUSTOM_PWM
      pwm_set_duty(pwm_state, 0);
      pwm_start(); // commit
  #else
      analogWrite(PWM_PIN, pwm_state);
  #endif
#endif
}

void setup() {
  pinMode(BUILTIN_LED1, OUTPUT);
  digitalWrite(BUILTIN_LED1, LOW);
  EEPROM.begin(256);
  
  pinMode(PWM_PIN, OUTPUT);
#ifdef PWM_WPIN
  pinMode(PWM_WPIN, OUTPUT);
#endif
  
  Serial.begin(115200);
  byte mac[6];
  WiFi.macAddress(mac);
  sprintf(mqtt_cls, MQTT_CLASS"-%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  for (int i = 0; i < sizeof(MQTT_CLASS) - 1; i++)
  {
    if (EEPROM.read(i) != mqtt_cls[i])
    {
      CFG_INIT();
      break;
    }
  }
  CFG_LOAD();
  if (PWM_FREQUENCE == 0)
  {
      CFG_INIT();
      CFG_LOAD();
  }

#ifdef USING_CUSTOM_PWM
  if (5000000 % PWM_FREQUENCE == 0)
  {
      PWM_PERIOD = 5000000 / PWM_FREQUENCE;
  } else 
  {
      PWM_PERIOD = 320;
      PWM_FREQUENCE = 5000000 / PWM_PERIOD;
  }  
#else
  analogWriteFreq(PWM_FREQUENCE);  // 10kHz
  analogWriteRange(PWM_PERIOD);
#endif

  WiFi.mode(WIFI_STA);
//  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);

  char *p;
  char chEEP;
  int iOffset;
  int rc = CFG_READ_STRING(96, msg_buf, sizeof(msg_buf));
  if (rc == -1)
  {
      CFG_INIT();
      ESP.reset();
  }
  ssid = msg_buf;  
  password = msg_buf + rc + 1;
  
  rc = CFG_READ_STRING(128, msg_buf + rc + 1, sizeof(msg_buf) - rc - 1);
  if (rc == -1)
  {
      CFG_INIT();
      ESP.reset();
  }
  
  Serial.println();
  Serial.printf("SSID = %s  PASSWORD = %s\n", ssid, password);
  if (strlen(ssid) == 0)
  {
      Serial.print("Please input SSID: ");
      ssid = msg_buf;
      p = msg_buf;
      chEEP = '\0';
      while (chEEP != '\n')
      {
          if (Serial.available())
          {
              chEEP = Serial.read();
              *p++ = chEEP == '\n' ? '\0' : chEEP;
          }
      }
      Serial.println();
      Serial.print("Please input Password: ");
      password = p;
      chEEP = '\0';
      while (chEEP != '\n')
      {
          if (Serial.available())
          {
              chEEP = Serial.read();
              *p++ = chEEP == '\n' ? '\0' : chEEP;
          }
      }
      Serial.println();
      Serial.print("Please input MQTT Server & Port: ");
      p = (char *)mqtt_server;
      chEEP = '\0';
      bool isPort = false;
      while (chEEP != '\n')
      {
          if (Serial.available())
          {
              chEEP = Serial.read();
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
                  *p++ = chEEP == '\n' ? '\0' : chEEP;
              }
          }
      }
      Serial.println();
      Serial.printf("Input SSID = %s  PASSWORD = %s  MQTT Server: %s:%d\n", ssid, password, mqtt_server, port);
      Serial.print("Confirm? [y/n]");
      while (chEEP != 'y')
      {
          if (Serial.available())
          {
              chEEP = Serial.read();
              if (chEEP == 'n')
              {
                  ESP.reset();
              }
          }
      }
      Serial.println();
      CFG_WRITE_STRING(96, ssid, sizeof(msg_buf));
      CFG_WRITE_STRING(128, password, sizeof(msg_buf) - (password - msg_buf));
      CFG_WRITE_STRING(160, (char *)mqtt_server, sizeof(mqtt_server));
      CFG_SAVE();
  }
  WiFi.begin(ssid, password);

#ifdef USING_CUSTOM_PWM
  // Initial duty -> all off
  for (uint8_t channel = 0; channel < PWM_CHANNELS; channel++) {
    pwm_duty_init[channel] = 0;
  }
  
  // Initialize
  pwm_init(PWM_PERIOD, pwm_duty_init, PWM_CHANNELS, io_info);

  // Commit
  pwm_start();
#endif
  updatePWMValue(set_state);
  last_state_hold = 0;
  
  Serial.println();
  Serial.printf("PWM Frequence = %d   Range = %d - %d\n", PWM_FREQUENCE, PWM_START, PWM_END);
  Serial.printf("Flash: %d\n", ESP.getFlashChipRealSize());
  Serial.printf("Version: %s\n", VERSION);
  Serial.printf("Device ID: %s\n", mqtt_cls+sizeof(MQTT_CLASS));
  Serial.print("Connecting to ");
  Serial.printf("%s:%d ", mqtt_server, port);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
//    while (Serial.available())
//    {
//        if (Serial.read() == 'r') { CFG_INIT(); ESP.reset(); }
//    }
  }
  
  Serial.println();
  
  Serial.print("Connected to wifi. My address:");
  IPAddress myAddress = WiFi.localIP();
  Serial.println(myAddress);  
  
  WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 1000);
  client.setServer((const char *)mqtt_server, port);//端口号
  client.setCallback(callback); //用于接收服务器接收的数据
}


void callback(char* topic, byte* payload, unsigned int length) {//用于接收数据
  int l=0;
  int p=1;
  hasPacket = true;
  if (strcmp(topic, "set_bright") == 0)
  {
    if (!inSmooth)
    {
        last_set_state = set_state;
        inSmooth = true;
    } else {
        last_set_state = last_set_state - (float)(last_set_state - set_state) * (millis() - last_state_hold) / SMOOTH_INTERVAL;
    }
    set_state = atoi((char *)payload);
    updatePWMValue(set_state);
    last_state_hold = millis();
    last_state = -1;
  } else if (strcmp(topic, "ota") == 0)
  {
    WiFiClient ota_client;

    char bufferByte = payload[length];
    payload[length] = 0;
    Serial.print("Start OTA from URL: ");
    Serial.println((char *)payload);
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
    Serial.printf("Write %d bytes\n", CFG_WRITE_STRING(16, (char *)payload, length));
    sendMeta();
  } else if (strcmp(topic, "set_pwm_freq") == 0)
  {
    char bufferByte = payload[length];
    payload[length] = 0;
    PWM_FREQUENCE = strtoul((char *)payload, NULL, 10);
    
#ifdef USING_CUSTOM_PWM
    if (5000000 % PWM_FREQUENCE == 0)
    {
        PWM_PERIOD = 5000000 / PWM_FREQUENCE;
    } else 
    {
        for (int i = sizeof(ACCEPT_FACTOR) / sizeof(uint16_t) - 1; i >= 0; i--)
        {
            if (ACCEPT_FACTOR[i] < PWM_FREQUENCE)
            {
                PWM_FREQUENCE = ACCEPT_FACTOR[i];
                PWM_PERIOD = 5000000 / PWM_FREQUENCE;
                break;
            }
        }
    }    
#endif
    payload[length] = bufferByte;
    CFG_SAVE();
#ifdef USING_CUSTOM_PWM
    ESP.reset();
#else
    analogWriteFreq(PWM_FREQUENCE);  // 10kHz
#endif
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

#ifdef PWM_WPIN
  } else if (strcmp(topic, "set_ct_abx") == 0)
  {
    set_ct = atoi((char *)payload);
    updatePWMValue(set_state);
    last_state_hold = millis();
    last_state = -1;
#endif
  } else if (strcmp(topic, "reset") == 0)
  {
    ESP.reset();
  }
}

void sendMeta()
{
  // max length = 64 - 21
    char *p = msg_buf + sprintf(msg_buf, "{\"name\":\"");
    p += CFG_READ_STRING(16, p, 32);
    p = p + sprintf(p, "\",\"pwm_freq\":%d,\"pwm_start\":%d,\"pwm_end\":%d}", PWM_FREQUENCE, PWM_START, PWM_END);
    client.publish("dev", msg_buf);
}

void reconnect() {//等待，直到连接上服务器
  if (!client.connected()){
      Serial.print("Connecting to MQTT server");    
  }
  while (!client.connected()) {//如果没有连接上
    if (client.connect(mqtt_cls)) {//接入时的用户名，尽量取一个很不常用的用户名
      Serial.println(" success");//连接失
      sendMeta();
      last_rssi = -1;
      last_state = -1;
    } else {
      Serial.print(" failed, rc=");//连接失败
      Serial.print(client.state());//重新连接
      Serial.println(" try again in 5 seconds");//延时5秒后重新连接
      delay(5000);
      ESP.reset();
    }
  }
}

int buflen = 0;
void loop() {
   hasPacket = false;
   reconnect();//确保连上服务器，否则一直等待。
   client.loop();//MUC接收数据的主循环函数。
   long rssi = WiFi.RSSI();
   if (inSmooth && last_state_hold != 0 && millis() - last_state_hold <= SMOOTH_INTERVAL && set_state != -1)
   {      
      updatePWMValue((int)(last_set_state - (float)(last_set_state - set_state) * (millis() - last_state_hold) / SMOOTH_INTERVAL));
   }
   if (inSmooth && millis() - last_state_hold > SMOOTH_INTERVAL && set_state != -1)
   {
      updatePWMValue(set_state);
      inSmooth = false;
   }
   if (last_state_hold != 0 && millis() - last_state_hold >= 2000 && set_state != -1)
   {
      CFG_SAVE();
      last_state_hold = 0;
   }
   if (last_state != set_state || (abs(rssi - last_rssi) >= 3 && millis() - last_send_rssi >= 5000))
   {
      last_send_rssi = millis();
      last_state = set_state;
      last_rssi = rssi;
#ifdef PWM_WPIN
      sprintf(msg_buf, "{\"bright\":%d,\"ct\":%d,\"rssi\":%ld,\"version\":\"%s\"}", set_state, set_ct, rssi, VERSION);
#else
      sprintf(msg_buf, "{\"bright\":%d,\"rssi\":%ld,\"version\":\"%s\"}", set_state, rssi, VERSION);
#endif
      client.publish("status", msg_buf);
   }

   if (!hasPacket)
   {
      delay(20);
   }
}
