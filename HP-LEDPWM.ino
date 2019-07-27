#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>

#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <EEPROM.h>

extern "C"{
  #include "pwm.h"
}

#define USING_CUSTOM_PWM
//#define OUTPUT_BY_MOSFET

// Period of PWM frequency -> default of SDK: 5000 -> * 200ns ^= 1 kHz
#define PWM_PERIOD 320

// PWM channels
#define PWM_CHANNELS 2

#ifdef USING_CUSTOM_PWM
  // PWM setup (choice all pins that you use PWM)
  uint32 io_info[PWM_CHANNELS][3] = {
    // MUX, FUNC, PIN
    {PERIPHS_IO_MUX_U0RXD_U,  FUNC_GPIO3, 3}, // D3
    {PERIPHS_IO_MUX_GPIO2_U,  FUNC_GPIO2, 2}, // D2
  };
  
  // PWM initial duty: all off
  uint32 pwm_duty_init[PWM_CHANNELS];
#endif

#ifndef MQTT_CLASS
#define MQTT_CLASS "HP-LEDPWM"
#define VERSION "1.6"
#endif

// Warm / Single LED PWM Pin  (ESP-01  RXD0)
#define PWM_PIN 3
// Cold LED PWM Pin
#define PWM_WPIN 2
const byte BUILTIN_LED1 = 1; //GPIO0

int PWM_START = 0;
int PWM_END = PWM_PERIOD;
//#define PWM_START 5
//#define PWM_END 18

#ifndef USING_CUSTOM_PWM
uint16_t PWM_FREQUENCE = 25000;
#endif

char* ssid;
char* password;
char* mqtt_server[16];//服务器的地址 
int port = 1883;//服务器端口号
char mqtt_cls[sizeof(MQTT_CLASS) + 13];
char msg_buf[64];

WiFiClient espClient;
PubSubClient client(espClient);

char last_state = -1;
int set_state = 0;
#ifdef PWM_WPIN
int set_ct = 4000;
#endif
long last_rssi = -1;
unsigned long last_send_rssi;
unsigned long last_state_hold;
bool otaMode = true;                             //OTA mode flag
bool hasPacket = false;

void updatePWMValue(int set_state)
{
#ifdef OUTPUT_BY_MOSFET
    int pwm_state = map(set_state, 0, 100, PWM_PERIOD - PWM_START, PWM_PERIOD - PWM_END);
#else
    int pwm_state = map(set_state, 0, 100, PWM_START, PWM_END);
#endif
    if (pwm_state == PWM_PERIOD-PWM_START) pwm_state = PWM_PERIOD;
    if (pwm_state == PWM_PERIOD-PWM_END) pwm_state = 0;
#ifdef PWM_WPIN
    float set_power = 0.5 + (set_ct - 4100) / 3000.;

    float warm_power = (1-set_power);
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

  Serial.printf("Set Power Power %d  CT: %d WARM: %d  COLD: %d\n", set_state, set_ct, (int)(pwm_state * warm_power), (int)(pwm_state * cold_power));
  #ifdef USING_CUSTOM_PWM
      pwm_set_duty(pwm_state * warm_power, 0);
      pwm_set_duty(pwm_state * cold_power, 1);
      pwm_start(); // commit
  #else
      analogWrite(PWM_PIN, pwm_state * warm_power);
      analogWrite(PWM_WPIN, pwm_state * cold_power);

  #endif
#else
    analogWrite(PWM_PIN, pwm_state);
#endif
}

void setup() {
  pinMode(BUILTIN_LED1, OUTPUT);
  digitalWrite(BUILTIN_LED1, LOW);
  EEPROM.begin(256);
  for (int i = 0; i < sizeof(MQTT_CLASS) - 1; i++)
  {
    if (EEPROM.read(i) != mqtt_cls[i])
    {
      initEEPROM();
      break;
    }
  }
#ifndef USING_CUSTOM_PWM
  EEPROM.get(64, PWM_FREQUENCE);
  if (PWM_FREQUENCE == 0)
  {
      initEEPROM();
      EEPROM.get(64, PWM_FREQUENCE);
  }
#endif

  PWM_START = EEPROM.read(66);
  PWM_END = EEPROM.read(67);
#ifdef PWM_WPIN
  EEPROM.get(68, set_ct);
#endif
#ifndef USING_CUSTOM_PWM
  analogWriteFreq(PWM_FREQUENCE);  // 10kHz
  analogWriteRange(PWM_PERIOD);
#endif

  pinMode(PWM_PIN, OUTPUT);
#ifdef PWM_WPIN
  pinMode(PWM_WPIN, OUTPUT);
#endif
  
  Serial.begin(115200);
  byte mac[6];
  WiFi.macAddress(mac);
  sprintf(mqtt_cls, MQTT_CLASS"-%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  WiFi.mode(WIFI_STA);
//  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);

  int iOffset = 96;
  char *p = msg_buf;
  char chEEP;
  while (chEEP = EEPROM.read(iOffset++))
  {
      *p++ = chEEP;
  }
  *p++ = '\0';

  ssid = msg_buf;
  password = p;

  iOffset = 128;
  while (chEEP = EEPROM.read(iOffset++))
  {
      *p++ = chEEP;
  }
  *p++ = '\0';

  p = (char *)mqtt_server;
  iOffset = 160;
  while (chEEP = EEPROM.read(iOffset++))
  {
      *p++ = chEEP;
  }
  *p = '\0';

  EEPROM.get(192, port);
    
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
      iOffset = 96;
      p = ssid;
      while (*p != '\0') EEPROM.write(iOffset++, *p++);
      EEPROM.write(iOffset, 0);

      p = password;
      iOffset = 128;
      while (*p != '\0') EEPROM.write(iOffset++, *p++);
      EEPROM.write(iOffset, 0);

      p = (char *)mqtt_server;
      iOffset = 160;
      while (*p != '\0') EEPROM.write(iOffset++, *p++);
      EEPROM.write(iOffset, 0);

      EEPROM.put(192, port);
      EEPROM.commit();
  }
  WiFi.begin(ssid, password);

  set_state = EEPROM.read(15);

#ifdef USING_CUSTOM_PWM
  // Initial duty -> all off
  for (uint8_t channel = 0; channel < PWM_CHANNELS; channel++) {
    pwm_duty_init[channel] = 0;
  }
  
  // Period
  uint32_t period = PWM_PERIOD;

  // Initialize
  pwm_init(period, pwm_duty_init, PWM_CHANNELS, io_info);

  // Commit
  pwm_start();
#endif
  updatePWMValue(set_state);
  last_state_hold = 0;
  
  Serial.println();
  Serial.printf("Flash: %d\n", ESP.getFlashChipRealSize());
  Serial.printf("Version: %s\n", VERSION);
  Serial.printf("Device ID: %s\n", mqtt_cls+sizeof(MQTT_CLASS));
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  
  Serial.print("Connected to wifi. My address:");
  IPAddress myAddress = WiFi.localIP();
  Serial.println(myAddress);  
  
  WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 1000);
  client.setServer((const char *)mqtt_server, port);//端口号
  client.setCallback(callback); //用于接收服务器接收的数据
}


void initEEPROM()
{
  for (int i = 0; i < sizeof(MQTT_CLASS) - 1; i++)
  {
    EEPROM.write(i, mqtt_cls[i]);
  }
  EEPROM.write(15, 10);
  EEPROM.write(16, 0);

  EEPROM.write(96, 0);  // SSID
  EEPROM.write(128, 0);  // PASSWORD
  EEPROM.write(160, 0);  // MQTT Server
  port = 1883;
  EEPROM.put(192, port);  // MQTT Server
  
  // PWM FREQUENCE
  EEPROM.put(64, (int16_t)20000);
  // PWM START
  EEPROM.write(66, 0);
  // PWM END
  EEPROM.write(67, PWM_PERIOD);
  EEPROM.commit();
}

char *loadEEPName(char *buffer)
{
    uint8_t len = EEPROM.read(16);
    for (uint8_t i = 0; i < len; i++)
    {
        buffer[i] = EEPROM.read(i + 17);
    }
    return buffer + len;
}


void callback(char* topic, byte* payload, unsigned int length) {//用于接收数据
  int l=0;
  int p=1;
  hasPacket = true;
  if (strcmp(topic, "set_bright") == 0)
  {
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
    if (length > 64 - 21 - EEPROM.read(64))
    {
      length = 64 - 21 - EEPROM.read(64);
    }
    EEPROM.write(16, length);
    for (int i = 0; i < length;i++)
    {
      EEPROM.write(17+i, payload[i]);
    }
    EEPROM.commit();
    sendMeta();
#ifndef USING_CUSTOM_PWM
  } else if (strcmp(topic, "set_pwm_freq") == 0)
  {
    char bufferByte = payload[length];
    payload[length] = 0;
    PWM_FREQUENCE = strtoul((char *)payload, NULL, 10);
    payload[length] = bufferByte;
    EEPROM.put(64, PWM_FREQUENCE);
    EEPROM.commit();
    analogWriteFreq(PWM_FREQUENCE);  // 10kHz
    updatePWMValue(set_state);
    sendMeta();
#endif
  } else if (strcmp(topic, "set_pwm_start") == 0)
  {
    char bufferByte = payload[length];
    payload[length] = 0;
    PWM_START = atoi((char *)payload);
    EEPROM.write(66, PWM_START);
    payload[length] = bufferByte;
    EEPROM.commit();
    updatePWMValue(set_state);
    sendMeta();
  } else if (strcmp(topic, "set_pwm_end") == 0)
  {
    char bufferByte = payload[length];
    payload[length] = 0;
    PWM_END = atoi((char *)payload);
    EEPROM.write(67, PWM_END);
    payload[length] = bufferByte;
    EEPROM.commit();
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
    p = loadEEPName(p);
    p = p + sprintf(p, "\","
#ifndef USING_CUSTOM_PWM
    "\"pwm_freq\":%d,"
#endif
    "\"pwm_start\":%d,\"pwm_end\":%d}", 
#ifndef USING_CUSTOM_PWM
PWM_FREQUENCE, 
#endif
    PWM_START, PWM_END);
    client.publish("dev", msg_buf);
}

void reconnect() {//等待，直到连接上服务器
  while (!client.connected()) {//如果没有连接上
    if (client.connect(mqtt_cls)) {//接入时的用户名，尽量取一个很不常用的用户名
      Serial.println("Connect to MQTT server Success!");
      sendMeta();
      last_rssi = -1;
      last_state = -1;
    } else {
      Serial.print("failed, rc=");//连接失败
      Serial.print(client.state());//重新连接
      Serial.println(" try again in 5 seconds");//延时5秒后重新连接
      delay(5000);
      ESP.reset();
    }
  }
}

char buffer[256];
int buflen = 0;
void loop() {
   hasPacket = false;
   reconnect();//确保连上服务器，否则一直等待。
   client.loop();//MUC接收数据的主循环函数。
   long rssi = WiFi.RSSI();
   if (last_state_hold != 0 && millis() - last_state_hold >= 3000 && set_state != -1)
   {
#ifdef PWM_WPIN
      EEPROM.put(68, set_ct);
#endif
      EEPROM.write(15, set_state);
      EEPROM.commit();
      last_state_hold = 0;
   }
   if (last_state != set_state || (abs(rssi - last_rssi) >= 3 && millis() - last_send_rssi >= 5000))
   {
      last_send_rssi = millis();
      last_state = set_state;
      last_rssi = rssi;
#ifdef PWM_WPIN
      sprintf(msg_buf, "{\"bright\":%d,\"ct\":%d,\"rssi\":%ld,\"version\":\"%s\",\"ota\":\"unset\"}", set_state, set_ct, rssi, VERSION);
#else
      sprintf(msg_buf, "{\"bright\":%d,\"rssi\":%ld,\"version\":\"%s\",\"ota\":\"unset\"}", set_state, rssi, VERSION);
#endif
      client.publish("status", msg_buf);
   }

   if (!hasPacket)
   {
      delay(100);
   }
}