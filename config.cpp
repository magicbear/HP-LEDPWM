#include <EEPROM.h>
#ifdef ARDUINO_ARCH_ESP32
  #include <SPIFFS.h>
  #include <WiFi.h>
#else
  #include <FS.h>
  #include <ESP8266WiFi.h>
#endif
#include "config.h"

static char *cfg_buffer = NULL;
static bool cfg_by_spiffs = true;

char *dev_name;
char *ssid;
char *password;
char *mqtt_server;//服务器的地址
uint16_t port;//服务器端口号 

void cfg_begin()
{
#ifdef ARDUINO_ARCH_ESP32
    if(SPIFFS.begin(true)){
        cfg_by_spiffs = true;
        cfg_buffer = (char *)calloc(255, 1);
        File f = SPIFFS.open("/config.bin", "r");
        
        if (f.read((uint8_t *)cfg_buffer, 255) != 255)
        {
            
        }
        f.close();
    } else 
    {
        cfg_by_spiffs = false;
        EEPROM.begin(256);
    }
#else
    SPIFFS.begin();
    FSInfo fs_info;
    if (!SPIFFS.info(fs_info))
    {
        cfg_by_spiffs = false;
        EEPROM.begin(256);
    } else {
        cfg_by_spiffs = true;
        cfg_buffer = (char *)calloc(255, 1);
        File f = SPIFFS.open("/config.bin", "r");
        
        if (f.read((uint8_t *)cfg_buffer, 255) != 255)
        {
            
        }
        f.close();
    }
#endif

}

bool cfg_spiffs()
{
    return cfg_by_spiffs;
}


bool cfg_check(const char *mqtt_cls, const hp_cfg_t *def_value)
{
    if (cfg_by_spiffs)
    {
      if (strncmp(cfg_buffer, mqtt_cls, strlen(mqtt_cls)) != 0)
      {
//        Serial.printf("INVALID CONFIG HEADER, RESET CONFIG FILES: \"%s\" != \"%s\"\n", mqtt_cls, cfg_buffer);
        cfg_init(mqtt_cls, def_value, true);
        return false;
      }
    } else {
      for (int i = 0; i < strlen(mqtt_cls); i++)
      {
          if (EEPROM.read(i) != mqtt_cls[i])
          {
            cfg_init(mqtt_cls, def_value, true);
            break;
          }
      } 
    }
    return true;
}

void cfg_init(const char *mqtt_cls, const hp_cfg_t *def_value, bool full_init)
{
    Serial.printf("Initalize configure\n");
    if (cfg_by_spiffs)
    {
        strcpy(cfg_buffer, mqtt_cls);
    } else {
        for (int i = 0; i < strlen(mqtt_cls); i++)
        {
          EEPROM.write(i, mqtt_cls[i]);
        }
    }
    for (int i = 0; def_value[i].offset != 0; i++)
    {
        if (!full_init && def_value[i].full_init == full_init) continue;
        switch (def_value[i].size)
        {
            case 1:
                if (cfg_by_spiffs)
                {
                    *(uint8_t *)(cfg_buffer + def_value[i].offset) = def_value[i].data.uint8;
                } else 
                {
                    EEPROM.write(def_value[i].offset, def_value[i].data.uint8);
                }                
                break;
            case 2:
                if (cfg_by_spiffs)
                {
                    *(uint16_t *)(cfg_buffer + def_value[i].offset) = def_value[i].data.uint16;
                } else 
                {
                    EEPROM.put(def_value[i].offset, def_value[i].data.uint16);
                }
                break;
            case 4:
                if (cfg_by_spiffs)
                {
                    *(uint32_t *)(cfg_buffer + def_value[i].offset) = def_value[i].data.uint32;
                } else 
                {
                    EEPROM.put(def_value[i].offset, def_value[i].data.uint32);
                }
                break;
            default:
                if (cfg_by_spiffs)
                {
                    *(uint8_t *)(cfg_buffer + def_value[i].offset) = 0;
                } else 
                {
                    EEPROM.write(def_value[i].offset, 0);
                }
                break;
        }        
    }

    if (cfg_by_spiffs)
    {
        File f = SPIFFS.open("/config.bin", "w");
#ifdef ARDUINO_ARCH_ESP32
        f.write((uint8_t *)cfg_buffer, 255);
#else
        f.write(cfg_buffer, 255);
#endif
        f.close();
    } else 
    {
        EEPROM.commit();
    }
}

uint16_t boot_count_increase()
{
    uint16_t boot_count = 0;
    if (cfg_by_spiffs)
    {
        File f = SPIFFS.open("/bootcount.txt", "r");
        if (f)
        {
            f.read((uint8_t *)&boot_count, 2);
            f.close();          
        }
        f = SPIFFS.open("/bootcount.txt", "w");
        boot_count++;
        f.write((uint8_t *)&boot_count, 2);
        f.close();
    }
    return boot_count;
}

void boot_count_reset()
{
    uint16_t boot_count;
    if (cfg_by_spiffs)
    {
        File f = SPIFFS.open("/bootcount.txt", "w");
        f.seek(0, SeekSet);
        boot_count = 0;
        f.write((uint8_t *)&boot_count, 2);
        f.close();
    }  
}

uint8_t cfg_read_uint8(const hp_cfg_t *def_value, int addr)
{
    uint8_t val;
    if (cfg_by_spiffs)
    {
        return *(uint8_t *)(cfg_buffer + addr);
    }
    EEPROM.get(addr, val);
    return val;
}


uint16_t cfg_read_uint16(const hp_cfg_t *def_value, int addr)
{
    uint16_t val;
    if (cfg_by_spiffs)
    {
        return *(uint16_t *)(cfg_buffer + addr);
    }
    EEPROM.get(addr, val);
    return val;
}

uint32_t cfg_read_uint32(const hp_cfg_t *def_value, int addr)
{
    uint32_t val;
    if (cfg_by_spiffs)
    {
        return *(uint32_t *)(cfg_buffer + addr);
    }
    EEPROM.get(addr, val);
    return val;
}

int cfg_read_string(const hp_cfg_t *def_value, int addr, char *buf, int bufsize)
{
    if (cfg_by_spiffs)
    {
        strncpy(buf, cfg_buffer + addr, bufsize);
        return strlen(cfg_buffer + addr);
    }
    char chEEP;
    char *p = buf;
    int iAddr = addr;    
    while (chEEP = EEPROM.read(iAddr++))
    {
        *p++ = chEEP;
    
        if (iAddr >= addr + bufsize)
        {
            return -1;
        }
    }
    *p = '\0';

    return p - buf;
}

int cfg_write_string(const hp_cfg_t *def_value, int addr, char *buf, int bufsize)
{
    if (cfg_by_spiffs)
    {
        strncpy(cfg_buffer + addr, buf, bufsize);
        return strlen(cfg_buffer + addr);
    }
    char *p;
    int iaddr;
    
    p = (char *)buf;
    iaddr = addr;
    while (true)
    {
        if (p - (char *)buf >= bufsize)
        {
            EEPROM.write(iaddr++, 0);
            p++;
            break;
        } else {
            EEPROM.write(iaddr++, *p++);
        }        
        if (*p == '\0') break;
    }
    return p - (char *)buf;
}

bool cfg_load(const hp_cfg_t *def_value)
{
    for (int i = 0; def_value[i].offset != 0; i++)
    {
        if (def_value[i].assign == NULL) continue;
        switch (def_value[i].size)
        {
            case 1:
                *(uint8_t *)def_value[i].assign = cfg_read_uint8(def_value, def_value[i].offset);
                break;
            case 2:
                *(uint16_t *)def_value[i].assign = cfg_read_uint16(def_value, def_value[i].offset);
                break;
            case 4:
                *(uint32_t *)def_value[i].assign = cfg_read_uint32(def_value, def_value[i].offset);
                break;
            case 0:
                *(char **)def_value[i].assign = cfg_buffer + def_value[i].offset;
                break;
            default:
                if (cfg_read_string(def_value, def_value[i].offset, (char *)def_value[i].assign, def_value[i].size) == -1)
                {
                    return false;
                }
                break;
        }
    }
    return true;
}

void cfg_save(const hp_cfg_t *def_value)
{
//    noInterrupts();
    for (int i = 0; def_value[i].offset != 0; i++)
    {
        if (def_value[i].assign == NULL) continue;
        switch (def_value[i].size)
        {
            case 1:
                if (cfg_by_spiffs)
                {
                    *(uint8_t *)(cfg_buffer + def_value[i].offset) = *(uint8_t *)def_value[i].assign;
                } else 
                {
                    EEPROM.put(def_value[i].offset, *(uint8_t *)def_value[i].assign);
                }
                break;
            case 2:
                if (cfg_by_spiffs)
                {
                    *(uint16_t *)(cfg_buffer + def_value[i].offset) = *(uint16_t *)def_value[i].assign;
                } else 
                {
                    EEPROM.put(def_value[i].offset, *(uint16_t *)def_value[i].assign);
                }
                break;
            case 4:
                if (cfg_by_spiffs)
                {
                    *(uint32_t *)(cfg_buffer + def_value[i].offset) = *(uint32_t *)def_value[i].assign;
                } else 
                {
                    EEPROM.put(def_value[i].offset, *(uint32_t *)def_value[i].assign);
                }
                break;
            case 0:
                break;
            default:
                cfg_write_string(def_value, def_value[i].offset, (char *)def_value[i].assign, def_value[i].size);
                break;
        }
    }
    if (cfg_by_spiffs)
    {
        File f = SPIFFS.open("/config.bin.bak", "w");
#ifdef ARDUINO_ARCH_ESP32
        f.write((uint8_t *)cfg_buffer, 255);
#else
        f.write(cfg_buffer, 255);
#endif
        f.close();
    } else 
    {
        EEPROM.commit();
    }
//    interrupts();
}

void cfg_confirm() {  
    SPIFFS.remove("/config.bin");
    SPIFFS.rename("/config.bin.bak", "/config.bin");
}

void startupWifiConfigure(const hp_cfg_t *def_value, char *msg_buf, uint8_t msg_buf_size, char *mqtt_cls)
{
  char *p;
  char chEEP;
  
  if (strlen(ssid) == 0)
  {
      WiFi.mode(WIFI_AP);
      
      uint32_t t_start = micros();
      while (micros() - t_start <= 100) yield();
//      
//#ifdef ARDUINO_ARCH_ESP8266
//      WiFi.softAPConfig(IPAddress(192,168,32,1), IPAddress(192,168,32,1), IPAddress(255, 255, 255, 0)); // Set AP Address
//#endif
      while(!WiFi.softAP(mqtt_cls)){}; // Startup AP
 
      WiFiServer telnetServer(23);
      WiFiClient telnetClient;

      telnetServer.begin(23);

      Serial.print("Please input SSID: ");
      p = ssid;
      chEEP = '\0';
      while (chEEP != '\n')
      {
          if (!telnetClient && (telnetClient = telnetServer.available()))
          {
//              delay(100);
              t_start = micros();
              while (micros() - t_start <= 100) yield();
              while (telnetClient.available()) telnetClient.read();
              telnetClient.printf("Welcome, Device ID: %s\n", mqtt_cls);
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
      p = password;
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
          }else if (Serial.available())
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
              ESP.restart();
          }
      }
      Serial.printf("\n");
//      CFG_WRITE_STRING(96, ssid, sizeof(msg_buf));
//      CFG_WRITE_STRING(128, password, sizeof(msg_buf) - (password - msg_buf));
//      CFG_WRITE_STRING(160, (char *)mqtt_server, sizeof(mqtt_server));
      cfg_save(def_value);
      cfg_confirm();
      if (telnetClient)
      {
          telnetClient.stop();
      }
  }
  WiFi.mode(WIFI_STA);
#ifdef ARDUINO_ARCH_ESP8266
  WiFi.hostname(mqtt_cls);
#elif ARDUINO_ARCH_ESP32
  WiFi.setHostname(mqtt_cls);
#endif
  WiFi.begin(ssid, password);
}

void cfg_reset()
{
    if (cfg_by_spiffs)
    {
        SPIFFS.remove("/config.bin");
    } else 
    {
        for (int i = 0; i < 255; i++)
        {
            EEPROM.write(i, 0);
        }
    }
}
