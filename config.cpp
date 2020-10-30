#include <EEPROM.h>
#ifdef ARDUINO_ARCH_ESP32
  #include <nvs.h>
  #include <SPIFFS.h>
  #include <WiFi.h>
#else
  #include <FS.h>
  #include <ESP8266WiFi.h>
#endif
#include "config.h"

static char *cfg_buffer = NULL;
enum cfg_storage_backend_t {
    STORAGE_NVS,
    STORAGE_SPIFFS,
    STORAGE_EEPROM
};
static enum cfg_storage_backend_t cfg_backend;

char *dev_name;
char *ssid;
char *password;
char *mqtt_server;//服务器的地址
uint16_t port;//服务器端口号 

#define BUFFER_SIZE 255

#ifdef ESP_NVS_H
nvs_handle nvs;
bool cfg_migrate2nvs(const char *mqtt_cls, const hp_cfg_t *def_value)
{
    Serial.printf("Migrate Config From SPIFFS to NVS\n");

    nvs_set_str(nvs, "class", mqtt_cls);
    
    File f = SPIFFS.open("/config.bin", "r");
    
    if (f.read((uint8_t *)cfg_buffer, BUFFER_SIZE) != BUFFER_SIZE)
    {
        Serial.printf("ERROR: Migrate config failed, buffer size not matched.\n");
        cfg_buffer[0] = '\0';
        f.close();
        SPIFFS.remove("/config.bin");
        return false;
    }
    f.close();
    cfg_backend = STORAGE_SPIFFS;
    cfg_load(def_value);
    cfg_backend = STORAGE_NVS;
    cfg_save(def_value, false, true);
    cfg_confirm();
    SPIFFS.remove("/config.bin");
    return true;
}
#endif

void cfg_begin()
{
#ifdef ARDUINO_ARCH_ESP32
//    nvs_stats_t nvs_stats;
//    nvs_get_stats("nvs", &nvs_stats);
//    Serial.printf("NVS Status: Count: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d)\n",
//          nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries);
    int rc = nvs_open("nvs", NVS_READWRITE, &nvs);
    if (rc == ESP_OK)
    {
        cfg_backend = STORAGE_NVS;
        cfg_buffer = (char *)calloc(BUFFER_SIZE, 1);
        
        bool checkPass = true;
        size_t required_size;
        int rc = nvs_get_str(nvs, "class", NULL, &required_size);
        if (rc == ESP_ERR_NVS_NOT_FOUND)
        {
            checkPass = false;
        } else {
            char *szClass = (char *)malloc(required_size);
            nvs_get_str(nvs, "class", szClass, &required_size);
            free(szClass);
        }
    } else if ((rc == ESP_ERR_NVS_PART_NOT_FOUND || rc == ESP_ERR_NVS_NOT_INITIALIZED) && SPIFFS.begin(true))
    {
        cfg_backend = STORAGE_SPIFFS;
        cfg_buffer = (char *)calloc(BUFFER_SIZE, 1);
        if (SPIFFS.exists("/config.bin"))
        {
            File f = SPIFFS.open("/config.bin", "r");
            if (f.read((uint8_t *)cfg_buffer, BUFFER_SIZE) != BUFFER_SIZE)
            {
                cfg_buffer[0] = '\0';
                f.close();
                SPIFFS.remove("/config.bin");
                return cfg_begin();
            }
            f.close();
        }
    } else 
    {
        cfg_backend = STORAGE_EEPROM;
        EEPROM.begin(256);
    }
#else
    SPIFFS.begin();
    FSInfo fs_info;
    if (!SPIFFS.info(fs_info))
    {
        cfg_backend = STORAGE_EEPROM;
        EEPROM.begin(256);
    } else {
        cfg_backend = STORAGE_SPIFFS;
        cfg_buffer = (char *)calloc(BUFFER_SIZE, 1);
        File f = SPIFFS.open("/config.bin", "r");
        
        if (f.read((uint8_t *)cfg_buffer, BUFFER_SIZE) != BUFFER_SIZE)
        {
            
        }
        f.close();
    }
#endif

}

bool cfg_spiffs()
{
    return cfg_backend == STORAGE_SPIFFS;
}

const char *cfg_get_backend()
{
    if (cfg_backend == STORAGE_NVS)
    {
          return "NVS";
    }else if (cfg_backend == STORAGE_SPIFFS)
    {
          return "SPIFFS";
    }else if (cfg_backend == STORAGE_EEPROM)
    {
          return "EEPROM";
    }
}


bool cfg_check(const char *mqtt_cls, const hp_cfg_t *def_value)
{
#ifdef ESP_NVS_H
    if (cfg_backend == STORAGE_NVS)
    {
        bool checkPass = true;
        size_t required_size;
        int rc = nvs_get_str(nvs, "class", NULL, &required_size);
        if (rc == ESP_ERR_NVS_NOT_FOUND || required_size != strlen(mqtt_cls) + 1)
        {
            checkPass = false;
        } else {
            char *szClass = (char *)malloc(required_size);
            nvs_get_str(nvs, "class", szClass, &required_size);
            if (strncmp(szClass, mqtt_cls, strlen(mqtt_cls)) != 0)
            {
                checkPass = false;
            }
            free(szClass);
        }
        if (!checkPass)
        {
            if (SPIFFS.begin(true) && SPIFFS.exists("/config.bin")) {
              checkPass = cfg_migrate2nvs(mqtt_cls, def_value);
            }
            else cfg_init(mqtt_cls, def_value, true);
        }
        return checkPass;
    } else 
#endif
    if (cfg_backend == STORAGE_SPIFFS)
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
    if (cfg_backend == STORAGE_NVS)
    {
#ifdef ESP_NVS_H
        nvs_set_str(nvs, "class", mqtt_cls);
#endif
    } else if (cfg_backend == STORAGE_SPIFFS)
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
                if (cfg_backend == STORAGE_NVS)
                {
#ifdef ESP_NVS_H
                    nvs_set_u8(nvs, def_value[i].nvs_name, def_value[i].data.uint8);
#endif
                } else if (cfg_backend == STORAGE_SPIFFS)
                {
                    *(uint8_t *)(cfg_buffer + def_value[i].offset) = def_value[i].data.uint8;
                } else 
                {
                    EEPROM.write(def_value[i].offset, def_value[i].data.uint8);
                }                
                break;
            case 2:
                if (cfg_backend == STORAGE_NVS)
                {
#ifdef ESP_NVS_H
                    nvs_set_u16(nvs, def_value[i].nvs_name, def_value[i].data.uint16);
#endif
                } else if (cfg_backend == STORAGE_SPIFFS)
                {
                    *(uint16_t *)(cfg_buffer + def_value[i].offset) = def_value[i].data.uint16;
                } else 
                {
                    EEPROM.put(def_value[i].offset, def_value[i].data.uint16);
                }
                break;
            case 4:
                if (cfg_backend == STORAGE_NVS)
                {
#ifdef ESP_NVS_H
                    nvs_set_u32(nvs, def_value[i].nvs_name, def_value[i].data.uint32);
#endif
                } else if (cfg_backend == STORAGE_SPIFFS)
                {
                    *(uint32_t *)(cfg_buffer + def_value[i].offset) = def_value[i].data.uint32;
                } else 
                {
                    EEPROM.put(def_value[i].offset, def_value[i].data.uint32);
                }
                break;
            default:
                if (cfg_backend == STORAGE_NVS)
                {
#ifdef ESP_NVS_H
                    nvs_set_str(nvs, def_value[i].nvs_name, "");
#endif
                } else if (cfg_backend == STORAGE_SPIFFS)
                {
                    *(uint8_t *)(cfg_buffer + def_value[i].offset) = 0;
                } else 
                {
                    EEPROM.write(def_value[i].offset, 0);
                }
                break;
        }        
    }

    if (cfg_backend == STORAGE_NVS)
    {
#ifdef ESP_NVS_H
        nvs_commit(nvs);
#endif
    } else if (cfg_backend == STORAGE_SPIFFS)
    {
        File f = SPIFFS.open("/config.bin", "w");
#ifdef ARDUINO_ARCH_ESP32
        f.write((uint8_t *)cfg_buffer, BUFFER_SIZE);
#else
        f.write(cfg_buffer, BUFFER_SIZE);
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
    if (cfg_backend == STORAGE_SPIFFS)
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
    if (cfg_backend == STORAGE_SPIFFS)
    {
        File f = SPIFFS.open("/bootcount.txt", "w");
        f.seek(0, SeekSet);
        boot_count = 0;
        f.write((uint8_t *)&boot_count, 2);
        f.close();
    }  
}

uint8_t cfg_read_uint8(const hp_cfg_t *def_value, int index)
{
    uint8_t val;
    if (cfg_backend == STORAGE_NVS)
    {
#ifdef ESP_NVS_H
        if (ESP_OK == nvs_get_u8(nvs, def_value[index].nvs_name, &val)) return val;
#endif
        return def_value[index].data.uint8;
    } else if (cfg_backend == STORAGE_SPIFFS)
    {
        return *(uint8_t *)(cfg_buffer + def_value[index].offset);
    }
    EEPROM.get(def_value[index].offset, val);
    return val;
}


uint16_t cfg_read_uint16(const hp_cfg_t *def_value, int index)
{
    uint16_t val;
    if (cfg_backend == STORAGE_NVS)
    {
#ifdef ESP_NVS_H
        if (ESP_OK == nvs_get_u16(nvs, def_value[index].nvs_name, &val)) return val;
#endif
        return def_value[index].data.uint16;
    } else if (cfg_backend == STORAGE_SPIFFS)
    {
        return *(uint16_t *)(cfg_buffer + def_value[index].offset);
    }
    EEPROM.get(def_value[index].offset, val);
    return val;
}

uint32_t cfg_read_uint32(const hp_cfg_t *def_value, int index)
{
    uint32_t val;
    if (cfg_backend == STORAGE_NVS)
    {
#ifdef ESP_NVS_H
        if (ESP_OK == nvs_get_u32(nvs, def_value[index].nvs_name, &val)) return val;
#endif
        return def_value[index].data.uint32;
    } else if (cfg_backend == STORAGE_SPIFFS)
    {
        return *(uint32_t *)(cfg_buffer + def_value[index].offset);
    }
    EEPROM.get(def_value[index].offset, val);
    return val;
}

int cfg_read_string(const hp_cfg_t *def_value, int index, char *buf, int bufsize)
{
    if (cfg_backend == STORAGE_NVS)
    {
#ifdef ESP_NVS_H
        size_t required_size;
        int rc;
        if (bufsize == 0)
        {
            required_size = def_value[index + 1].offset != 0 ? def_value[index+1].offset - def_value[index].offset : BUFFER_SIZE - def_value[index].offset;
            rc = nvs_get_str(nvs, def_value[index].nvs_name, buf, &required_size);
            if (ESP_OK == rc) return required_size;
        } else 
        {
            required_size = bufsize;
            rc = nvs_get_blob(nvs, def_value[index].nvs_name, buf, &required_size);
            if (ESP_OK == rc) return required_size;
        }
        switch (rc)
        {
            case ESP_ERR_NVS_NOT_FOUND:
                Serial.printf("NVS ERROR: %s ESP_ERR_NVS_NOT_FOUND\n", def_value[index].nvs_name);
                break;
            case ESP_ERR_NVS_INVALID_HANDLE:
                Serial.printf("NVS ERROR: %s ESP_ERR_NVS_INVALID_HANDLE\n", def_value[index].nvs_name);
                break;
            case ESP_ERR_NVS_INVALID_NAME:
                Serial.printf("NVS ERROR: %s ESP_ERR_NVS_INVALID_NAME\n", def_value[index].nvs_name);
                break;
            case ESP_ERR_NVS_INVALID_LENGTH:
                Serial.printf("NVS ERROR: %s ESP_ERR_NVS_INVALID_LENGTH\n", def_value[index].nvs_name);
                break;
        }
        
#endif
        return -1;
    } else if (cfg_backend == STORAGE_SPIFFS)
    {
        strncpy(buf, cfg_buffer + def_value[index].offset, bufsize);
        return strlen(cfg_buffer + def_value[index].offset);
    }
    char chEEP;
    char *p = buf;
    int iAddr = def_value[index].offset;    
    while (chEEP = EEPROM.read(iAddr++))
    {
        *p++ = chEEP;
    
        if (iAddr >= def_value[index].offset + bufsize)
        {
            return -1;
        }
    }
    *p = '\0';

    return p - buf;
}

int cfg_write_string(const hp_cfg_t *def_value, int index, char *buf, int bufsize)
{
    if (cfg_backend == STORAGE_NVS) {
        int rc = -1;
#ifdef ESP_NVS_H
        if (bufsize == 0)
        {
            bufsize = strlen(buf);
            rc = nvs_set_str(nvs, def_value[index].nvs_name, buf);
        } else {
            rc = nvs_set_blob(nvs, def_value[index].nvs_name, buf, bufsize);
        }
        if (rc == ESP_OK) return bufsize;
        else 
#endif
            return rc;
    } else if (cfg_backend == STORAGE_SPIFFS)
    {
        if (bufsize == 0) return strlen(buf);
        strncpy(cfg_buffer + def_value[index].offset, buf, bufsize);
        return strlen(cfg_buffer + def_value[index].offset);
    }
    char *p;
    int iaddr;
    
    p = (char *)buf;
    iaddr = def_value[index].offset;
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
                *(uint8_t *)def_value[i].assign = cfg_read_uint8(def_value, i);
                if (cfg_backend == STORAGE_NVS) {
                    *(uint8_t *)(cfg_buffer + def_value[i].offset) = *(uint8_t *)def_value[i].assign;
                }
                break;
            case 2:
                *(uint16_t *)def_value[i].assign = cfg_read_uint16(def_value, i);
                if (cfg_backend == STORAGE_NVS) {
                    *(uint16_t *)(cfg_buffer + def_value[i].offset) = *(uint16_t *)def_value[i].assign;
                }
                break;
            case 4:
                *(uint32_t *)def_value[i].assign = cfg_read_uint32(def_value, i);
                if (cfg_backend == STORAGE_NVS) {
                    *(uint32_t *)(cfg_buffer + def_value[i].offset) = *(uint32_t *)def_value[i].assign;
                }
                break;
            case 0:
                if (cfg_backend == STORAGE_NVS) {
                    size_t required_size = 0;
                    if (cfg_read_string(def_value, i, (char *)(cfg_buffer + def_value[i].offset), 0) == -1)
                    {
                        Serial.printf("Failed to read NVS key %s\n", def_value[i].nvs_name);
                        return false;
                    }
                }
                *(char **)def_value[i].assign = cfg_buffer + def_value[i].offset;
//                Serial.printf("ENGINE: %s  %s -> %s (OFFSET = %d)\n", cfg_backend == STORAGE_NVS ? "NVS" : "SPIFFS", def_value[i].nvs_name, *(char **)def_value[i].assign, cfg_buffer + def_value[i].offset);
                break;
            default:
                if (cfg_read_string(def_value, i, (char *)def_value[i].assign, def_value[i].size) == -1)
                {
                    return false;
                }
                break;
        }
    }
    return true;
}

void cfg_save(const hp_cfg_t *def_value, bool ignoreString, bool forceSave)
{
    char current_value[32];
//    noInterrupts();
    for (int i = 0; def_value[i].offset != 0; i++)
    {
        if (def_value[i].assign == NULL) continue;
        switch (def_value[i].size)
        {
            case 1:
                if (cfg_backend == STORAGE_NVS)
                {
                    if (*(uint8_t *)(cfg_buffer + def_value[i].offset) == *(uint8_t *)def_value[i].assign && !forceSave) continue;
#ifdef ESP_NVS_H
                    nvs_set_u8(nvs, def_value[i].nvs_name, *(uint8_t *)def_value[i].assign);
#endif
                    *(uint8_t *)(cfg_buffer + def_value[i].offset) = *(uint8_t *)def_value[i].assign;
                }else if (cfg_backend == STORAGE_SPIFFS)
                {
                    *(uint8_t *)(cfg_buffer + def_value[i].offset) = *(uint8_t *)def_value[i].assign;
                } else 
                {
                    EEPROM.put(def_value[i].offset, *(uint8_t *)def_value[i].assign);
                }
                break;
            case 2:
                if (cfg_backend == STORAGE_NVS)
                {
                      if (*(uint16_t *)(cfg_buffer + def_value[i].offset) == *(uint16_t *)def_value[i].assign && !forceSave) continue;
#ifdef ESP_NVS_H
                    nvs_set_u16(nvs, def_value[i].nvs_name, *(uint16_t *)def_value[i].assign);
#endif
                    *(uint16_t *)(cfg_buffer + def_value[i].offset) = *(uint16_t *)def_value[i].assign;
                }else if (cfg_backend == STORAGE_SPIFFS)
                {
                    *(uint16_t *)(cfg_buffer + def_value[i].offset) = *(uint16_t *)def_value[i].assign;
                } else 
                {
                    EEPROM.put(def_value[i].offset, *(uint16_t *)def_value[i].assign);
                }
                break;
            case 4:
                if (cfg_backend == STORAGE_NVS)
                {
                    if (*(uint32_t *)(cfg_buffer + def_value[i].offset) == *(uint32_t *)def_value[i].assign && !forceSave) continue;
#ifdef ESP_NVS_H
                    nvs_set_u32(nvs, def_value[i].nvs_name, *(uint32_t *)def_value[i].assign);
#endif
                    *(uint32_t *)(cfg_buffer + def_value[i].offset) = *(uint32_t *)def_value[i].assign;
                }else if (cfg_backend == STORAGE_SPIFFS)
                {
                    *(uint32_t *)(cfg_buffer + def_value[i].offset) = *(uint32_t *)def_value[i].assign;
                } else 
                {
                    EEPROM.put(def_value[i].offset, *(uint32_t *)def_value[i].assign);
                }
                break;
            case 0:
                if (ignoreString && !forceSave) continue;
#ifdef ESP_NVS_H
                if (cfg_backend == STORAGE_NVS)
                {
                    size_t required_size = 32;
                    if (ESP_OK == nvs_get_str(nvs, def_value[i].nvs_name, current_value, &required_size) && !forceSave)
                    {
                        if (required_size == strlen(*(char **)def_value[i].assign) + 1 && strncmp(current_value, *(char **)def_value[i].assign, required_size) == 0)
                        {
                            continue;
                        }
                    }
                }
#endif
                cfg_write_string(def_value, i, *(char **)def_value[i].assign, 0);
                break;
            default:
                if (ignoreString && !forceSave) continue;
                cfg_write_string(def_value, i, *(char **)def_value[i].assign, def_value[i].size);
                break;
        }
    }
    if (cfg_backend == STORAGE_NVS)
    {
        
    } else if (cfg_backend == STORAGE_SPIFFS)
    {
        File f = SPIFFS.open("/config.bin.bak", "w");
#ifdef ARDUINO_ARCH_ESP32
        f.write((uint8_t *)cfg_buffer, BUFFER_SIZE);
#else
        f.write(cfg_buffer, BUFFER_SIZE);
#endif
        f.close();
    } else 
    {
        EEPROM.commit();
    }
//    interrupts();
}

void cfg_confirm() {  
    if (cfg_backend == STORAGE_NVS)
    {
#ifdef ARDUINO_ARCH_ESP32
        nvs_commit(nvs);
#endif
    } else if (cfg_backend == STORAGE_SPIFFS){
        SPIFFS.remove("/config.bin");
        SPIFFS.rename("/config.bin.bak", "/config.bin");
    }
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

void cfg_reset(const hp_cfg_t *def_value)
{
    if (cfg_backend == STORAGE_NVS) {
#ifdef ESP_NVS_H
        nvs_erase_key(nvs, "class");
        nvs_erase_key(nvs, "boot_count");
        for (int i = 0; def_value[i].offset != 0; i++)
        {
            nvs_erase_key(nvs, def_value[i].nvs_name);
        }
//        nvs_erase_all(nvs);
#endif
    } else if (cfg_backend == STORAGE_SPIFFS)
    {
        SPIFFS.remove("/config.bin");
    } else 
    {
        for (int i = 0; i < BUFFER_SIZE; i++)
        {
            EEPROM.write(i, 0);
        }
    }
}
