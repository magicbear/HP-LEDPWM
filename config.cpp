#include <EEPROM.h>
#ifdef ARDUINO_ARCH_ESP32
  #include <nvs.h>
  #include <SPIFFS.h>
  #include <WiFi.h>
  #include <esp_task_wdt.h>
  #include <rom/rtc.h>
//  #include <esp_bt_main.h>
//  #include <esp_bt_device.h>
  #define ESPhttpUpdate httpUpdate
  #include <HTTPUpdate.h>
  #include <esp_wifi.h>
#else
  #include <FS.h>
  #include <ESP8266WiFi.h>
  #include <ESP8266httpUpdate.h>
#endif
#include "config.h"

static char *cfg_buffer = NULL;
static enum cfg_storage_backend_t cfg_backend;
static mqtt_callback on_message = NULL;
static cfg_callback  on_cfgUpdate = NULL;
static char *global_msg_buf = NULL;
static PubSubClient *global_client;

char *dev_name;
char *ssid;
char *password;
char *mqtt_server;//服务器的地址
uint16_t port;//服务器端口号 

#define BUFFER_SIZE 255

#ifdef ESP_NVS_H
nvs_handle nvs;
#endif

void cfg_begin()
{
#ifdef ARDUINO_ARCH_ESP32
#ifdef CFG_DEBUG
    nvs_stats_t nvs_stats;
    nvs_get_stats("nvs", &nvs_stats);
    printLog(LOG_INFO, "NVS Status: Count: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d)\n",
          nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries);
#endif
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
        nvs_close(nvs);
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
    cfg_buffer = (char *)calloc(BUFFER_SIZE, 1);
    if (!SPIFFS.info(fs_info) || true)
    {
        cfg_backend = STORAGE_EEPROM;
        EEPROM.begin(256);
    } else {
        cfg_backend = STORAGE_SPIFFS;
//        if (SPIFFS.exists("/config.bin"))
//          Serial.printf("Config is exists\n");
//        else
//          Serial.printf("Config is not exists\n");
        File f = SPIFFS.open("/config.bin", "r");
        int r;
        if ((r = f.read((uint8_t *)cfg_buffer, BUFFER_SIZE)) != BUFFER_SIZE)
        {
            printLog(LOG_ERROR, "Read config failed, read: %d\n", r);
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

enum cfg_storage_backend_t cfg_get_backend_t()
{
    return cfg_backend;
}

bool cfg_check(const char *mqtt_cls, const hp_cfg_t *def_value)
{
#ifdef ESP_NVS_H
    if (cfg_backend == STORAGE_NVS)
    {
        int rc = nvs_open("nvs", NVS_READWRITE, &nvs);
        if (rc != ESP_OK) return false;
        bool checkPass = true;
        size_t required_size;
        rc = nvs_get_str(nvs, "class", NULL, &required_size);
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
            cfg_init(mqtt_cls, def_value, true);
        }
        nvs_close(nvs);
        return checkPass;
    } else 
#endif
    if (cfg_backend == STORAGE_SPIFFS)
    {
      if (strncmp(cfg_buffer, mqtt_cls, strlen(mqtt_cls)) != 0)
      {
        printLog(LOG_FATAL, "INVALID CONFIG HEADER, RESET CONFIG FILES: \"%s\" != \"%s\"\n", mqtt_cls, cfg_buffer);
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
    printLog(LOG_INFO, "Initalize configure\n");
    if (cfg_backend == STORAGE_NVS)
    {
#ifdef ESP_NVS_H
        int rc = nvs_open("nvs", NVS_READWRITE, &nvs);
        if (rc != ESP_OK) {
            printLog(LOG_FATAL, "ERROR: Initalize NVS core failed\n");
            return;
        }
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
        uint32_t storage_value;
        switch (def_value[i].size)
        {
            case 1:
                storage_value = def_value[i].data.uint8;
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
                storage_value = def_value[i].data.uint16;
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
                storage_value = def_value[i].data.uint32;
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
#ifdef CFG_DEBUG
        if (def_value[i].size == 0)
        {
            printLog(LOG_INFO, "Initalize %s[%d:%d] -> \"%s\"\n", def_value[i].nvs_name, def_value[i].offset, def_value[i].offset+strlen(*(char **)def_value[i].assign), *(char **)def_value[i].assign);
        } else 
        {
            printLog(LOG_INFO, "Initalize %s[%d:%d] -> %ld\n", def_value[i].nvs_name, def_value[i].offset, def_value[i].offset+def_value[i].size, storage_value);
        }
#endif
    }

    if (cfg_backend == STORAGE_NVS)
    {
#ifdef ESP_NVS_H
        nvs_commit(nvs);
        nvs_close(nvs);
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
        printLog(LOG_INFO, "Init save %s\n", SPIFFS.exists("/config.bin") ? "Success" : "Failed");
    } else 
    {
        EEPROM.commit();
    }
}

uint16_t boot_count_increase()
{
    uint8_t boot_count = 0;
#if ARDUINO_ARCH_ESP32
    if (cfg_backend == STORAGE_NVS)
    {
        int rc = nvs_open("nvs", NVS_READWRITE, &nvs);
        if (rc != ESP_OK) return false;
        if (ESP_OK != nvs_get_u8(nvs, "bootcount", &boot_count))
        {
            boot_count = 0;
        }
        boot_count++;
        nvs_set_u8(nvs, "bootcount", boot_count);
        nvs_commit(nvs);
        nvs_close(nvs);
        return boot_count;
    }else 
#endif
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
    } else if (cfg_backend == STORAGE_EEPROM)
    {
        uint8_t tmp_bootcount = 0;
        EEPROM.get(255, tmp_bootcount);
        EEPROM.put(255, tmp_bootcount++);
        EEPROM.commit();
        boot_count = tmp_bootcount;
    }
    return boot_count;
}

void boot_count_reset()
{
    uint16_t boot_count;
#if ARDUINO_ARCH_ESP32
    if (cfg_backend == STORAGE_NVS)
    {
        int rc = nvs_open("nvs", NVS_READWRITE, &nvs);
        if (rc != ESP_OK) return;
        nvs_erase_key(nvs, "bootcount");
        nvs_commit(nvs);
        nvs_close(nvs);
    }else 
#endif
    if (cfg_backend == STORAGE_SPIFFS)
    {
        File f = SPIFFS.open("/bootcount.txt", "w");
        f.seek(0, SeekSet);
        boot_count = 0;
        f.write((uint8_t *)&boot_count, 2);
        f.close();
    } else if (cfg_backend == STORAGE_EEPROM)
    {
        EEPROM.put(255, 0);
        EEPROM.commit();
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
                printLog(LOG_ERROR, "NVS ERROR: %s ESP_ERR_NVS_NOT_FOUND\n", def_value[index].nvs_name);
                break;
            case ESP_ERR_NVS_INVALID_HANDLE:
                printLog(LOG_ERROR, "NVS ERROR: %s ESP_ERR_NVS_INVALID_HANDLE\n", def_value[index].nvs_name);
                break;
            case ESP_ERR_NVS_INVALID_NAME:
                printLog(LOG_ERROR, "NVS ERROR: %s ESP_ERR_NVS_INVALID_NAME\n", def_value[index].nvs_name);
                break;
            case ESP_ERR_NVS_INVALID_LENGTH:
                printLog(LOG_ERROR, "NVS ERROR: %s ESP_ERR_NVS_INVALID_LENGTH\n", def_value[index].nvs_name);
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
        if (bufsize != 0 && iAddr >= def_value[index].offset + bufsize)
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

    if (bufsize == 0) bufsize = strlen(buf) + 1;
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
        if (*(p-1) == '\0') break;
    }
    return p - (char *)buf;
}

bool cfg_load(const hp_cfg_t *def_value)
{
#ifdef ESP_NVS_H
    if (cfg_backend == STORAGE_NVS)
    {
        int rc = nvs_open("nvs", NVS_READWRITE, &nvs);
        if (rc != ESP_OK) return false;
    }
#endif
    for (int i = 0; def_value[i].offset != 0; i++)
    {
        if (def_value[i].assign == NULL) continue;
        uint32_t storage_value;
        switch (def_value[i].size)
        {
            case 1:
                *(uint8_t *)def_value[i].assign = cfg_read_uint8(def_value, i);
                storage_value = *(uint8_t *)def_value[i].assign;
                if (cfg_backend == STORAGE_NVS) {
                    *(uint8_t *)(cfg_buffer + def_value[i].offset) = *(uint8_t *)def_value[i].assign;
                }
                break;
            case 2:
                *(uint16_t *)def_value[i].assign = cfg_read_uint16(def_value, i);
                storage_value = *(uint16_t *)def_value[i].assign;
                if (cfg_backend == STORAGE_NVS) {
                    *(uint16_t *)(cfg_buffer + def_value[i].offset) = *(uint16_t *)def_value[i].assign;
                }
                break;
            case 4:
                *(uint32_t *)def_value[i].assign = cfg_read_uint32(def_value, i);
                storage_value = *(uint32_t *)def_value[i].assign;
                if (cfg_backend == STORAGE_NVS) {
                    *(uint32_t *)(cfg_buffer + def_value[i].offset) = *(uint32_t *)def_value[i].assign;
                }
                break;
            case 0:
                if (cfg_backend == STORAGE_NVS) {
                    size_t required_size = 0;
                    if (cfg_read_string(def_value, i, (char *)(cfg_buffer + def_value[i].offset), 0) == -1)
                    {
                        printLog(LOG_ERROR, "Failed to read NVS key %s\n", def_value[i].nvs_name);
                        goto failed;
                        return false;
                    }
                } else if (cfg_backend == STORAGE_EEPROM) {
                    if (cfg_read_string(def_value, i, (char *)cfg_buffer + def_value[i].offset, 0) == -1)
                    {
                        goto failed;
                        return false;
                    }
                }
                *(char **)def_value[i].assign = cfg_buffer + def_value[i].offset;
//                Serial.printf("ENGINE: %s  %s -> %s (OFFSET = %d)\n", cfg_backend == STORAGE_NVS ? "NVS" : "SPIFFS", def_value[i].nvs_name, *(char **)def_value[i].assign, cfg_buffer + def_value[i].offset);
                break;
            default:
                if (cfg_read_string(def_value, i, (char *)def_value[i].assign, def_value[i].size) == -1)
                {
                    goto failed;
                    return false;
                }
                break;
        }
#ifdef CFG_DEBUG
        if (def_value[i].size == 0)
        {
            printLog(LOG_DEBUG, "Loading %s[%d:%d] -> \"%s\"\n", def_value[i].nvs_name, def_value[i].offset, def_value[i].offset+strlen(*(char **)def_value[i].assign), *(char **)def_value[i].assign);
        } else 
        {
            printLog(LOG_DEBUG, "Loading %s[%d:%d] -> %ld\n", def_value[i].nvs_name, def_value[i].offset, def_value[i].offset+def_value[i].size, storage_value);
        }
#endif
    }
#ifdef ESP_NVS_H
    if (cfg_backend == STORAGE_NVS)
    {
        nvs_close(nvs);
    }
#endif
    return true;
failed:
#ifdef ESP_NVS_H
    if (cfg_backend == STORAGE_NVS)
    {
        nvs_close(nvs);
    }
#endif
    return false;
}

void cfg_save(const hp_cfg_t *def_value, bool ignoreString, bool forceSave)
{
#ifdef ESP_NVS_H
    if (cfg_backend == STORAGE_NVS)
    {
        int rc = nvs_open("nvs", NVS_READWRITE, &nvs);
        if (rc != ESP_OK) return ;
    }
#endif
    char current_value[32];
//    noInterrupts();
    for (int i = 0; def_value[i].offset != 0; i++)
    {
        if (def_value[i].assign == NULL) continue;
        uint32_t storage_value;
        switch (def_value[i].size)
        {
            case 1:
                storage_value = *(uint8_t *)def_value[i].assign;
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
                storage_value = *(uint16_t *)def_value[i].assign;
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
                storage_value = *(uint32_t *)def_value[i].assign;
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
#ifdef CFG_DEBUG
        if (def_value[i].size == 0)
        {
            printLog(LOG_DEBUG, "Saving %s[%d:%d] -> \"%s\"\n", def_value[i].nvs_name, def_value[i].offset, def_value[i].offset+strlen(*(char **)def_value[i].assign), *(char **)def_value[i].assign);
        } else 
        {
            printLog(LOG_DEBUG, "Saving %s[%d:%d] -> %ld\n", def_value[i].nvs_name, def_value[i].offset, def_value[i].offset+def_value[i].size, storage_value);
        }
#endif
    }
    if (cfg_backend == STORAGE_NVS)
    {
#ifdef ARDUINO_ARCH_ESP32
        nvs_commit(nvs);
        nvs_close(nvs);
#endif
    } else if (cfg_backend == STORAGE_SPIFFS)
    {
        File f = SPIFFS.open("/config.bin.bak", "w");
#ifdef ARDUINO_ARCH_ESP32
        f.write((uint8_t *)cfg_buffer, BUFFER_SIZE);
#else
        f.write(cfg_buffer, BUFFER_SIZE);
#endif
        f.close();
        SPIFFS.remove("/config.bin");
        SPIFFS.rename("/config.bin.bak", "/config.bin");
    } else 
    {
        EEPROM.commit();
    }
//    interrupts();
}

void startupWifiConfigure(const hp_cfg_t *def_value, char *msg_buf, uint8_t msg_buf_size, char *mqtt_cls, led_callback led_cb)
{
  char *p;
  char chEEP;

  global_msg_buf = msg_buf;
  if (strlen(ssid) == 0)
  {
      WiFi.mode(WIFI_AP);
      
      uint32_t t_start = micros();
      while (micros() - t_start <= 100) yield();
//      
//#ifdef ARDUINO_ARCH_ESP8266
//      WiFi.softAPConfig(IPAddress(192,168,32,1), IPAddress(192,168,32,1), IPAddress(255, 255, 255, 0)); // Set AP Address
//#endif
//      // if you are connected, scan for available WiFi networks and print the number discovered:
//      Serial.println("** Scan Networks **");
//      byte numSsid = WiFi.scanNetworks();
//      if (numSsid != WIFI_SCAN_FAILED)
//      {
//          Serial.print("Number of available WiFi networks discovered:");
//          Serial.println(numSsid);
//          for (int i = 0; i < numSsid; ++i) {
//              // Print SSID and RSSI for each network found
//              Serial.printf("  %d: ", i + 1);
//              Serial.print(WiFi.SSID(i));
//              Serial.print(" (");
//              Serial.print(WiFi.RSSI(i));
//              Serial.print(")");
//#if ARDUINO_ARCH_ESP32
//              Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
//#endif
//              delay(10);
//          }
//      }
//      
      WiFi.disconnect();
      
      printLog(LOG_INFO, "Startup WiFi Station...\n");
      while(!WiFi.softAP(mqtt_cls)){ delay(10); yield(); }; // Startup AP
      printLog(LOG_INFO, "Startup Telet Server...\n");
      
      uint8_t primaryChan = 6;
      wifi_second_chan_t secondChan = WIFI_SECOND_CHAN_NONE;
      esp_wifi_set_channel(primaryChan, secondChan);

      WiFiServer telnetServer(23);
      WiFiClient telnetClient;

      telnetServer.begin(23);
ssid_input:
      Serial.print("Please input SSID: ");
      p = ssid;
      chEEP = '\0';
      while (chEEP != '\n')
      {
          if (led_cb != NULL)
              led_cb(telnetClient ? CLIENT_CONNECTED : INITALIZE);
          if (!telnetClient && (telnetClient = telnetServer.available()))
          {
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
                  if (chEEP <= 31 && chEEP != '\r' && chEEP != '\n')
                  {
                      telnetClient.printf("Invalid character %d<%c> input, retry input!\n", chEEP, chEEP);
                      telnetClient.printf("Please input SSID: ");
                      goto ssid_input;
                  }
                  *p++ = chEEP == '\n' || chEEP == '\r' ? '\0' : chEEP;
              }
          }
          if (Serial.available())
          {
              chEEP = Serial.read();
              *p++ = chEEP == '\n' || chEEP == '\r' ? '\0' : chEEP;
          }
          delay(1);
          yield();
      }
//      if (strlen(ssid) <= 2 && atoi(ssid) != 0)
//      {
//          strcpy(ssid, WiFi.SSID(atoi(ssid)-1).c_str());
//      }
      if (led_cb != NULL)
          led_cb(CLIENT_CONNECTED);
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
          if (led_cb != NULL)
              led_cb(CLIENT_CONNECTED);
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
                  if (chEEP <= 31 && chEEP != '\r' && chEEP != '\n')
                  {
                      telnetClient.printf("Invalid character %d<%c> input, retry input!\n", chEEP, chEEP);
                      telnetClient.printf("Please input SSID: ");
                      goto ssid_input;
                  }
                  *p++ = chEEP == '\n' || chEEP == '\r' ? '\0' : chEEP;
              }
          }
          if (Serial.available())
          {
              chEEP = Serial.read();
              *p++ = chEEP == '\n' || chEEP == '\r' ? '\0' : chEEP;
          }
          delay(1);
          yield();
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
          if (led_cb != NULL)
              led_cb(CLIENT_CONNECTED);
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
          delay(1);
          yield();
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
          if (led_cb != NULL)
              led_cb(CLIENT_CONNECTED);
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
              if (telnetClient)
              {
                telnetClient.print("\nPlease input SSID: ");
                goto ssid_input;
              }
              ESP.restart();
          }
      }
      Serial.printf("\n");
      cfg_save(def_value);
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
  WiFi.setAutoReconnect(true);
}

void cfg_reset(const hp_cfg_t *def_value)
{
    if (cfg_backend == STORAGE_NVS) {
#ifdef ESP_NVS_H
        int rc = nvs_open("nvs", NVS_READWRITE, &nvs);
        if (rc != ESP_OK) {
            printLog(LOG_ERROR, "Fatal to open NVS\n");
            return;
        }
        nvs_erase_key(nvs, "class");
        nvs_erase_key(nvs, "boot_count");
        for (int i = 0; def_value[i].offset != 0; i++)
        {
            printLog(LOG_DEBUG, "Erase Key: %s\n", def_value[i].nvs_name);
            nvs_erase_key(nvs, def_value[i].nvs_name);
        }
        nvs_commit(nvs);
        nvs_close(nvs);
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
        EEPROM.commit();
    }
}


const char* szWlStatusToStr(wl_status_t wlStatus)
{
  switch (wlStatus)
  {
  case WL_NO_SHIELD: return "WL_NO_SHIELD";
  case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
  case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
  case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
  case WL_CONNECTED: return "WL_CONNECTED";
  case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
  case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
  case WL_DISCONNECTED: return "WL_DISCONNECTED";
  default: return "Unknown";
  }
}


void sendBufferedLog(PubSubClient *client)
{
#ifdef ARDUINO_ARCH_ESP32
    log_chain_t *p = log_chain;
    while (p != NULL)
    {
        if (client->connected())
        {
            client->publish("log", p->data);
            log_chain_buffer -= p->len;
            free(p->data);
            log_chain_t *q = p;
            p = p->next;
            log_chain = p;
            free(q);
            if (p == NULL)
            {
                log_chain = NULL;
                log_chain_tail = NULL;
            }
        } else {
            break;
        }
    }
#endif
}


bool check_connect(char *mqtt_cls, PubSubClient *client, void (*onConnect)(void)) { //等待，直到连接上服务器
  static uint32_t disconnectTime = 0;
  static wl_status_t lastWiFiStatus = WL_NO_SHIELD;
  static int retry_failed_count = 0;
  static uint32_t lastWiFiDebugMessage = 0;
  wl_status_t wifiStatus = WiFi.status();

  // Disconnected -> CONNECT_FAILED -> WL_NO_SHIELD -> WL_DISCONNECTED
  if (wifiStatus != lastWiFiStatus && wifiStatus != WL_IDLE_STATUS)
  {
      if (wifiStatus == WL_CONNECTION_LOST)
      {
          disconnectTime = millis();
          WiFi.disconnect();
          printLog(LOG_DEBUG, "\nWiFi: %s, Connecting", szWlStatusToStr(wifiStatus));
      } else if (wifiStatus == WL_CONNECT_FAILED) {
          disconnectTime = millis();
          printLog(LOG_ERROR, "\nWiFi: WL_CONNECT_FAILED, disable wifi");
          if(!WiFi.enableSTA(false)) {
              printLog(LOG_FATAL, "WiFi: STA disable failed! Force reset system!!\n");
              ESP.restart();
              return false;
          }
      } else if (wifiStatus == WL_NO_SHIELD)
      {
          disconnectTime = millis();
          Serial.printf("\nWiFi: %s, waiting", szWlStatusToStr(wifiStatus));
      } else if (wifiStatus == WL_DISCONNECTED || wifiStatus == WL_NO_SSID_AVAIL) {
          disconnectTime = millis();
          WiFi.reconnect();
          printLog(LOG_DEBUG, "\nWiFi: %s, Connecting", szWlStatusToStr(wifiStatus));
      }else if (wifiStatus != WL_CONNECTED) {
          printLog(LOG_DEBUG, "\nWiFi: %s, Connecting", szWlStatusToStr(wifiStatus));
      } else 
      {
          disconnectTime = 0;
          IPAddress myAddress = WiFi.localIP();
          printLog(LOG_INFO, "\nWiFi: Connected, IP: %s\n", myAddress.toString().c_str());
//          Serial.print(myAddress);
//          Serial.printf("\n");
      }
      lastWiFiStatus = wifiStatus;
  }
  if (wifiStatus == WL_NO_SHIELD && millis() - disconnectTime >= 2000)
  {
      disconnectTime = millis();
      Serial.printf("\nWiFi: %s, reset wifi", szWlStatusToStr(wifiStatus));
      if(!WiFi.enableSTA(true)) {
          printLog(LOG_FATAL, "WiFi: STA enable failed! Force reset system!!\n");
          ESP.restart();
          return false;
      }
  }
  if (wifiStatus != WL_CONNECTED && millis() - lastWiFiDebugMessage >= 250)
  {
      Serial.printf(".");
      lastWiFiDebugMessage = millis();
  }
  if (wifiStatus != WL_CONNECTED && millis() - disconnectTime >= 30000) {
      printLog(LOG_FATAL, "\nWiFi: Status = %d (Disconnected), RESET SYSTEM\n", wifiStatus);
      ESP.restart();
  }
  if (wifiStatus == WL_CONNECTED && !client->connected()){
      printLog(LOG_INFO, "MQTT: Connecting to %s:%d, state: %d\n", mqtt_server, port, client->state());
#ifdef ARDUINO_ARCH_ESP32
      esp_task_wdt_init(30, true);
      esp_task_wdt_add(NULL);
#endif
      if (client->connect(mqtt_cls)) {//接入时的用户名，尽量取一个很不常用的用户名
        retry_failed_count = 0;
        printLog(LOG_INFO, "MQTT: Connected, login by: %s\n",mqtt_cls);
  #ifdef ARDUINO_ARCH_ESP32
        esp_task_wdt_delete(NULL);
        esp_task_wdt_deinit();
  #endif
        if (onConnect != NULL) onConnect();
      } else {
  #ifdef ARDUINO_ARCH_ESP32
        esp_task_wdt_reset();
  #endif
        retry_failed_count++;
        printLog(LOG_ERROR, "MQTT: Connect failed, rc=%d\n", client->state());//连接失败
        client->disconnect();
        delay(1000);
        if (retry_failed_count >= 10)
        {
            printLog(LOG_FATAL, "MQTT: Reconnect Too many times, RESET SYSTEM\n");
            ESP.restart();
        }
      }
  }
  if (client->connected())
  {
      sendBufferedLog(client);
  }
  return wifiStatus == WL_CONNECTED && client->connected();
}


void cfg_initalize_info(size_t size_conf)
{
#ifdef ARDUINO_ARCH_ESP8266
  rst_info *resetInfo;
  resetInfo = ESP.getResetInfoPtr();
  Serial.printf("Flash: %d\n", ESP.getFlashChipRealSize());
  printLog(LOG_INFO, "Reset Reason: %d -> %s\n", resetInfo->reason, ESP.getResetReason().c_str());
#elif ARDUINO_ARCH_ESP32
//  if (log_chain_magic != 0x22336543 ||
//      (rtc_get_reset_reason(0) == 12 && rtc_get_reset_reason(1) == 12) ||
//      ((rtc_get_reset_reason(0) == 1 || rtc_get_reset_reason(0) == 12) && rtc_get_reset_reason(1) == 14))
//  {
//      log_chain = NULL;
//      log_chain_tail = NULL;
//      log_chain_buffer = 0;
//      log_chain_magic = 0x22336543;
//  }
  printLog(LOG_INFO, "Reset Reason: %d / %d\n", rtc_get_reset_reason(0), rtc_get_reset_reason(1));
//  Serial.printf("RTC Reset Reason: %d\n", rtc_get_reset_reason());
//  esp_bluedroid_init();
//  esp_bluedroid_enable();
//  const uint8_t *BT_macAddress = esp_bt_dev_get_address();
//  Serial.printf("BLE Address: %02x:%02x:%02x:%02x:%02x:%02x", BT_macAddress);
  Serial.printf("CPU Speed: %d MHz  XTAL: %d MHz  APB: %d Hz\n", getCpuFrequencyMhz(), getXtalFrequencyMhz(), getApbFrequency());
#endif
    printLog(LOG_INFO, "Build Date: %s %s  Board: %s  CFG FROM %s SIZE: %d\n", __DATE__, __TIME__, ARDUINO_BOARD, cfg_get_backend(), size_conf);
//    Serial.printf("WiFi: AutoConnect: %d  AutoReconnect: %d\n", WiFi.getAutoConnect(), WiFi.getAutoReconnect());
}


void cfg_mqtt_callback(char* topic, byte* payload, unsigned int length) {//用于接收数据
  int l=0;
  int p=1;
  if (on_message != NULL && on_message(topic, payload, length) == true) return;
  if (strcmp(topic, "ota") == 0)
  {
    WiFiClient ota_client;

    char bufferByte = payload[length];
    payload[length] = 0;
    printLog(LOG_INFO, "Start OTA from URL: %s\n", (char *)payload);
    t_httpUpdate_return ret = ESPhttpUpdate.update(ota_client, (char *)payload);

    payload[length] = bufferByte;

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        sprintf(global_msg_buf, "{\"ota\":\"%s\"}", ESPhttpUpdate.getLastErrorString().c_str());
        global_client->publish("status", global_msg_buf);
        printLog(LOG_FATAL, "HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        ESP.restart();
        break;

      case HTTP_UPDATE_NO_UPDATES:
        sprintf(global_msg_buf, "{\"ota\":\"no updates\"}");
        global_client->publish("status", global_msg_buf);
        printLog(LOG_ERROR, "HTTP_UPDATE_NO_UPDATES");
        ESP.restart();
        break;

      case HTTP_UPDATE_OK:
        sprintf(global_msg_buf, "{\"ota\":\"success\"}");
        global_client->publish("status", global_msg_buf);
        printLog(LOG_INFO, "HTTP_UPDATE_OK");
        ESP.restart();
        break;
    }
  } else if (strcmp(topic, "setName") == 0)
  {
      if (length < 32)
      {
          strncpy(dev_name, (const char *)payload, length);
          dev_name[length] = 0;
          printLog(LOG_INFO, "Set Device Name to %s\n", dev_name);
      }
      if (on_cfgUpdate != NULL)
      {
          on_cfgUpdate();
      }
  } else if (strcmp(topic, "reboot") == 0)
  {
      printLog(LOG_INFO, "Remote reboot request\n");
      ESP.restart();
  }
}

void init_mqtt_client(PubSubClient *client, mqtt_callback callback, cfg_callback OnCfgUpdate)
{
    on_message = callback;
    on_cfgUpdate = OnCfgUpdate;
    global_client = client;
    client->setServer((const char *)mqtt_server, port);//端口号
    client->setCallback(cfg_mqtt_callback);
}


void printLog(int debugLevel, char *fmt, ...)
{
    va_list arglist;
    /* Initializing arguments to store all values after num */
    va_start ( arglist, fmt );
    size_t needed = vsnprintf(NULL, 0, fmt, arglist);
    char *buffer = (char *)malloc(needed+1+13);  // 2KB Buffer
    if (buffer == NULL) return;
    char *p = buffer + sprintf(buffer, "%ld|%d|", millis(), debugLevel);
    // buffer
    vsprintf( p, fmt, arglist );
    Serial.printf("%s", p);
    va_end ( arglist );                  // Cleans up the list
    
#ifdef ARDUINO_ARCH_ESP32
    if (log_chain_buffer + needed <= 4096)   // Max 4K Log Buffer
    {
        log_chain_t *node = (log_chain_t *)calloc(1, sizeof(log_chain_t));
        if (node == NULL)
        {
            free(buffer);
            Serial.printf("Log: Buffer Overflowed\n");
            return;
        }
        node->data = buffer;
        node->len = needed;
        node->next = NULL;
        
        if (log_chain_tail == NULL)
        {
            log_chain = node;
            log_chain_tail = node;
        }else {
            log_chain_tail->next = node;
            log_chain_tail = node;
        }
        log_chain_buffer += needed;
    } else {
        free(buffer);
        Serial.printf("Log: Buffer Full\n");
    }
#else
    free(buffer);
#endif
}

int uint16_cmpfunc (const void * a, const void * b) {
   return ( *(uint16_t*)a - *(uint16_t*)b );
}

int uint32_rcmpfunc (const void * a, const void * b) {
   return ( *(uint32_t*)b - *(uint32_t*)a );
}

#ifdef ARDUINO_ARCH_ESP32
uint16_t adc_filter_value(adc1_channel_t ch, uint8_t samples)
{
    uint32_t sensor_adc = 0;
    uint16_t *sensor_adcValues = (uint16_t *)calloc(sizeof(uint16_t), samples);
    for (int i = 0; i < samples; i++)
    {
        sensor_adcValues[i] = adc1_get_raw(ch);
    }
    if (samples >= 2)
        qsort(sensor_adcValues, samples, sizeof(uint16_t), uint16_cmpfunc);
    if (samples >= 16)
    {
        for (int i = samples / 2 - 8; i < samples / 2 + 8; i++)
        {
            sensor_adc += sensor_adcValues[i];
        }
        sensor_adc >>= 4;
        free(sensor_adcValues);
        return sensor_adc;
    } else if (samples >= 4) {
        for (int i = samples / 2 - 2; i < samples / 2 + 2; i++)
        {
            sensor_adc += sensor_adcValues[i];
        }
        sensor_adc >>= 2;
        free(sensor_adcValues);
        return sensor_adc;
    } else if (samples >= 2) {
        for (int i = samples / 2 - 1; i < samples / 2 + 1; i++)
        {
            sensor_adc += sensor_adcValues[i];
        }
        sensor_adc >>= 1;
        free(sensor_adcValues);
        return sensor_adc;
    }
    uint16_t final_adcValues = sensor_adcValues[0];
    free(sensor_adcValues);
    return final_adcValues;
}

uint32_t filter_uint32_array(uint32_t *a, int samples)
{
    uint32_t sensor_adc = 0;
    if (samples >= 4)
        qsort(a, samples, sizeof(uint32_t), uint32_rcmpfunc);
    if (samples >= 16)
    {
        for (int i = samples / 2 - 8; i < samples / 2 + 8; i++)
        {
            sensor_adc += a[i];
        }
        sensor_adc >>= 4;
    } else if (samples >= 4) {
        for (int i = samples / 2 - 2; i < samples / 2 + 2; i++)
        {
            sensor_adc += a[i];
        }
        sensor_adc >>= 2;
    } else if (samples >= 2) {
        sensor_adc >>= 1;
    }
    return sensor_adc;
}
#endif

float mapfloat(float x, long in_min, long in_max, long out_min, long out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
