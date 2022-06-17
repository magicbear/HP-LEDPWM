#include <EEPROM.h>
#define USE_LITTLE_FS
#ifdef USE_LITTLE_FS
#include <LittleFS.h>
#else
  #ifdef ARDUINO_ARCH_ESP32
    #include <SPIFFS.h>
  #else
    #include <FS.h>
  #endif
#endif
#ifdef ARDUINO_ARCH_ESP32
  #include <nvs.h>
  #include <WiFi.h>
  #include <esp_task_wdt.h>
  #include <rom/rtc.h>
//  #include <esp_bt_main.h>
//  #include <esp_bt_device.h>
  #define ESPhttpUpdate httpUpdate
  #include <HTTPUpdate.h>
  #include <esp_wifi.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESP8266httpUpdate.h>
#endif
#include "config.h"

// ESP-Toolset Define 
// 1.0.6
// F_CPU  ARDUINO  ARDUINO_ESP32_DEV  ARDUINO_ARCH_ESP32  ARDUINO_BOARD  ARDUINO_VARIANT  ESP32  CORE_DEBUG_LEVEL IDF_VER WITH_POSIX _GNU_SOURCE
// 2.0.0
// ARDUINO_EVENT_RUNNING_CORE ARDUINO_RUNNING_CORE ARDUINO_USB_CDC_ON_BOOT

static char *cfg_buffer = NULL;
static enum cfg_storage_backend_t cfg_backend;
static mqtt_callback on_message = NULL;
static cfg_callback  on_cfgUpdate = NULL;
static char *global_msg_buf = NULL;
static PubSubClient *global_client;
static uint32_t max_log_buffer = 0;
bool extFsLogExists = false;
WiFiUDP udp;
bool fallbackEnabled = false;
WiFiServer telnetServer(23);
WiFiClient telnetClient;

char *dev_name;
char *ssid;
char *password;
char *mqtt_server;//服务器的地址
uint16_t port;//服务器端口号 
const hp_cfg_t *last_def_cfg = NULL;
led_callback last_led_cb = NULL;

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
//    } else if ((rc == ESP_ERR_NVS_PART_NOT_FOUND || rc == ESP_ERR_NVS_NOT_INITIALIZED) && SPIFFS.begin(true))
//    {
//        cfg_backend = STORAGE_FS;
//        cfg_buffer = (char *)calloc(BUFFER_SIZE, 1);
//        if (SPIFFS.exists("/config.bin"))
//        {
//            File f = SPIFFS.open("/config.bin", "r");
//            if (f.read((uint8_t *)cfg_buffer, BUFFER_SIZE) != BUFFER_SIZE)
//            {
//                cfg_buffer[0] = '\0';
//                f.close();
//                SPIFFS.remove("/config.bin");
//                return cfg_begin();
//            }
//            f.close();
//        }
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
        cfg_backend = STORAGE_FS;
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
    return cfg_backend == STORAGE_FS;
}

const char *cfg_get_backend()
{
    if (cfg_backend == STORAGE_NVS)
    {
          return "NVS";
    }else if (cfg_backend == STORAGE_FS)
    {
          return "SPIFFS";
    }else if (cfg_backend == STORAGE_EEPROM)
    {
          return "EEPROM";
    }
    return NULL;
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
    if (cfg_backend == STORAGE_FS)
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
    } else if (cfg_backend == STORAGE_FS)
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
                } else if (cfg_backend == STORAGE_FS)
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
                } else if (cfg_backend == STORAGE_FS)
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
                } else if (cfg_backend == STORAGE_FS)
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
                } else if (cfg_backend == STORAGE_FS)
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
    } else if (cfg_backend == STORAGE_FS)
    {
#ifdef USE_LITTLE_FS
        File f = LITTLEFS.open("/config.bin", "w");
#else
        File f = SPIFFS.open("/config.bin", "w");
#endif
#ifdef ARDUINO_ARCH_ESP32
        f.write((uint8_t *)cfg_buffer, BUFFER_SIZE);
#else
        f.write(cfg_buffer, BUFFER_SIZE);
#endif
        f.close();
//        printLog(LOG_INFO, "Init save %s\n", SPIFFS.exists("/config.bin") ? "Success" : "Failed");
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
    } 
#endif
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
    } else if (cfg_backend == STORAGE_FS)
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
    } else if (cfg_backend == STORAGE_FS)
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
    } else if (cfg_backend == STORAGE_FS)
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
    } else if (cfg_backend == STORAGE_FS)
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
    } else if (cfg_backend == STORAGE_FS)
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
                }else if (cfg_backend == STORAGE_FS)
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
                }else if (cfg_backend == STORAGE_FS)
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
                }else if (cfg_backend == STORAGE_FS)
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
    } else if (cfg_backend == STORAGE_FS)
    {
#ifdef USE_LITTLE_FS
        File f = LITTLEFS.open("/config.bin.bak", "w");
#else
        File f = SPIFFS.open("/config.bin.bak", "w");
#endif
#ifdef ARDUINO_ARCH_ESP32
        f.write((uint8_t *)cfg_buffer, BUFFER_SIZE);
#else
        f.write(cfg_buffer, BUFFER_SIZE);
#endif
        f.close();
#ifdef USE_LITTLE_FS
        LITTLEFS.remove("/config.bin");
        LITTLEFS.rename("/config.bin.bak", "/config.bin");
#else
        SPIFFS.remove("/config.bin");
        SPIFFS.rename("/config.bin.bak", "/config.bin");
#endif        
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
  last_def_cfg = def_value;
  last_led_cb = led_cb;
  
  if (strlen(ssid) == 0)
  {
      WiFi.mode(WIFI_AP_STA);
      
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
              rebootSystem();
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
  udp.beginMulticast(IPAddress(224, 0, 0, 52), 12345);
  WiFi.setAutoReconnect(true);
  
#if defined(CFG_ENABLE_GLOBAL_WDT)
    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);
#endif
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
    } else if (cfg_backend == STORAGE_FS)
    {
      #ifdef USE_LITTLE_FS
        LITTLEFS.remove("/config.bin");
      #else
        SPIFFS.remove("/config.bin");
      #endif
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
    if (client->connected() && extFsLogExists)
    {
        char buf[128];
        int len;
        int fsize;
        bool sendFSLogFailed = false;
        #ifdef USE_LITTLE_FS
        File f = LITTLEFS.open("/logbuf.bin", "r");
        #else
        File f = SPIFFS.open("/logbuf.bin", "r");
        #endif
        fsize = f.size();
        while (f.position() < f.size())
        {
            if (!client->connected())
            {
                printLog(LOG_ERROR, "LOG: send extfs archived log failed");
                #ifdef USE_LITTLE_FS
                File fcopy = LITTLEFS.open("/logbuf.bin.bak", "w");
                #else
                File fcopy = SPIFFS.open("/logbuf.bin.bak", "w");
                #endif
                while (f.position() < f.size())
                {
                    len = f.read((uint8_t *)buf, sizeof(buf));
                    fcopy.write((uint8_t *)buf, len);
                    if (len != sizeof(buf)) break;
                }
                fcopy.close();
                f.close();
                #ifdef USE_LITTLE_FS
                LITTLEFS.remove("/logbuf.bin");
                LITTLEFS.rename("/logbuf.bin.bak", "/logbuf.bin");
                #else
                SPIFFS.remove("/logbuf.bin");
                SPIFFS.rename("/logbuf.bin.bak", "/logbuf.bin");
                #endif
                sendFSLogFailed = true;
                break;
            }
            if (sizeof(int) != f.read((uint8_t *)&len, sizeof(int))) break;
            if (len > sizeof(buf)) f.seek(len, SeekCur);
            else{
                int rc = f.read((uint8_t *)buf, len);
                buf[len] = 0;
                client->publish("log", buf);
            }
            
        }
        if (!sendFSLogFailed)
        {
            f.close();
            #ifdef USE_LITTLE_FS
              LITTLEFS.remove("/logbuf.bin");
            #else
              SPIFFS.remove("/logbuf.bin");
            #endif
            extFsLogExists = false;
        }
            
    }
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


typedef enum
{
    SSID,
    PASSWORD,
    MQTT_SERVER,
    MQTT_PORT,
    CONFIRM
} hp_input_stage_t;

void processInput(char chEEP)
{
    static hp_input_stage_t inputStage = SSID;
    static char *p = ssid;
    static char *pOffset = ssid;

    if (chEEP == 0)
    {
        inputStage = SSID;
        p = ssid;
        pOffset = ssid;
        telnetClient.printf("\nPlease input SSID: ");
        return;
    }
    if (chEEP == '\b')
    {
        *p--;
        if (p < pOffset) p = pOffset;
    } else if (chEEP == '\r')
    {
        
    } else if (inputStage == SSID)
    {
        *p++ = chEEP == '\n' ? '\0' : chEEP;
        if (chEEP == '\n')
        {
            if (p == pOffset + 1)
            {
                p--;
                return;
            }
            inputStage = PASSWORD;
            telnetClient.printf("Please input Password: ");
            p = password;
            pOffset = p;
        }
    } else if (inputStage == PASSWORD)
    {
        *p++ = chEEP == '\n' ? '\0' : chEEP;
        if (chEEP == '\n')
        {
            inputStage = MQTT_SERVER;
            telnetClient.printf("Please input MQTT Server: ");
            p = mqtt_server;
            pOffset = p;
        }
    } else if (inputStage == MQTT_SERVER)
    {
        *p++ = chEEP == '\n' || chEEP == ':' ? '\0' : chEEP;
        if (chEEP == ':')
        {
            inputStage = MQTT_PORT;
            port = 0;
        } else if (chEEP == '\n')
        {
            inputStage = CONFIRM;
            telnetClient.printf("Input SSID = %s  PASSWORD = %s  MQTT Server: %s:%d\n", ssid, password, mqtt_server, port);
            telnetClient.print("Confirm? [y/n]");
        }
    } else if (inputStage == MQTT_PORT)
    {
        if (chEEP != '\n') {
            port = port * 10 + chEEP - '0';
        } else 
        {
            inputStage = CONFIRM;
            telnetClient.printf("Input SSID = %s  PASSWORD = %s  MQTT Server: %s:%d\n", ssid, password, mqtt_server, port);
            telnetClient.print("Confirm? [y/n]");
        }
    } else if (inputStage == CONFIRM)
    {
        if (chEEP == 'y')
        {
            Serial.printf("\n");
            cfg_save(last_def_cfg);
            if (telnetClient)
            {
                telnetClient.stop();
            }
            rebootSystem();
        } else if (chEEP != 'n')
        {
            telnetClient.print("Confirm? [y/n]");
        } else 
        {
            processInput(0);
        }
    }
}

bool check_connect(char *mqtt_cls, PubSubClient *client, void (*onConnect)(void)) { //等待，直到连接上服务器
  static uint32_t disconnectTime = 0;
  static uint32_t offlineTime = 0xffffffff;
  static wl_status_t lastWiFiStatus = WL_NO_SHIELD;
  static int retry_failed_count = 0;
  static uint32_t last_try_connect_mqtt = 0;
  static uint32_t lastWiFiDebugMessage = 0;
  wl_status_t wifiStatus = WiFi.status();

#if defined(ARDUINO_ARCH_ESP32) && defined(CFG_ENABLE_GLOBAL_WDT)
  esp_task_wdt_reset();
#endif
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
              rebootSystem();
              return false;
          }
      } else if (wifiStatus == WL_NO_SHIELD)
      {
          disconnectTime = millis();
          printLog(LOG_DEBUG, "\nWiFi: %s, waiting", szWlStatusToStr(wifiStatus));
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
      }
      lastWiFiStatus = wifiStatus;
  }
  if (wifiStatus == WL_NO_SHIELD && millis() - disconnectTime >= 2000)
  {
      disconnectTime = millis();
      printLog(LOG_DEBUG, "\nWiFi: %s, reset wifi", szWlStatusToStr(wifiStatus));
      if(!WiFi.enableSTA(true)) {
          printLog(LOG_FATAL, "WiFi: STA enable failed! Force reset system!!\n");
          rebootSystem();
          return false;
      }
  }
  boolean mqttRequestFallback = false;
  if (wifiStatus != WL_CONNECTED && millis() - lastWiFiDebugMessage >= 250)
  {
#if !defined(CORE_DEBUG_LEVEL) || CORE_DEBUG_LEVEL >= 4 // VERBOSE
      Serial.printf(".");
#endif
      lastWiFiDebugMessage = millis();
  }
  if (wifiStatus == WL_CONNECTED && !client->connected() && millis() - last_try_connect_mqtt >= 3000){
      printLog(LOG_INFO, "MQTT: Connecting to %s:%d, state: %d\n", mqtt_server, port, client->state());
#if defined(ARDUINO_ARCH_ESP32) && !defined(CFG_ENABLE_GLOBAL_WDT)
      esp_task_wdt_init(30, true);
      esp_task_wdt_add(NULL);
#endif
      if (client->connect(mqtt_cls)) {//接入时的用户名，尽量取一个很不常用的用户名
        retry_failed_count = 0;
        printLog(LOG_INFO, "MQTT: Connected, login by: %s\n",mqtt_cls);
  #if defined(ARDUINO_ARCH_ESP32) && !defined(CFG_ENABLE_GLOBAL_WDT)
        esp_task_wdt_delete(NULL);
        esp_task_wdt_deinit();
  #endif
        if (fallbackEnabled)
        {
            fallbackEnabled = false;
            printLog(LOG_INFO, "Fallback: Connected to WiFi success, close fallback WiFi.\n");
            WiFi.mode(WIFI_STA);
        }
        last_try_connect_mqtt = 0;
        if (onConnect != NULL) onConnect();
      } else {
  #ifdef ARDUINO_ARCH_ESP32
        esp_task_wdt_reset();
  #endif
        retry_failed_count++;
        int fail_reason = client->state();
        printLog(LOG_ERROR, "MQTT: Connect failed, rc=%d\n", fail_reason);//连接失败
        client->disconnect();
        last_try_connect_mqtt = millis();
        if (retry_failed_count >= 10) // && fail_reason != MQTT_CONNECT_FAILED)
        {
            printLog(LOG_FATAL, "MQTT: Reconnect Too many times, RESET WiFi\n");
            WiFi.disconnect();
            retry_failed_count = 0;
        }
//        else if (retry_failed_count >= 10) {
//            mqttRequestFallback = true;
//            printLog(LOG_FATAL, "MQTT: Server connect denied, startup fallback wifi\n");
//        }
      }
  }
  if (mqttRequestFallback || (wifiStatus != WL_CONNECTED && millis() - disconnectTime >= 60000)) {
      if (!fallbackEnabled)
      {
          fallbackEnabled = true;

          WiFi.disconnect();
          
          WiFi.mode(WIFI_AP_STA);
          
          uint8_t primaryChan = 6;
          wifi_second_chan_t secondChan = WIFI_SECOND_CHAN_NONE;
          esp_wifi_set_channel(primaryChan, secondChan);
          
          printLog(LOG_INFO, "Startup WiFi Station -> %s...\n", mqtt_cls);
          while(!WiFi.softAP(mqtt_cls)){ delay(10); yield(); }; // Startup AP
          printLog(LOG_INFO, "Startup Telet Server...\n");

          if (strlen(ssid) > 0)
          {
              WiFi.begin(ssid, password);
          }          
          
          telnetServer.begin(23);
          printLog(LOG_FATAL, "\nWiFi: Status = %d (Disconnected), Startup Fallback WiFi\n", wifiStatus);
      }
//      rebootSystem();
  }
  if (fallbackEnabled)
  {
      char chEEP;
      if (!telnetClient && (telnetClient = telnetServer.available())) {
          uint32_t t_start = micros();
          while (micros() - t_start <= 100) yield();
          while (telnetClient.available()) telnetClient.read();
          
          telnetClient.printf("Welcome, Device ID: %s\n", mqtt_cls);
          processInput(0);
      }
      if (telnetClient)
      {
          if (telnetClient.connected() && telnetClient.available())
          {
              chEEP = telnetClient.read();              
          }
          if (chEEP != 0)
          {
              processInput(chEEP);
          }
      }
  }
  if (client->connected())
  {
      sendBufferedLog(client);
  }
  uint16_t udpPktSize;
  if (wifiStatus == WL_CONNECTED && (udpPktSize = udp.parsePacket()))
  {
      if (udpPktSize <= 512)
      {
          char rcvBuf[64];
          char *udpBuf = (char *)malloc(udpPktSize+1);
          if (udpBuf == NULL)
          {
              printLog(LOG_FATAL, "UDP: Buffer overflow\n");
              uint16_t bufLen = 0;
              while (udpPktSize > 0)
              {
                  udpPktSize -= udp.read(rcvBuf, udpPktSize > sizeof(rcvBuf) ? sizeof(rcvBuf) : udpPktSize);
              }
          } else 
          {
              udp.read(udpBuf, udpPktSize);
              
              if (udpPktSize >= 19 && strncmp(udpBuf, "{\"cmd\":\"discovery\"}", 19) == 0)
              {
                  printLog(LOG_INFO, "UDP: receive discovery command\n");
                  sprintf(rcvBuf, "{\"dev\":\"%s\"}", mqtt_cls);
                  if (udp.beginMulticastPacket()) //udp.remoteIP(), udp.remotePort()))
                  {
                      udp.write((uint8_t *)rcvBuf, strlen(rcvBuf));
                      printLog(LOG_INFO, "UDP: send discovery result: %s\n", rcvBuf);
                      udp.endPacket();
                  }
              } else if (udpPktSize >= 22 + strlen(mqtt_cls) + 2 && strncmp(udpBuf, "{\"cmd\":\"reset\",\"dev\":\"", 22) == 0
                && strncmp(udpBuf + 22, mqtt_cls, strlen(mqtt_cls)) == 0
                && strncmp(udpBuf + 22 + strlen(mqtt_cls), "\"}", 2) == 0 )
              {
                  printLog(LOG_INFO, "UDP: receive reset command\n");
                  if (client->connected())
                  {
                      sendBufferedLog(client);
                  }
                  if (cfg_backend == STORAGE_NVS)
                  {
                      int rc = nvs_open("nvs", NVS_READWRITE, &nvs);
                      if (rc != ESP_OK) {
                          printLog(LOG_ERROR, "Fatal to open NVS\n");
                          return false;
                      }
                      nvs_erase_key(nvs, "class");
                      nvs_commit(nvs);
                      nvs_close(nvs);
                      rebootSystem();
                  }
              }
              free(udpBuf);
          }
      }
  }
  if (wifiStatus == WL_CONNECTED) {
      offlineTime = 0;
  } else if (offlineTime == 0)
  {
      offlineTime = time(NULL);
  } else if (fallbackEnabled && time(NULL) - offlineTime >= 900 && !telnetClient)
  {
      // Over 15 Minutes cannot conect to WiFi
      printLog(LOG_ERROR, "Failed to connect WiFi and no fallback request, reboot\n");
      rebootSystem();
  }
  return wifiStatus == WL_CONNECTED && client->connected();
}


void cfg_initalize_info(size_t size_conf)
{
#ifdef ARDUINO_ARCH_ESP8266
  rst_info *resetInfo;
  resetInfo = ESP.getResetInfoPtr();
#if !defined(CORE_DEBUG_LEVEL) || CORE_DEBUG_LEVEL >= 3 // INFO
  Serial.printf("Flash: %d\n", ESP.getFlashChipRealSize());
#endif
  printLog(LOG_INFO, "Reset Reason: %d -> %s\n", resetInfo->reason, ESP.getResetReason().c_str());
#elif ARDUINO_ARCH_ESP32
  esp_task_wdt_init(10, true);
  esp_task_wdt_add(NULL);
  
  extFsLogExists = false;
#ifdef USE_LITTLE_FS
  if (!LITTLEFS.begin())
  {
      printLog(LOG_INFO, "Initalize LITTLEFS failed, try to format\n");
      LITTLEFS.format();
  }
  if (LITTLEFS.begin())
  {
      if (LITTLEFS.totalBytes() / 2 > LITTLEFS.usedBytes())
      {
          max_log_buffer = LITTLEFS.totalBytes() / 2 - LITTLEFS.usedBytes();
      }
      extFsLogExists = LITTLEFS.exists("/logbuf.bin");
      printLog(LOG_INFO, "LITTLEFS exists: used: %ld / total: %ld LOG: %d\n", LITTLEFS.usedBytes(), LITTLEFS.totalBytes(), extFsLogExists);
  } else 
  {
      printLog(LOG_ERROR, "Initalize LITTLEFS failed\n");
  }
#else
  if (SPIFFS.begin(false))
  {
      if (SPIFFS.totalBytes() / 2 > SPIFFS.usedBytes())
      {
          max_log_buffer = SPIFFS.totalBytes() / 2 - SPIFFS.usedBytes();
      }
      extFsLogExists = SPIFFS.exists("/logbuf.bin");
      printLog(LOG_INFO, "SPIFFS exists: used: %ld / total: %ld LOG: %d\n", SPIFFS.usedBytes(), SPIFFS.totalBytes(), extFsLogExists);
  } else 
  {
      printLog(LOG_ERROR, "Initalize SPIFFFS failed\n");
  }
#endif
  printLog(LOG_INFO, "Reset Reason: %d / %d\n", rtc_get_reset_reason(0), rtc_get_reset_reason(1));

  // Hook System Log to our log handler
  esp_log_set_vprintf(&esp_system_log);
  
  esp_task_wdt_delete(NULL);
  esp_task_wdt_deinit();
  
//  esp_bluedroid_init();
//  esp_bluedroid_enable();
//  const uint8_t *BT_macAddress = esp_bt_dev_get_address();
//  Serial.printf("BLE Address: %02x:%02x:%02x:%02x:%02x:%02x", BT_macAddress);

#if !defined(CORE_DEBUG_LEVEL) || CORE_DEBUG_LEVEL >= 3 // INFO
  Serial.printf("CPU Speed: %d MHz  XTAL: %d MHz  APB: %d Hz\n", getCpuFrequencyMhz(), getXtalFrequencyMhz(), getApbFrequency());
#endif

#endif
    printLog(LOG_INFO, "Build Date: %s %s  Board: %s  CFG FROM %s SIZE: %d\n", __DATE__, __TIME__, ARDUINO_BOARD, cfg_get_backend(), size_conf);
//    Serial.printf("WiFi: AutoConnect: %d  AutoReconnect: %d\n", WiFi.getAutoConnect(), WiFi.getAutoReconnect());
}


void rebootSystem()
{
    if (max_log_buffer > 0)
    {   
        log_chain_t *p = log_chain;
        if (p != NULL)
        {
            #ifdef USE_LITTLE_FS
                File LogBufferFile = LITTLEFS.open("/logbuf.bin", "a+");
            #else
                File LogBufferFile = SPIFFS.open("/logbuf.bin", "a+");
            #endif
            while (p != NULL)
            {
                if (max_log_buffer < p->len + 1)
                {
                    break;
                }
                LogBufferFile.write((const uint8_t *)&p->len, sizeof(int));
                LogBufferFile.write((const uint8_t *)p->data, p->len);
                max_log_buffer -= p->len + 1;
                
                p = p->next;
            }
            LogBufferFile.close();
        }
    }
    ESP.restart();
//#ifdef ARDUINO_ARCH_ESP32
//
//#else
//    ESP.reset();
//#endif
}


// MAGIC DEFINE FOR ESP-Toolkit >= 2.0
#ifdef ARDUINO_USB_CDC_ON_BOOT
void update_started() {
  Serial.println("CALLBACK:  HTTP update process started");
}

void update_finished() {
  Serial.println("CALLBACK:  HTTP update process finished");
}

void update_progress(int cur, int total) {
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err) {
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}
#endif

void cfg_mqtt_callback(char* topic, byte* payload, unsigned int length) {//用于接收数据
  int l=0;
  int p=1;
  if (on_message != NULL && on_message(topic, payload, length) == true) return;
  if (strcmp(topic, "ota") == 0)
  {
    WiFiClient ota_client;

#if defined(ARDUINO_ARCH_ESP32) && defined(CFG_ENABLE_GLOBAL_WDT)
    esp_task_wdt_delete(NULL);
    esp_task_wdt_deinit();
#endif

#if defined(ARDUINO_ARCH_ESP32) && defined(ARDUINO_USB_CDC_ON_BOOT)
    httpUpdate.onStart(update_started);
    httpUpdate.onEnd(update_finished);
    httpUpdate.onProgress(update_progress);
    httpUpdate.onError(update_error);
//    printLog(LOG_INFO, "Current Sketch: %s\n", getSketchSHA256().c_str());
#endif
    
    char bufferByte = payload[length];
    payload[length] = 0;
    printLog(LOG_INFO, "Start OTA from URL: %s\n", (char *)payload);
    t_httpUpdate_return ret = ESPhttpUpdate.update(ota_client, (char *)payload);

    payload[length] = bufferByte;

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        printLog(LOG_FATAL, "HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        rebootSystem();
        break;

      case HTTP_UPDATE_NO_UPDATES:
        printLog(LOG_ERROR, "HTTP_UPDATE_NO_UPDATES");
        rebootSystem();
        break;

      case HTTP_UPDATE_OK:
        printLog(LOG_INFO, "HTTP_UPDATE_OK");
        rebootSystem();
        break;
      default:
        printLog(LOG_INFO, "OTA Status Unknow: %d\n", ret);
        rebootSystem();
        break;
    }
#if defined(ARDUINO_ARCH_ESP32) && defined(CFG_ENABLE_GLOBAL_WDT)
    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);
#endif
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
      rebootSystem();
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

int vprintLog(int debugLevel, const char *fmt, va_list arglist)
{
    size_t needed = vsnprintf(NULL, 0, fmt, arglist)+1+14;
    char *buffer = (char *)malloc(needed);  // 2KB Buffer
    if (buffer == NULL) {
#if !defined(CORE_DEBUG_LEVEL) || CORE_DEBUG_LEVEL >= 1 // ERROR
        Serial.printf("Log: Allocate buffer failed.\n");
#endif
        return 0;
    }
    char *p = buffer + sprintf(buffer, "%ld|%d|", millis(), debugLevel);
    // buffer
    vsprintf( p, fmt, arglist );
#if defined(CORE_DEBUG_LEVEL) && CORE_DEBUG_LEVEL > 0
    if (CORE_DEBUG_LEVEL >= debugLevel)
        Serial.write(p);
#elif !defined(CORE_DEBUG_LEVEL)
    Serial.write(p);
#endif
    
#ifdef ARDUINO_ARCH_ESP32
    if (log_chain_buffer + needed <= 4096)   // Max 4K Log Buffer
    {
        log_chain_t *node = (log_chain_t *)calloc(1, sizeof(log_chain_t));
        if (node == NULL)
        {
            free(buffer);
#if !defined(CORE_DEBUG_LEVEL) || CORE_DEBUG_LEVEL >= 2 // Warning
            Serial.printf("Log: Buffer Overflowed\n");
#endif
            return needed - 1 - 14;
        }
        node->data = buffer;
        node->len = needed;
        node->next = NULL;
        
        if (log_chain_tail == NULL)
        {
            log_chain = node;
            log_chain_tail = node;
        } else {
            log_chain_tail->next = node;
            log_chain_tail = node;
        }
        log_chain_buffer += needed;
    } else {
        free(buffer);
#if !defined(CORE_DEBUG_LEVEL) || CORE_DEBUG_LEVEL >= 2 // Warning
        Serial.printf("Log: Buffer Full\n");
#endif
    }
#else
    free(buffer);
#endif
  return needed - 1 - 14;
}

void printLog(int debugLevel, const char *fmt, ...)
{
    va_list arglist;
    /* Initializing arguments to store all values after num */
    va_start ( arglist, fmt );
    vprintLog(debugLevel, fmt, arglist);
    va_end ( arglist );                  // Cleans up the list
}

int esp_system_log(const char *fmt, va_list arg)
{
    return vprintLog(LOG_INFO, fmt, arg);
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
