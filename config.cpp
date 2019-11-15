#include <EEPROM.h>
#ifdef ARDUINO_ARCH_ESP32
  #include <SPIFFS.h>
#else
  #include <FS.h>
#endif
#include "config.h"

static bool cfg_by_spiffs = true;
static char *cfg_buffer = NULL;

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


void cfg_check(const char *mqtt_cls, const hp_cfg_t *def_value, int count)
{
    if (cfg_by_spiffs)
    {
      if (strncmp(cfg_buffer, mqtt_cls, strlen(mqtt_cls)) != 0)
      {
//        Serial.printf("INVALID CONFIG HEADER, RESET CONFIG FILES: \"%s\" != \"%s\"\n", mqtt_cls, cfg_buffer);
        cfg_init(mqtt_cls, def_value, count, true);
      }
    } else {     
      for (int i = 0; i < strlen(mqtt_cls); i++)
      {
          if (EEPROM.read(i) != mqtt_cls[i])
          {
            cfg_init(mqtt_cls, def_value, count, true);
            break;
          }
      } 
    }
}

void cfg_init(const char *mqtt_cls, const hp_cfg_t *def_value, int count, bool full_init)
{
    if (cfg_by_spiffs)
    {
        strcpy(cfg_buffer, mqtt_cls);
    } else {
        for (int i = 0; i < strlen(mqtt_cls); i++)
        {
          EEPROM.write(i, mqtt_cls[i]);
        }
    }
    for (int i = 0; i < count; i++)
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

uint8_t cfg_read_uint8(const hp_cfg_t *def_value, int count, int addr)
{
    uint8_t val;
    if (cfg_by_spiffs)
    {
        return *(uint8_t *)(cfg_buffer + addr);
    }
    EEPROM.get(addr, val);
    return val;
}


uint16_t cfg_read_uint16(const hp_cfg_t *def_value, int count, int addr)
{
    uint16_t val;
    if (cfg_by_spiffs)
    {
        return *(uint16_t *)(cfg_buffer + addr);
    }
    EEPROM.get(addr, val);
    return val;
}

uint32_t cfg_read_uint32(const hp_cfg_t *def_value, int count, int addr)
{
    uint32_t val;
    if (cfg_by_spiffs)
    {
        return *(uint32_t *)(cfg_buffer + addr);
    }
    EEPROM.get(addr, val);
    return val;
}

int cfg_read_string(const hp_cfg_t *def_value, int count, int addr, char *buf, int bufsize)
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

int cfg_write_string(const hp_cfg_t *def_value, int count, int addr, char *buf, int bufsize)
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

void cfg_load(const hp_cfg_t *def_value, int count)
{
    for (int i = 0; i < count; i++)
    {
        if (def_value[i].assign == NULL) continue;
        switch (def_value[i].size)
        {
            case 1:
                *(uint8_t *)def_value[i].assign = cfg_read_uint8(def_value, count, def_value[i].offset);
                break;
            case 2:
                *(uint16_t *)def_value[i].assign = cfg_read_uint16(def_value, count, def_value[i].offset);
                break;
            case 4:
                *(uint32_t *)def_value[i].assign = cfg_read_uint32(def_value, count, def_value[i].offset);
                break;
            default:
                cfg_read_string(def_value, count, def_value[i].offset, (char *)def_value[i].assign, def_value[i].size);
                break;
        }
    }
}

void cfg_save(const hp_cfg_t *def_value, int count)
{
//    noInterrupts();
    for (int i = 0; i < count; i++)
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
            default:
                cfg_write_string(def_value, count, def_value[i].offset, (char *)def_value[i].assign, def_value[i].size);
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
//    interrupts();
}
