#include <EEPROM.h>
#include "config.h"

void cfg_init(const char *mqtt_cls, const hp_cfg_t *def_value, int count)
{
    for (int i = 0; i < strlen(mqtt_cls); i++)
    {
      EEPROM.write(i, mqtt_cls[i]);
    }
    for (int i = 0; i < count; i++)
    {
        switch (def_value[i].size)
        {
            case 1:
                EEPROM.write(def_value[i].offset, def_value[i].data.uint8);
                break;
            case 2:
                EEPROM.put(def_value[i].offset, def_value[i].data.uint16);
                break;
            case 4:
                EEPROM.put(def_value[i].offset, def_value[i].data.uint32);
                break;
            default:
                EEPROM.write(def_value[i].offset, 0);
                break;
        }        
    }

    EEPROM.commit();
}

uint8_t cfg_read_uint8(const hp_cfg_t *def_value, int count, int addr)
{
    uint8_t val;
    EEPROM.get(addr, val);
    return val;
}


uint16_t cfg_read_uint16(const hp_cfg_t *def_value, int count, int addr)
{
    uint16_t val;
    EEPROM.get(addr, val);
    return val;
}

uint32_t cfg_read_uint32(const hp_cfg_t *def_value, int count, int addr)
{
    uint32_t val;
    EEPROM.get(addr, val);
    return val;
}

int cfg_read_string(const hp_cfg_t *def_value, int count, int addr, char *buf, int bufsize)
{
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
    for (int i = 0; i < count; i++)
    {
        if (def_value[i].assign == NULL) continue;
        switch (def_value[i].size)
        {
            case 1:
                EEPROM.put(def_value[i].offset, *(uint8_t *)def_value[i].assign);
                break;
            case 2:
                EEPROM.put(def_value[i].offset, *(uint16_t *)def_value[i].assign);
                break;
            case 4:
                EEPROM.put(def_value[i].offset, *(uint32_t *)def_value[i].assign);
                break;
            default:
                cfg_write_string(def_value, count, def_value[i].offset, (char *)def_value[i].assign, def_value[i].size);
                break;
        }
    }
    EEPROM.commit();
}
