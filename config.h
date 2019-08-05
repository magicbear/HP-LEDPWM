#ifndef HP_CONFIG_H

#define HP_CONFIG_H

typedef struct hp_cfg {
    int offset;
    int size;
    union {
      uint8_t  uint8;
      uint16_t uint16;
      uint32_t uint32;
    } data;
    void *assign;
} hp_cfg_t;

void cfg_init(const char *mqtt_cls, const hp_cfg_t *def_value, int count);
uint8_t cfg_read_uint8(const hp_cfg_t *def_value, int count, int addr);
uint16_t cfg_read_uint16(const hp_cfg_t *def_value, int count, int addr);
uint32_t cfg_read_uint32(const hp_cfg_t *def_value, int count, int addr);
void cfg_load(const hp_cfg_t *def_value, int count);
void cfg_save(const hp_cfg_t *def_value, int count);
int cfg_read_string(const hp_cfg_t *def_value, int count, int addr, char *buf, int bufsize);
int cfg_write_string(const hp_cfg_t *def_value, int count, int addr, char *buf, int bufsize);

#define CFG_INIT() cfg_init(mqtt_cls, def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t))
#define CFG_LOAD() cfg_load(def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t))
#define CFG_SAVE() cfg_save(def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t))
#define CFG_READ_UINT8(addr)  cfg_read_uint8(def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t), addr)
#define CFG_READ_UINT16(addr)  cfg_read_uint16(def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t), addr)
#define CFG_READ_UINT32(addr)  cfg_read_uint32(def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t), addr)
#define CFG_READ_STRING(addr, buf, bufsize)  cfg_read_string(def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t), addr, buf, bufsize)
#define CFG_WRITE_STRING(addr, buf, bufsize)  cfg_write_string(def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t), addr, buf, bufsize)

#endif
