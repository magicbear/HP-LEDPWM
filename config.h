#ifndef HP_CONFIG_H

#define HP_CONFIG_H

#define STORAGE_BY_SPIFFS

typedef struct hp_cfg {
    int offset;
    int size;
    union {
      uint32_t uint32;
      uint16_t uint16;
      uint8_t  uint8;
    } data;
    void *assign;
    bool full_init;
} hp_cfg_t;

void cfg_begin();
bool cfg_spiffs();
void cfg_check(const char *mqtt_cls, const hp_cfg_t *def_value, int count);
void cfg_init(const char *mqtt_cls, const hp_cfg_t *def_value, int count, bool full_init);
uint8_t cfg_read_uint8(const hp_cfg_t *def_value, int count, int addr);
uint16_t cfg_read_uint16(const hp_cfg_t *def_value, int count, int addr);
uint32_t cfg_read_uint32(const hp_cfg_t *def_value, int count, int addr);
void cfg_load(const hp_cfg_t *def_value, int count);
void cfg_save(const hp_cfg_t *def_value, int count);
int cfg_read_string(const hp_cfg_t *def_value, int count, int addr, char *buf, int bufsize);
int cfg_write_string(const hp_cfg_t *def_value, int count, int addr, char *buf, int bufsize);
uint16_t boot_count_increase();
void boot_count_reset();

#define CFG_CHECK() cfg_check(mqtt_cls, def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t))
#define CFG_INIT(FULL_INIT) cfg_init(mqtt_cls, def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t), FULL_INIT)
#define CFG_LOAD() cfg_load(def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t))
#define CFG_SAVE() cfg_save(def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t))
#define CFG_READ_UINT8(addr)  cfg_read_uint8(def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t), addr)
#define CFG_READ_UINT16(addr)  cfg_read_uint16(def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t), addr)
#define CFG_READ_UINT32(addr)  cfg_read_uint32(def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t), addr)
#define CFG_READ_STRING(addr, buf, bufsize)  cfg_read_string(def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t), addr, buf, bufsize)
#define CFG_WRITE_STRING(addr, buf, bufsize)  cfg_write_string(def_cfg, sizeof(def_cfg) / sizeof(hp_cfg_t), addr, buf, bufsize)

#endif
