#ifndef HP_CONFIG_H

#define HP_CONFIG_H

#define STORAGE_BY_SPIFFS

extern char *dev_name;
extern char *ssid;
extern char *password;
extern char *mqtt_server;//服务器的地址
extern uint16_t port;//服务器端口号 

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
    const char *nvs_name;
} hp_cfg_t;

void cfg_begin();
bool cfg_spiffs();
const char *cfg_get_backend();
bool cfg_check(const char *mqtt_cls, const hp_cfg_t *def_value);
void cfg_init(const char *mqtt_cls, const hp_cfg_t *def_value, bool full_init);
uint8_t cfg_read_uint8(const hp_cfg_t *def_value, int index);
uint16_t cfg_read_uint16(const hp_cfg_t *def_value, int index);
uint32_t cfg_read_uint32(const hp_cfg_t *def_value, int index);
bool cfg_load(const hp_cfg_t *def_value);
void cfg_save(const hp_cfg_t *def_value, bool ignoreString = false);
void cfg_confirm();
int cfg_read_string(const hp_cfg_t *def_value, int index, char *buf, int bufsize);
int cfg_write_string(const hp_cfg_t *def_value, int index, char *buf, int bufsize);
void cfg_reset(const hp_cfg_t *def_value);
uint16_t boot_count_increase();
void boot_count_reset();
void startupWifiConfigure(const hp_cfg_t *def_value, char *msg_buf, uint8_t msg_buf_size,  char *mqtt_cls);

#define CFG_CHECK() cfg_check(mqtt_cls, def_cfg)
#define CFG_INIT(FULL_INIT) cfg_init(mqtt_cls, def_cfg, FULL_INIT)
#define CFG_LOAD() cfg_load(def_cfg)
#define CFG_SAVE() { cfg_save(def_cfg); cfg_confirm(); }
#define CFG_SAFE_SAVE() cfg_save(def_cfg);
#define CFG_READ_UINT8(addr)  cfg_read_uint8(def_cfg, addr)
#define CFG_READ_UINT16(addr)  cfg_read_uint16(def_cfg, addr)
#define CFG_READ_UINT32(addr)  cfg_read_uint32(def_cfg, addr)
#define CFG_READ_STRING(addr, buf, bufsize)  cfg_read_string(def_cfg, addr, buf, bufsize)
#define CFG_WRITE_STRING(addr, buf, bufsize)  cfg_write_string(def_cfg, addr, buf, bufsize)

#endif
