#ifndef HP_CONFIG_H

#define HP_CONFIG_H

#ifdef ARDUINO_ARCH_ESP8266
  #ifndef BEARSSL_SSL_BASIC
  #error Please check config for ESP8266 compile params, required: SSL Support -> Basic SSL ciphers / Flash Size: with at least 32K SPIFFS
  #endif
#endif

extern char *dev_name;
extern char *ssid;
extern char *password;
extern char *mqtt_server;//服务器的地址
extern uint16_t port;//服务器端口号 

enum cfg_storage_backend_t {
    STORAGE_NVS,
    STORAGE_SPIFFS,
    STORAGE_EEPROM
};

typedef struct hp_cfg {
    int offset;  /*!< offset for config buffer */
    int size;    /*!< offset for config variable size (1: uint8, 2:uint16, 4: uint32, 0: string, other: blob) */
    union {
      uint32_t uint32;
      uint16_t uint16;
      uint8_t  uint8;
    } data;    /*!< default value */
    void *assign;    /*!< pointer to data */
    bool full_init;    /*!< init when cfg_init with full_init = false */
    const char *nvs_name;    /*!< NVS storage engine key name */
} hp_cfg_t;

/**@{*/
/**
 * @brief      create config buffer and init storage backend
 * 
 * \code{c}
 * // Example to use
 * #define mqtt_cls "DEMO_CLASS"
 * uint8_t bright;
 * uint16_t port;
 * char *ssid, *password, *mqtt_server;
 * const hp_cfg_t def_cfg[] = {
 *   {31, sizeof(uint8_t), (uint8_t)10,      &bright, true, "bright"},        // BRIGHT
 *   {96, 0, (uint8_t)0, &ssid, true, "ssid"},                    // STRING: SSID
 *   {128, 0, (uint8_t)0, &password, true, "password"},                   // STRING: WIFI PASSWORD 
 *   {160, 0, (uint8_t)0, &mqtt_server, true, "mqtt_srv"},        // STRING: MQTT SERVER
 *   {192, sizeof(uint16_t), (uint16_t)1234, &port, true, "mqtt_port"},             // UINT16: PORT
 *   {NULL, 0,0, NULL, false, NULL}
 * };
 * void setup() {
 *    cfg_begin();
 *    bool cfg_ok = cfg_check(mqtt_cls, def_cfg);
 *    if (!cfg_load(def_cfg))
 *    {
 *        cfg_init(mqtt_cls, def_cfg, true);
 *    }
 *    if (!cfg_ok)
 *    {
 *        // write your auto init code here
 *        // such as PCB version auto check
 *    }
 *    
 *    // update value and save
 *    bright = 100;
 *    cfg_save(def_cfg);
 *    cfg_confirm();
 *    
 *    // clear all configure
 *    cfg_reset();
 * }
 * \endcode
 *
 */
void cfg_begin();

/**@{*/
/**
 * @brief      check storage backend is SPIFFS
 *
 * @return
 *             - true if the backend is using SPIFFS
 *             - false if the backend is another
 */
bool cfg_spiffs();

/**@{*/
/**
 * @brief      get storage backend
 *
 * @return
 *             - string for the backend using
 */
const char *cfg_get_backend();
enum cfg_storage_backend_t cfg_get_backend_t();

/**@{*/
/**
 * @brief      check the config in storages is valid
 *
 *
 * @param[in]  mqtt_cls    class name for configs (use to check the conf).
 * @param[in]  hp_cfg_t    configure variables
 *                         
 * @return
 *             - true if config is valid
 *             - false if config is invalid
 */
bool cfg_check(const char *mqtt_cls, const hp_cfg_t *def_value);

/**@{*/
/**
 * @brief      initalize the config to default value
 *
 *
 * @param[in]  mqtt_cls    class name for configs (use to check the conf).
 * @param[in]  hp_cfg_t    configure variables
 * @param[in]  full_init   initalize all variable, if false will only init the 
 *                         full_init == true variables
 */
void cfg_init(const char *mqtt_cls, const hp_cfg_t *def_value, bool full_init);

/**@{*/
/**
 * @brief      clear config
 *
 *
 * @param[in]  hp_cfg_t    configure variables
 */
void cfg_reset(const hp_cfg_t *def_value);

/**@{*/
/**
 * @brief      load config to configure variables
 *
 *
 * @param[in]  hp_cfg_t    configure variables
 *                         
 * @return
 *             - true if config is valid
 *             - false if config is invalid
 */
bool cfg_load(const hp_cfg_t *def_value);

/**@{*/
/**
 * @brief      save configure variables to buffer
 *
 *
 * @param[in]  hp_cfg_t      configure variables
 * @param[in]  ignoreString  ignore check string to increase speed on NVS
 * @param[in]  forceSave     ignore check data consistent
 */
void cfg_save(const hp_cfg_t *def_value, bool ignoreString = false, bool forceSave = false);

/**@{*/
/**
 * @brief      setup wifi configure tools
 * 
 * This function will startup a WiFi Telnet server to wait telnet conn and
 * waiting Serial console to input WiFi configure and MQTT server.
 * 
 * @param[in]  hp_cfg_t      configure variables
 * @param[in]  msg_buf       buffer for temporarily save inputs.
 * @param[in]  msg_buf_size  buffer size
 * @param[in]  mqtt_cls    class name for configs (use to check the conf).
 *
 */
void startupWifiConfigure(const hp_cfg_t *def_value, char *msg_buf, uint8_t msg_buf_size,  char *mqtt_cls);

//uint8_t cfg_read_uint8(const hp_cfg_t *def_value, int index);
//uint16_t cfg_read_uint16(const hp_cfg_t *def_value, int index);
//uint32_t cfg_read_uint32(const hp_cfg_t *def_value, int index);
//int cfg_read_string(const hp_cfg_t *def_value, int index, char *buf, int bufsize);
//int cfg_write_string(const hp_cfg_t *def_value, int index, char *buf, int bufsize);
uint16_t boot_count_increase();
void boot_count_reset();

#define CFG_CHECK() cfg_check(mqtt_cls, def_cfg)
#define CFG_INIT(FULL_INIT) cfg_init(mqtt_cls, def_cfg, FULL_INIT)
#define CFG_LOAD() cfg_load(def_cfg)
#define CFG_SAVE() cfg_save(def_cfg);
#define CFG_READ_UINT8(addr)  cfg_read_uint8(def_cfg, addr)
#define CFG_READ_UINT16(addr)  cfg_read_uint16(def_cfg, addr)
#define CFG_READ_UINT32(addr)  cfg_read_uint32(def_cfg, addr)
#define CFG_READ_STRING(addr, buf, bufsize)  cfg_read_string(def_cfg, addr, buf, bufsize)
#define CFG_WRITE_STRING(addr, buf, bufsize)  cfg_write_string(def_cfg, addr, buf, bufsize)

#endif
