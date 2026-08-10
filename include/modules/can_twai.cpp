#include "can_twai.h"
#include "driver/twai.h"

static bool s_inited = false;

bool TWAI_Init(int rx_gpio, int tx_gpio, long baud, bool listen_only) {
  if (s_inited) return true;

  // gpio_num_t cast şart (IDF API böyle istiyor)
  twai_general_config_t g =
    TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx_gpio,
                                (gpio_num_t)rx_gpio,
                                listen_only ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);

  twai_timing_config_t t;
  switch (baud) {
    case 250000:  t = TWAI_TIMING_CONFIG_250KBITS(); break;
    case 1000000: t = TWAI_TIMING_CONFIG_1MBITS();   break;
    default:      t = TWAI_TIMING_CONFIG_500KBITS(); break;
  }

  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }

  s_inited = true;
  return true;
}

bool TWAI_Send(uint32_t id, const uint8_t* data, uint8_t len, bool extended) {
  if (!s_inited || len > 8) return false;
  twai_message_t m = {};
  m.identifier = id;
  m.extd = extended ? 1 : 0;
  m.data_length_code = len;
  if (len) memcpy(m.data, data, len);
  return twai_transmit(&m, pdMS_TO_TICKS(100)) == ESP_OK;
}

bool TWAI_Read(uint32_t &id, uint8_t* data, uint8_t &len, bool &extended, uint32_t timeout_ms) {
  if (!s_inited) return false;
  twai_message_t m;
  if (twai_receive(&m, pdMS_TO_TICKS(timeout_ms)) != ESP_OK) return false;
  id = m.identifier; extended = m.extd; len = m.data_length_code;
  if (len && data) memcpy(data, m.data, len);
  return true;
}

int TWAI_Available() {
  if (!s_inited) return 0;
  twai_status_info_t st; twai_get_status_info(&st);
  return (int)st.msgs_to_rx;
}

void TWAI_Deinit() {
  if (!s_inited) return;
  twai_stop();
  twai_driver_uninstall();
  s_inited = false;
}
