/* Basic Driver setup for the ESP32-S3 2.1 inch LCD Driver board  */
/* Author: Andy Valentine - Garage Tinkering                      */

#include <Arduino.h>
#include "CANBus_Driver.h"
#include "LVGL_Driver.h"
#include "I2C_Driver.h"

#define TAG "TWAI"

// Struct for CAN signal configuration
typedef enum {
    STATIC,
    RANGE,
    BITS
} ByteType;

typedef enum {
    NONE,
    MSB,
    LSB
} EndianType;

typedef enum {
    BUTTON_MODE_NONE,
    BUTTON_MODE_TOGGLE,
    BUTTON_MODE_MOMENTARY
} ButtonMode;

typedef struct {
    int value;              // 0 or 1
    ButtonMode button_mode; // toggle/momentary/none
    int exio_pin;           // EXIO pin number, or -1 if not used
} BitConfig;

typedef struct {
    ByteType type;
    union {
        int static_value; // For STATIC
        struct { int min, max, value, speed, direction; EndianType endian; } range; // For RANGE
        BitConfig bits[8]; // For BITS (up to 8 bits per byte)
    } data;
} ByteConfig;

typedef struct {
    uint32_t can_id;
    ByteConfig bytes[8]; // 8 bytes per CAN message
} CanMessageConfig;

const int DATA_RATE = 200; // ms between CAN sends

CanMessageConfig can_signals[] = {
  {
    0x551,
    {
      {RANGE, { .range = {0, 180, 0, 5, 1, NONE} }},
      {STATIC, {0}},
      {STATIC, {0}},
      {RANGE, { .range = {0, 4, 0, 1, 1, MSB} }},
      {RANGE, { .range = {0, 255, 0, 30, 1, LSB} }},
      {STATIC, {0}},
      {STATIC, {0}},
      {STATIC, {0}}
    }
  },
  {
    0x555,
    {
      {STATIC, {0}},
      {STATIC, {0}},
      {BITS, { .bits = {
        {0, BUTTON_MODE_NONE, -1}, // bit 0
        {0, BUTTON_MODE_NONE, -1}, // bit 1
        {0, BUTTON_MODE_NONE, -1}, // bit 2
        {0, BUTTON_MODE_NONE, -1}, // bit 3
        {0, BUTTON_MODE_MOMENTARY, EXIO_PIN7}, // bit 4
        {0, BUTTON_MODE_TOGGLE, EXIO_PIN5}, // bit 5
        {0, BUTTON_MODE_NONE, -1}, // bit 6
        {0, BUTTON_MODE_NONE, -1}  // bit 7
      }}},
      {STATIC, {0}},
      {STATIC, {0}},
      {STATIC, {0}},
      {STATIC, {0}},
      {STATIC, {0}}
    }
  },
};

const int num_signals = sizeof(can_signals) / sizeof(CanMessageConfig);

// CONTROL VARIABLE INIT
bool initial_load             = false; // has the first data been received

lv_obj_t *main_scr;

void drivers_init(void) {
  i2c_init();

  Serial.println("Scanning for TCA9554...");
  bool found = false;
  for (int attempt = 0; attempt < 10; attempt++) {
    if (i2c_scan_address(0x20)) { // 0x20 is default for TCA9554
      found = true;
      break;
    }
    delay(50); // wait a bit before retrying
  }

  if (!found) {
    Serial.println("TCA9554 not detected! Skipping expander init.");
  } else {
    tca9554pwr_init(0x00);
  }
  lcd_init();
  canbus_init();
  lvgl_init();
}


// create the elements on the main scr
void main_scr_ui(void) {
   
}

// build the screens
void screens_init(void) {
  main_scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(main_scr, lv_color_make(0,0,0), 0);
  lv_screen_load(main_scr);

  main_scr_ui();
}

  // Task to send CAN message ID 0x551, byte 0, value 0-180-0, 10Hz
void send_can_sim_task(void *arg) {
  const uint8_t dlc = 8;
  while (1) {

    // update values
    for (int i = 0; i < num_signals; i++) {
      int msb_position = -1, lsb_position = -1;
      for (int j = 0; j < 8; j++) {
        ByteConfig &byte_cfg = can_signals[i].bytes[j];
        
        if (byte_cfg.type == RANGE) {
          if (byte_cfg.data.range.endian == NONE) {
            byte_cfg.data.range.value += byte_cfg.data.range.direction * byte_cfg.data.range.speed;
            if (byte_cfg.data.range.value >= byte_cfg.data.range.max) byte_cfg.data.range.direction = -1;
            if (byte_cfg.data.range.value <= byte_cfg.data.range.min) byte_cfg.data.range.direction = 1;
          } else {
            // see if msb / lsb positions are set
            if (byte_cfg.data.range.endian == MSB) {
              msb_position = j;
            }
            if (byte_cfg.data.range.endian == LSB) {
              lsb_position = j;
            }
          }
        }
      }

      if (msb_position >= 0 && lsb_position >= 0) {
        if (can_signals[i].bytes[lsb_position].data.range.direction == 1) {
          if (can_signals[i].bytes[lsb_position].data.range.value < can_signals[i].bytes[lsb_position].data.range.max) {
            can_signals[i].bytes[lsb_position].data.range.value += can_signals[i].bytes[lsb_position].data.range.speed;
          } else {
            if (can_signals[i].bytes[msb_position].data.range.value < can_signals[i].bytes[msb_position].data.range.max) {
              can_signals[i].bytes[msb_position].data.range.value += 1;
              can_signals[i].bytes[lsb_position].data.range.value = can_signals[i].bytes[lsb_position].data.range.min;
            } else {
              can_signals[i].bytes[lsb_position].data.range.direction = -1;
            }
          }
        } else {
          if (can_signals[i].bytes[lsb_position].data.range.value > can_signals[i].bytes[lsb_position].data.range.min) {
            can_signals[i].bytes[lsb_position].data.range.value -= can_signals[i].bytes[lsb_position].data.range.speed;
          } else {
            if (can_signals[i].bytes[msb_position].data.range.value > can_signals[i].bytes[msb_position].data.range.min) {
              can_signals[i].bytes[msb_position].data.range.value -= 1;
              can_signals[i].bytes[lsb_position].data.range.value = can_signals[i].bytes[lsb_position].data.range.max;
            } else {
              can_signals[i].bytes[lsb_position].data.range.direction = 1;
            }
          }
        }  
      }
    }

    // build messages
    for (int i = 0; i < num_signals; i++) {
      uint8_t data[8] = {0};
      for (int j = 0; j < 8; j++) {
        ByteConfig &byte_cfg = can_signals[i].bytes[j];
        switch (byte_cfg.type) {
          case STATIC:
            data[j] = byte_cfg.data.static_value;
            break;
          case RANGE:
            data[j] = byte_cfg.data.range.value;
            //Serial.printf("CAN ID 0x%03X Byte %d Value: %d\n", can_signals[i].can_id, j, byte_cfg.data.range.value);
            break;
          case BITS:
            for (int b = 0; b < 8; b++) {
              if (byte_cfg.data.bits[b].value) {
                data[j] |= (1 << b);
              } else {
                data[j] &= ~(1 << b);
              }
            }
            break;
        }
      }

      // send messages
      twai_message_t message = {};
      message.identifier = can_signals[i].can_id;
      message.data_length_code = dlc;
      memcpy(message.data, data, dlc);
      message.flags = 0;

      esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(10));
      if (err == ESP_OK) {
        // Successfully queued for transmission
      } else {
        Serial.printf("Failed to transmit TWAI message ID 0x%03X: %d\n", can_signals[i].can_id, err);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(DATA_RATE)); // 100Hz
  }
}

  
void setup(void) {
  Serial.begin(115200);
  Serial.println("begin");
  drivers_init();
  set_backlight(80);

  screens_init();
  set_exio(EXIO_PIN4, Low);

  // Start CAN simulation sender task
  xTaskCreate(send_can_sim_task, "send_can_sim_task", 4096, NULL, 1, NULL);
}

void loop(void) {
  lv_timer_handler();

  // handle EXIO inputs - button press logic
  static bool last_exio_state[8] = {0};
  for (int i = 0; i < num_signals; i++) {
    for (int j = 0; j < 8; j++) {
      ByteConfig &byte_cfg = can_signals[i].bytes[j];
      if (byte_cfg.type == BITS) {
        for (int b = 0; b < 8; b++) {
          BitConfig &bit = byte_cfg.data.bits[b];
          if (bit.exio_pin >= 0) {
            bool exio_state = read_exio(bit.exio_pin);
            if (bit.button_mode == BUTTON_MODE_TOGGLE) {
              if (exio_state && !last_exio_state[bit.exio_pin]) {
                bit.value = !bit.value;
              }
            } else if (bit.button_mode == BUTTON_MODE_MOMENTARY) {
              bit.value = exio_state ? 0 : 1;
            }
            last_exio_state[bit.exio_pin] = exio_state;
          }
        }
      }
    }
  }

  delay(5);
}