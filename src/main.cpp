/* Basic Driver setup for the ESP32-S3 2.1 inch LCD Driver board  */
/* Author: Andy Valentine - Garage Tinkering                      */

#include <Arduino.h>
#include "CANBus_Driver.h"
#include "LVGL_Driver.h"
#include "I2C_Driver.h"

#define TAG "TWAI"

// Struct for CAN signal configuration

typedef enum {
  BUTTON_MODE_NONE,
  BUTTON_MODE_TOGGLE,
  BUTTON_MODE_MOMENTARY
} ButtonMode;

typedef enum {
  NONE,
  MSB,
  LSB
} SignificanceType;

typedef struct {
  uint8_t bit_pos;
  int value; // 0 or 1
  ButtonMode button_mode;
  int exio_pin; // EXIO pin number, or -1 if not used
} BitConfig;

typedef enum {
  SIGNAL_TYPE_BYTE,
  SIGNAL_TYPE_MULTI_BYTE,
  SIGNAL_TYPE_BIT,
  SIGNAL_TYPE_MULTI_BIT,
} CanSignalType;

typedef struct {
  uint32_t can_id;
  CanSignalType type;
  uint8_t byte_pos;
  union {
    struct { // For byte signals
      int min_value;
      int max_value;
      int value;
      int direction;
    } byte;
    BitConfig bit;
    struct { // For multi-bit signals
      BitConfig bits[8]; // Up to 8 bits per byte
      int num_bits;
    } multi_bit;
    struct {
      struct {
        uint8_t byte_pos;
        int min_value;
        int max_value;
        int value;
        int direction;
        SignificanceType significance;
      } bytes[8];
    } multi_byte;
  } data;
} CanSignalConfig;



const int DATA_RATE = 200; // ms between CAN sends

// SETUP THE CAN SIGNALS HERE
CanSignalConfig can_signals[] = {
  {0x551, SIGNAL_TYPE_BYTE, 0, {.byte={0,180,0,1}}},
  {0x552, SIGNAL_TYPE_BYTE, 4, {.byte={0,255,0,1}}},
  {0x555, SIGNAL_TYPE_MULTI_BIT, 2, {.multi_bit = { .bits = {
    {4,0,BUTTON_MODE_MOMENTARY,EXIO_PIN7},
    {5,0,BUTTON_MODE_TOGGLE,EXIO_PIN5}
  }, .num_bits = 2 }}},
  {0x556, SIGNAL_TYPE_MULTI_BYTE, 2, {.multi_byte = { .bytes = {
    {0,0,0,0,1,NONE},
    {1,0,0,0,1,NONE},
    {2,0,2,0,1,MSB},
    {3,0,255,0,1,LSB}
  }}}},
};

const int num_signals = sizeof(can_signals) / sizeof(CanSignalConfig);

// CONTROL VARIABLE INIT
bool initial_load             = false; // has the first data been received

lv_obj_t *main_scr;

void Drivers_Init(void) {
  I2C_Init();

  Serial.println("Scanning for TCA9554...");
  bool found = false;
  for (int attempt = 0; attempt < 10; attempt++) {
    if (I2C_ScanAddress(0x20)) { // 0x20 is default for TCA9554
      found = true;
      break;
    }
    delay(50); // wait a bit before retrying
  }

  if (!found) {
    Serial.println("TCA9554 not detected! Skipping expander init.");
  } else {
    TCA9554PWR_Init(0x00);
  }
  LCD_Init();
  CANBus_Init();
  Lvgl_Init();
}


// create the elements on the main scr
void Main_Scr_UI(void) {
   
}

// build the screens
void Screens_Init(void) {
  main_scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(main_scr, lv_color_make(0,0,0), 0);
  lv_screen_load(main_scr);

  Main_Scr_UI();
}

// task to avoid blocking LVGL with CAN receive
void Receive_CAN_Task(void *arg) {

  while (1) {
    twai_message_t message;
    // Use shorter wait time so we can periodically reset WDT
    esp_err_t err = twai_receive(&message, pdMS_TO_TICKS(100));

    if (err == ESP_OK) {
      
        Serial.print("Received CAN message: ID=0x");
        Serial.print(message.identifier, HEX);
        Serial.print(" DLC=");
        Serial.print(message.data_length_code);
        Serial.print(" Data=");
        for (int i = 0; i < message.data_length_code; i++) {
          Serial.printf("%02X ", message.data[i]);
        }
        Serial.println();

    } else if (err == ESP_ERR_TIMEOUT) {
        // No message within 100ms — that’s fine
        ESP_LOGW(TAG, "Reception timed out");
    } else {
        ESP_LOGE(TAG, "Message reception failed: %s", esp_err_to_name(err));
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // let scheduler breathe
  }
}

  // Task to send CAN message ID 0x551, byte 0, value 0-180-0, 10Hz
void Send_CAN_Sim_Task(void *arg) {
  const uint8_t dlc = 8;
  while (1) {
    for (int i = 0; i < num_signals; i++) {
      uint8_t data[8] = {0};
      if (can_signals[i].type == SIGNAL_TYPE_BYTE) {
        data[can_signals[i].byte_pos] = can_signals[i].data.byte.value;
      } else if (can_signals[i].type == SIGNAL_TYPE_BIT) {
        if (can_signals[i].data.bit.value) {
          data[can_signals[i].byte_pos] |= (1 << can_signals[i].data.bit.bit_pos);
        } else {
          data[can_signals[i].byte_pos] &= ~(1 << can_signals[i].data.bit.bit_pos);
        }
      } else if (can_signals[i].type == SIGNAL_TYPE_MULTI_BIT) {
        for (int b = 0; b < can_signals[i].data.multi_bit.num_bits; b++) {
          BitConfig* bit = &can_signals[i].data.multi_bit.bits[b];
          if (bit->value) {
            data[can_signals[i].byte_pos] |= (1 << bit->bit_pos);
          } else {
               data[can_signals[i].byte_pos] &= ~(1 << bit->bit_pos);
          }
        }
      } else if (can_signals[i].type == SIGNAL_TYPE_MULTI_BYTE) {
        for (int b = 0; b < 8; b++) {
          data[b] = can_signals[i].data.multi_byte.bytes[b].value;
        }
      }

      twai_message_t message = {};
      message.identifier = can_signals[i].can_id;
      message.data_length_code = dlc;
      memcpy(message.data, data, dlc);
      message.flags = 0;

      esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(10));
      // if (err == ESP_OK) {
      //   if (can_signals[i].type == SIGNAL_TYPE_BYTE) {
      //     Serial.printf("Sent CAN: ID=0x%03X Byte%d=%d\n", can_signals[i].can_id, can_signals[i].byte_pos, can_signals[i].data.byte.value);
      //   } else if (can_signals[i].type == SIGNAL_TYPE_BIT) {
      //     Serial.printf("Sent CAN: ID=0x%03X Byte%d.Bit%d=%d\n", can_signals[i].can_id, can_signals[i].byte_pos, can_signals[i].data.bit.bit_pos, can_signals[i].data.bit.value);
      //   } else if (can_signals[i].type == SIGNAL_TYPE_MULTI_BIT) {
      //     Serial.printf("Sent CAN: ID=0x%03X Byte%d MultiBits:", can_signals[i].can_id, can_signals[i].byte_pos);
      //     for (int b = 0; b < can_signals[i].data.multi_bit.num_bits; b++) {
      //       BitConfig* bit = &can_signals[i].data.multi_bit.bits[b];
      //       Serial.printf(" Bit%d=%d", bit->bit_pos, bit->value);
      //     }
      //     Serial.println();
      //   } else if (can_signals[i].type == SIGNAL_TYPE_MULTI_BYTE) {
      //     Serial.printf("Sent CAN: ID=0x%03X MultiBytes:", can_signals[i].can_id);
      //     for (int b = 0; b < 8; b++) {
      //       Serial.printf(" Byte%d=%d", b, can_signals[i].data.multi_byte.bytes[b].value);
      //     }
      //     Serial.println();
      //   }
      // } else {
      //   Serial.printf("CAN send error: %s\n", esp_err_to_name(err));
      // }

      // Update value for sweep/toggle
      if (can_signals[i].type == SIGNAL_TYPE_BYTE) {
        can_signals[i].data.byte.value += can_signals[i].data.byte.direction;
        if (can_signals[i].data.byte.value >= can_signals[i].data.byte.max_value) can_signals[i].data.byte.direction = -1;
        if (can_signals[i].data.byte.value <= can_signals[i].data.byte.min_value) can_signals[i].data.byte.direction = 1;
      } else if (can_signals[i].type == SIGNAL_TYPE_BIT) {
        // Only allow 0 or 1 for bit value
        if (can_signals[i].data.bit.value > 1) can_signals[i].data.bit.value = 1;
        if (can_signals[i].data.bit.value < 0) can_signals[i].data.bit.value = 0;
      } else if (can_signals[i].type == SIGNAL_TYPE_MULTI_BYTE) {
        int msb_position = -1, lsb_position = -1;
        for (int b = 0; b < 8; b++) {
          if (can_signals[i].data.multi_byte.bytes[b].significance == MSB) {
            msb_position = b;
          }
          if (can_signals[i].data.multi_byte.bytes[b].significance == LSB) {
            lsb_position = b;
          }
        }

        if (can_signals[i].data.multi_byte.bytes[lsb_position].direction == 1) {
          if (can_signals[i].data.multi_byte.bytes[lsb_position].value < can_signals[i].data.multi_byte.bytes[lsb_position].max_value) {
            can_signals[i].data.multi_byte.bytes[lsb_position].value++;
          } else {
            if (can_signals[i].data.multi_byte.bytes[msb_position].value < can_signals[i].data.multi_byte.bytes[msb_position].max_value) {
              can_signals[i].data.multi_byte.bytes[msb_position].value++;
              can_signals[i].data.multi_byte.bytes[lsb_position].value = can_signals[i].data.multi_byte.bytes[lsb_position].min_value;
            } else {
              can_signals[i].data.multi_byte.bytes[lsb_position].direction = -1;
            }
          }
        } else {
          if (can_signals[i].data.multi_byte.bytes[lsb_position].value > can_signals[i].data.multi_byte.bytes[lsb_position].min_value) {
            can_signals[i].data.multi_byte.bytes[lsb_position].value--;
          } else {
            if (can_signals[i].data.multi_byte.bytes[msb_position].value > can_signals[i].data.multi_byte.bytes[msb_position].min_value) {
              can_signals[i].data.multi_byte.bytes[msb_position].value--;
              can_signals[i].data.multi_byte.bytes[lsb_position].value = can_signals[i].data.multi_byte.bytes[lsb_position].max_value;
            } else {
              can_signals[i].data.multi_byte.bytes[lsb_position].direction = 1;
            }
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(DATA_RATE)); // 100Hz
  }
}
  
void setup(void) {
  Serial.begin(115200);
  Serial.println("begin");
  Drivers_Init();
  Set_Backlight(80);

  Screens_Init();
  Set_EXIO(EXIO_PIN4, Low);

  xTaskCreate(Receive_CAN_Task, "Receive_CAN_Task", 8192, NULL, 1, NULL);
  Serial.print("setup complete");

  // Start CAN simulation sender task
  xTaskCreate(Send_CAN_Sim_Task, "Send_CAN_Sim_Task", 4096, NULL, 1, NULL);
}

void loop(void) {
  lv_timer_handler();

  static bool last_exio_state[8] = {0}; // Support up to EXIO_PIN7
  for (int i = 0; i < num_signals; i++) {
    if (can_signals[i].type == SIGNAL_TYPE_BIT && can_signals[i].data.bit.exio_pin >= 0) {
      bool exio_state = Read_EXIO(can_signals[i].data.bit.exio_pin);
      if (can_signals[i].data.bit.button_mode == BUTTON_MODE_TOGGLE) {
        if (exio_state && !last_exio_state[can_signals[i].data.bit.exio_pin]) {
          can_signals[i].data.bit.value = !can_signals[i].data.bit.value;
        }
      } else if (can_signals[i].data.bit.button_mode == BUTTON_MODE_MOMENTARY) {
                can_signals[i].data.bit.value = exio_state ? 0 : 1;
      }
      last_exio_state[can_signals[i].data.bit.exio_pin] = exio_state;
    } else if (can_signals[i].type == SIGNAL_TYPE_MULTI_BIT) {
      for (int b = 0; b < can_signals[i].data.multi_bit.num_bits; b++) {
        BitConfig* bit = &can_signals[i].data.multi_bit.bits[b];
        if (bit->exio_pin >= 0) {
          bool exio_state = Read_EXIO(bit->exio_pin);
          if (bit->button_mode == BUTTON_MODE_TOGGLE) {
            if (exio_state && !last_exio_state[bit->exio_pin]) {
              bit->value = !bit->value;
            }
          } else if (bit->button_mode == BUTTON_MODE_MOMENTARY) {
            bit->value = exio_state ? 0 : 1;
          }
          last_exio_state[bit->exio_pin] = exio_state;
        }
      }
    }
  }

  delay(5);
}