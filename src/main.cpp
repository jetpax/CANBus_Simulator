/* Basic Driver setup for the ESP32-S3 2.1 inch LCD Driver board  */
/* Author: Andy Valentine - Garage Tinkering                      */

#include <Arduino.h>
#include "CANBus_Driver.h"


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
    MODE_NONE,
    MODE_TOGGLE,
    MODE_MOMENTARY,
    MODE_TIMER
} ChangeMode;

typedef struct {
    int value;              // 0 or 1
    ChangeMode change_mode; // toggle/momentary/timer/none
    int gpio_pin;           // GPIO pin number, or -1 if not used
    int timer_duration;     // in ms, for timer mode
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

const int DATA_RATE = 100; // ms between CAN sends

CanMessageConfig can_signals[1];  // Declare array

void init_can_signals() {
  // Initialize CAN signal 0x551
  can_signals[0].can_id = 0x551;

  // Byte 0 - coolant temp
  can_signals[0].bytes[0].type = RANGE;
  can_signals[0].bytes[0].data.range.min = 0;
  can_signals[0].bytes[0].data.range.max = 180;
  can_signals[0].bytes[0].data.range.value = 0;
  can_signals[0].bytes[0].data.range.speed = 1;
  can_signals[0].bytes[0].data.range.direction = 1;
  can_signals[0].bytes[0].data.range.endian = NONE;

  // Byte 1 - oil temp
  can_signals[0].bytes[1].type = RANGE;
  can_signals[0].bytes[1].data.range.min = 0;
  can_signals[0].bytes[1].data.range.max = 180;
  can_signals[0].bytes[1].data.range.value = 0;
  can_signals[0].bytes[1].data.range.speed = 1;
  can_signals[0].bytes[1].data.range.direction = 1;
  can_signals[0].bytes[1].data.range.endian = NONE;

  // Byte 2 - oil pressure
  can_signals[0].bytes[2].type = RANGE;
  can_signals[0].bytes[2].data.range.min = 0;
  can_signals[0].bytes[2].data.range.max = 120;
  can_signals[0].bytes[2].data.range.value = 0;
  can_signals[0].bytes[2].data.range.speed = 1;
  can_signals[0].bytes[2].data.range.direction = 1;
  can_signals[0].bytes[2].data.range.endian = NONE;

  // Byte 3 - speed
  can_signals[0].bytes[3].type = RANGE;
  can_signals[0].bytes[3].data.range.min = 0;
  can_signals[0].bytes[3].data.range.max = 120;
  can_signals[0].bytes[3].data.range.value = 0;
  can_signals[0].bytes[3].data.range.speed = 1;
  can_signals[0].bytes[3].data.range.direction = 1;
  can_signals[0].bytes[3].data.range.endian = NONE;

  // Byte 4 - rpm msb
  can_signals[0].bytes[4].type = RANGE;
  can_signals[0].bytes[4].data.range.min = 0;
  can_signals[0].bytes[4].data.range.max = 4;
  can_signals[0].bytes[4].data.range.value = 0;
  can_signals[0].bytes[4].data.range.speed = 1;
  can_signals[0].bytes[4].data.range.direction = 1;
  can_signals[0].bytes[4].data.range.endian = MSB;

  // Byte 5 - rpm lsb
  can_signals[0].bytes[5].type = RANGE;
  can_signals[0].bytes[5].data.range.min = 0;
  can_signals[0].bytes[5].data.range.max = 255;
  can_signals[0].bytes[5].data.range.value = 0;
  can_signals[0].bytes[5].data.range.speed = 5;
  can_signals[0].bytes[5].data.range.direction = 1;
  can_signals[0].bytes[5].data.range.endian = LSB;

  // Byte 6 - bits
  can_signals[0].bytes[6].type = BITS;
  // Bit 0 - indicator left (timer mode)
  can_signals[0].bytes[6].data.bits[0].value = 1;
  can_signals[0].bytes[6].data.bits[0].change_mode = MODE_TIMER;
  can_signals[0].bytes[6].data.bits[0].gpio_pin = -1;
  can_signals[0].bytes[6].data.bits[0].timer_duration = 1000;
  // Bit 1 - indicator right (timer mode)
  can_signals[0].bytes[6].data.bits[1].value = 0;
  can_signals[0].bytes[6].data.bits[1].change_mode = MODE_TIMER;
  can_signals[0].bytes[6].data.bits[1].gpio_pin = -1;
  can_signals[0].bytes[6].data.bits[1].timer_duration = 1000;
  // Bits 2-6 - unused
  for (int i = 2; i <= 6; i++) {
    can_signals[0].bytes[6].data.bits[i].value = (i == 4 || i == 5) ? 1 : 0;
    can_signals[0].bytes[6].data.bits[i].change_mode = MODE_NONE;
    can_signals[0].bytes[6].data.bits[i].gpio_pin = -1;
    can_signals[0].bytes[6].data.bits[i].timer_duration = 0;
  }
  // Bit 7 - GPIO0 toggle
  can_signals[0].bytes[6].data.bits[7].value = 0;
  can_signals[0].bytes[6].data.bits[7].change_mode = MODE_TOGGLE;
  can_signals[0].bytes[6].data.bits[7].gpio_pin = 0;
  can_signals[0].bytes[6].data.bits[7].timer_duration = 0;

  // Byte 7 - unused
  can_signals[0].bytes[7].type = STATIC;
  can_signals[0].bytes[7].data.static_value = 0;
}

const int num_signals = sizeof(can_signals) / sizeof(CanMessageConfig);

// CONTROL VARIABLE INITWell t
bool initial_load             = false; // has the first data been received

// Function to read built-in GPIO pin (returns true when button is pressed)
bool read_gpio(uint8_t pin) {
  // Note: With INPUT_PULLUP, pin reads LOW when pressed, HIGH when not pressed
  return !digitalRead(pin);  // Invert to return true when pressed
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
    vTaskDelay(pdMS_TO_TICKS(DATA_RATE)); // 10Hz
  }
}

  
void setup(void) {
  Serial.begin(115200);
  Serial.println("begin");

  // Small delay to ensure boot process is complete
  delay(100);

  // Initialize CAN signals data
  init_can_signals();

  canbus_init();

  // Initialize GPIO0 as input with pull-up for button detection
  // GPIO0 can be used normally after boot
  pinMode(0, INPUT_PULLUP);
  Serial.println("GPIO0 configured as input with pull-up");

  // Start CAN simulation sender task
  xTaskCreate(send_can_sim_task, "send_can_sim_task", 4096, NULL, 1, NULL);
}

void loop(void) {

  // handle GPIO inputs - button press logic
  static bool last_gpio_state[40] = {0};  // Support up to GPIO39 on ESP32
  static unsigned long last_timer_toggle[8][8] = {{0}}; // [can_signal][bit]
  unsigned long now = millis();

  for (int i = 0; i < num_signals; i++) {
    for (int j = 0; j < 8; j++) {
      ByteConfig &byte_cfg = can_signals[i].bytes[j];
      if (byte_cfg.type == BITS) {
        for (int b = 0; b < 8; b++) {
          BitConfig &bit = byte_cfg.data.bits[b];
          if (bit.gpio_pin >= 0) {
            bool gpio_state = read_gpio(bit.gpio_pin);
            if (bit.change_mode == MODE_TOGGLE) {
              if (gpio_state && !last_gpio_state[bit.gpio_pin]) {
                bit.value = !bit.value;
                Serial.printf("GPIO%d pressed - Toggled bit %d in byte %d to %d (CAN ID: 0x%03X)\n",
                             bit.gpio_pin, b, j, bit.value, can_signals[i].can_id);
              }
            } else if (bit.change_mode == MODE_MOMENTARY) {
              bit.value = gpio_state ? 0 : 1;
            }
            last_gpio_state[bit.gpio_pin] = gpio_state;
          }
          // TIMER mode: toggle value if timer_duration has passed
          if (bit.change_mode == MODE_TIMER && bit.timer_duration > 0) {
            if (now - last_timer_toggle[i][b] >= (unsigned long)bit.timer_duration) {
              bit.value = !bit.value;
              last_timer_toggle[i][b] = now;
            }
          }
        }
      }
    }
  }

  delay(5);
}