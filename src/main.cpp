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

const int DATA_RATE = 200; // ms between CAN sends

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
  
  // Diagnostic counters
  uint32_t frame_count = 0;
  uint32_t success_count = 0;
  uint32_t error_count = 0;
  uint32_t last_status_log = 0;
  uint32_t last_bus_state = 0xFF; // Invalid initial state
  uint32_t last_tx_err = 0;
  uint32_t last_rx_err = 0;
  uint32_t loop_count = 0;
  uint32_t consecutive_errors = 0;
  
  Serial.println("[TX] Starting CAN transmission task");
  
  // Log initial bus state
  twai_status_info_t init_status;
  if (twai_get_status_info(&init_status) == ESP_OK) {
    Serial.printf("[TX] Initial bus state: %d, tx_err=%d, rx_err=%d\n\r", 
                 init_status.state, init_status.tx_error_counter, init_status.rx_error_counter);
    last_bus_state = init_status.state;
    last_tx_err = init_status.tx_error_counter;
    last_rx_err = init_status.rx_error_counter;
  }
  
  while (1) {
    loop_count++;
    uint32_t now = millis();

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
            //Serial.printf("CAN ID 0x%03X Byte %d Value: %d\n\r", can_signals[i].can_id, j, byte_cfg.data.range.value);
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

      // Check bus state before attempting transmission
      twai_status_info_t pre_tx_status;
      bool skip_transmission = false;
      uint32_t tec_before = 0;
      if (twai_get_status_info(&pre_tx_status) == ESP_OK) {
        tec_before = pre_tx_status.tx_error_counter;
        if (pre_tx_status.state == TWAI_STATE_RECOVERING || pre_tx_status.state == TWAI_STATE_BUS_OFF) {
          skip_transmission = true;
          if (frame_count % 10 == 0) { // Log every 10th skipped frame to avoid spam
            Serial.printf("[TX] Skipping transmission - bus in state %d (RECOVERING/BUS_OFF)\n\r", pre_tx_status.state);
          }
        }
      }
      
      if (skip_transmission) {
        frame_count++;
        error_count++;
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay when skipping
        continue; // Skip to next message
      }
      
      frame_count++;
      
      esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(10));
      
      // Get TEC after transmission (wait longer to ensure TEC updates)
      // TEC updates happen asynchronously after frame transmission completes
      vTaskDelay(pdMS_TO_TICKS(20)); // Wait 20ms for transmission to complete and TEC to update
      uint32_t tec_after = 0;
      twai_status_info_t post_tx_status;
      if (twai_get_status_info(&post_tx_status) == ESP_OK) {
        tec_after = post_tx_status.tx_error_counter;
        int32_t tec_delta = (int32_t)tec_after - (int32_t)tec_before;
        
        // Log TEC changes to understand ACK pattern
        // CAN bus TEC rules:
        // - Successful TX with ACK: TEC decreases by 1 (min 0)
        // - ACK error (no ACK): TEC increases by 8
        // - Bit error: TEC increases by 1
        if (tec_delta != 0) {
          if (tec_delta == -1) {
            // Successful transmission (ACK received)
            // Don't log every success to avoid spam
          } else if (tec_delta >= 8 && (tec_delta % 8 == 0)) {
            // Multiple ACK errors (8, 16, 24, etc.)
            uint32_t ack_errors = tec_delta / 8;
            Serial.printf("[TX] Frame #%lu: %lu ACK ERROR(S) (TEC +%d) - No receiver ACKed %lu frame(s)!\n\r", 
                         frame_count, ack_errors, tec_delta, ack_errors);
          } else if (tec_delta == 8) {
            Serial.printf("[TX] Frame #%lu: ACK ERROR (TEC +8) - No receiver ACKed!\n\r", frame_count);
          } else if (tec_delta > 0 && tec_delta < 8) {
            Serial.printf("[TX] Frame #%lu: Bit/Form/Stuff error (TEC +%d)\n\r", frame_count, tec_delta);
          } else if (tec_delta > 8) {
            Serial.printf("[TX] Frame #%lu: Multiple errors accumulated (TEC +%d) - check queue!\n\r", frame_count, tec_delta);
          } else {
            Serial.printf("[TX] Frame #%lu: TEC decreased by %d (recovery/success)\n\r", frame_count, -tec_delta);
          }
        }
      }
      
      if (err == ESP_OK) {
        success_count++;
        consecutive_errors = 0; // Reset on success
      } else {
        error_count++;
        consecutive_errors++;
        Serial.printf("[TX] ERROR: Frame #%lu, ID 0x%03X failed: err=%d (0x%X), consecutive=%lu\n\r", 
                     frame_count, can_signals[i].can_id, err, err, consecutive_errors);
        
        // Get detailed bus status on error
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
          // Track error counter changes
          if (status.tx_error_counter != last_tx_err) {
            Serial.printf("[TX] TX ERROR COUNTER INCREASED: %d -> %d (+%d) at frame #%lu\n\r",
                         last_tx_err, status.tx_error_counter, 
                         status.tx_error_counter - last_tx_err, frame_count);
            last_tx_err = status.tx_error_counter;
          }
          if (status.rx_error_counter != last_rx_err) {
            Serial.printf("[TX] RX ERROR COUNTER INCREASED: %d -> %d (+%d) at frame #%lu\n\r",
                         last_rx_err, status.rx_error_counter,
                         status.rx_error_counter - last_rx_err, frame_count);
            last_rx_err = status.rx_error_counter;
          }
          
          Serial.printf("[TX] Bus Status: state=%d, tx_err=%d, rx_err=%d, msgs_to_tx=%d, msgs_to_rx=%d, tx_failed=%d, rx_missed=%d, arb_lost=%d, bus_err=%d\n\r",
                       status.state, status.tx_error_counter, status.rx_error_counter,
                       status.msgs_to_tx, status.msgs_to_rx, status.tx_failed_count,
                       status.rx_missed_count, status.arb_lost_count, status.bus_error_count);
          
          // Interpret state value
          const char* state_str = "UNKNOWN";
          switch(status.state) {
            case 0: state_str = "STOPPED"; break;
            case 1: state_str = "RUNNING"; break;
            case 2: state_str = "RECOVERING"; break;
            case 3: state_str = "BUS_OFF"; break;
          }
          Serial.printf("[TX] Bus State: %d (%s)\n\r", status.state, state_str);
          
          // Analyze error pattern
          if (status.tx_error_counter > last_tx_err && status.tx_error_counter < 256) {
            // TEC is increasing - likely ACK errors (no receiver ACKing frames)
            // In CAN bus, if no node ACKs a frame, TEC increases by 8
            // If a node ACKs but there's a bit error, TEC increases by 1
            uint32_t tec_increase = status.tx_error_counter - last_tx_err;
            if (tec_increase >= 8) {
              Serial.printf("[TX] *** ACK ERROR DETECTED: TEC increased by %d (typical ACK error = +8) ***\n\r", tec_increase);
              Serial.printf("[TX] *** NO RECEIVER IS ACKING FRAMES - Check GVRET receiver! ***\n\r");
            } else {
              Serial.printf("[TX] TEC increased by %d (bit/form/stuff error = +1 to +8)\n\r", tec_increase);
            }
          }
          
          // Log state changes
          if (status.state != last_bus_state) {
            Serial.printf("[TX] BUS STATE CHANGE: %d -> %d (at frame #%lu)\n\r", 
                         last_bus_state, status.state, frame_count);
            last_bus_state = status.state;
            
            if (status.state == TWAI_STATE_BUS_OFF) {
              Serial.println("[TX] WARNING: Bus entered BUS_OFF state! Attempting recovery...");
              
              // Attempt bus recovery
              esp_err_t recover_err = twai_initiate_recovery();
              if (recover_err == ESP_OK) {
                Serial.println("[TX] Recovery initiated, waiting for completion...");
                // Wait for recovery to complete (can take up to 128 * 11 bit times)
                vTaskDelay(pdMS_TO_TICKS(100));
                
                // Check if recovery completed
                twai_status_info_t recovery_status;
                if (twai_get_status_info(&recovery_status) == ESP_OK) {
                  if (recovery_status.state == TWAI_STATE_RECOVERING) {
                    Serial.println("[TX] Bus still recovering, waiting more...");
                    vTaskDelay(pdMS_TO_TICKS(100));
                  }
                  
                  // Get final status
                  twai_get_status_info(&recovery_status);
                  if (recovery_status.state == TWAI_STATE_RUNNING) {
                    Serial.printf("[TX] Recovery successful! Bus back to RUNNING state (tx_err=%d, rx_err=%d)\n\r",
                                 recovery_status.tx_error_counter, recovery_status.rx_error_counter);
                    last_bus_state = recovery_status.state;
                  } else {
                    Serial.printf("[TX] Recovery incomplete, bus state: %d. Restarting driver...\n\r", recovery_status.state);
                    // Try restarting the driver
                    twai_stop();
                    vTaskDelay(pdMS_TO_TICKS(50));
                    if (twai_start() == ESP_OK) {
                      Serial.println("[TX] Driver restarted successfully");
                      last_bus_state = TWAI_STATE_RUNNING;
                    } else {
                      Serial.println("[TX] ERROR: Failed to restart driver!");
                    }
                  }
                }
              } else {
                Serial.printf("[TX] ERROR: Failed to initiate recovery: %d\n\r", recover_err);
                // Fallback: try stopping and restarting
                Serial.println("[TX] Attempting driver restart as fallback...");
                twai_stop();
                vTaskDelay(pdMS_TO_TICKS(50));
                if (twai_start() == ESP_OK) {
                  Serial.println("[TX] Driver restarted successfully");
                  last_bus_state = TWAI_STATE_RUNNING;
                } else {
                  Serial.println("[TX] ERROR: Failed to restart driver!");
                }
              }
            }
          }
          
          // Also check for BUS_OFF state on every error (not just state changes)
          if (status.state == TWAI_STATE_BUS_OFF && last_bus_state != TWAI_STATE_BUS_OFF) {
            // This handles the case where we're already in BUS_OFF but didn't catch the transition
            Serial.println("[TX] Detected BUS_OFF state, attempting recovery...");
            esp_err_t recover_err = twai_initiate_recovery();
            if (recover_err == ESP_OK) {
              vTaskDelay(pdMS_TO_TICKS(100));
              twai_status_info_t recovery_status;
              if (twai_get_status_info(&recovery_status) == ESP_OK && recovery_status.state == TWAI_STATE_RUNNING) {
                Serial.println("[TX] Recovery successful!");
                last_bus_state = TWAI_STATE_RUNNING;
              } else {
                twai_stop();
                vTaskDelay(pdMS_TO_TICKS(50));
                twai_start();
                last_bus_state = TWAI_STATE_RUNNING;
              }
            }
          }
        } else {
          Serial.println("[TX] ERROR: Failed to get bus status");
        }
      }
    }
    
    // Periodic status report every 5 seconds
    if (now - last_status_log >= 5000) {
      twai_status_info_t status;
      if (twai_get_status_info(&status) == ESP_OK) {
        float success_rate = frame_count > 0 ? (100.0f * success_count / frame_count) : 0.0f;
        Serial.printf("[TX] STATUS REPORT (loop #%lu):\n\r", loop_count);
        Serial.printf("  Frames: total=%lu, success=%lu, errors=%lu (%.1f%% success)\n\r",
                     frame_count, success_count, error_count, success_rate);
        Serial.printf("  Bus: state=%d, tx_err=%d, rx_err=%d\n\r",
                     status.state, status.tx_error_counter, status.rx_error_counter);
        Serial.printf("  Queue: tx=%d, rx=%d\n\r", status.msgs_to_tx, status.msgs_to_rx);
        Serial.printf("  Errors: tx_failed=%d, rx_missed=%d, arb_lost=%d, bus_err=%d\n\r",
                     status.tx_failed_count, status.rx_missed_count, status.arb_lost_count, status.bus_error_count);
        
        // Diagnostic: If TEC is increasing but tx_failed=0, frames are queued but not ACKed
        if (status.tx_error_counter > 0 && status.tx_failed_count == 0 && status.tx_error_counter < 256) {
          Serial.printf("  *** DIAGNOSIS: TEC=%d but tx_failed=0 -> Frames queued but NOT ACKed! ***\n\r", status.tx_error_counter);
          Serial.printf("  *** This indicates NO RECEIVER is ACKing frames (ACK errors) ***\n\r");
        }
        
        if (status.state != TWAI_STATE_RUNNING) {
          Serial.printf("[TX] WARNING: Bus not in RUNNING state! Current state: %d\n\r", status.state);
        }
      }
      last_status_log = now;
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
                Serial.printf("GPIO%d pressed - Toggled bit %d in byte %d to %d (CAN ID: 0x%03X)\n\r",
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