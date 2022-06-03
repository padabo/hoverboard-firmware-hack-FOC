#define ADC_MIN 0                  // min ADC1-value while poti at minimum-position (0 - 1023)
#define ADC_MAX ((1 << 10) -1)               // max ADC1-value while poti at maximum-position (0 - 1023)
#define ADC_MID (ADC_MAX / 2)
#define DEAD_ZONE 64

#define STEERING_EAGLE_FACTOR (45.0f/1000.0f)
#define WHEELBASE 35
#define WHEEL_WIDTH 10
#define STEERING_TO_WHEEL_DIST 5

#define THROTTLE_MAX 1000
#define THROTTLE_REVERSE_MAX (THROTTLE_MAX * 3 / 10)

#define THROTTLE0_PIN 33
#define THROTTLE1_PIN 32
#define STEERING_PIN 35

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD 115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD 115200       // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME 0xABCD       // [-] Start frame definition for reliable serial communication
#define TIME_SEND 20             // [ms] Sending time interval
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#define BUFFERSIZE 128
#define VAL_CNT 2
