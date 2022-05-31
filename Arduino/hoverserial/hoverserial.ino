// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// 
// The code starts with zero speed and moves towards +
//
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frame definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
#define SPEED_STEP          20          // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)


#define BUFFERSIZE 32
#define VAL_CNT 4

#include <SoftwareSerial.h>

#include <math.h>
#include <stdlib.h>
#include "bldc.h"  // for the main control variables

SoftwareSerial HoverSerial0(2,3);        // RX, TX
SoftwareSerial HoverSerial1(4,5);        // RX, TX

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setup() 
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");

  HoverSerial0.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial0.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void Receive()
{
    // Check for new data availability in the Serial buffer
    if (HoverSerial0.available()) {
        incomingByte 	  = HoverSerial0.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.print(incomingByte);
        return;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;	
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    }	
    
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            Serial.print("1: ");   Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
            Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
        } else {
          Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;

uint32_t index[VAL_CNT];
uint32_t buff_vals[VAL_CNT][BUFFERSIZE];
uint32_t cur_buff_val_sum[VAL_CNT];

uint32_t value_buffer(uint32_t in,int val){
  cur_buff_val_sum[val] -= buff_vals[val][index[val]];
  cur_buff_val_sum[val] += (buff_vals[val][index[val]] = (in >> 16));
  index[val] = (index[val] + 1) % BUFFERSIZE;
  return (cur_buff_val_sum[val] / (BUFFERSIZE)) << 16;
}

uint8_t val_len[20][4];

void init_debug_screen(){
  for(int x = 0; x < 20; x++)
    for(int y = 0; y < 4; y++)
      val_len[x][y] = 0;
  Display_Clear();
  Display_show_string(0,0, "Phase:");
  Display_show_string(0,1, "Pos:");
  Display_show_string(0,2, "blockcur:");
  Display_show_string(0,3, "Pwr:               V");
}

void update_num(uint8_t x, uint8_t y, int value){
  char buff[] = "                    ";
  int8_t tmp_len = Display_show_int(x, y, value);
  int8_t old_len = val_len[x][y];
  if(tmp_len < old_len){
    buff[old_len - tmp_len]='\0';
    Display_show_string(x-tmp_len,y,buff);
  }
  for(int i = 0; x < MAX(tmp_len,old_len); i++)
    val_len[x-i][y] = MAX(tmp_len-i,0);
}

void update_debug_screen(){
  update_num(19,0, phase_period[0]+phase_period[1]/2);
  update_num(17,1, last_pos[0]);
  update_num(19,1, last_pos[1]);
  update_num(19,2, blockcurlr[0]+blockcurlr[1]);
  Display_show_float(18,3,ADC122BATTERY_VOLTAGE(battery_voltage),5);
}

//bobbycar
int clean_adc_full(uint32_t inval){
  int outval = (uint32_t)(inval >> 16) - ADC_MID;
  if(abs(outval) < (DEAD_ZONE / 2))
    return 0;
  else
    outval -= (DEAD_ZONE / 2) * SIGN(outval);
  if(abs(outval) > (ADC_MAX / 2 - ((DEAD_ZONE * 3) / 2)))
    return THROTTLE_MAX * SIGN(outval);
  return outval * THROTTLE_MAX / (ADC_MAX / 2 - ((DEAD_ZONE*3)/2));
}

int clean_adc_half(uint32_t inval){
  int outval = (uint32_t)(inval >> 16);
  if(abs(outval) > (ADC_MAX - ((DEAD_ZONE * 3) / 2)))
    return THROTTLE_MAX;
  return outval * THROTTLE_MAX / (ADC_MAX - ((DEAD_ZONE*3)/2));
}

int throttle_calc(int cleaned_adc){
  return cleaned_adc < 0 ?
    ((cleaned_adc * cleaned_adc  / THROTTLE_REVERSE_MAX ) * 2 + cleaned_adc ) / 3 :
    ((cleaned_adc * cleaned_adc / THROTTLE_MAX ) * 2 + cleaned_adc ) / 3;
}

int calc_torque(int throttle,int breaks){
  if(breaks == 0){  // drive forward
    return throttle;
}
  else if(breaks == PWM_MAX){  // drive backwards
    return -throttle;
  }
  else{
    return throttle-breaks;
  }
}

float calc_steering_eagle(int inval){
  return (float)inval * STEERING_EAGLE_FACTOR;
}

inline void calc_torque_per_wheel(int throttle, float steering_eagle, int* torque){
  int back_wheel = (float)WHEELBASE / tan(abs(steering_eagle));
  int radius_main = sqrt(pow(back_wheel, 2)+pow(WHEELBASE / 2 ,2));
  int wheel_bl = (back_wheel + (WHEEL_WIDTH * sign(steering_eagle) / 2 - STEERING_TO_WHEEL_DIST));
  int wheel_br = (back_wheel - (WHEEL_WIDTH * sign(steering_eagle) / 2 - STEERING_TO_WHEEL_DIST));
  torque[0] = (sqrt(pow(wheel_bl, 2)+pow(WHEELBASE,2)) + STEERING_TO_WHEEL_DIST * sign(steering_eagle)) * throttle / radius_main;
  torque[1] = (sqrt(pow(wheel_br, 2)+pow(WHEELBASE,2)) - STEERING_TO_WHEEL_DIST * sign(steering_eagle)) * throttle / radius_main;
  torque[2] = (back_wheel + WHEEL_WIDTH / 2 * sign(steering_eagle)) * throttle / radius_main;
  torque[3] = (back_wheel - WHEEL_WIDTH / 2 * sign(steering_eagle)) * throttle / radius_main;
}



void device_init(){
  for(int i = 0; i < VAL_CNT ; i++){
    cur_buff_val_sum[i] = index[i] = 0;
    for(int j = 0; j < BUFFERSIZE;j++)
      cur_buff_val_sum[i] += (buff_vals[i][j] = ADC_MID);
  }
  init_debug_screen();
}

void device_specific(){
  int tmp_throttle_per_wheel[2];
  int tmp_throttle;
    calc_torque_per_wheel(
      tmp_throttle = throttle_calc(
        clean_adc_full(virtual_ival[0][0])
      ),
      calc_steering_eagle(clean_adc_full(virtual_ival[0][1])),
      tmp_throttle_per_wheel);
    #ifdef BEEPS_BACKWARD
    if(tmp_throttle < THROTTLE_REVERSE_MAX / 10)
      set_buzzer(reverseSound);
    else
      stop_buzzer();
    #endif
    set_throttle(tmp_throttle_per_wheel[0], tmp_throttle_per_wheel[1]);
  if(!get_mainCounter()%1000)
    update_debug_screen();
}

void loop(void)
{ 
  unsigned long timeNow = millis();

  // Check for new received data
  Receive();

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  Send(0, iTest);

  // Calculate test command signal
  iTest += iStep;

  // invert step if reaching limit
  if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST)
    iStep = -iStep;

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
}

// ########################## END ##########################
