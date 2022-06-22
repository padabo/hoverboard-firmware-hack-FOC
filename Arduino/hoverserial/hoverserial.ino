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
//   it is recommended to use the built-in Serial interface for full speed_per_wheel perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
//
// The code starts with zero speed_per_wheel and moves towards +
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
#include <SoftwareSerial.h>
// #include <BluetoothSerial.h> //Header File for Serial Bluetooth, will be added by default into Arduino
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

//#include <Wire.h>  // Wire Bibliothek hochladen
//#include <LiquidCrystal_I2C.h> // Vorher hinzugefügte LiquidCrystal_I2C Bibliothek hochladen

#include "defines.h"
#include "config.h"

//LiquidCrystal_I2C lcd(0x27, 20, 4);  //Hier wird das Display benannt (Adresse/Zeichen pro Zeile/Anzahl Zeilen). In unserem Fall „lcd“. Die Adresse des I²C Displays kann je nach Modul variieren.
SoftwareSerial HoverSerial_front(RX0, TX0); // RX, TX
SoftwareSerial HoverSerial_rear(RX1, TX1);  // RX, TX
// BluetoothSerial ESP_BT; //Object for Bluetooth


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);


uint32_t index_buff_vals[VAL_CNT];
uint32_t buff_vals[VAL_CNT][BUFFERSIZE];
uint32_t cur_buff_val_sum[VAL_CNT];
bool dsp_connected;
typedef struct
{
  uint16_t start;
  int16_t steer;
  int16_t speed_per_wheel;
  uint16_t checksum;
} SerialCommand;

typedef struct
{
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;

// ########################## SETUP ##########################
void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");
  Wire.begin(I2C_SDA, I2C_SCL);
  // ESP_BT.begin("ESP32_BobbyCon"); //Name of your Bluetooth Signal
  pinMode(THROTTLE0_PIN,INPUT);
  pinMode(STEERING_PIN,INPUT);
  scan_i2c();
  if(!(dsp_connected = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)))
    Serial.println("SSD1306 allocation failed");
  else
    display.display();
  //lcd.init(); //Im Setup wird der LCD gestartet
  //lcd.backlight(); //Hintergrundbeleuchtung einschalten (0 schaltet die Beleuchtung aus).


  HoverSerial_front.begin(HOVER_SERIAL_BAUD);

  HoverSerial_rear.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);

  for (int i = 0; i < VAL_CNT; i++)
  {
    cur_buff_val_sum[i] = index_buff_vals[i] = 0;
    for (int j = 0; j < BUFFERSIZE; j++)
      cur_buff_val_sum[i] += (buff_vals[i][j] = ADC_MID);
  }
  //init_debug_screen();
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;

uint32_t value_buffer(uint32_t in, int val)
{
  cur_buff_val_sum[val] -= buff_vals[val][index_buff_vals[val]];
  cur_buff_val_sum[val] += (buff_vals[val][index_buff_vals[val]] = in);
  index_buff_vals[val] = (index_buff_vals[val] + 1) % (BUFFERSIZE);
  return (cur_buff_val_sum[val] / (BUFFERSIZE));
}

void draw(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}

void testscrolltext(void) {
  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(F("scroll"));
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
  delay(1000);
}

/*
uint8_t val_len[20][4];

void init_debug_screen()
{
  for (int x = 0; x < 20; x++)
    for (int y = 0; y < 4; y++)
      val_len[x][y] = 0;
  lcd.clear();
  //Display_show_string(0, 0, "Phase:");
  //Display_show_string(0, 1, "Pos:");
  //Display_show_string(0, 2, "blockcur:");
  //Display_show_string(0, 3, "Pwr:               V");
}

void update_num(uint8_t x, uint8_t y, int value)
{
  char buff[] = "                    ";
  int8_t tmp_len;// = Display_show_int(x, y, value);
  int8_t old_len = val_len[x][y];
  if (tmp_len < old_len)
  {
    buff[old_len - tmp_len] = '\0';
    //Display_show_string(x - tmp_len, y, buff);
  }
  for (int i = 0; x < MAX(tmp_len, old_len); i++)
    val_len[x - i][y] = MAX(tmp_len - i, 0);
}

void update_debug_screen()
{
  //update_num(19, 0, phase_period[0] + phase_period[1] / 2);
  //update_num(17, 1, last_pos[0]);
  //update_num(19, 1, last_pos[1]);
  //update_num(19, 2, blockcurlr[0] + blockcurlr[1]);
  //Display_show_float(18, 3, ADC122BATTERY_VOLTAGE(battery_voltage), 5);
}
*/
// bobbycar
int sign(float in){
  if(in > 0.0f)
    return 1;
   else if(in < 0.0f)
    return -1;
   else
    return 0;
}

int clean_adc_full(uint32_t inval)
{
  int outval = (int)(inval) - ADC_MID;
  int abs_outval = abs(outval);
  if (abs_outval < (DEAD_ZONE / 2)) // deadzone
    return 0;
  else
    abs_outval -= (DEAD_ZONE / 2);
  if (abs_outval > (ADC_MAX - ADC_MID - DEAD_ZONE * 3 / 2))
    return THROTTLE_MAX * sign(outval);
  return abs_outval * THROTTLE_MAX / (ADC_MAX - ADC_MID - DEAD_ZONE * 3 / 2) * sign(outval);
}

int clean_adc_half(uint32_t inval)
{
  int outval = (uint32_t)inval;
  if (abs(outval) > (ADC_MAX - ((DEAD_ZONE * 3) / 2)))
    return THROTTLE_MAX;
  else if(abs(outval) < (((DEAD_ZONE * 3) / 2)))
    return 0;
  return outval * (THROTTLE_MAX / 2) / (ADC_MAX - DEAD_ZONE * 3);
}

int throttle_calc(int cleaned_adc)
{
  return cleaned_adc < 0 ?
    ((cleaned_adc * cleaned_adc / THROTTLE_MAX) * (-2) + cleaned_adc) / 3 *THROTTLE_REVERSE_MAX / THROTTLE_MAX
    : ((cleaned_adc * cleaned_adc / THROTTLE_MAX) * 2 + cleaned_adc) / 3;
}

int calc_torque(int throttle, int breaks)
{
  if (breaks == 0)
  { // drive forward
    return throttle;
  }
  else if (breaks == THROTTLE_MAX)
  { // drive backwards
    return -throttle;
  }
  else
  {
    return throttle - breaks;
  }
}

float calc_steering_eagle(int inval)
{
  return (float)inval * STEERING_EAGLE_FACTOR;
}

inline void calc_torque_per_wheel(int throttle, float steering_eagle, int *torque)
{
    torque[0] = torque[1] = torque[2] = torque[3] = throttle;
}

inline void swp(int *x, int *y)
{
  int tmp = *x;
  *x = *y;
  *y = tmp;
}
inline void sort_array(int x[], int cnt)
{
  for (int y = 0; y < cnt - 1; y++)
    for (int z = y + 1; z < cnt; z++)
      if (x[y] > x[z])
        swp(&x[y], &x[z]);
}
int calc_median(int x[], int cnt)
{
  sort_array(x, cnt);
  if (cnt % 2)
    return x[cnt / 2];
  else
    return (x[cnt / 2] + x[cnt / 2 + 1]) / 2;
}

// ########################## SEND ##########################
void Send(SoftwareSerial *board, int16_t speed0, int16_t speed1)
{
  SerialCommand Command;
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)speed0;
  Command.speed_per_wheel = (int16_t)speed1;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed_per_wheel);

  // Write to Serial
  board->write((uint8_t *)&Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
// Global variables
uint8_t idx = 0; // index_buff_vals for new data pointer
byte *p;         // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
SerialFeedback NewFeedback;
bool Receive(SoftwareSerial *board, SerialFeedback *out)
{

  uint16_t bufStartFrame; // Buffer Start Frame
  // byte buffer[sizeof(SerialFeedback)];
  //  Check for new data availability in the Serial buffer
  if (board->available())
  {
    incomingByte = board-> read();                            // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev; // Construct the start frame
  }
  else
  {
    return false;
  }

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.println(incomingByte, HEX);
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME)
  { // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  }
  else if (idx >= 2 && idx < sizeof(SerialFeedback))
  { // Save the new received data
    *p++ = incomingByte;
    idx++;
  }
  // Update previous states
  incomingBytePrev = incomingByte;
  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback))
  {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
    idx = 0; // Reset the index_buff_vals (it prevents to enter in this if condition in the next cycle)

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum)
    {
      // Copy the new data
      memcpy(&out, &NewFeedback, sizeof(SerialFeedback));

      // Print data to built-in Serial
      Serial.print("1: ");
      Serial.print(out->cmd1);
      Serial.print(" 2: ");
      Serial.print(out->cmd2);
      Serial.print(" 3: ");
      Serial.print(out->speedR_meas);
      Serial.print(" 4: ");
      Serial.print(out->speedL_meas);
      Serial.print(" 5: ");
      Serial.print(out->batVoltage);
      Serial.print(" 6: ");
      Serial.print(out->boardTemp);
      Serial.print(" 7: ");
      Serial.println(out->cmdLed);
      return true;
    }
    else
    {
      Serial.println("Non-valid data skipped");
      return false;
    }
  }
  else{
    return false;
  }
}

SerialFeedback SerialFeedback_front;
SerialFeedback SerialFeedback_rear;

int torgue[4];
int speed_per_wheel[4];
int speed;

void scan_i2c(){
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }  
}

void loop(void)
{
  int a0;
  unsigned long timeNow = millis();
  int throttle = throttle_calc(clean_adc_full(value_buffer(analogRead(THROTTLE0_PIN),0)));
  float steering = calc_steering_eagle(clean_adc_full(a0 = value_buffer(analogRead(STEERING_PIN),1)));
  // Check for new received data
  //if(Receive(&HoverSerial_front, &SerialFeedback_front) || Receive(&HoverSerial_rear, &SerialFeedback_rear)){
  //  speed_per_wheel[0] = SerialFeedback_front.speedL_meas;
  //  speed_per_wheel[1] = SerialFeedback_front.speedR_meas;
  //  speed_per_wheel[2] = SerialFeedback_rear.speedL_meas;
  //  speed_per_wheel[3] = SerialFeedback_rear.speedR_meas;
  //  speed = calc_median(speed_per_wheel,4);
  //}  // Send commands
  if (iTimeSend > timeNow)
    return;
  iTimeSend = timeNow + TIME_SEND;
  calc_torque_per_wheel(throttle, steering, torgue);
  Send(&HoverSerial_front, torgue[0], torgue[1]);
  Send(&HoverSerial_rear, torgue[2], torgue[3]);
  Serial.print("Set: Throttle: ");
  Serial.print(throttle);
  Serial.print("  steering: ");
  Serial.print(a0);
  Serial.print("  torgue: ");
  Serial.print(torgue[0]);
  Serial.print(" ");
  Serial.print(torgue[1]);
  Serial.print(" ");
  Serial.print(torgue[2]);
  Serial.print(" ");
  Serial.println(torgue[3]);
  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);
}

// ########################## END ##########################
