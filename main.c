/*********************************************************************
 *   A demo example using several of the peripherals on the Embedded
 *   Artists Base Board using the LPC1769 Cortex M3 Processer
 **********************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "stdbool.h"
#include "joystick.h"
#include "pca9532.h" //LED Array
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "led7seg.h"
#include "stdio.h"
#include "light.h"
#include "lpc17xx_uart.h"
#include "uart2.h"
#include <string.h>
#define UART_PORT (LPC_UART_TypeDef *)LPC_UART3

typedef enum {
 MODE_STATIONARY, MODE_LAUNCH, MODE_RETURN
} system_mode_t;
volatile int state;
volatile system_mode_t mode;

//booleans for UART
int isUARTStationaryModeSent = FALSE;
int isUARTLaunchModeSent = FALSE;
int isUARTVeerSent = FALSE;
int isUARTTempSent = FALSE;
int isUARTReturnModeSent = FALSE;
int CLEAR_WARNING = FALSE;
int isObstacleNear = FALSE;
float temp_threshold = 32.0; //change fuel tank temp threshold here

volatile int obstacleNearTime = 0;

uint8_t rev_buf[255];
uint8_t rev_cnt = 0;
uint32_t isReceived = 0;


//Strings for OLED
int8_t OLED_EXT_MODE[15];
int8_t OLED_X[15];
int8_t OLED_Y[15];
int8_t OLED_Z[15];
int8_t OLED_LIGHT[15];
int8_t OLED_TEMPERATURE[15];

void pinsel_uart3(void){
 PINSEL_CFG_Type PinCfg;
 PinCfg.Funcnum = 2;
 PinCfg.Pinnum = 0;
 PinCfg.Portnum = 0;
 PINSEL_ConfigPin(&PinCfg);
 PinCfg.Pinnum = 1;
 PINSEL_ConfigPin(&PinCfg);
}

// UART Receive Callback Function
// It will be called when a message is received
void UART_IntReceive(void)
{
 /* Read the received data */
 if(UART_Receive(UART_PORT, &rev_buf[rev_cnt], 1, NONE_BLOCKING) == 1) {
  if(rev_buf[rev_cnt] == '\r'){
   isReceived = 1;
  }
  rev_cnt++;
  if(rev_cnt == 255) rev_cnt = 0;
 }
}

//set up UART interrupt
void UART_Receive_Int_Init()
{
 // UART Config
 UART_CFG_Type UARTConfigStruct;
 // UART FIFO config
 UART_FIFO_CFG_Type UARTFIFOConfigStruct;
 // Pin config
 PINSEL_CFG_Type PinCfg;
 PinCfg.Funcnum = 2;
 PinCfg.OpenDrain = 0;
 PinCfg.Pinmode = 0;
 PinCfg.Pinnum = 0;
 PinCfg.Portnum = 0;
 PINSEL_ConfigPin(&PinCfg);
 PinCfg.Pinnum = 1;
 PINSEL_ConfigPin(&PinCfg);
 /* Init UART Config to default state:
  * Baudrate = 9600bps 8N1 */
 UART_ConfigStructInit(&UARTConfigStruct);
 /* Set Baudrate to 115200 */
 UARTConfigStruct.Baud_rate = 115200;
 // Init UART3
 UART_Init(UART_PORT, &UARTConfigStruct);
 //------------------------------------—

 /* Init FIFOConfig to default state,
  * using FIFO will allow LPC to have more time to handle interrupts,
  * and also prevent data loss at high rate
  */
 UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
 // Init FIFO for UART3
 UART_FIFOConfig(UART_PORT, &UARTFIFOConfigStruct);
 //------------------------------------------------

 // Setup callback for Receive Message
 UART_SetupCbs(UART_PORT, 0, (void *)UART_IntReceive);
 // Enable UART Transmit
 UART_TxCmd(UART_PORT, ENABLE);
 /* Enable UART Rx interrupt */
 UART_IntConfig(UART_PORT, UART_INTCFG_RBR, ENABLE);
 /* Enable Interrupt for UART3 */
 NVIC_EnableIRQ(UART3_IRQn);
}

//UART3 interrupt handler
void UART3_IRQHandler(void)
{
 UART3_StdIntHandler();
}

//UART Sent, which includes message pre-processing that
//change '\0' to '\r\n'
void UART_Send_Message(char* msg){
 //handling message, eg \0 to \r\n
 int len = strlen(msg);
 msg[len] = '\r';// \0 -> \r
 msg[++len] = '\n';// append \n behind
 UART_Send(UART_PORT, (uint8_t*)msg, (uint32_t)len, BLOCKING);
}

//UART Receive Post-processing that
//change '\r' to '\0'
void UART_RcvMsgHandling(){
 rev_buf[rev_cnt-1] = '\0';
 rev_cnt = 0;
}


void init_uart(void){
 UART_CFG_Type uartCfg;
 uartCfg.Baud_rate = 115200;
 uartCfg.Databits = UART_DATABIT_8;
 uartCfg.Parity = UART_PARITY_NONE;
 uartCfg.Stopbits = UART_STOPBIT_1;
 //pin select for uart3;
 pinsel_uart3();
 //supply power & setup working parameters for uart3
 UART_Init(LPC_UART3, &uartCfg);
 //enable transmit for uart3
 UART_TxCmd(LPC_UART3, ENABLE);
}

static uint8_t barPos = 2;
static uint32_t tempMax = 28;

//Local functions
volatile uint32_t msTicks;
volatile uint32_t countTicksR = 0;
volatile uint32_t countTicksL = 0;
volatile uint32_t countTicksS = 0;

void SysTick_Handler(void) {
 msTicks++;
}

uint32_t getTicks(void) {
 return msTicks;
}

void Delay(uint32_t dlyTicks)
{
 uint32_t curTicks;

 curTicks = msTicks;
 while ((msTicks - curTicks) < dlyTicks) ;
}

static void moveBar(uint8_t steps, uint8_t dir)
{
 uint16_t ledOn = 0;

 if (barPos == 0)
  ledOn = (1 << 0) | (3 << 14);
 else if (barPos == 1)
  ledOn = (3 << 0) | (1 << 15);
 else
  ledOn = 0x07 << (barPos-2);

 barPos += (dir * steps);
 barPos = (barPos % 16);

 //uint16_t showTemperatureValue = (temperature / tempMax) * 0xffff; //

 pca9532_setLeds(ledOn, 0xffff);//showTemperatureValue);
}

static void drawOled(uint8_t joyState) {
 static int wait = 0;
 static uint8_t currX = 48;
 static uint8_t currY = 32;
 static uint8_t lastX = 0;
 static uint8_t lastY = 0;

 if ((joyState & JOYSTICK_CENTER) != 0) {
  oled_clearScreen(OLED_COLOR_BLACK);
  return;
 }

 if (wait++ < 3)
  return;

 wait = 0;

 if ((joyState & JOYSTICK_UP) != 0 && currY > 0) {
  currY--;
 }

 if ((joyState & JOYSTICK_DOWN) != 0 && currY < OLED_DISPLAY_HEIGHT - 1) {
  currY++;
 }

 if ((joyState & JOYSTICK_RIGHT) != 0 && currX < OLED_DISPLAY_WIDTH - 1) {
  currX++;
 }

 if ((joyState & JOYSTICK_LEFT) != 0 && currX > 0) {
  currX--;
 }

 if (lastX != currX || lastY != currY) {
  oled_putPixel(currX, currY, OLED_COLOR_WHITE);
  lastX = currX;
  lastY = currY;
 }
}

#define NOTE_PIN_HIGH() GPIO_SetValue(0, 1<<26);
#define NOTE_PIN_LOW()  GPIO_ClearValue(0, 1<<26);

static int accelorometerReadings(int8_t xoff, int8_t yoff, int8_t zoff) {

 int8_t x = 0;
 int8_t y = 0;
 int8_t z = 0;
 //returns value in Gs
 acc_read(&x, &y, &z);
 xoff = 0 - x;
 yoff = 0 - y;
 zoff = 64 - z;
}

static uint32_t notes[] = { 2272, // A - 440 Hz
  2024, // B - 494 Hz
  3816, // C - 262 Hz
  3401, // D - 294 Hz
  3030, // E - 330 Hz
  2865, // F - 349 Hz
  2551, // G - 392 Hz
  1136, // a - 880 Hz
  1012, // b - 988 Hz
  1912, // c - 523 Hz
  1703, // d - 587 Hz
  1517, // e - 659 Hz
  1432, // f - 698 Hz
  1275, // g - 784 Hz
};

static void playNote(uint32_t note, uint32_t durationMs) {

 uint32_t t = 0;

 if (note > 0) {

  while (t < (durationMs * 1000)) {
   NOTE_PIN_HIGH();
   Timer0_us_Wait(note / 2);
   //delay32Us(0, note / 2);

   NOTE_PIN_LOW();
   Timer0_us_Wait(note / 2);
   //delay32Us(0, note / 2);

   t += note;
  }

 } else {
  Timer0_Wait(durationMs);
  //delay32Ms(0, durationMs);
 }
}

static uint32_t getNote(uint8_t ch) {
 if (ch >= 'A' && ch <= 'G')
  return notes[ch - 'A'];

 if (ch >= 'a' && ch <= 'g')
  return notes[ch - 'a' + 7];

 return 0;
}

static uint32_t getDuration(uint8_t ch) {
 if (ch < '0' || ch > '9')
  return 400;

 /* number of ms */

 return (ch - '0') * 200;
}

static uint32_t getPause(uint8_t ch) {
 switch (ch) {
 case '+':
  return 0;
 case ',':
  return 5;
 case '.':
  return 20;
 case '_':
  return 30;
 default:
  return 5;
 }
}

static void playSong(uint8_t *song) {
 uint32_t note = 0;
 uint32_t dur = 0;
 uint32_t pause = 0;

 /*
  * A song is a collection of tones where each tone is
  * a note, duration and pause, e.g.
  *
  * "E2,F4,"
  */
 while (*song != '\0') {
  note = getNote(*song++);
  if (*song == '\0')
   break;
  dur = getDuration(*song++);
  if (*song == '\0')
   break;
  pause = getPause(*song++);

  playNote(note, dur);
  //dela

  y32Ms(0, pause);
  Timer0_Wait(pause);

 }
}

static uint8_t * song = (uint8_t*) "C2.C2,D4,C4,F4,E8,";
//(uint8_t*)"C2.C2,D4,C4,F4,E8,C2.C2,D4,C4,G4,F8,C2.C2,c4,A4,F4,E4,D4,A2.A2,H4,F4,G4,F8,";
//"D4,B4,B4,A4,A4,G4,E4,D4.D2,E4,E4,A4,F4,D8.D4,d4,d4,c4,c4,B4,G4,E4.E2,F4,F4,A4,A4,G8,";

static void init_ssp(void) {
 SSP_CFG_Type SSP_ConfigStruct;
 PINSEL_CFG_Type PinCfg;
 /*
  * Initialize SPI pin connect
  * P0.7 - SCK;
  * P0.8 - MISO
  * P0.9 - MOSI
  * P2.2 - SSEL - used as GPIO
  */
 PinCfg.Funcnum = 2;
 PinCfg.OpenDrain = 0;
 PinCfg.Pinmode = 0;
 PinCfg.Portnum = 0;
 PinCfg.Pinnum = 7;
 PINSEL_ConfigPin(&PinCfg);
 PinCfg.Pinnum = 8;
 PINSEL_ConfigPin(&PinCfg);
 PinCfg.Pinnum = 9;
 PINSEL_ConfigPin(&PinCfg);
 PinCfg.Funcnum = 0;
 PinCfg.Portnum = 2;
 PinCfg.Pinnum = 2;
 PINSEL_ConfigPin(&PinCfg);

 SSP_ConfigStructInit(&SSP_ConfigStruct);

 // Initialize SSP peripheral with parameter given in structure above
 SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

 // Enable SSP peripheral
 SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void) {
 PINSEL_CFG_Type PinCfg;

 /* Initialize I2C2 pin connect */
 PinCfg.Funcnum = 2;
 PinCfg.Pinnum = 10;
 PinCfg.Portnum = 0;
 PINSEL_ConfigPin(&PinCfg);
 PinCfg.Pinnum = 11;
 PINSEL_ConfigPin(&PinCfg);

 // Initialize I2C peripheral
 I2C_Init(LPC_I2C2, 100000);

 /* Enable I2C1 operation */
 I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void) {
 /*// Initialize button 3
 PINSEL_CFG_Type PinCfg;
 PinCfg.Funcnum = 0;
 PinCfg.Portnum = 1;
 PinCfg.Pinnum = 31;
 PINSEL_ConfigPin(&PinCfg);
 PinCfg.Portnum = 2;
 PinCfg.Pinnum = 10;
 PINSEL_ConfigPin(&PinCfg);
 GPIO_SetDir(1, 1<<31, 0);
 GPIO_SetDir(2, 1<<10, 0);*/
 //Initialize button sw4
 PINSEL_CFG_Type PinCfg;
 PinCfg.Funcnum = 0;
 PinCfg.OpenDrain = 0;
 PinCfg.Pinmode = 0;
 PinCfg.Portnum = 1;
 PinCfg.Pinnum = 31;
 PINSEL_ConfigPin(&PinCfg);
 GPIO_SetDir(1, 1 << 31, 0);

 //Initialize button sw3
 PinCfg.Funcnum = 0;
 PinCfg.OpenDrain = 0;
 PinCfg.Pinmode = 0;
 PinCfg.Portnum = 2;
 PinCfg.Pinnum = 10;
 PINSEL_ConfigPin(&PinCfg);
 GPIO_SetDir(2, 1 << 10, 0);

}

volatile int SW3 = 0;
volatile int SW4 = 0;
volatile int SW3_Flag = 0;

void EINT3_IRQHandler(void) { //come back here
 if ((LPC_GPIOINT->IO2IntStatF >> 10) & 0x1) {
  SW3 = 0;
  LPC_GPIOINT->IO2IntClr = (1 << 10);

  //  if (mode == MODE_RETURN && (msTicks - countTicksR <= 1000)) {
  //   //do we have to go back to launch mode again?
  //   if(msTicks - countTicksR < 1000) {
  //    SW3 = 0;
  //   }
  //  }
  //  else if (mode == MODE_RETURN) {
  //   countTicksR = msTicks;
  //  }
  //
  //  if (mode == MODE_STATIONARY && (msTicks - countTicksS <= 1000)) {
  //   //do we have to go back to launch mode again?
  //   if(msTicks - countTicksS < 1000) {
  //    SW3 = 0;
  //   }
  //  }
  //  else if (mode == MODE_STATIONARY) {
  //   countTicksS = msTicks;
  //  }

  if ((mode == MODE_RETURN) && (msTicks - countTicksS > 1000)) {
   SW3 = 1;
  }


  if ((mode == MODE_STATIONARY) && (msTicks - countTicksS > 1000)) {
   SW3 = 1;
  }

  if (mode == MODE_LAUNCH && (msTicks - countTicksS <= 1000)) {
   if (msTicks - countTicksS < 1000) {
    //oled_clearScreen(OLED_COLOR_BLACK); //come back here to oled not working
    /*oled_putString(0, 10, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(0, 20, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(0, 30, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(0, 40, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(0, 50, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);*/
    mode = MODE_RETURN;
    isUARTReturnModeSent = FALSE;
    SW3 = 0;
   }
  }
  countTicksS = msTicks;
 }
}

int veerOffCourse(float x, float y) {
 if (x > 0.4 || y > 0.4) {
  return TRUE;
 }
 else if (x < -0.4 || y < -0.4) {
  return TRUE;
 }
 else {
  return FALSE;
 }
}

int tempTooHigh(int32_t temp) {
 if (temp >= temp_threshold) {
  return TRUE;
 }
 else {
  return FALSE;
 }
}

void blink_blue() {
 rgb_setLeds(RGB_BLUE);
 Delay(333);
 GPIO_ClearValue( 0, (1<<26) );
 Delay(333);
}

void blink_red() {
 rgb_setLeds(RGB_RED);
 Delay(333);
 GPIO_ClearValue( 2, 1 << 0);
 Delay(333);
}


int lightValueToBinaryMaskAndTurnOn(int value) {
 if (value >= 3000) {
  pca9532_setLeds(0b1111111111111111, 0xffff);
 }
 else if (value >= 2800) {
  pca9532_setLeds(0b0111111111111111, 0xffff);
 }
 else if (value >= 2600) {
  pca9532_setLeds(0b0011111111111111, 0xffff);
 }
 else if (value >= 2400) {
  pca9532_setLeds(0b0001111111111111, 0xffff);
 }
 else if (value >= 2200) {
  pca9532_setLeds(0b000011111111111, 0xffff);
 }
 else if (value >= 2000) {
  pca9532_setLeds(0b00000011111111111, 0xffff);
 }
 else if (value >= 1800) {
  pca9532_setLeds(0b00000001111111111, 0xffff);
 }
 else if (value >= 1600) {
  pca9532_setLeds(0b00000000111111111, 0xffff);
 }
 else if (value >= 1400) {
  pca9532_setLeds(0b0000000001111111, 0xffff);
 }
 else if (value >= 1200) {
  pca9532_setLeds(0b0000000000111111, 0xffff);
 }
 else if (value >= 1000) {
  pca9532_setLeds(0b0000000000011111, 0xffff);
 }
 else if (value >= 800) {
  pca9532_setLeds(0b0000000000001111, 0xffff);
 }
 else if (value >= 600) {
  pca9532_setLeds(0b0000000000000111, 0xffff);
 }
 else if (value >= 400) {
  pca9532_setLeds(0b0000000000000011, 0xffff);
 }
 else if (value >= 200) {
  pca9532_setLeds(0b0000000000000001, 0xffff);
 }
 else {
  pca9532_setLeds(0b0000000000000000, 0xffff);
 }
}

int main(void) {
 mode = MODE_STATIONARY; //set the starting mode
 state = 1;

 int8_t xoff = 0;
 int8_t yoff = 0;
 int8_t zoff = 0;

 uint16_t ledOn = 0;

 int8_t x = 0;
 int8_t y = 0;
 int8_t z = 0;

 uint8_t dir = 1;
 uint8_t wait = 0;

 uint8_t state = 0;

 uint8_t btn1 = 1;
 uint8_t btn2 = 1;

 init_i2c();
 init_ssp();
 init_GPIO();

 pca9532_init();
 joystick_init();
 acc_init();
 oled_init();
 rgb_init();
 led7seg_init();
 light_init();
 light_enable();

 init_uart(); //UART

 unsigned char msg[100] = "";
 unsigned char msg2[100] = "";
 uint8_t data = 0;
 uint32_t len = 0;
 uint32_t counter = 0;
 uint8_t lines[64];
 uint8_t letterR = 82;
 uint8_t letterP = 80;
 uint8_t letterT = 84;

 UART_Receive_Int_Init();

 NVIC_EnableIRQ(EINT3_IRQn);
 LPC_GPIOINT->IO2IntClr = 1 << 10; //SW3
 LPC_GPIOINT->IO2IntEnF |= 1 << 10; //SW3
 LPC_GPIOINT->IO2IntClr = 1 << 31; //SW4
 LPC_GPIOINT->IO2IntEnF |= 1 << 31; //SW4
 LPC_GPIOINT ->IO2IntEnF |= 1 << 5;
 light_setHiThreshold(700);
 light_setLoThreshold(200);
 light_setRange(LIGHT_RANGE_4000);
 light_clearIrqStatus();
 NVIC_ClearPendingIRQ(EINT3_IRQn);

 /*
  * Assume base board in zero-g position when reading first value.
  */
 acc_read(&x, &y, &z);
 xoff = 0 - x;
 yoff = 0 - y;
 zoff = 0 - z;
 /* —— Temperature Sensor —----> */
 //dislays value of temperature on OLED

 SysTick_Config(SystemCoreClock / 1000);

 int32_t my_temp_value;

 temp_init(getTicks);

 btn1 = (GPIO_ReadValue(1) >> 31) & 0x01;
 btn2 = (GPIO_ReadValue(2) >> 10) & 0x01;

 /* <—— Temperature Sensor —---- */

 /* —— LED 7-Segment Display 0 to F —----> */
 //used for countdown

 uint8_t letter[16] = {
   /* digits 0-9 */
   0x24, 0x7D, 0xE0, 0x70, 0x39, 0x32, 0x22, 0x7C, 0x20, 0x38,
   /* A-F */
   0x28, 0x23, 0xA6, 0x61, 0xA2, 0xAA,
 };
 int32_t i = 15;
 uint32_t currentTime, initialTime, currentTimeLaunch, currentTimeReturn, initialTimeLaunch, initialTimeReturn;
 uint32_t currentTimeSw3, currentTimeSw32;
 initialTimeLaunch = 0;
 initialTimeReturn = 0;

 /* —— LED 7-Segment Display 0 to F —----> */

 /* —— Speaker —----> */

 // GPIO_SetDir(2, 1<<0, 1);
 // GPIO_SetDir(2, 1<<1, 1);
 //
 // GPIO_SetDir(0, 1<<27, 1);
 // GPIO_SetDir(0, 1<<28, 1);
 // GPIO_SetDir(2, 1 << 13, 1);
 // GPIO_SetDir(0, 1 << 26, 1);
 //
 // GPIO_ClearValue(0, 1<<27); //LM4811-clk
 // GPIO_ClearValue(0, 1<<28);//LM4811-up/dn
 // GPIO_ClearValue(2, 1<<13);//LM4811-shutdn

 /* <---- Speaker —---- */

 //moveBar(1, dir);
 oled_clearScreen(OLED_COLOR_BLACK);

 led7seg_setChar(letter[15], TRUE);

 my_temp_value = temp_read();

 while (1) { //main while loop

  /* ####### Accelerometer and LEDs  ###### */
  /* # */

  //acc_setRange(100);

  //accelorometerReadings(xoff,yoff,zoff);

  acc_read(&x, &y, &z);

  float xfinal, yfinal;

  xfinal = (float) (x + xoff)/256 * 4;

  yfinal = (float) (y + yoff)/256 * 4;

  /* ###############      MODES    ################### */
  switch(mode) {
  case MODE_STATIONARY:


   if (isReceived){
    isReceived = 0;
    UART_RcvMsgHandling();

    if (strcmp(rev_buf, "INFO\r") == 0) {
     //isReadyMsgSent = 0;
     sprintf(msg, "Temp: %.1f, ACC_X:%.3g, Y:%.3g \r\n", my_temp_value/10.0, xfinal, yfinal);
     UART_Send_Message(msg);

     if (my_temp_value/10.0 <= temp_threshold) {
      sprintf(msg, "Safe to launch, Current Temperature Threshold at: %.1f  \r\n", temp_threshold);
      UART_Send_Message(msg);
     }

     else {
      sprintf(msg, "DO NOT LAUNCH FUEL TEMPERATURE WARNING! Current Temperature Threshold at: %.1f \r\n", temp_threshold);
      UART_Send_Message(msg);
     }

    }

   }

   //UART_SEND();
   // This is the default mode, i.e., the mode when the system is powered on.
   if(!isUARTStationaryModeSent) {
    UART_Send(LPC_UART3, (uint8_t *) "Entering STATIONARY mode. \r\n", strlen("Entering STATIONARY mode. \r\n"), BLOCKING);
    isUARTStationaryModeSent = TRUE;
   }
   // OLED_DISPLAY - shows “STATIONARY”.
   oled_putString(20,0, "STATIONARY", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

   // The temperature from TEMPERATURE_SENSOR is also displayed on OLED_DISPLAY.
   sprintf(OLED_TEMPERATURE, "Temp: %.1f", my_temp_value/10.0);
   oled_putString(0, 10, (uint8_t *) OLED_TEMPERATURE, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

   // The SEGMENT_DISPLAY shows F initially.
   if (state == 1)
   {
    led7seg_setChar(letter[15], TRUE);
   }

   // Countdown for MODE_TOGGLE
   if(SW3 == 1)
   {
    SW3 = 0;
    state = 0;
    while (i >= 0) {
     if(my_temp_value/10.0 > temp_threshold) {
      led7seg_setChar(letter[15], TRUE);
      break;
     }
     currentTime = getTicks();
     if (currentTime - initialTime < 1000) {
      led7seg_setChar(letter[i], TRUE);
     } else {
      initialTime = getTicks();
      if (i == 0 && mode == MODE_STATIONARY) {
       mode = MODE_LAUNCH;
       isUARTLaunchModeSent = FALSE;
      } else {
       i--;
      }
     }
    }
   }
   break;

   //If TEMP_WARNING occurs, countdown is aborted and the SEGMENT_DISPLAY goes back to F. (TBD)

   //"Entering STATIONARY Mode \r\n" is sent to NUSCloud. (TBD)
  case MODE_LAUNCH:

   if (isReceived){
    isReceived = 0;
    UART_RcvMsgHandling();

    if (strcmp(rev_buf, "RPT\r") == 0) {
     //isReadyMsgSent = 0;
     sprintf(msg, "Temp: %.1f, ACC_X:%.3g, Y:%.3g \r\n", my_temp_value/10.0, xfinal, yfinal);
     UART_Send_Message(msg);
    }

   }

   if(!isUARTLaunchModeSent) {
    UART_Send(LPC_UART3, (uint8_t *) "Entering LAUNCH mode \r\n", strlen("Entering LAUNCH MODE \r\n"), BLOCKING);
    isUARTLaunchModeSent = TRUE;
   }

   currentTimeLaunch = getTicks();
   if(currentTimeLaunch - initialTimeLaunch >= 10000) {

    //msg = ("Temp: %.1f, ACC_X: %.3g, Y: %.3g \r\n", my_temp_value/10.0,xfinal,yfinal);
    sprintf(msg, "Temp: %.1f, ACC_X:%.3g, Y:%.3g \r\n", my_temp_value/10.0, xfinal, yfinal);
    UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg), BLOCKING);
    initialTimeLaunch = currentTimeLaunch;
   }
   // OLED_DISPLAY - shows “LAUNCH”.
   oled_putString(20,0, "  LAUNCH  ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

   // The temperature from TEMPERATURE_SENSOR is also displayed on OLED_DISPLAY.
   sprintf(OLED_TEMPERATURE, "Temp: %.1f", my_temp_value/10.0);

   // The ACCELEROMETER X and Y readings (must be in 'g's, and not in m/s2 or the raw value) is shown on the OLED_DISPLAY.
   sprintf(OLED_X, "X=%.3g", xfinal);
   sprintf(OLED_Y, "Y=%.3g", yfinal);
   oled_putString(0, 20, (uint8_t *) OLED_X, OLED_COLOR_WHITE,
     OLED_COLOR_BLACK);
   oled_putString(0, 30, (uint8_t *) OLED_Y, OLED_COLOR_WHITE,
     OLED_COLOR_BLACK);
   if(tempTooHigh(my_temp_value/10.0)) {
    oled_putString(0, 50, "Temp. too high", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    blink_red();
    if(!isUARTTempSent) {
     UART_Send(LPC_UART3, (uint8_t *) "Temp. too high. \r\n", strlen("Temp. too high. \r\n"), BLOCKING);
     isUARTTempSent = TRUE;
    }
   }
   if(veerOffCourse(xfinal, yfinal)) {
    oled_putString(0, 40, "Veer off course", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    blink_blue();
    if(!isUARTVeerSent) {
     UART_Send(LPC_UART3, (uint8_t *) "Veer off course. \r\n", strlen("Veer off course. \r\n"), BLOCKING);
     isUARTVeerSent = TRUE;
    }

   }
   SW4 = (GPIO_ReadValue(1) >> 31) & 0x01;
   if(SW4 == 0) {
    CLEAR_WARNING = TRUE;
   }
   if(CLEAR_WARNING == TRUE) {
    oled_putString(0, 50, "              ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(0, 40, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    CLEAR_WARNING=FALSE;
    isUARTVeerSent = FALSE;
    isUARTTempSent = FALSE;
   }

   break;

  case MODE_RETURN:
   if (isReceived){
    isReceived = 0;
    UART_RcvMsgHandling();

    if (strcmp(rev_buf, "RPT\r") == 0) {
     //isReadyMsgSent = 0;
     sprintf(msg2, "Obstacle distance: %u \r\n", light_read());
     UART_Send_Message(msg2);
    }

   }

   //OLED_DISPLAY - shows “RETURN”.

   oled_putString(0, 10, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
   oled_putString(0, 20, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
   oled_putString(0, 30, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

   oled_putString(30,0, "RETURN ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
   if(!isUARTReturnModeSent) {
    UART_Send(LPC_UART3, (uint8_t *) "Entering RETURN mode \r\n", strlen("Entering RETURN MODE \r\n"), BLOCKING);
    isUARTReturnModeSent = TRUE;
   }
   currentTimeReturn = getTicks();
   if(currentTimeReturn - initialTimeReturn >= 10000) {
    sprintf(msg2, "Obstacle distance: %u \r\n", light_read());
    UART_Send(LPC_UART3, (uint8_t *) msg2, strlen(msg2), BLOCKING);
    initialTimeReturn = currentTimeReturn;
   }

   //SEGMENT_DISPLAY shows 0.
   led7seg_setChar('0', FALSE);


   //LED_ARRAY depicts the distance of the system from obstacles.
   lightValueToBinaryMaskAndTurnOn(light_read());

   if(light_read() >= 3000 && isObstacleNear == FALSE) {
    oled_putString(0, 50, "Obstacle near", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    UART_Send(LPC_UART3, (uint8_t *) "Obstacle Near. \r\n", strlen("Obstacle Near. \r\n"), BLOCKING); //come back here
    isObstacleNear = TRUE;
   }


   //   if (light_read() >= 3000) {
   //    if (isReceived){
   //     isReceived = 0;
   //     UART_RcvMsgHandling();
   //
   //     if (strcmp(rev_buf, "SOS\r") == 0) {
   //      //isReadyMsgSent = 0;
   //      UART_Send_Message("SOS MESSAGE RECEIVED PLEASE ASSIST ROCKET");
   //     }
   //
   //    }
   //   }

   if(light_read() < 3000 && isObstacleNear == TRUE) {
    oled_putString(0, 10, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(0, 20, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(0, 30, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(0, 40, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(0, 50, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    UART_Send(LPC_UART3, (uint8_t *) "Obstacle Avoided. \r\n", strlen("Obstacle Avoided. \r\n"), BLOCKING);
    isObstacleNear = FALSE;
   }

   if(SW3 == 1) {
    SW3 = 0;
    mode = MODE_STATIONARY;
    isUARTStationaryModeSent = FALSE;
    led7seg_setChar(letter[15], TRUE);
    i = 15;
   }

  }

  Timer0_Wait(1);
 }

}

void check_failed(uint8_t *file, uint32_t line) {
 /* User can add his own implementation to report the file name and line number,
 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

 /* Infinite loop */
 while (1)
  ;
}
