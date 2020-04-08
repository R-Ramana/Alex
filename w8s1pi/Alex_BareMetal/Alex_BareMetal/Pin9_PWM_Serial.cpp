#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include "packet.h"
#include "constants.h"
#include <avr/sleep.h> 

// include buffer files
#ifndef F_CPU
#define F_CPU   16000000UL
#endif
#define PRR_TWI_MASK            0b10000000 
#define PRR_SPI_MASK            0b00000100 
#define ADCSRA_ADC_MASK         0b10000000 
#define PRR_ADC_MASK            0b00000001 
#define PRR_TIMER2_MASK         0b01000000 
#define PRR_TIMER0_MASK         0b00100000 
#define PRR_TIMER1_MASK         0b00001000 
#define SMCR_SLEEP_ENABLE_MASK  0b00000001 
#define SMCR_IDLE_MODE_MASK     0b11110001
#define PIN_7 (1 << 7)
#define PIN_6 (1 << 6)
#define PIN_5 (1 << 5)
#define PIN_4 (1 << 4)
#define PIN_3 (1 << 3)
#define PIN_2 (1 << 2)
#define PIN_1 (1 << 1)
#define PIN_0 (1 << 0)
#define PIN_9 (1 << 1)
#define PIN_10 (1 << 2)
#define PIN_11 (1 << 3)

typedef enum
{
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
} TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      200
#define ALEX_LENGTH         23.5 //18.5 previously
#define ALEX_BREADTH         13.2

#define COUNTS_PER_REV_L     150 // initially 252
#define COUNTS_PER_REV_R     296
//#define PI                  3.141592654

#define TURN_FACTOR 3.6 //3.7ish on table// could be affected by grooves calculated around 2.8 but ended up using 3.5 for better turning

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.42035


float AlexDiagonal = 0;//22.73
float AlexCirc = 0; //71.39

// Motor control pins.
// Alex moves in the correct direction
#define LF                  5   // Left forward pin (OCR0B)
#define LR                  6   // Left reverse pin (OCR0A)
#define RF                  10  // Right forward pin (OCR1B)
#define RR                  9  // Right reverse pin (OCR1A)

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;
unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;

TBuffer _recvBuffer;
initBuffer(_recvBuffer, PACKET_SIZE);
TBuffer _xmitBuffer;
initBuffer(_xmitBuffer, PACKET_SIZE);

/*
 * 
 * Alex Communication Routines.
 * 
 */
void WDT_off(void) { /* Global interrupt should be turned OFF here if not already done so */ 
 
  /* Clear WDRF in MCUSR */ 
  MCUSR &= ~(1<<WDRF); 
  
  /* Write logical one to WDCE and WDE */ /* Keep old prescaler setting to prevent unintentional time-out */ 
  WDTCSR |= (1<<WDCE) | (1<<WDE); 
  
  /* Turn off WDT */ 
  WDTCSR = 0x00; 
 
  /* Global interrupt should be turned ON here if subsequent operations after calling this function do not require turning off global interrupt */ 
}

void setupPowerSaving()
{   
  // Turn off the Watchdog Timer   
  WDT_off();
  // Modify PRR to shut down TWI 
  PRR &= ~PRR_TWI_MASK;
  // Modify PRR to shut down SPI 
  PRR &= ~PRR_SPI_MASK;
  // Modify ADCSRA to disable ADC,    
  ADCSRA &= ~ADCSRA_ADC_MASK;
  // then modify PRR to shut down ADC  
  PRR &= ~PRR_ADC_MASK;
  // Set the SMCR to choose the IDLE sleep mode   
  SMCR |= SMCR_IDLE_MODE_MASK;
  // Do not set the Sleep Enable (SE) bit yet 
 
  // Set Port B Pin 5 as output pin, then write a logic LOW   
  DDRB &= ~PIN5;
  // to it so that the LED tied to Arduino's Pin 13 is OFF. 
  }

 void putArduinoToIdle() {   
  // Modify PRR to shut down TIMER 0, 1, and 2
  PRR &= ~PRR_TIMER2_MASK;
  PRR &= ~PRR_TIMER0_MASK;
  PRR &= ~PRR_TIMER1_MASK;    
  // Modify SE bit in SMCR to enable (i.e., allow) sleep  
  SMCR |= SMCR_SLEEP_ENABLE_MASK;    
  // The following function puts ATmega328P’s MCU into sleep;   
  // it wakes up from sleep when USART serial data arrives   
  sleep_cpu();     
  // Modify SE bit in SMCR to disable (i.e., disallow) sleep 
  SMCR &= ~SMCR_SLEEP_ENABLE_MASK;
  // Modify PRR to power up TIMER 0, 1, and 2
  PRR |= PRR_TIMER2_MASK;
  PRR |= PRR_TIMER0_MASK;
  PRR |= PRR_TIMER1_MASK;   
  } 
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType=PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
} 

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= 0b11110011;
  PORTD |= 0b00001100; 
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }

 if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }

  if (dir == LEFT) {
    // left reverse
    leftReverseTicksTurns++;
  }

  if (dir == RIGHT) {
    // left forward
    leftForwardTicksTurns++;
  }
  
}

void rightISR()
{
  if (dir == FORWARD) {
    rightForwardTicks++;
//    forwardDist = (unsigned long) ((float) rightForwardTicks / COU/NTS_PER_REV_R * WHEEL_CIRC);
  }

  if (dir == BACKWARD) {
    rightReverseTicks++;
//   / reverseDist = (unsigned long) ((float) rightReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }

  if (dir == LEFT) {
    // right forward
    rightForwardTicksTurns++;
  }

  if (dir == RIGHT) {
    // right reverse
    rightReverseTicksTurns++;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EICRA =  0b00001010;
  EIMSK = 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

// Implement INT0 and INT1 ISRs above.
ISR(INT0_vect){
  leftISR();
}

ISR(INT1_vect){
 rightISR();
}

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
    /*
    setup:
    1. frame format - using UCSR0C
    2. baud/bit rate
    3. enable/disable interrupts and other features like transmitter/receiver
    4. can we send / read data? parity error? (polling mode)

    =====
    PART ONE: frame format
    =====
    Use UCSR0C register to configure:
    1. mode - select asynchrounous USART (i.e. UART)
    2. parity 
    3. stop bits
    4. number of bit - use 8 bits (need to use UCSR0B register as well)
    5. clock polarity - set to 0 since we are in asynchrounous mode

    ====
    PART TWO: baud/bit rate
    ====
    Set the baud rate register, UBRR [further broken down into UBRR0H for high byte and UBRR0L for low byte], 
    to the value B where is given by,
    B = round(fclock/16*baud) - 1
    Note, for Arduino, fclock = 16 Mhz
    Refer to setBaud() function below.

    ====
    PART THREE: enable/disable interrupts and other features like transmitter/receiver
    ====
    The UCSR0B allows us to enable the triggering of interrupts when data is received, transmitted. 
    Also to set number of bits for frame (Refer to part 1, point 4)

    ====
    PART FOUR: can we send / read data? (polling mode) parity error?
    ====
    Bits 6 and 7 of UCSR0A changes as data is transmitted or received.
    Also, can be used to check if there is a parity error.

  */

    /*
        9600 bps, 8N1 configuration (9600 baudrate, 8 bits of data, no parity and 1 stop bit)
    */
    
    // PART ONE: frame format using UCSR0C
    UCSR0C = 0b00000110;
    // PART TWO: baud rate
    setBaud(9600);
    // PART THREE: enable transmission/receipt and interrupts (and size of data. ref: part 1 point 4)
    // is done in startSerial because as soon as transmit / receive bits are 1, serial begins

    // PART FOUR: clear UCSR0A before starting
    UCSR0A = 0;

}

void setBaud(unsigned long baudRate) {
    unsigned int B;
    B = (unsigned int) round(F_CPU/16*baudRate) - 1;
    UBRR0H = (unsigned char) B >> 8;
    UBRR0L = B;
}


// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
    UCSR0B = 0b10111000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{
    /*
        Data can be read from the 8 bit UDR0 register
        A circular buffer is used to read/write data for efficiency reasons

        declare _recvBuffer and also include buffer.zip library
    */
    int count=0;
    TBufferResult result;
    do {
        result = readBuffer(&_recvBuffer, &buffer[count]);
        if (result == BUFFER_OK)
            count++;
        
    } while(result == BUFFER_OK)

    return count;
}

// "Receive complete" interrupt is triggered whenever a new character comes in
ISR(USART_RX_vect) {
    unsigned char data = UDR0;

    writeBuffer(&_recvBuffer, data);
}

// "Transmit complete" interrupt is triggered whenever a new character is sent
ISR(USART_UDRE_vect) {
    unsigned char data;
    TBufferResult result = readBuffer(&_xmitBuffer, &data);
    if (result == BUFFER_OK)
        UDRO = data;
    else
        if (result == BUFFER_EMPTY)
            UCSR0B &= 0b11011111;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
    TBufferResult result = BUFFER_OK;
    int i;
    for (i = 1; i < len && result == BUFFER_OK; i++)
        result = writeBuffer(&_xmitBuffer, buffer[1]);
  
    UDRO = buffer[0];

    // Enable the UDRE interrupt. The enable is bit 5 of UCSR0B
    UCSR0B |= 0b0010000;
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
  DDRD |= ((PIN_5) | (PIN_6));
  DDRB |= ((PIN_10) | (PIN_11));

  TCNT0 = 0;
  TCCR0A = 0b10100001; 
  TCCR0B = 0b00000011;
  TCNT1 = 0;
  TCCR1A = 0b10100001;
  TCCR0B = 0b00000011;


}

void startMotors()
{

  OCR0A = 0;
  OCR0B = 0;
  OCR1A = 0;
  OCR1B = 0;

}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}


// helper functions for setting LF, LR, RF, RR

void setLF(int val)
{
  OCR0B = val;
}

void setLR(int val)
{
  OCR0A = val;
}

void setRF(int val)
{
  OCR1B = val;
}

void setRR(int val)
{
  OCR1A = val;
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  dir = FORWARD;
  
  int val = pwmVal(speed);

  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;

  newDist = forwardDist + deltaDist;

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  setLF(val);
  setRF(val);
  setLR(0);
  setRR(0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{

  dir = BACKWARD;
  int val = pwmVal(speed);

  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;

  newDist = reverseDist + deltaDist;

  setLR(val);
  setRR(val);
  setLF(0);
  setRF(0);
}

// function for wheel ticks
unsigned long computeDeltaTicks(float ang)
{
  unsigned long ticks = (unsigned long) ((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks/TURN_FACTOR;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;
  int val = pwmVal(speed);

  if (ang == 0) 
    deltaTicks = 9999999;
  else {
    deltaTicks = computeDeltaTicks(ang);
  }

  targetTicks = leftReverseTicksTurns + deltaTicks;

  setLR(val);
  setRF(val);
  setLF(0);
  setRR(0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;
  int val = pwmVal(speed);

  if (ang == 0) 
    deltaTicks = 9999999;
  else {
    deltaTicks = computeDeltaTicks(ang);
  }

  targetTicks = rightReverseTicksTurns + deltaTicks;

  setRR(val);
  setLF(val);
  setLR(0);
  setRF(0);

}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  setLF(0);
  setLR(0);
  setRF(0);
  setRR(0);
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0; 
  rightForwardTicks = 0;
  leftReverseTicks = 0; 
  rightReverseTicks = 0;
  
  leftForwardTicksTurns = 0; 
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0; 
  rightReverseTicksTurns = 0;
  
  //volatile unsigned long leftRevs;
  //volatile unsigned long rightRevs;
  
  // Forward and backward distance traveled
  forwardDist = 0;
  reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Alex's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_GET_STATS:
        sendOK();
        sendStatus();
      break;  
    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
    break;

    case COMMAND_STOP:
        sendOK();
        stop();
      break;
      
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  setupPowerSaving();
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  AlexCirc = PI * AlexDiagonal;
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
  putArduinoToIdle();
  if (deltaTicks > 0) 
  {
    if (dir == LEFT)
    {
      if(leftReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else {
      if (dir == RIGHT) {
        if (rightReverseTicksTurns >= targetTicks) {
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
      } else {
        if (dir == STOP) {
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
      }
    }
  }
  if (deltaDist > 0)
  {
    if (dir == FORWARD) {
      if (forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD) {
      if (reverseDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if (dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }


  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 
}


