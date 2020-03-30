#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include "packet.h"
#include "constants.h"

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

#define COUNTS_PER_REV      197
#define ALEX_LENGTH         18.5
#define ALEX_BREADTH         13.2

//#define COUNTS_PER_REV_L     150 // initially 252
//#define COUNTS_PER_REV_R     296
//#define PI                  3.141592654

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.42035


float AlexDiagonal = 0;
float AlexCirc = 0;

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  5   // Left forward pin
#define LR                  6   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin

//PINS
#define PIN_5 (1 << 5)
#define PIN_6 (1 << 6)
#define PIN_10 (1 << 2)
#define PIN_11 (1 << 3)

#define UDRIEMASK 0b00100000

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

/*
 * PID Variables
 */
long error;
long adjustL = 0;
long adjustR = 0;
long Kp = 1500L;
long Ki = 900L;
long Kd = 100L;


/*
 * 
 * Alex Communication Routines.
 * 
 */
 
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

ISR(USART_RX_vect) {
  dataRecv = UDR0;
}

ISR(USART_UDRE_vect) {
  UDR0 = dataSend;
  UCSR0B &= ~UDRIEMASK;
}

void sendData(const char data) {
  dataSend = buttonVal+'0';
  UCSR0B |= UDRIEMASK;
}

char recvData() {
  return dataRecv-'0'; 
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
  // To replace later with bare-metal.
  // Serial.begin(9600);
  UBRR0L = 103;    // for 9600 baud rate 
  UBRR0H = 0;
  UCSR0C = 0b00000110;  // disabled parity , 1 stop bit,  8 bit character size, lastbit set to 0 for asynchronous
  UCSR0A = 0; // 
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  UCSR0B = 0b10111000; // triggers rx vect interrupt when character received
  //but not when sending character or when data register empty 
  // enabled usart receiver and transmitter , 8 bit character size, no 9th bit to receive/transmit (checks out w 8 bit char size)
  // should be 10111000 for interrupt mode if using buffer else, 10011000  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{
  unsigned char data = UDR0;
// Will fail silently and data will be lost if recvBuffer is full
  writeBuffer(&_recvBuffer, data); 
//   int count=0;

//   while(Serial.available())
//     buffer[count++] = Serial.read();

//   return count;
}

// Write to the serial port. Replaced later with
// bare-metal code                                      //i think this change to interrupt mode?

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
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
   TCCR0A = 0b10000001;       // phase correct mode  // why is 1st bit 1 (notsure)
   TIMSK0 |= 0b110; // OCIEA = 1 OCIEB = 1   //switch on ociea and b 
   OCR0A = 128;   
   OCR0B = 128;           
   TCCR0B = 0b00000011;  //prescalar 64
   prevMillis = millis();
  // sei();
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  PORTB = PIN_10;   //seems right?? not sure
  PORTB = PIN_11;
  PORTD = PIN_5;
  PORTD = PIN_6;
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

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  dir = FORWARD;
  
  int val = pwmVal(speed);

  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.

  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;

  newDist = forwardDist + deltaDist;

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  
  analogWrite(LF, val); //still need to edit this part
  analogWrite(RF, val);
  analogWrite(LR,0);
  analogWrite(RR, 0);
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

  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.
  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;

  newDist = reverseDist + deltaDist;

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  analogWrite(LR, val);
  analogWrite(RR, val);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
}

// function for wheel ticks
unsigned long computeDeltaTicks(float ang)
{
  unsigned long ticks = (unsigned long) ((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
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
  analogWrite(LR, val);
  analogWrite(RF, val);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
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

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  analogWrite(RR, val);
  analogWrite(LF, val);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
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
   
  //leftRevs = 0;
  //rightRevs = 0;
  
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
  // put your setup code here, to run once:
  EICRA |= 0b00000011;
  DDRB |= 0b00001100;
  DDRD |= 0b00110000;
  EIMSK |= 0b00000011;

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
  // PID code
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

// Uncomment the code below for Week 9 Studio 2

 // put your main code here, to run repeatedly:
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
