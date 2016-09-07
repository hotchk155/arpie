////////////////////////////////////////////////////////////////////////////////
//
//                                  //
//
//      //////   //  //// //////    //    ////
//            // ////     //    //  //  //    //
//      //////// //       //    //  //  ////////
//    //      // //       //    //  //  //
//      //////   //       //////    //    ////
//      MIDI ARPEGGIATOR  //
//      hotchk155/2013-15 //
//
//    Revision History   
//    1.00  16Apr13  Baseline 
//    1.01  21Apr13  Improvements for MIDI thru control
//    1.02  26Apr13  Synch source/input lockout options
//    1.03  12May13  Fix issue with synch thru/change lockout blink rate
//    1.04  09Nov13  Support MIDI stop/continue on external synch
//    1.05  16May14  Force to scale options
//    1.06  19May14  Poly Gate/MIDI transpose/Skip on rest
//    1.07  16Nov14  Revert to primary menu function, glide mode
//    A4    Feb2015  New release version
//    XXXX  Working on changes
//
#define VERSION_HI  7
#define VERSION_LO  7

//
// INCLUDE FILES
//
#include <avr/interrupt.h>  
#include <avr/io.h>
#include <EEPROM.h>

// Midi CC numbers to associate with hack header inputs
#define HH_CC_PC0             16
#define HH_CC_PC4             17
#define HH_CC_PC5             18

// The controller number used for Velocity CC mode
#define VELOCITY_CC           41 // Korg Volca FM


// Hack header pulse clock config
#define SYNCH_TICK_TO_PULSE_RATIO  12
#define SYNCH_CLOCK_PULSE_WIDTH    10
#define SYNCH_CLOCK_MIN_PERIOD     20
#define SYNCH_CLOCK_INVERT         0

//
// PREFERENCES WORD
//
enum {
  PREF_HACKHEADER=       (unsigned int)0b1111111100000000,
  PREF_HH_TYPE=          (unsigned int)0b1000000000000000,  
  PREF_HHTYPE_POTS=      (unsigned int)0b0000000000000000,
  
  PREF_HH_SYNCHTAB=      (unsigned int)0b1000000000000000,

  PREF_HHSW_PB3=         (unsigned int)0b0100000000000000,
 
  PREF_HHPOT_PC0=        (unsigned int)0b0011000000000000,
  PREF_HHPOT_PC0_TEMPO=  (unsigned int)0b0001000000000000,
  PREF_HHPOT_PC0_GATE=   (unsigned int)0b0010000000000000,
  PREF_HHPOT_PC0_CC=     (unsigned int)0b0011000000000000,

  PREF_HHPOT_PC4=        (unsigned int)0b0000110000000000,
  PREF_HHPOT_PC4_VEL=    (unsigned int)0b0000010000000000,
  PREF_HHPOT_PC4_PB=     (unsigned int)0b0000100000000000,
  PREF_HHPOT_PC4_CC=     (unsigned int)0b0000110000000000,

  PREF_HHPOT_PC5=        (unsigned int)0b0000001100000000,
  PREF_HHPOT_PC5_MOD=    (unsigned int)0b0000000100000000,
  PREF_HHPOT_PC5_TRANS=  (unsigned int)0b0000001000000000,
  PREF_HHPOT_PC5_CC=     (unsigned int)0b0000001100000000,
 
  PREF_AUTOREVERT=   (unsigned int)0b0000000000010000,

  PREF_LONGPRESS=    (unsigned int)0b0000000000001100, //Mask
  PREF_LONGPRESS0=   (unsigned int)0b0000000000000000, //Shortest
  PREF_LONGPRESS1=   (unsigned int)0b0000000000000100, //:
  PREF_LONGPRESS2=   (unsigned int)0b0000000000001000, //:
  PREF_LONGPRESS3=   (unsigned int)0b0000000000001100, //Longest
  
  PREF_LEDPROFILE =  (unsigned int)0b0000000000000011,  // Mask
  PREF_LEDPROFILE0 = (unsigned int)0b0000000000000000,  // STD GREEN
  PREF_LEDPROFILE1 = (unsigned int)0b0000000000000001,  // STD BLUE
  PREF_LEDPROFILE2 = (unsigned int)0b0000000000000010,  // SUPER BRIGHT BLUE
  PREF_LEDPROFILE3 = (unsigned int)0b0000000000000011,  // SUPER BRIGHT WHITE
  
  PREF_MASK        = (unsigned int)0b1111111100011111 // Which bits of the prefs register are mapped to actual prefs
};
#define IS_HH_CLOCK  ((gPreferences & PREF_HACKHEADER) == PREF_HH_SYNCHTAB)
#define IS_HH_POTS   ((gPreferences & PREF_HH_TYPE) == PREF_HHTYPE_POTS)


// The preferences word
unsigned int gPreferences;

// Forward declare the UI refresh flag
#define EDIT_FLAG_FORCE_REFRESH 0x01
#define EDIT_FLAG_IS_HELD       0x02
#define EDIT_FLAG_IS_NEW        0x04
extern byte editFlags;

#define SYNCH_TO_MIDI                  0x0001    // This flag indicates we are slaving to MIDI clock
#define SYNCH_SEND_CLOCK               0x0002    // This flag indicates that we should send MIDI clock to output
#define SYNCH_BEAT                     0x0004    // This flag indicates that the beat clock should flash
#define SYNCH_SEND_START               0x0008    // This flag indicates that a START message needs to be sent to slaves
#define SYNCH_SEND_STOP                0x0010    // This flag indicates that a STOP message needs to be sent to slaves
#define SYNCH_SEND_CONTINUE            0x0020    // This flag indicates that a CONTINUE message needs to be sent to slaves
#define SYNCH_RESET_NEXT_STEP_TIME     0x0040    // This flag indicates that the next step will be timed from now
#define SYNCH_RESTART_ON_BEAT          0x0080    // This flag indicates that the sequence will restart at the next beat
#define SYNCH_AUX_RUNNING              0x0100    // This flag indicates that the AUX SYNCH input in a running state 
#define SYNCH_ZERO_TICK_COUNT          0x0200    // This flag indicates cause the tick count to restart (and get beat LED in synch)
#define SYNCH_HOLD_AT_ZERO             0x0400    // This flag resets and suspends arpeggiator while still passing MIDI clock
#define SYNCH_PLAY_ADVANCE             0x0800    // Whether to advance play counter to next step
extern volatile unsigned int synchFlags;


////////////////////////////////////////////////////////////////////////////////
//
//
//
// LOW LEVEL USER INTERFACE FUNCTIONS
// - Drive the LEDs and read the switches
//
//
//
////////////////////////////////////////////////////////////////////////////////

// pin definitions
#define P_UI_HOLDSW      12

#define P_UI_IN_LED      8
#define P_UI_SYNCH_LED   16
#define P_UI_OUT_LED     17
#define P_UI_HOLD_LED     9

#define P_UI_CLK     5   //PD5
#define P_UI_DATA    7   //PD7
#define P_UI_STROBE  15  //PC1
#define P_UI_READ1   10  //PB2
#define P_UI_READ0   6   //PD6

#define DBIT_UI_CLK     0b00100000  // D5
#define DBIT_UI_DATA    0b10000000  // D7
#define CBIT_UI_STROBE  0b00000010  // C1

#define DBIT_UI_READ0   0b01000000  // D6
#define BBIT_UI_READ1   0b00000100  // B2

#define NO_VALUE (-1)
#define DEBOUNCE_COUNT 50

// Time in ms that counts as a long button press
enum 
{
  UI_HOLD_TIME_0 = 250,
  UI_HOLD_TIME_1 = 500,
  UI_HOLD_TIME_2 = 1000,
  UI_HOLD_TIME_3 = 1500
};
unsigned int uiLongHoldTime;

// Brightness levels of LEDs
enum 
{
  UI_LEDPROFILE0_HI   = 100,
  UI_LEDPROFILE0_MED  = 3,
  UI_LEDPROFILE0_LO   = 1,
  UI_LEDPROFILE1_HI   = 255,
  UI_LEDPROFILE1_MED  = 3,
  UI_LEDPROFILE1_LO   = 1,
  UI_LEDPROFILE2_HI   = 255,
  UI_LEDPROFILE2_MED  = 25,
  UI_LEDPROFILE2_LO   = 4,
  UI_LEDPROFILE3_HI   = 255,
  UI_LEDPROFILE3_MED  = 35,
  UI_LEDPROFILE3_LO   = 10
};
byte uiLedBright; 
byte uiLedMedium;
byte uiLedDim;

#define UI_IN_LED_TIME     20
#define UI_OUT_LED_TIME    20
#define UI_SYNCH_LED_TIME  5

#define UI_HOLD_PRESSED  0x01
#define UI_HOLD_HELD     0x02
#define UI_HOLD_CHORD    0x04
#define UI_HOLD_LOCKED   0x08

#define SYNCH_SOURCE_NONE      0
#define SYNCH_SOURCE_INTERNAL  1
#define SYNCH_SOURCE_INPUT     2
#define SYNCH_SOURCE_AUX       3
#define SYNCH_SOURCE_MANUAL    4

volatile byte uiLeds[16];
volatile char uiDataKey;
volatile char uiLastDataKey;
volatile char uiMenuKey;
volatile char uiLastMenuKey;
volatile byte uiDebounce;
volatile byte uiScanPosition;
volatile byte uiLedOffPeriod;
volatile byte uiFlashHold;

unsigned long uiUnflashInLED;
unsigned long uiUnflashOutLED;
unsigned long uiUnflashSynchLED;
byte uiHoldType;
unsigned long uiHoldPressedTime;
#define UI_DEBOUNCE_MS 100

// Enumerate the menu keys
// Columns A, B, C 
// Rows 1,2,3,4
enum {
  UI_KEY_A1  = 0,
  UI_KEY_A2  = 1,
  UI_KEY_A3  = 2,
  UI_KEY_A4  = 3,

  UI_KEY_B1  = 4,
  UI_KEY_B2  = 5,
  UI_KEY_B3  = 6,
  UI_KEY_B4  = 7,

  UI_KEY_C1  = 8,
  UI_KEY_C2  = 9,
  UI_KEY_C3  = 10,
  UI_KEY_C4  = 11  
};

////////////////////////////////////////////////////////////////////////////////
//
// INTERRUPT SERVICE ROUTINE FOR UPDATING THE DISPLAY AND READING KEYBOARD
//
////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_OVF_vect) 
{  
  // turn off LED strobe
  PORTC &= ~CBIT_UI_STROBE;

  // Do we need to wait until turning the next LED on? (this is used implement
  // crude PWM brightness control)
  if(uiLedOffPeriod)
  {
    // ok, we need another interrupt before we illuminate the next LED
    TCNT2 = 255 - uiLedOffPeriod;
    uiLedOffPeriod = 0;
  }
  else
  {         
    // used to flash hold light
    ++uiFlashHold;  

    // need to start scanning from the start of the led row again?
    if(uiScanPosition >= 15)
    {
      // Clock a single bit into the shift registers
      PORTD &= ~DBIT_UI_CLK;
      PORTD |= DBIT_UI_DATA;
      PORTD |= DBIT_UI_CLK;
    }

    // Shift the bit along (NB we need to shift it once 
    // before it will appear at shift reg output 0)
    PORTD &= ~DBIT_UI_DATA;
    PORTD &= ~DBIT_UI_CLK;
    PORTD |= DBIT_UI_CLK;

    // look up the button index
    byte buttonIndex = 15 - uiScanPosition;

    // does this LED need to be lit?    
    byte ledOnPeriod = uiLeds[buttonIndex];
    if(ledOnPeriod)
    {
      // ok enable led strobe line
      PORTC |= CBIT_UI_STROBE;      
    }     
    // set up the off period to make up a 255 clock tick cycle
    uiLedOffPeriod = 255 - ledOnPeriod;

    // check we're not debouncing the data entry keys
    if(!uiDebounce)
    {
      // is a data entry key pressed?
      if(!!(PINB & BBIT_UI_READ1))
      {
        // is it a new keypress?
        if(uiLastDataKey != buttonIndex)
        {
          // store it            
          uiDataKey = buttonIndex;
          uiLastDataKey = uiDataKey;
          uiDebounce = DEBOUNCE_COUNT;
        }
      }
      // has the previously pressed key been released?
      else if(buttonIndex == uiLastDataKey)
      {
        // no key is pressed
        uiLastDataKey = NO_VALUE;
      }
    }    
    else
    {
      // debouncing
      --uiDebounce;
    }

    // check for input at the menu keys
    // here we don't bother with a debounce
    if(!!(PIND & DBIT_UI_READ0))
    {
      // is a new key pressed
      if(buttonIndex != uiLastMenuKey)
      {
        // report it
        uiMenuKey = buttonIndex;
        uiLastMenuKey = uiMenuKey;
      }
    }
    else if(buttonIndex == uiLastMenuKey)
    {
      // previous key no longer pressed
      uiLastMenuKey = NO_VALUE;
    }

    if(!uiScanPosition)
      uiScanPosition = 15;
    else
      --uiScanPosition;

    TCNT2 = 255 - ledOnPeriod;
  }  
}

////////////////////////////////////////////////////////////////////////////////
// UI INIT
void uiInit()
{
  pinMode(P_UI_CLK, OUTPUT);     
  pinMode(P_UI_DATA, OUTPUT);     
  pinMode(P_UI_READ1, INPUT);     
  pinMode(P_UI_READ0, INPUT);     
  pinMode(P_UI_STROBE, OUTPUT);     

  pinMode(P_UI_HOLDSW, INPUT_PULLUP); // If you get an error here, get latest Arduino IDE
  pinMode(P_UI_IN_LED, OUTPUT);
  pinMode(P_UI_OUT_LED, OUTPUT);
  pinMode(P_UI_SYNCH_LED, OUTPUT);
  pinMode(P_UI_HOLD_LED, OUTPUT);

  // enable pullups
//  digitalWrite(P_UI_HOLDSW, HIGH);

  for(int i=0;i<16;++i)
    uiLeds[i] = 0;  

  uiDataKey = NO_VALUE;
  uiLastDataKey = NO_VALUE;
  uiMenuKey = NO_VALUE;
  uiDebounce = 0;
  uiScanPosition = 15;
  uiLedOffPeriod = 0;
  uiUnflashInLED = 0;
  uiUnflashOutLED = 0;
  uiUnflashSynchLED = 0;
  uiHoldPressedTime = 0;
  uiHoldType = 0;
  uiFlashHold = 0;
  uiLongHoldTime = UI_HOLD_TIME_3;
  uiLedBright = UI_LEDPROFILE2_HI; 
  uiLedMedium = UI_LEDPROFILE2_MED; 
  uiLedDim = UI_LEDPROFILE2_LO;
  
  // start the interrupt to service the UI   
  TCCR2A = 0;
  TCCR2B = 1<<CS21 | 1<<CS20;
  TIMSK2 = 1<<TOIE2;
  TCNT2 = 0; 
}

////////////////////////////////////////////////////////////////////////////////
// SHOW FIRMWARE VERSION (BCD)
void uiShowVersion()
{
  if(digitalRead(P_UI_HOLDSW) == LOW) {
    uiLeds[0] =  !!((VERSION_HI/10)&0x8) ? uiLedBright:uiLedDim;
    uiLeds[1] =  !!((VERSION_HI/10)&0x4) ? uiLedBright:uiLedDim;
    uiLeds[2] =  !!((VERSION_HI/10)&0x2) ? uiLedBright:uiLedDim;
    uiLeds[3] =  !!((VERSION_HI/10)&0x1) ? uiLedBright:uiLedDim;

    uiLeds[4] =  !!((VERSION_HI%10)&0x8) ? uiLedBright:uiLedDim;
    uiLeds[5] =  !!((VERSION_HI%10)&0x4) ? uiLedBright:uiLedDim;
    uiLeds[6] =  !!((VERSION_HI%10)&0x2) ? uiLedBright:uiLedDim;
    uiLeds[7] =  !!((VERSION_HI%10)&0x1) ? uiLedBright:uiLedDim;

    uiLeds[8] =  !!((VERSION_LO/10)&0x8) ? uiLedBright:uiLedDim;
    uiLeds[9] =  !!((VERSION_LO/10)&0x4) ? uiLedBright:uiLedDim;
    uiLeds[10] = !!((VERSION_LO/10)&0x2) ? uiLedBright:uiLedDim;
    uiLeds[11] = !!((VERSION_LO/10)&0x1) ? uiLedBright:uiLedDim;

    uiLeds[12] = !!((VERSION_LO%10)&0x8) ? uiLedBright:uiLedDim;
    uiLeds[13] = !!((VERSION_LO%10)&0x4) ? uiLedBright:uiLedDim;
    uiLeds[14] = !!((VERSION_LO%10)&0x2) ? uiLedBright:uiLedDim;
    uiLeds[15] = !!((VERSION_LO%10)&0x1) ? uiLedBright:uiLedDim;

    while(digitalRead(P_UI_HOLDSW) == LOW);
  }
}

////////////////////////////////////////////////////////////////////////////////
// FLASH IN LED
void uiFlashInLED(unsigned long milliseconds)
{
  digitalWrite(P_UI_IN_LED, HIGH);
  uiUnflashInLED = milliseconds + UI_IN_LED_TIME;
}

////////////////////////////////////////////////////////////////////////////////
// FLASH OUT LED
void uiFlashOutLED(unsigned long milliseconds)
{
  digitalWrite(P_UI_OUT_LED, HIGH);
  uiUnflashOutLED = milliseconds + UI_OUT_LED_TIME;
}

////////////////////////////////////////////////////////////////////////////////
// FLASH SYNCH LED
void uiFlashSynchLED(unsigned long milliseconds)
{
  digitalWrite(P_UI_SYNCH_LED, HIGH);
  uiUnflashSynchLED = milliseconds + UI_SYNCH_LED_TIME;
}

////////////////////////////////////////////////////////////////////////////////
//  CLEAR ALL LEDS
void uiClearLeds()
{
  for(int i=0;i<16;++i)
    uiLeds[i] = 0;
}

////////////////////////////////////////////////////////////////////////////////
// SET A RANGE OF LEDS TO SAME STATUS
void uiSetLeds(int start, int len, byte newStatus)
{
  while(start < 16 && len-- > 0)
    uiLeds[start++] = newStatus;
}

////////////////////////////////////////////////////////////////////////////////
// SET ALL LEDS FROM 16-BIT UINT
void uiSetLeds(unsigned int enabled, unsigned int status)
{
  unsigned int m=0x8000;
  for(int i=0; i<16;++i)
  {
    if(status & m)
      uiLeds[i] = uiLedBright;
    else if(enabled & m)
      uiLeds[i] = uiLedDim;
    else 
      uiLeds[i] = 0;
     m>>=1;
  }
}

////////////////////////////////////////////////////////////////////////////////
// RUN THE UI STATE MACHINE
void uiRun(unsigned long milliseconds)
{
  if(uiUnflashInLED && uiUnflashInLED < milliseconds)
  {
    digitalWrite(P_UI_IN_LED, LOW);
    uiUnflashInLED = 0;
  }
  if(uiUnflashOutLED && uiUnflashOutLED < milliseconds)
  {
    digitalWrite(P_UI_OUT_LED, LOW);
    uiUnflashOutLED = 0;
  }
  if(uiUnflashSynchLED && uiUnflashSynchLED < milliseconds)
  {
    digitalWrite(P_UI_SYNCH_LED, LOW);
    uiUnflashSynchLED = 0;
  }

  // Hold button logic  
  if(milliseconds < uiHoldPressedTime + UI_DEBOUNCE_MS)
  {
    // debouncing... ignore everything
  }
  else
    if(!digitalRead(P_UI_HOLDSW))
    {
      // button pressed... new press?
      if(!(uiHoldType & UI_HOLD_PRESSED))
      {
        // record and debounce it
        uiHoldType |= UI_HOLD_PRESSED;
        uiHoldPressedTime = milliseconds;
      }
      else if(!(uiHoldType & UI_HOLD_HELD) && (milliseconds > uiHoldPressedTime + uiLongHoldTime))
      {
        // record a long hold and set LOCK flag
        uiHoldType |= UI_HOLD_HELD;          
        uiHoldType |= UI_HOLD_LOCKED;          
        uiFlashHold = 0;
      }
    }  
    else
      if(!!(uiHoldType & UI_HOLD_PRESSED))
      {
        if(!(uiHoldType & UI_HOLD_HELD))
        {
          // release after short hold clears lock flag
          // and toggles hold
          if(!!(uiHoldType & UI_HOLD_LOCKED))
            uiHoldType &= ~UI_HOLD_LOCKED;
          else
            uiHoldType ^= UI_HOLD_CHORD;
        }
        uiHoldType &= ~UI_HOLD_PRESSED;
        uiHoldType &= ~UI_HOLD_HELD;
        uiHoldPressedTime = milliseconds;
      }    

  if(uiHoldType & UI_HOLD_LOCKED)
    digitalWrite(P_UI_HOLD_LED, !!(uiFlashHold & 0x80));
  else
    digitalWrite(P_UI_HOLD_LED, !!(uiHoldType & UI_HOLD_CHORD));   
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//
//
// EEPROM PROXY FUNCTIONS
//
//
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

enum {
  EEPROM_MAGIC_COOKIE = 99,
  EEPROM_INPUT_CHAN = 100,
  EEPROM_OUTPUT_CHAN,
  EEPROM_SYNCH_SOURCE,
  EEPROM_SYNCH_SEND,
  EEPROM_MIDI_OPTS,
  EEPROM_PREFS0,
  EEPROM_PREFS1,
  EEPROM_ARPOPTIONS0,
  EEPROM_ARPOPTIONS1
};
#define EEPROM_MAGIC_COOKIE_VALUE  0x22

////////////////////////////////////////////////////////////////////////////////
// SET A VALUE IN EEPROM
void eepromSet(byte which, byte value)
{
  EEPROM.write(which, value);
}

////////////////////////////////////////////////////////////////////////////////
// GET A VALUE FROM EEPROM
byte eepromGet(byte which, byte minValue = 0x00, byte maxValue = 0xFF, byte defaultValue = 0x00)
{
  byte value = EEPROM.read(which);
  if(value == defaultValue)
    return value;
  if(value < minValue || value > maxValue)
  {
    value = defaultValue;
    eepromSet(which, value);
  }
  return value;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//
//
// LOW LEVEL MIDI HANDLING
//
//
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// max ms we will wait for a mandatory midi parameter to arrive
#define MIDI_PARAM_TIMEOUT  50

// state variables
byte midiInRunningStatus;
byte midiOutRunningStatus;
byte midiNumParams;
byte midiParams[2];
char midiParamIndex;
byte midiSendChannel;
byte midiReceiveChannel;
byte midiOptions;

// midi options
#define MIDI_OPTS_SEND_CHMSG       0x01
#define MIDI_OPTS_PASS_INPUT_NOTES 0x02
#define MIDI_OPTS_PASS_INPUT_CHMSG 0x04
#define MIDI_OPTS_SYNCH_INPUT      0x08
#define MIDI_OPTS_SYNCH_AUX        0x10
#define MIDI_OPTS_FILTER_CHMODE    0x20
#define MIDI_OPTS_VOLCAFM_VEL      0x40

#define MIDI_OPTS_MAX_VALUE        0x3F
#define MIDI_OPTS_DEFAULT_VALUE    (MIDI_OPTS_SEND_CHMSG|MIDI_OPTS_SYNCH_INPUT|MIDI_OPTS_SYNCH_AUX)

// macros
#define MIDI_IS_NOTE_ON(msg) ((msg & 0xf0) == 0x90)
#define MIDI_IS_NOTE_OFF(msg) ((msg & 0xf0) == 0x80)
#define MIDI_MK_NOTE (0x90 | midiSendChannel)
#define MIDI_MK_CTRL_CHANGE (0xB0 | midiSendChannel)
#define MIDI_MK_PITCHBEND   (0xE0 | midiSendChannel)

// realtime synch messages
#define MIDI_SYNCH_TICK     0xf8
#define MIDI_SYNCH_START    0xfa
#define MIDI_SYNCH_CONTINUE 0xfb
#define MIDI_SYNCH_STOP     0xfc

#define MIDI_OMNI           0xff

////////////////////////////////////////////////////////////////////////////////
// MIDI INIT
void midiInit()
{
  // init the serial port
  Serial.begin(31250);
  Serial.flush();

  midiInRunningStatus = 0;
  midiOutRunningStatus = 0;
  midiNumParams = 0;
  midiParamIndex = 0;
  midiSendChannel = eepromGet(EEPROM_OUTPUT_CHAN, 0, 15, 0);
  midiReceiveChannel = eepromGet(EEPROM_INPUT_CHAN, 0, 15, MIDI_OMNI);
  midiOptions = eepromGet(EEPROM_MIDI_OPTS, 0, MIDI_OPTS_MAX_VALUE, MIDI_OPTS_DEFAULT_VALUE);

}

////////////////////////////////////////////////////////////////////////////////
// MIDI WRITE
void midiWrite(byte statusByte, byte param1, byte param2, byte numParams, unsigned long milliseconds)
{
  if((statusByte & 0xf0) == 0xf0)
  {
    // realtime byte pass straight through
    Serial.write(statusByte);
  }
  else
  {
    // send channel message
    if(midiOutRunningStatus != statusByte)
    {
      Serial.write(statusByte);
      midiOutRunningStatus = statusByte;
    }
    if(numParams > 0)
      Serial.write(param1);
    if(numParams > 1)
      Serial.write(param2);    
  }

  // indicate activity
  uiFlashOutLED(milliseconds);
}

////////////////////////////////////////////////////////////////////////////////
// MIDI READ
byte midiRead(unsigned long milliseconds, byte passThru)
{


  // loop while we have incoming MIDI serial data
  while(Serial.available())
  {    
    // fetch the next byte
    byte ch = Serial.read();

    // REALTIME MESSAGE
    if((ch & 0xf0) == 0xf0)
    {
      switch(ch)
      {
      case MIDI_SYNCH_TICK:
        if(!!(synchFlags & SYNCH_TO_MIDI) && !!(midiOptions & MIDI_OPTS_SYNCH_INPUT))
          synchTick(SYNCH_SOURCE_INPUT);
        break;            
      case MIDI_SYNCH_START:
        if(!!(synchFlags & SYNCH_TO_MIDI) && !!(midiOptions & MIDI_OPTS_SYNCH_INPUT))
          synchStart(SYNCH_SOURCE_INPUT);
        break;
      case MIDI_SYNCH_CONTINUE:
        if(!!(synchFlags & SYNCH_TO_MIDI) && !!(midiOptions & MIDI_OPTS_SYNCH_INPUT))
          synchContinue(SYNCH_SOURCE_INPUT);
        break;
      case MIDI_SYNCH_STOP:
        if(!!(synchFlags & SYNCH_TO_MIDI) && !!(midiOptions & MIDI_OPTS_SYNCH_INPUT))
          synchStop(SYNCH_SOURCE_INPUT);
        break;
      }
    }      
    // CHANNEL STATUS MESSAGE
    else if(!!(ch & 0x80))
    {
      midiParamIndex = 0;
      midiInRunningStatus = ch; 
      switch(ch & 0xF0)
      {
      case 0xA0: //  Aftertouch  1  key  touch  
      case 0xC0: //  Patch change  1  instrument #   
      case 0xD0: //  Channel Pressure  1  pressure  
        midiNumParams = 1;
        break;    
      case 0x80: //  Note-off  2  key  velocity  
      case 0x90: //  Note-on  2  key  veolcity  
      case 0xB0: //  Continuous controller  2  controller #  controller value  
      case 0xE0: //  Pitch bend  2  lsb (7 bits)  msb (7 bits)  
      default:
        midiNumParams = 2;
        break;        
      }
    }    
    else if(midiInRunningStatus)
    {
      // gathering parameters
      midiParams[midiParamIndex++] = ch;
      if(midiParamIndex >= midiNumParams)
      {
        midiParamIndex = 0;

        // flash the LED
        uiFlashInLED(milliseconds);

        // is it a channel message for our channel?
        if(MIDI_OMNI == midiReceiveChannel ||
          (midiInRunningStatus & 0x0F) == midiReceiveChannel)
        {
          switch(midiInRunningStatus & 0xF0)
          {
          case 0x80: // note off
          case 0x90: // note on
            if(!!(midiOptions & MIDI_OPTS_PASS_INPUT_NOTES) || !!(uiHoldType & UI_HOLD_LOCKED))
              midiWrite(midiInRunningStatus, midiParams[0], midiParams[1], midiNumParams, milliseconds);                
            return midiInRunningStatus; // return to the arp engine
          case 0xB0: // CC
            if(!!(midiOptions & MIDI_OPTS_FILTER_CHMODE)) {
              if(midiParams[0] >= 120) {
                // break from switch - ignore message
                break;
              }
            }
            // otherwise fall through
          default:
            if(!!(midiOptions & MIDI_OPTS_PASS_INPUT_CHMSG) || !!(uiHoldType & UI_HOLD_LOCKED))
              midiWrite(midiInRunningStatus, midiParams[0], midiParams[1], midiNumParams, milliseconds);                  
            // send to the new channel
            if(!!(midiOptions & MIDI_OPTS_SEND_CHMSG))
              midiWrite((midiInRunningStatus & 0xF0)|midiSendChannel, midiParams[0], midiParams[1], midiNumParams, milliseconds);
          }
        }
        else
        {                
          // send thru to output
          midiWrite(midiInRunningStatus, midiParams[0], midiParams[1], midiNumParams, milliseconds);
        }
      }
    }
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// MIDI SEND REALTIME
void midiSendRealTime(byte msg)
{
  Serial.write(msg);
}

////////////////////////////////////////////////////////////////////////////////
// MIDI PANIC
void midiPanic()
{  
  midiOutRunningStatus = 0;
  for(int i=0;i<128;++i)
    midiWrite(MIDI_MK_NOTE, i, 0, 2, millis());    
}

////////////////////////////////////////////////////////////////////////////////
// CLEAR RUUNNING STATUS
void midiClearRunningStatus()
{
  midiOutRunningStatus = 0;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//
//
// SYNCHRONISATION
// - Functions for managing the beat clock
//
//
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// variables
volatile unsigned long synchTickCount;         // tick count
volatile int synchPlayRate;                    // ratio of ticks per arp note
volatile unsigned long synchPlayIndex;         // arp note count
volatile int synchBPM;                         // internal synch bpm
volatile float synchInternalTickPeriod;        // internal synch millseconds per tick
volatile float synchNextInternalTick;          // internal synch next tick time
volatile unsigned long synchLastStepTime;      // the last step time
volatile unsigned long synchStepPeriod;          // period between steps
volatile unsigned int  synchFlags;                      // synch events to send
volatile char synchTicksToSend;                // ticks to send
volatile int synchThisPulseClockPeriod;         
volatile unsigned long synchLastPulseClockTime;         
byte synchInternalTicksSinceLastPulseClock;
volatile char synchPulseClockTickCount;        // number of pending clock out pulses
byte synchClockSendState;                      // ms counter for pulse width
byte synchClockSendStateTimer;                 // used to detect change in ms
byte synchStartSource;                         // In ext sync mode, remembers how clock was started

#define SYNCH_HH_EXT_CLOCK  (0xFF)             // special sendState value meaning external clock in use

// Define pins used for the hack header
#define P_SYNCH_HH_DETECT                19
#define P_SYNCH_HH_CLKOUT                18  
#define P_SYNCH_HH_CLKIN                 14
#if SYNCH_HH_CLOCK_ACTIVELOW    
  #define SYNCH_HH_CLOCK_ON           PORTC &= ~(1<<4)
  #define SYNCH_HH_CLOCK_OFF          PORTC |= (1<<4)
#else
  #define SYNCH_HH_CLOCK_OFF          PORTC &= ~(1<<4)
  #define SYNCH_HH_CLOCK_ON           PORTC |= (1<<4)
#endif
#define SYCH_HH_CLOCK_IN              (PINC & (1<<0))
#define SYNCH_HH_INPUT_DETECT         (!(PINC & (1<<5)))
#define SYNCH_HH_ENABLE_PCINT         (PCMSK1 |= (1<<0))
#define SYNCH_HH_DISABLE_PCINT        (PCMSK1 &= ~(1<<0))

// PIN DEFS (From PIC MCU servicing MIDI SYNCH input)
#define P_SYNCH_TICK     2
#define P_SYNCH_RESTART  3
#define P_SYNCH_RUN      4

#define DBIT_SYNCH_RUN   (1<<4) // port D bit that indicates a run status on AUX SYNCH port

#define MILLISECONDS_PER_MINUTE 60000
#define TICKS_PER_QUARTER_NOTE 24
#define SYNCH_DEFAULT_BPM 120

// Values for synchRate
enum
{
  SYNCH_RATE_1    = 96,
  SYNCH_RATE_2D   = 72,
  SYNCH_RATE_2    = 48,
  SYNCH_RATE_4D   = 36,
  SYNCH_RATE_2T   = 32,  
  SYNCH_RATE_4    = 24,
  SYNCH_RATE_8D   = 18,
  SYNCH_RATE_4T   = 16,
  SYNCH_RATE_8    = 12,
  SYNCH_RATE_16D  = 9,
  SYNCH_RATE_8T   = 8,
  SYNCH_RATE_16   = 6,
  SYNCH_RATE_16T  = 4,
  SYNCH_RATE_32   = 3
};

//////////////////////////////////////////////////////////////////////////
// HANDLER FOR A MIDI CLOCK TICK
// This could be internally generated or come from an external source
void synchTick(byte source)
{
  if(synchFlags & SYNCH_HOLD_AT_ZERO) {
      synchTickCount = 0;
      synchPlayIndex = 0;
  }
  else 
  {
    if(synchFlags & SYNCH_ZERO_TICK_COUNT) {
      synchFlags &= ~SYNCH_ZERO_TICK_COUNT;
      synchTickCount = 0;
    }
    else {
      ++synchTickCount;
    }
    if(!(synchTickCount % synchPlayRate))//ready for next step?
    {
      // store step length in ms.. this will be used
      // when calculating step length
      unsigned long ms = millis();
      if(synchLastStepTime > 0)
        synchStepPeriod = ms - synchLastStepTime;
      synchLastStepTime = ms;

      if((synchFlags & SYNCH_TO_MIDI) && synchStartSource != source && synchStartSource != SYNCH_SOURCE_MANUAL)
      {
        // do not advance playback - ext synch is stopped
      }
      else 
      {
        if(synchFlags & SYNCH_RESTART_ON_BEAT)
        {
          synchPlayIndex = 0;
        }
        else 
        {
          synchPlayIndex++;
        }
        synchFlags |= SYNCH_PLAY_ADVANCE;
      }
    }
    synchFlags &= ~SYNCH_RESTART_ON_BEAT;
    
    if(!(synchTickCount % TICKS_PER_QUARTER_NOTE))
      synchFlags |= SYNCH_BEAT;
  }

  if(synchFlags & SYNCH_SEND_CLOCK)
    synchTicksToSend++;
    
  if(IS_HH_CLOCK)
    synchPulseClockTickCount++;
}

//////////////////////////////////////////////////////////////////////////
// RESTART PLAY FROM START OF SEQUENCE IMMEDIATELY
void synchRestartSequence()
{
  synchLastStepTime = millis();
  synchTickCount = 0;
  synchPlayIndex = 0;
  synchTicksToSend = 0;
  synchFlags |= SYNCH_BEAT;
  synchFlags |= SYNCH_PLAY_ADVANCE;
}

//////////////////////////////////////////////////////////////////////////
// START PLAYING
void synchStart(byte source)
{
  synchStartSource = source;  
  synchFlags |= SYNCH_SEND_START;  
  synchRestartSequence();
}

//////////////////////////////////////////////////////////////////////////
// STOP PLAYING
void synchStop(byte source)
{
  synchStartSource = SYNCH_SOURCE_NONE;  
  synchFlags |= SYNCH_SEND_STOP;
}

//////////////////////////////////////////////////////////////////////////
// CONTINUE PLAYING FROM CURRENT POSITION
void synchContinue(byte source)
{
  synchStartSource = source;  
  synchFlags |= SYNCH_SEND_CONTINUE;
}

//////////////////////////////////////////////////////////////////////////
//
// synchReset_ISR
// Called at start of bar
// 
//////////////////////////////////////////////////////////////////////////
ISR(synchReset_ISR)
{
  if(!!(synchFlags & SYNCH_TO_MIDI) && !!(midiOptions & MIDI_OPTS_SYNCH_AUX))     
    synchStart(SYNCH_SOURCE_AUX);
}

//////////////////////////////////////////////////////////////////////////
//
// synchTick_ISR
// Called on midi synch
// 
//////////////////////////////////////////////////////////////////////////
ISR(synchTick_ISR)
{
  // For AUX midi tick to be actioned we need to synch to AUX MIDI and we need a RUNNING status
  if(!!(synchFlags & SYNCH_TO_MIDI) && !!(midiOptions & MIDI_OPTS_SYNCH_AUX))
    synchTick(SYNCH_SOURCE_AUX);
}

//////////////////////////////////////////////////////////////////////////
//
// PCINT1_vect
// Pin change interrupt on external pulse clock
// 
//////////////////////////////////////////////////////////////////////////
ISR(PCINT1_vect)
{
  if(SYCH_HH_CLOCK_IN)  // rising edge
  {
    unsigned long ms = millis();
    if(synchLastPulseClockTime && synchLastPulseClockTime < ms)
      synchThisPulseClockPeriod = ms - synchLastPulseClockTime;
    else      
      synchFlags |= SYNCH_ZERO_TICK_COUNT; // synch the beat LED
    synchLastPulseClockTime = ms;
  }
}

//////////////////////////////////////////////////////////////////////////
// SET TEMPO
void synchSetTempo(int bpm)
{
  synchBPM = bpm;
  synchInternalTickPeriod = (float)MILLISECONDS_PER_MINUTE/(bpm * TICKS_PER_QUARTER_NOTE);
}
void synchSetInternalTickPeriod(float period)
{
  synchInternalTickPeriod = period;
  int bpm = (float)MILLISECONDS_PER_MINUTE / (TICKS_PER_QUARTER_NOTE * period);
  if(synchBPM != bpm)
  {
    synchBPM = bpm;
    editFlags |= EDIT_FLAG_FORCE_REFRESH;
  }
}
void synchResynch() 
{
  synchFlags |= SYNCH_RESTART_ON_BEAT;
  synchLastPulseClockTime = 0;
}

////////////////////////////////////////////////////////////////////////////////
// SYNCH INIT
void synchInit()
{
  synchFlags = 0;
  
  // by default don't synch
  if(eepromGet(EEPROM_SYNCH_SOURCE,0,1,0)) 
    synchFlags |= SYNCH_TO_MIDI;

  // by default do not send synch
  if(eepromGet(EEPROM_SYNCH_SEND,0,1,0))
    synchFlags |= SYNCH_SEND_CLOCK;
  
  synchTicksToSend = 0;
  synchStartSource = SYNCH_SOURCE_NONE;  
  
  // set default play rate
  synchPlayRate = SYNCH_RATE_16;

  synchLastStepTime = 0;
  synchStepPeriod = 0;

  // reset the counters
  synchRestartSequence();

  // initialise internal synch generator
  synchSetTempo(SYNCH_DEFAULT_BPM);
  synchNextInternalTick = 0;

  pinMode(P_SYNCH_TICK, INPUT_PULLUP);
  pinMode(P_SYNCH_RESTART, INPUT_PULLUP);
  pinMode(P_SYNCH_RUN, INPUT_PULLUP);
  attachInterrupt(0, synchReset_ISR, RISING);
  attachInterrupt(1, synchTick_ISR, RISING);

  synchPulseClockTickCount = 0;
  synchClockSendState = 0;
  synchClockSendStateTimer = 0;
  synchLastPulseClockTime  = 0;
  synchThisPulseClockPeriod = 0;         
  synchInternalTicksSinceLastPulseClock = 0;
  
  if(IS_HH_CLOCK) {
     pinMode(P_SYNCH_HH_CLKOUT, OUTPUT);
     pinMode(P_SYNCH_HH_CLKIN, INPUT);
     pinMode(P_SYNCH_HH_DETECT, INPUT_PULLUP);
     delay(10);
     if(SYNCH_HH_INPUT_DETECT) 
     {    
       synchClockSendState = SYNCH_HH_EXT_CLOCK;  // disable clock output
       PCICR |= (1<<1);                           // enable pin change interrupt 1
       PCMSK1 = 0;                                // initially all pins disabled for PC interrupt
       SYNCH_HH_ENABLE_PCINT;                     // PCINT enabled on input pin 
     }
     else
     {
       SYNCH_HH_DISABLE_PCINT;                    // no pin change interrupt
     } 
  }
  interrupts();
}

////////////////////////////////////////////////////////////////////////////////
// SYNCH RUN
void synchRun(unsigned long milliseconds)
{
  if(!!(synchFlags & SYNCH_TO_MIDI) && !!(midiOptions & MIDI_OPTS_SYNCH_AUX)) // Check if we are interested
  {
    byte auxRunning = !!(PIND & DBIT_SYNCH_RUN); // see if aux port is running
    if(auxRunning && !(synchFlags & SYNCH_AUX_RUNNING)) // transition STOP->RUN
    {
      synchFlags |= SYNCH_AUX_RUNNING;
      synchContinue(SYNCH_SOURCE_AUX);
    }
    else if(!auxRunning && !!(synchFlags & SYNCH_AUX_RUNNING)) // transition RUN->STOP
    {
      synchFlags &= ~SYNCH_AUX_RUNNING;
      synchStop(SYNCH_SOURCE_AUX);
    }
  }  
//  synchFlags = auxRunning? (synchFlags|SYNCH_AUX_RUNNING) : (synchFlags&~SYNCH_AUX_RUNNING);

  // else check if we are using internal clock
  if(!(synchFlags & SYNCH_TO_MIDI))
  { 
    if(synchThisPulseClockPeriod) // have we got a clock period measurement?
    {      
      synchSetInternalTickPeriod((float)synchThisPulseClockPeriod / SYNCH_TICK_TO_PULSE_RATIO);  // infer bpm
      synchThisPulseClockPeriod = 0;
      synchNextInternalTick = milliseconds;  // tick immediately
      while(synchInternalTicksSinceLastPulseClock++ < SYNCH_TICK_TO_PULSE_RATIO)
        synchTick(SYNCH_SOURCE_INTERNAL); // make up any missed ticks (when tempo is being increased)
      synchInternalTicksSinceLastPulseClock = 0;
    }
    
    if(synchClockSendState == SYNCH_HH_EXT_CLOCK && 
      synchInternalTicksSinceLastPulseClock >= SYNCH_TICK_TO_PULSE_RATIO)
    {
      // hold off any ticks until the next external pulse. External tempo is being reduced
    }
    else if(synchFlags & SYNCH_RESET_NEXT_STEP_TIME)
    {      
      synchNextInternalTick = milliseconds + synchInternalTickPeriod;
    }
    // need to generate our own ticks
    else if(synchNextInternalTick < milliseconds)
    {
      ++synchInternalTicksSinceLastPulseClock;
      synchTick(SYNCH_SOURCE_INTERNAL);
      synchNextInternalTick += synchInternalTickPeriod;
      if(synchNextInternalTick < milliseconds)
        synchNextInternalTick = milliseconds + synchInternalTickPeriod;
    }
  }
  synchFlags &= ~SYNCH_RESET_NEXT_STEP_TIME;//clear flag



  if(synchFlags & SYNCH_SEND_CLOCK)//Are we sending MIDI synch to slaves?
  {
    if(synchFlags & SYNCH_SEND_STOP)//Stop
    {
      midiSendRealTime(MIDI_SYNCH_STOP);
      synchTicksToSend = 0;
    }
    else if(synchFlags & SYNCH_SEND_START)//Start
    {
      midiSendRealTime(MIDI_SYNCH_START);
    }  
    else if(synchFlags & SYNCH_SEND_CONTINUE)//Continue?
    {
      midiSendRealTime(MIDI_SYNCH_CONTINUE);
    }  

    if(synchTicksToSend>0) // need to send a tick?
    {
      synchTicksToSend--;
      midiSendRealTime(MIDI_SYNCH_TICK);
    }    
  }
  else
  {
    if(synchFlags & SYNCH_SEND_STOP)//Stop
      midiSendRealTime(MIDI_SYNCH_STOP);
    synchTicksToSend = 0; // ensure no ticks will be sent
  }
  synchFlags &= ~(SYNCH_SEND_STOP|SYNCH_SEND_START|SYNCH_SEND_CONTINUE); // clear all realtime msg flags

  ///////////////////////////////////////////////////////
  // Handling pulse clock output 
  if(synchClockSendState != SYNCH_HH_EXT_CLOCK) 
  {    
    if(!synchClockSendState)
    {
      // check if a new pulse is required
      if(synchPulseClockTickCount>=SYNCH_TICK_TO_PULSE_RATIO)
      {
        // start a new pulse
        SYNCH_HH_CLOCK_ON;
        synchPulseClockTickCount -= SYNCH_TICK_TO_PULSE_RATIO;
        synchClockSendState = 1;
      }
    }
    else if(synchClockSendStateTimer != (byte)milliseconds)
    {
      // pulse is in progress
      synchClockSendStateTimer = (byte)milliseconds;
      ++synchClockSendState;
      if(synchClockSendState >= SYNCH_CLOCK_MIN_PERIOD)
        synchClockSendState = 0;
  
      if(synchClockSendState == SYNCH_CLOCK_PULSE_WIDTH)
        SYNCH_HH_CLOCK_OFF;
    }
  }
  ///////////////////////////////////////////////////////

  // check if we need to report a beat
  if(!!(synchFlags & SYNCH_BEAT))
  {
    uiFlashSynchLED(milliseconds);
    synchFlags&=~SYNCH_BEAT;
  } 
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//
//
//
// ARPEGGIATOR FUNCTIONS
//
//
//
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Macro defs
#define ARP_MAX_CHORD 12
#define ARP_MAKE_NOTE(note, vel) ((((unsigned int)vel)<<8)|(note))
#define ARP_GET_NOTE(x) ((x)&0x7f)
#define ARP_GET_VELOCITY(x) (((x)>>8)&0x7f)
#define ARP_MAX_SEQUENCE 60
#define ARP_NOTE_HELD 0x8000
#define ARP_PLAY_THRU 0x8000
#define ARP_PATN_PLAY     0x01
#define ARP_PATN_GLIDE    0x02
#define ARP_PATN_TIE      0x04
#define ARP_PATN_ACCENT   0x08
#define ARP_PATN_OCTAVE   0x10
#define ARP_PATN_OCTDN    0x20
#define ARP_PATN_4TH      0x40
#define ARP_PATN_PLAYTHRU 0x80

// Values for arpType
enum 
{
  ARP_TYPE_UP = 0,
  ARP_TYPE_DOWN,
  ARP_TYPE_UP_DOWN,
  ARP_TYPE_RANDOM,
  ARP_TYPE_MANUAL,
  ARP_TYPE_POLYGATE
};

// Values for arpInsertMode
enum
{
  ARP_INSERT_OFF = 0,
  ARP_INSERT_HI,
  ARP_INSERT_LOW,
  ARP_INSERT_3_1,
  ARP_INSERT_4_2
};

// Values for force to scale masks
enum 
{
  //                               0123456789012345
  ARP_SCALE_CHROMATIC = (unsigned)0b0000111111111111,  //no scale
  ARP_SCALE_IONIAN    = (unsigned)0b0000101011010101,  //diatonic modes
  ARP_SCALE_DORIAN    = (unsigned)0b0000101101010110,  //:
  ARP_SCALE_PHRYGIAN  = (unsigned)0b0000110101011010,  //:
  ARP_SCALE_LYDIAN    = (unsigned)0b0000101010110101,  //:
  ARP_SCALE_MIXOLYDIAN= (unsigned)0b0000101011010110,  //:
  ARP_SCALE_AEOLIAN   = (unsigned)0b0000101101011010,  //:
  ARP_SCALE_LOCRIAN   = (unsigned)0b0000110101101010,  //:
};

// Force to scale mode for out of scale notes
enum
{
    ARP_SCALE_ADJUST_SKIP   = (unsigned)0x0000,  // Out of scale notes are skipped
    ARP_SCALE_ADJUST_MUTE   = (unsigned)0x1000,  // Out of scale notes are muted
    ARP_SCALE_ADJUST_FLAT   = (unsigned)0x2000,  // Out of scale notes are flattened to bring them to scale
    ARP_SCALE_ADJUST_SHARP  = (unsigned)0x3000,  // Out of scale notes are sharpened to bring them to scale
    ARP_SCALE_ADJUST_TOGGLE = (unsigned)0x4000,  // Out of scale notes are sharpened/flattened alternately
    
    ARP_SCALE_ADJUST_MASK   = (unsigned)0x7000,        
    ARP_SCALE_TOGGLE_FLAG   = (unsigned)0x8000
};

// ARP PARAMETERS
byte arpType;              // arpeggio type
char arpOctaveShift;       // octave transpose
byte arpOctaveSpan;        // number of octaves to span with the arpeggio
byte arpInsertMode;        // additional note insertion mode
byte arpVelocityMode;      // (0 = original, 1 = override)
byte arpVelocity;          // velocity 
byte arpGateLength;        // gate length (0 = tie notes, 1-127 = fraction of beat)
char arpTranspose;         // up/down transpose
char arpForceToScaleRoot;  // Defines the root note of the scale (0 = C)
unsigned int arpForceToScaleMask;   // Force to scale interval mask (defines the scale)

// CHORD INFO - notes held by user
unsigned int arpChord[ARP_MAX_CHORD];
int arpChordLength;        // number of notes in the chord
int arpNotesHeld;          // number of notes physically held
char arpChordRootNote;

// Manual chord selected by the user
unsigned int arpManualChord;

// ARPEGGIO SEQUENCE - the arpeggio build from chord/inserts etc
unsigned int arpSequence[ARP_MAX_SEQUENCE];
int arpSequenceLength;     // number of notes in the sequence
int arpSequenceIndex;

// NOTE PATTERN - the rythmic pattern of played/muted notes
byte arpPattern[ARP_MAX_SEQUENCE];
byte arpPatternLength;   // user-defined pattern length (1-16)
byte arpPatternIndex;    // position in the pattern (for display)

byte arpRefresh;  // whether the pattern index is changed

// STOP NOTE - remembers which (single) note is playing
// and when it should be stopped
byte arpPlayingNotes[16];
unsigned long arpStopNoteTime;

// used to time the length of a step
unsigned long arpLastPlayAdvance;

#define ARP_MAX_TRAN_SEQ 16
char arpTransposeSequence[ARP_MAX_TRAN_SEQ];
byte arpTransposeSequenceLen;
byte arpTransposeSequencePos;
unsigned int arpTransposeSequenceMask;

byte arpLastVelocityCC;

enum {
  ARP_FLAG_REBUILD = 0x01,
  ARP_FLAG_MUTE    = 0x02
};

// ARP STATUS FLAGS
byte arpFlags;          // whether the sequence needs to be rebuilt
unsigned int arpOptions;

enum {
                                      //012345678901234567
  ARP_OPT_MIDITRANSPOSE   = (unsigned)0b1000000000000000, // Hold button secondary function
  ARP_OPT_SKIPONREST      = (unsigned)0b0010000000000000, // Whether rests are skipped or held
  ARP_OPTS_MASK           = (unsigned)0b1010000000000000,

  ARP_LAYER_ACCENT        = (unsigned)0b0000000000000001,
  ARP_LAYER_GLIDE         = (unsigned)0b0000000000000010,
  ARP_LAYER_TIE           = (unsigned)0b0000000000000100,
  ARP_LAYER_OCTUP         = (unsigned)0b0000000000001000,
  ARP_LAYER_OCTDN         = (unsigned)0b0000000000010000,
  ARP_LAYER_4THDN         = (unsigned)0b0000000000100000,
  ARP_LAYER_PLAYTHRU      = (unsigned)0b0000000001000000,
  ARP_LAYER_MASK          = (unsigned)0b0000000001111111
};

////////////////////////////////////////////////////////////////////////////////
// APPLY ARP OPTIONS BITS TO VARIABLES
void arpOptionsApply()
{
}

////////////////////////////////////////////////////////////////////////////////
// LOAD ARP OPTIONS
void arpOptionsLoad()
{
  unsigned int q;
  q = eepromGet(EEPROM_ARPOPTIONS1); 
  q<<=8;
  q |= eepromGet(EEPROM_ARPOPTIONS0);   
  arpOptions &= ~ARP_OPTS_MASK;
  arpOptions |= (q & ARP_OPTS_MASK);
  arpOptionsApply();  
}

////////////////////////////////////////////////////////////////////////////////
// SAVE NEW PRFERENCES TO EEPROM
void arpOptionsSave()
{
  eepromSet(EEPROM_ARPOPTIONS0,(arpOptions&0xFF));  
  eepromSet(EEPROM_ARPOPTIONS1,((arpOptions>>8)&0xFF));  
}

////////////////////////////////////////////////////////////////////////////////
// ARP INIT
void arpInit()
{
  int i;
  
  //  arpHold = 0;
  arpType = ARP_TYPE_UP;
  arpOctaveShift = 0;
  arpOctaveSpan = 1;
  arpInsertMode = ARP_INSERT_OFF;
  arpVelocity = 127;
  arpVelocityMode = 1;
  arpChordLength = 0;
  arpNotesHeld = 0;
  arpPatternLength = 16;
  arpRefresh = 0;
  arpFlags = 0;
  arpGateLength = 100;
  arpSequenceLength = 0;
  arpLastPlayAdvance = 0;
  arpTranspose = 0;
  arpForceToScaleRoot=0;
  arpForceToScaleMask=ARP_SCALE_CHROMATIC|ARP_SCALE_ADJUST_SHARP;
  arpChordRootNote = -1;
  arpSequenceIndex = 0;
  arpManualChord = 0;
  arpTransposeSequenceLen = 0;
  arpTransposeSequencePos = 0;
  arpTransposeSequenceMask = 0;
  arpOptionsLoad();
  arpOptions &= ~ARP_LAYER_MASK;
  arpOptions |= ARP_LAYER_ACCENT;
  arpLastVelocityCC = 0xFF;
  
  // the pattern starts with all beats on
  for(i=0;i<16;++i)
    arpPattern[i] = ARP_PATN_PLAY;

  // no notes playing
  for(i=0;i<16;++i)
    arpPlayingNotes[i] = 0;    
}

////////////////////////////////////////////////////////////////////////////////
// START A NOTE PLAYING 
// We remember the note is playing so we can stop it later. If an additional 
// "note set" is provided then we log it there too
void arpStartNote(byte note, byte velocity, unsigned long milliseconds, byte *noteSet)
{  
  if(note<128)
  {
    if((midiOptions & MIDI_OPTS_VOLCAFM_VEL) && (arpLastVelocityCC != velocity)) {
      midiWrite(MIDI_MK_CTRL_CHANGE, VELOCITY_CC, velocity, 2, millis());    
      arpLastVelocityCC = velocity;
    }  
    midiWrite(MIDI_MK_NOTE, note, velocity, 2, milliseconds);          
    byte n = (1<<(note&0x07));
    arpPlayingNotes[note>>3] |= n;
    if(noteSet)
      noteSet[note>>3] |= n;
  }    
}

////////////////////////////////////////////////////////////////////////////////
// STOP PLAYING NOTES
// Stop all currently playing notes. If a "note set" is provided then any notes
// present in it are left alone and remain playing
void arpStopNotes(unsigned long milliseconds, byte *excludedNoteSet)
{ 
  for(int i=0; i<16; ++i)  
  {
    if(arpPlayingNotes[i])
    {
      byte note = i<<3;
      byte n = arpPlayingNotes[i];
      byte m = 0x01;
      while(m)
      {
        if(n&m)
        {
          if(!excludedNoteSet || !(excludedNoteSet[i]&m))
          {
            midiWrite(MIDI_MK_NOTE, note, 0, 2, milliseconds);
            arpPlayingNotes[i] &= ~m;
          }
        }
        ++note;
        m<<=1;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// CLEAR CHORD
void arpClear()
{
  arpChordLength = 0;
  arpManualChord = 0;
  editFlags |= EDIT_FLAG_FORCE_REFRESH;
  arpFlags |= ARP_FLAG_REBUILD;
}

////////////////////////////////////////////////////////////////////////////////
// COPY CHORD - RETURN LOWEST NOTE
char arpCopyChord(int *dest)
{
  char m =-1;
  int i;
  for(i=0; i<arpChordLength; ++i)
  {
    dest[i] = arpChord[i];
    if(m == -1 || arpChord[i] < m)
      m = arpChord[i];
  }
  return m;
}

////////////////////////////////////////////////////////////////////////////////
// SORT NOTES OF CHORD - RETURN LOWEST NOTE
// Crappy bubblesort.. but there are not too many notes
char arpSortChord(int *dest)
{
  arpCopyChord(dest);
  byte sorted = 0;
  while(!sorted)
  {
    sorted = 1;
    for(int i=0; i<arpChordLength-1; ++i)
    {
      if(ARP_GET_NOTE(dest[i]) > ARP_GET_NOTE(dest[i+1]))
      {
        int t = dest[i];
        dest[i] = dest[i+1];
        dest[i+1] = t;
        sorted = 0;
      }
    }
  }
 return arpChord[0];
}

////////////////////////////////////////////////////////////////////////////////
// RANDOMIZE CHORD
void arpRandomizeChord(int *dest)
{
  int i,j;

  // clear destination buffer
  for(i=0; i<arpChordLength; ++i)
    dest[i] = 0;

  // loop through the source chord
  for(i=0; i<arpChordLength; ++i)
  {
    // loop until we find a place to 
    // put this note in dest buffer
    for(;;)
    {
      // look for a place
      j = random(arpChordLength);
      if(!dest[j])
      {
        // its empty, so we can use it
        dest[j] = arpChord[i];
        break;
      }
    }        
  }
}

////////////////////////////////////////////////////////////////////////////////
// BUILD A NEW SEQUENCE
void arpBuildSequence()
{  
  // make sure the velocity CC is refreshed
  arpLastVelocityCC = 0xFF;
  
  // sequence is empty if no chord notes present
  arpChordRootNote=-1;
  arpSequenceLength=0;
  if(!arpChordLength)
    return;

  // sort the chord info if needed
  int chord[ARP_MAX_CHORD];
  if(arpType == ARP_TYPE_UP || arpType == ARP_TYPE_DOWN || arpType == ARP_TYPE_UP_DOWN)
    arpChordRootNote = arpSortChord(chord);
  else
    arpChordRootNote = arpCopyChord(chord);

  int tempSequence[ARP_MAX_SEQUENCE];
  int tempSequenceLength = 0;        
  int highestNote = ARP_MAKE_NOTE(0,0);
  int lowestNote = ARP_MAKE_NOTE(127,0);    

  // This outer loop allows us two passes for UP-DOWN mode
  int nextPass = 1;    
  while(nextPass && 
    tempSequenceLength < ARP_MAX_SEQUENCE)
  {
    arpForceToScaleMask ^= ARP_SCALE_TOGGLE_FLAG;
    byte adjustToggle = !!(arpForceToScaleMask & ARP_SCALE_TOGGLE_FLAG);
    
    // this loop is for the octave span
    int octaveCount;
    for(octaveCount = 0; 
      octaveCount < arpOctaveSpan && tempSequenceLength < ARP_MAX_SEQUENCE; 
      ++octaveCount)
    {
      // Set up depending on arp type
      int transpose;
      int chordIndex;
      int lastChordIndex;
      int chordIndexDelta;      
      switch(arpType)
      {
      case ARP_TYPE_RANDOM:
        arpRandomizeChord(chord);
        // fall thru
      case ARP_TYPE_UP:
      case ARP_TYPE_MANUAL:
      case ARP_TYPE_POLYGATE:
        chordIndex = 0;
        lastChordIndex = arpChordLength - 1;
        transpose = arpTranspose + 12 * (arpOctaveShift + octaveCount);    
        chordIndexDelta = 1;
        nextPass = 0;
        break;          

      case ARP_TYPE_DOWN:
        chordIndex = arpChordLength - 1;
        lastChordIndex = 0;
        transpose = arpTranspose + 12 * (arpOctaveShift + arpOctaveSpan - octaveCount - 1);    
        chordIndexDelta = -1;
        nextPass = 0;
        break;          

      case ARP_TYPE_UP_DOWN:        
        if(nextPass == 1)
        {
          // going up we can play all the notes
          chordIndex = 0;
          lastChordIndex = arpChordLength - 1;
          chordIndexDelta = 1;
          transpose = arpTranspose + 12 * (arpOctaveShift + octaveCount);    
          if(octaveCount == arpOctaveSpan - 1)
            nextPass = 2;
        }
        else
        {
          // GOING DOWN!
          // Is the range just one octave?
          if(arpOctaveSpan == 1)
          {
            // On the way down we don't play top or bottom notes of the chord
            chordIndex = arpChordLength - 2;
            lastChordIndex = 1;
            nextPass = 0;
          }
          // are we on the top octave of the descent?
          else if(octaveCount == 0)
          {
            // the top note is skipped, the bottom note can be played
            chordIndex = arpChordLength - 2;
            lastChordIndex = 0;
          }
          // are we on the bottom octave of the descent?
          else if(octaveCount == arpOctaveSpan - 1)
          {
            // top note can be played but bottom note is not
            chordIndex = arpChordLength - 1;
            lastChordIndex = 1;

            // this the the last octave to play
            nextPass = 0;
          }
          else
          {
            // this is not first or last octave of the descent, so there
            // is no need to skip any of the notes
            chordIndex = arpChordLength - 1;
            lastChordIndex = 0;
          }
          transpose = arpTranspose + 12 * (arpOctaveShift + arpOctaveSpan - octaveCount - 1);    
          chordIndexDelta = -1;
        }
        break;
      }        

      // Write notes from the chord into the arpeggio sequence
      while(chordIndex >= 0 && 
        chordIndex < arpChordLength && 
        tempSequenceLength < ARP_MAX_SEQUENCE)
      {
        // fetch the current note
        int note = ARP_GET_NOTE(chord[chordIndex]);
        byte velocity = ARP_GET_VELOCITY(chord[chordIndex]);
        byte skipNote = 0;

        // transpose as needed
        note += transpose;

        // force to scale
        int scaleNote = note - arpForceToScaleRoot;
        while(scaleNote<0)
          scaleNote+=12;
        if(!(arpForceToScaleMask & ((int)0x0800>>(scaleNote % 12))))
        {
          switch(arpForceToScaleMask & ARP_SCALE_ADJUST_MASK)
          { 
              case ARP_SCALE_ADJUST_SKIP: 
                skipNote = 1;
                break;
              case ARP_SCALE_ADJUST_MUTE: 
                note = 0; 
                break;
              case ARP_SCALE_ADJUST_FLAT: 
                --note; 
                break;
              case ARP_SCALE_ADJUST_SHARP: 
                ++note; 
                break;
              case ARP_SCALE_ADJUST_TOGGLE: 
                if(adjustToggle)
                  ++note;
                else
                  --note;
                adjustToggle = !adjustToggle;
                break;
          }
        }
        
        if(!skipNote)
        {
          // force to MIDI range           
          while(note>127)
            note -= 12;
          while(note<0)
            note += 12;          
          int newNote = ARP_MAKE_NOTE(note, velocity);
  
          // track lowest and highest notes
          if(note > ARP_GET_NOTE(highestNote))
            highestNote = newNote;
          if(note < ARP_GET_NOTE(lowestNote))
            lowestNote = newNote;
  
          // insert into sequence
          tempSequence[tempSequenceLength++] = newNote;
        }

        // have we reached the last note we want?
        if(chordIndex == lastChordIndex)
          break;

        // skip to next note in the chord
        chordIndex += chordIndexDelta;
      }      
    }           
  }  


  int i, j;
  arpSequenceLength = 0;
  if(arpType == ARP_TYPE_POLYGATE)
  {
      // Polyphonic gate mode, copy the notes over and flag to be 
      // played at the same time
      for(i=0; i<tempSequenceLength; ++i)
        arpSequence[arpSequenceLength++] = ARP_PLAY_THRU|tempSequence[i];
  }
  else
  {
    // we have the expanded sequence for one octave... now we need to 
    // perform any necessary note insertions
    switch(arpInsertMode)
    {
    case ARP_INSERT_OFF:
      for(i=0; i<tempSequenceLength; ++i)
        arpSequence[arpSequenceLength++] = tempSequence[i];
      break;
    case ARP_INSERT_HI:
      for(i=0; i<tempSequenceLength && arpSequenceLength < ARP_MAX_SEQUENCE; ++i)
      {
        if(tempSequence[i] != highestNote)
        {
          arpSequence[arpSequenceLength++] = highestNote;
          arpSequence[arpSequenceLength++] = tempSequence[i];
        }
      }
      break;
    case ARP_INSERT_LOW:
      for(i=0; i<tempSequenceLength && arpSequenceLength < ARP_MAX_SEQUENCE; ++i)
      {
        if(tempSequence[i] != lowestNote)
        {
          arpSequence[arpSequenceLength++] = lowestNote;
          arpSequence[arpSequenceLength++] = tempSequence[i];
        }
      }
      break;
    case ARP_INSERT_3_1: // 3 steps forward and one back 012123234345456
      i = 0;
      j = 0;
      while(i<tempSequenceLength && arpSequenceLength < ARP_MAX_SEQUENCE)
      {
        arpSequence[arpSequenceLength++] = tempSequence[i];
        if(!(++j%3))
          i--;
        else
          i++;
      }
      break;
    case ARP_INSERT_4_2: // 4 steps forward and 2 back 0123123423453456
      i = 0;
      j = 0;
      while(i<tempSequenceLength && arpSequenceLength < ARP_MAX_SEQUENCE)
      {
        arpSequence[arpSequenceLength++] = tempSequence[i];
        if(!(++j%4))
          i-=2;
        else
          i++;
      }
      break;
    } 
  }
}  

////////////////////////////////////////////////////////////////////////////////
// READ THE MIDI INPUT AND UPDATE THE CHORD BUFFER
void arpReadInput(unsigned long milliseconds)
{
  int i;
  char noteIndexInChord; 

  // we may have multiple notes to read
  for(;;)
  {
    // read the MIDI port
    byte msg = midiRead(milliseconds, 0);      
    if(!msg)
      break;      
      
    byte note = midiParams[0];
    byte velocity = midiParams[1];
    
    // Note on message
    if(MIDI_IS_NOTE_ON(msg) && velocity && note)
    {
      // Check for a lock out (This prevents any new notes
      // being added to the chord, but we still need to track
      // notes that get released)
      if(!!(uiHoldType & UI_HOLD_LOCKED))
      {
       // transpose by MIDI?
       if(!!(arpOptions & ARP_OPT_MIDITRANSPOSE))
       {
         // transpose things
         if(arpChordRootNote != -1)
         {
            arpTranspose = note - arpChordRootNote;                        
            arpFlags |= ARP_FLAG_REBUILD;
            editFlags |= EDIT_FLAG_FORCE_REFRESH;
         }
       }
      }
      else
      {
        // scan the current chord for this note
        // to see if it is already part of the chord      
        noteIndexInChord = -1;
        arpNotesHeld = 0;
        for(i=0;i<arpChordLength;++i)
        {        
          if(ARP_GET_NOTE(arpChord[i])== note)
            noteIndexInChord = i;
          if(arpChord[i] & ARP_NOTE_HELD)
            arpNotesHeld++;
        }
  
        // is the note already part of the current chord?
        if(noteIndexInChord >= 0 && arpNotesHeld)
        {
          // Mark the key as held. There is no change to the arpeggio
          if(!(arpChord[noteIndexInChord] & ARP_NOTE_HELD))
          {        
            arpChord[noteIndexInChord] |= ARP_NOTE_HELD;
            arpNotesHeld++;
          }
        }
        else 
        {
          // if its the first note of a new chord then
          // we need to restart play
          if(!arpNotesHeld)
          {
            arpChordLength = 0;
            if(!!(uiHoldType & UI_HOLD_CHORD))
              synchFlags |= SYNCH_RESTART_ON_BEAT; // wait till next beat before restarting
            else
              synchRestartSequence();
          }
  
          // insert the new note into the chord                   
          if(arpChordLength < ARP_MAX_CHORD-1)
          {        
            arpChord[arpChordLength] = ARP_MAKE_NOTE(note,velocity);
            arpChord[arpChordLength] |= ARP_NOTE_HELD;
            arpChordLength++;
            arpNotesHeld++;
  
            // flag that the arp sequence needs to be rebuilt
            arpFlags |= ARP_FLAG_REBUILD;
          }  
        }
      }
    }
    // NOTE OFF MESSAGE
    // (NB might be note on with zero velocity)
    else if(MIDI_IS_NOTE_ON(msg) || MIDI_IS_NOTE_OFF(msg))
    {
      // unflag the note as "held" and count how many notes of
      // the chord are actually still held
      noteIndexInChord = -1;
      arpNotesHeld = 0;
      for(i=0; i<arpChordLength; ++i)
      {
        // did we find the released key in the chord?
        if(ARP_GET_NOTE(arpChord[i]) == note)
        {
          arpChord[i] &= ~ARP_NOTE_HELD;
          noteIndexInChord = i;
        }
        else if(arpChord[i] & ARP_NOTE_HELD)
        {
          arpNotesHeld++;
        }
      }

      // should the note be removed from the chord?
      if(!(uiHoldType & UI_HOLD_CHORD) && noteIndexInChord >= 0)
      {     

        // shift higher notes down one position
        // to remove the released note
        for(i = noteIndexInChord;i < arpChordLength-1; ++i)
          arpChord[i] = arpChord[i+1];

        // rebuild the sequence
        --arpChordLength;
        arpFlags |= ARP_FLAG_REBUILD;
      }
    }
  }

  // check if the hold switch is released while
  // there are notes being held
  if(!(uiHoldType & UI_HOLD_CHORD) && !arpNotesHeld && arpChordLength)
    arpClear();
}

void arpSetManualChord()
{
  arpChordLength = 0;
  arpNotesHeld = 0;  
  unsigned int m=1;
  for(int i=0; i<16; ++i) {
     if((arpManualChord & m) && (arpChordLength < ARP_MAX_CHORD-1))
     {        
       arpChord[arpChordLength] = ARP_MAKE_NOTE(48+i,127);
       arpChordLength++;
     }
     m<<=1;
  }
  arpFlags |= ARP_FLAG_REBUILD;
}

////////////////////////////////////////////////////////////////////////////////
// RUN ARPEGGIATOR
void arpRun(unsigned long milliseconds)
{  
  byte noteSet[16] = {0};
  
  // update the chord based on user input
  arpReadInput(milliseconds);

  // see if user has changed a setting that would mean the
  // sequence needs to be rebuilt
  if(arpFlags & ARP_FLAG_REBUILD)
  {
    // rebuild the sequence 
    arpBuildSequence();                
    arpFlags &= ~ARP_FLAG_REBUILD;

    // ensure that the receiving device gets the
    // initial note/channel on status for the arpeggio
    midiClearRunningStatus();
  }

  // have we updated the play position?
  if(!!(synchFlags & SYNCH_PLAY_ADVANCE) && arpSequenceLength && arpPatternLength)
  {                 
    // get the index into the pattern
    arpPatternIndex = synchPlayIndex % arpPatternLength;
    
    // Check if we need to transpose
    if(!arpPatternIndex && arpTransposeSequenceLen) {
      arpTranspose = arpTransposeSequence[arpTransposeSequencePos];
      arpBuildSequence();                
      midiClearRunningStatus();
      if(++arpTransposeSequencePos >= arpTransposeSequenceLen) {
        arpTransposeSequencePos = 0;
      }      
      editFlags |= EDIT_FLAG_FORCE_REFRESH;
    }
 
    // check there is a note (not a rest at this) point in the pattern
    if((arpPattern[arpPatternIndex] & ARP_PATN_PLAY) || (arpOptions & ARP_OPT_SKIPONREST))
    {
      byte glide = 0;
      byte newNote = 0;
      if(arpPattern[arpPatternIndex] & ARP_PATN_TIE)
        glide = 2;
      else if(arpPattern[arpPatternIndex] & ARP_PATN_GLIDE)
        glide = 1;
        
      // Keep the sequence index within range      
      if(arpSequenceIndex >= arpSequenceLength)
        arpSequenceIndex = 0;
      
      // Loop to action play-through flag
      byte playThru = (arpPattern[arpPatternIndex] & ARP_PATN_PLAYTHRU)? 2:0;      
      do {      
        if(playThru == 2) {
          playThru = 1;
        }
        else {
          // check play thru flag
          playThru = !!(arpSequence[arpSequenceIndex] & ARP_PLAY_THRU);
        }

        
        // Play the note if applicable
        if(arpPattern[arpPatternIndex] & ARP_PATN_PLAY)
        {
          byte note = ARP_GET_NOTE(arpSequence[arpSequenceIndex]);
          if((arpPattern[arpPatternIndex] & ARP_PATN_OCTDN) && (note > 12))
            note-=12;
          else if((arpPattern[arpPatternIndex] & ARP_PATN_OCTAVE) && (note <= 115))
            note+=12;
          else if((arpPattern[arpPatternIndex] & ARP_PATN_4TH) && (note >= 5))
            note-=5;
          // determine note velocity
          byte velocity;          
          if(arpFlags & ARP_FLAG_MUTE)
            velocity = 0;
          else if(arpPattern[arpPatternIndex] & ARP_PATN_ACCENT)
            velocity = 127;
          else
            velocity = arpVelocityMode? arpVelocity : ARP_GET_VELOCITY(arpSequence[arpSequenceIndex]);        
    
          // start the note playing
          if(note > 0)
          {
            arpStartNote(note, velocity, milliseconds, noteSet);
            newNote = 1;
          }
        }
        
        // next note
        ++arpSequenceIndex;
      } while(playThru && arpSequenceIndex < arpSequenceLength);

      // if the previous note is still playing when a new one is played
      // then stop it (should be the case only for "tie" mode)
      if(newNote)
      {
        arpStopNotes(milliseconds, noteSet);
      }

      // check if we need to "glide"
      if(glide == 2) 
      {
          // tie          
          arpStopNoteTime = 0;
      }
      else if(glide == 1) 
      {
         // full step
         arpStopNoteTime = milliseconds + synchStepPeriod;
      }
      else if(arpGateLength)
      {              
        // work out the gate length for this note
        arpStopNoteTime = milliseconds + (synchStepPeriod * arpGateLength) / 150;
      }
      else
      {
        // note till play till the next one starts
        arpStopNoteTime = 0;               
      }
      arpLastPlayAdvance = milliseconds;
    }

    // need to update the arp display
    arpRefresh = 1;
    synchFlags &= ~SYNCH_PLAY_ADVANCE;
  }
  // check if a note needs to be stopped.. either at end of playing or if there is no sequence
  // and we're in tied note mode
  else if((arpStopNoteTime && arpStopNoteTime < milliseconds) || 
    (!arpStopNoteTime && !arpSequenceLength))
  {
    // stop the ringing notes
    arpStopNotes(milliseconds, NULL);
    arpStopNoteTime = 0;
  }
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//
//
//
// PREFERENCES
//
//
//
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// LOAD USER PREFS
void prefsInit()
{
  gPreferences = eepromGet(EEPROM_PREFS1); 
  gPreferences<<=8;
  gPreferences |= eepromGet(EEPROM_PREFS0); 
  prefsApply();  
}

////////////////////////////////////////////////////////////////////////////////
// SAVE NEW PRFERENCES TO EEPROM
void prefsSave()
{
  eepromSet(EEPROM_PREFS0,(gPreferences&0xFF));  
  eepromSet(EEPROM_PREFS1,((gPreferences>>8)&0xFF));  
}

////////////////////////////////////////////////////////////////////////////////
// APPLY PREFERENCES BITS TO VARIABLES
void prefsApply()
{
  switch(gPreferences & PREF_LEDPROFILE)
  {
    case PREF_LEDPROFILE0:
      uiLedBright = UI_LEDPROFILE0_HI;
      uiLedMedium = UI_LEDPROFILE0_MED;
      uiLedDim    = UI_LEDPROFILE0_LO;
      break;
    case PREF_LEDPROFILE1:
      uiLedBright = UI_LEDPROFILE1_HI;
      uiLedMedium = UI_LEDPROFILE1_MED;
      uiLedDim    = UI_LEDPROFILE1_LO;
      break;
    case PREF_LEDPROFILE2:
      uiLedBright = UI_LEDPROFILE2_HI;
      uiLedMedium = UI_LEDPROFILE2_MED;
      uiLedDim    = UI_LEDPROFILE2_LO;
      break;
    case PREF_LEDPROFILE3:
      uiLedBright = UI_LEDPROFILE3_HI;
      uiLedMedium = UI_LEDPROFILE3_MED;
      uiLedDim    = UI_LEDPROFILE3_LO;
      break;
  }
  
  switch(gPreferences & PREF_LONGPRESS)
  {
    case PREF_LONGPRESS0:
      uiLongHoldTime = UI_HOLD_TIME_0;
      break;
    case PREF_LONGPRESS1:
      uiLongHoldTime = UI_HOLD_TIME_1;
      break;
    case PREF_LONGPRESS2:
      uiLongHoldTime = UI_HOLD_TIME_2;
      break;
    case PREF_LONGPRESS3:
      uiLongHoldTime = UI_HOLD_TIME_3;
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//
//
//
// CONTROL SURFACE
//
//
//
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


// map the edit keys to physical buttons
//
// PATN  LEN   MODE
// SHFT  SPAN  RATE
// VELO  GATE  INS
// TMPO  CHAN  TRAN
enum {
  EDIT_MODE_PATTERN          = UI_KEY_A1,
  EDIT_MODE_PATTERN_LENGTH   = UI_KEY_B1,
  EDIT_MODE_ARP_TYPE         = UI_KEY_C1,
  EDIT_MODE_OCTAVE_SHIFT     = UI_KEY_A2,
  EDIT_MODE_OCTAVE_SPAN      = UI_KEY_B2,
  EDIT_MODE_RATE             = UI_KEY_C2,
  EDIT_MODE_VELOCITY         = UI_KEY_A3,
  EDIT_MODE_GATE_LENGTH      = UI_KEY_B3,
  EDIT_MODE_INSERT           = UI_KEY_C3,
  EDIT_MODE_TEMPO_SYNCH      = UI_KEY_A4,
  EDIT_MODE_CHANNEL          = UI_KEY_B4,
  EDIT_MODE_TRANSPOSE        = UI_KEY_C4
};

// Time in ms when we go back to pattern
// edit mode after last button press
#define EDIT_REVERT_TIME 10000

enum {
  EDIT_NO_PRESS = 0,
  EDIT_PRESS,        // button pressed
  EDIT_LONG_PRESS,   // button held for long hold threshold
  EDIT_LONG_HOLD,    // button held for more than long hold threshold
  EDIT_LONG_RELEASED // was a long hold, now released
};

// current editing mode
byte editMode;

// track the revert time
unsigned long editRevertTime;

// track when a menu button is held for a long period of time
unsigned long editLongHoldTime;
byte editPressType;
byte editFlags;
unsigned long editTapTempoTime;

////////////////////////////////////////////////////////////////////////////////
// INIT EDITING
void editInit()
{
  editMode = EDIT_MODE_PATTERN;
  editPressType = EDIT_NO_PRESS;
  editFlags = EDIT_FLAG_FORCE_REFRESH;
  editRevertTime = 0;
  editLongHoldTime = 0;
  editTapTempoTime = 0;
}

////////////////////////////////////////////////////////////////////////////////
// EDIT PATTERN
void editPattern(char keyPress, byte forceRefresh)
{
  if(keyPress != NO_VALUE)
  {
    arpPattern[keyPress] = (arpPattern[keyPress] ^ ARP_PATN_PLAY);
    forceRefresh = 1;
  }

  if(forceRefresh || arpRefresh)
  {    
    // copy the leds
    for(int i=0; i<16; ++i)
      uiLeds[i] = (arpPattern[i] & ARP_PATN_PLAY) ? uiLedMedium : 0;

    // only display the play position if we have a sequence
    if(arpSequenceLength) {
        uiLeds[arpPatternIndex] = uiLedBright;
    }

    // reset the flag
    arpRefresh = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////
// EDIT PATTERN EXTENDED
void editPatternExt(char keyPress, byte forceRefresh)
{
  byte extBit = ARP_PATN_ACCENT;
  if(arpOptions & ARP_LAYER_ACCENT) {
    extBit = ARP_PATN_ACCENT;
  }
  else if(arpOptions & ARP_LAYER_GLIDE) {
    extBit = ARP_PATN_GLIDE;
  }
  else if(arpOptions & ARP_LAYER_TIE) {
    extBit = ARP_PATN_TIE;
  }
  else if(arpOptions & ARP_LAYER_OCTUP) {
    extBit = ARP_PATN_OCTAVE;
  }
  else if(arpOptions & ARP_LAYER_OCTDN) {
    extBit = ARP_PATN_OCTDN;
  }
  else if(arpOptions & ARP_LAYER_4THDN) {
    extBit = ARP_PATN_4TH;
  }
  else if(arpOptions & ARP_LAYER_PLAYTHRU) {
    extBit = ARP_PATN_PLAYTHRU; 
  }
    
  if(keyPress != NO_VALUE)
  {
    arpPattern[keyPress] = (arpPattern[keyPress] ^ extBit);
    forceRefresh = 1;
  }

  if(forceRefresh || arpRefresh)
  {    
    // copy the leds
    for(int i=0; i<16; ++i) {
      uiLeds[i] = (arpPattern[i] & extBit) ? uiLedMedium : (arpPattern[i] & ARP_PATN_PLAY) ? uiLedDim : 0;
    }

    // only display the play position if we have a sequence
    if(arpSequenceLength)    
      uiLeds[arpPatternIndex] = uiLedBright;

    // reset the flag
    arpRefresh = 0;
  }
}

/////////////////////////////////////////////////////
// EDIT PATTERN LENGTH
void editPatternLength(char keyPress, byte forceRefresh)
{
  int i;
  if(keyPress >= 0 && keyPress <= 15)
  {
    arpPatternLength = keyPress + 1;
    forceRefresh = 1;
  }

  if(forceRefresh)
  {    
    uiClearLeds();
    uiSetLeds(0, arpPatternLength, uiLedDim);
    uiLeds[arpPatternLength-1] = uiLedBright;
  }
}

/////////////////////////////////////////////////////
// EDIT ARPEGGIO TYPE
void editArpType(char keyPress, byte forceRefresh)
{
  int i;
  switch(keyPress)
  {
  case 0: 
  case 1:  
  case 2:  
  case 3: 
  case 4:
  case 5:
    arpType = keyPress;
    arpFlags |= ARP_FLAG_REBUILD;
    forceRefresh = 1;
    break;
  case 12: 
    arpPatternLength = 8+random(8);
    for(i = 0;i<16;++i) arpPattern[i] = random(256);
    break;
  case 13: 
    arpPatternLength = 8+random(8);
    for(i = 0;i<16;++i) arpPattern[i] = random(2)? ARP_PATN_PLAY:0;
    break;
  case 14:
    for(i = 0;i<16;++i) arpPattern[i] = 0;
    arpPatternLength = 16;
    break;
  case 15:
    for(i = 0;i<16;++i) arpPattern[i] = ARP_PATN_PLAY;
    arpPatternLength = 16;
    break;
  } 

  if(forceRefresh)
  {
    uiClearLeds();
    uiSetLeds(0, 6, uiLedMedium);
    uiLeds[arpType] = uiLedBright;
    uiSetLeds(12, 4, uiLedMedium);
  }
}

/////////////////////////////////////////////////////
// EDIT ARP OPTIONS
void editArpOptions(char keyPress, byte forceRefresh)
{
  int i;
  if(keyPress >= 0 && keyPress < 16) {
    unsigned int b = (1<<(15-keyPress));
    if(b & (ARP_LAYER_MASK|ARP_OPTS_MASK))
    {
      if(b & ARP_LAYER_MASK) {
        arpOptions &= ~ARP_LAYER_MASK;
        arpOptions |= b;
      }
      else {
        arpOptions^=b;
        arpOptionsSave();
        arpOptionsApply();
      }
      forceRefresh = 1;
    } 
  }

  if(forceRefresh)
  {
    uiSetLeds(ARP_OPTS_MASK|ARP_LAYER_MASK, arpOptions);
  }
}

/////////////////////////////////////////////////////
// EDIT PREFERENCES
void editPreferences(char keyPress, byte forceRefresh)
{
  int i;
  if(keyPress >= 0 && keyPress < 16) {
    unsigned int b = (1<<(15-keyPress));
    if(PREF_MASK & b)
    {
      gPreferences^=b;
      prefsSave();
      prefsApply();
      forceRefresh = 1;
    } 
  }

  if(forceRefresh)
  {
    uiSetLeds(PREF_MASK, gPreferences);
  }
}

/////////////////////////////////////////////////////
// EDIT OCTAVE SHIFT
void editOctaveShift(char keyPress, byte forceRefresh)
{
  if(keyPress >= 0 && keyPress <= 6)
  {
    arpOctaveShift = keyPress - 3;
    arpFlags |= ARP_FLAG_REBUILD;
    forceRefresh = 1;
  }

  if(forceRefresh)
  {
    uiClearLeds();
    uiSetLeds(0, 7, uiLedDim);
    uiLeds[3] = uiLedMedium;
    uiLeds[3 + arpOctaveShift] = uiLedBright;
  }
}

/////////////////////////////////////////////////////
// EDIT OCTAVE SPAN
void editOctaveSpan(char keyPress, byte forceRefresh)
{
  if(keyPress >= 0 && keyPress <= 3)
  {
    arpOctaveSpan = keyPress + 1;
    arpFlags |= ARP_FLAG_REBUILD;
    forceRefresh = 1;
  }

  if(forceRefresh)
  {
    uiClearLeds();
    uiSetLeds(0, 4, uiLedDim);
    uiLeds[arpOctaveSpan - 1] = uiLedBright;
  }
}

/////////////////////////////////////////////////////
// EDIT ARP RATE
void editRate(char keyPress, byte forceRefresh)
{
  byte rates[] = {
    SYNCH_RATE_1,
    SYNCH_RATE_2D,
    SYNCH_RATE_2,
    SYNCH_RATE_4D,
    SYNCH_RATE_2T,
    SYNCH_RATE_4,
    SYNCH_RATE_8D,
    SYNCH_RATE_4T,
    SYNCH_RATE_8,
    SYNCH_RATE_16D,
    SYNCH_RATE_8T,
    SYNCH_RATE_16,
    SYNCH_RATE_16T,
    SYNCH_RATE_32
  };    

  if(keyPress >= 0 && keyPress < 14)
  {
    synchPlayRate = rates[keyPress];
    forceRefresh = 1;
  }

  if(forceRefresh)
  {
    uiClearLeds();
    uiSetLeds(0, 13, uiLedDim);
    uiLeds[0] = uiLedMedium;
    uiLeds[2] = uiLedMedium;
    uiLeds[5] = uiLedMedium;
    uiLeds[8] = uiLedMedium;
    uiLeds[11] = uiLedMedium;
    uiLeds[13] = uiLedMedium;
    for(int i=0; i<14; ++i)
    {
      if(synchPlayRate == rates[i]) 
      {
        uiLeds[i] = uiLedBright;
        break;
      }
    }
  }
}


/////////////////////////////////////////////////////
// EDIT VELOCITY
void editVelocity(char keyPress, byte forceRefresh)
{
  byte vel[16] = {0,9,17,26,34,43,51,60,68,77,85,94,102,111,119,127};    
  if(keyPress == 0 && !arpVelocity && arpVelocityMode)
  {    
    arpVelocityMode = 0;
    forceRefresh = 1;
  }
  else if(keyPress >= 0 && keyPress <= 15)
  {    
    arpVelocity = vel[keyPress];
    arpVelocityMode = 1;
    forceRefresh = 1;
  }

  if(forceRefresh)
  {
    uiClearLeds();
    if(!arpVelocityMode) 
    {
      // original velocity
      uiLeds[0] = uiLedBright;
      uiLeds[15] = uiLedBright;
    }
    else
    {
      for(int i=0; i<16; ++i)
      {
        if(arpVelocity <= vel[i]) {
          uiLeds[i] = uiLedBright;
          break;
        }
        uiLeds[i] = uiLedMedium;
      }
    }
  }
}

/////////////////////////////////////////////////////
// EDIT GATE LENGTH
void editGateLength(char keyPress, byte forceRefresh)
{
  if(keyPress >= 0 && keyPress <= 14)
  {    
    arpGateLength = 10*(keyPress + 1);
    forceRefresh = 1;
  }
  else if(keyPress == 15)
  {
    arpGateLength = 0;
    forceRefresh = 1;
  }
  if(forceRefresh)
  {
    uiClearLeds();
    if(arpGateLength > 0)
    {      
      uiSetLeds(0, arpGateLength/10, uiLedMedium);
      uiLeds[arpGateLength/10 - 1] = uiLedBright;
    }
    else
    {
      uiSetLeds(0, 16, uiLedMedium);
      uiLeds[15] = uiLedBright;
    }
  }    
}

/////////////////////////////////////////////////////
// EDIT MIDI OPTIONS
void editMidiOptions(char keyPress, byte forceRefresh)
{
  if(0 == keyPress)
  {    
    midiOptions ^= MIDI_OPTS_SEND_CHMSG;
    eepromSet(EEPROM_MIDI_OPTS, midiOptions);    
    forceRefresh = 1;
  }
  else if(1 == keyPress)
  {    
    midiOptions ^= MIDI_OPTS_PASS_INPUT_NOTES;
    eepromSet(EEPROM_MIDI_OPTS, midiOptions);    
    forceRefresh = 1;
  }
  else if(2 == keyPress)
  {    
    midiOptions ^= MIDI_OPTS_PASS_INPUT_CHMSG;
    eepromSet(EEPROM_MIDI_OPTS, midiOptions);    
    forceRefresh = 1;
  }
  else if(3 == keyPress)
  {    
    midiOptions ^= MIDI_OPTS_SYNCH_INPUT;
    eepromSet(EEPROM_MIDI_OPTS, midiOptions);    
    forceRefresh = 1;
  }
  else if(4 == keyPress)
  {    
    midiOptions ^= MIDI_OPTS_SYNCH_AUX;
    eepromSet(EEPROM_MIDI_OPTS, midiOptions);    
    forceRefresh = 1;
  }
  else if(5 == keyPress)
  {    
    midiOptions ^= MIDI_OPTS_FILTER_CHMODE;
    eepromSet(EEPROM_MIDI_OPTS, midiOptions);    
    forceRefresh = 1;
  }
  else if(6 == keyPress)
  {    
    midiOptions ^= MIDI_OPTS_VOLCAFM_VEL;
    eepromSet(EEPROM_MIDI_OPTS, midiOptions);    
    forceRefresh = 1;
  }
  if(forceRefresh)
  {
    uiClearLeds();
    uiLeds[0] = !!(midiOptions&MIDI_OPTS_SEND_CHMSG)? uiLedBright : uiLedDim;
    uiLeds[1] = !!(midiOptions&MIDI_OPTS_PASS_INPUT_NOTES)? uiLedBright : uiLedDim;
    uiLeds[2] = !!(midiOptions&MIDI_OPTS_PASS_INPUT_CHMSG)? uiLedBright : uiLedDim;
    uiLeds[3] = !!(midiOptions&MIDI_OPTS_SYNCH_INPUT)? uiLedBright : uiLedDim;
    uiLeds[4] = !!(midiOptions&MIDI_OPTS_SYNCH_AUX)? uiLedBright : uiLedDim;
    uiLeds[5] = !!(midiOptions&MIDI_OPTS_FILTER_CHMODE)? uiLedBright : uiLedDim;    
    uiLeds[6] = !!(midiOptions&MIDI_OPTS_VOLCAFM_VEL)? uiLedBright : uiLedDim;    
  }    
}

/////////////////////////////////////////////////////
// EDIT NOTE INSERT MODE
void editInsertMode(char keyPress, byte forceRefresh)
{
  int i,j,note,newChord=0;
  switch(keyPress)
  {
  case 0: 
  case 1: 
  case 2: 
  case 3: 
  case 4:  
    arpInsertMode = keyPress;
    arpFlags |= ARP_FLAG_REBUILD;
    forceRefresh = 1;
    break;
  case 10:
    arpChordLength=2+random(3);
    for(i=0; i<arpChordLength; ++i)
    {
      for(;;)
      {
        note = 48+random(12); 
        for(j = 0; j<i; ++j)
        {
          if(ARP_GET_NOTE(arpChord[j]) == note)
            break;
        }
        if(j>=i)
          break;
      }           
      arpChord[i] = ARP_MAKE_NOTE(note,64+random(64));
    }
    newChord = 1;
    break;
  case 11: // MIN7
    arpChord[0] = ARP_MAKE_NOTE(48,127);       
    arpChord[1] = ARP_MAKE_NOTE(51,127); 
    arpChord[2] = ARP_MAKE_NOTE(55,127);
    arpChord[3] = ARP_MAKE_NOTE(58,127);
    arpChordLength = 4;
    newChord = 1;
    break;
  case 12: // MAJ7
    arpChord[0] = ARP_MAKE_NOTE(48,127);       
    arpChord[1] = ARP_MAKE_NOTE(52,127); 
    arpChord[2] = ARP_MAKE_NOTE(55,127);
    arpChord[3] = ARP_MAKE_NOTE(59,127);
    arpChordLength = 4;
    newChord = 1;
    break;
  case 13: // DOM7
    arpChord[0] = ARP_MAKE_NOTE(48,127);       
    arpChord[1] = ARP_MAKE_NOTE(52,127); 
    arpChord[2] = ARP_MAKE_NOTE(55,127);
    arpChord[3] = ARP_MAKE_NOTE(58,127);
    arpChordLength = 4;
    newChord = 1;
    break;
  case 14: // MIN
    arpChord[0] = ARP_MAKE_NOTE(48,127);       
    arpChord[1] = ARP_MAKE_NOTE(51,127);       
    arpChord[2] = ARP_MAKE_NOTE(55,127);       
    arpChordLength = 3;
    newChord = 1;
    break;
  case 15: // MAJ
    arpChord[0] = ARP_MAKE_NOTE(48,127);       
    arpChord[1] = ARP_MAKE_NOTE(52,127);       
    arpChord[2] = ARP_MAKE_NOTE(55,127);       
    arpChordLength = 3;
    newChord = 1;
    break;
  }

  if(newChord) {
    arpManualChord = 0;
    arpFlags |= ARP_FLAG_REBUILD;
    uiHoldType |= UI_HOLD_CHORD;
  }
  if(forceRefresh)
  {
    uiClearLeds();
    uiSetLeds(0, 5, uiLedDim);
    uiLeds[arpInsertMode] = uiLedBright;
    uiSetLeds(10, 6, uiLedMedium);
  }
}

/////////////////////////////////////////////////////
// MANUAL CHORD INPUT
void editManualChord(char keyPress, byte forceRefresh) 
{
  
  if(keyPress >= 0 && keyPress < 16) {
    arpManualChord ^= (1<<keyPress);
    arpSetManualChord();
    forceRefresh = 1;
    uiHoldType |= UI_HOLD_CHORD;
  }
  
  if(forceRefresh)
  {
    unsigned int m = 1;
    for(int i=0; i<16; ++i) {
      switch(i) {
        case 0: case 2: case 4: case 5: case 7: case 9: case 11: case 12: case 14:
          uiLeds[i] = (arpManualChord & m) ? uiLedBright:uiLedMedium;
          break;
        default:
          uiLeds[i] = (arpManualChord & m) ? uiLedBright:uiLedDim;
          break;
      }
      m<<=1;
    }      
  }  
}

/////////////////////////////////////////////////////
// EDIT SYNCH MODE AND TEMPO
void editTempoSynch(char keyPress, byte forceRefresh)
{
  switch(keyPress)
  {
  case 0: // Toggle MIDI synch
     synchFlags ^= SYNCH_TO_MIDI;
    if(!(synchFlags & SYNCH_TO_MIDI)) 
      synchResynch();
    eepromSet(EEPROM_SYNCH_SOURCE, !!(synchFlags & SYNCH_TO_MIDI));    
    forceRefresh = 1;
    break;
  case 1: // Toggle MIDI clock send
    if(synchFlags & SYNCH_SEND_CLOCK)
    {
      synchFlags |= SYNCH_SEND_STOP;
      synchFlags &= ~SYNCH_SEND_CLOCK;
    }
    else
    {
      synchFlags |= SYNCH_SEND_START;    // a start message will be sent to slaves
      synchFlags |= SYNCH_RESET_NEXT_STEP_TIME; // need to reset internal counter to ensure synch with slave
      synchFlags |= SYNCH_SEND_START;
      synchFlags |= SYNCH_SEND_CLOCK;
      synchRestartSequence();
    }
    eepromSet(EEPROM_SYNCH_SEND, !!(synchFlags & SYNCH_SEND_CLOCK));    
    forceRefresh = 1;
    break;
  case 3: 
  case 4: 
  case 5: 
  case 6: 
  case 7: 
  case 8: 
  case 9: 
  case 10: 
  case 11: 
    synchSetTempo(keyPress * 20);
    forceRefresh = 1;
    break;

  case 13: 
    if(synchFlags & SYNCH_TO_MIDI) { // Force stop in ext sync
        synchStop(SYNCH_SOURCE_MANUAL);
    }
    else// Tap tempo
    {
      unsigned long ms = millis();
      if(ms > editTapTempoTime && ms - editTapTempoTime < 1000)
      {
        synchSetTempo(60000L/(ms-editTapTempoTime));
        forceRefresh = 1;            
      }
      editTapTempoTime = ms;
    }
    break;
  case 14: 
    if(synchFlags & SYNCH_TO_MIDI) { // Force start in ext sync
        synchStart(SYNCH_SOURCE_MANUAL);
    }
    else if(synchBPM > 20)// Manual tempo dec
    {
      synchSetTempo(synchBPM-1);
      forceRefresh = 1;
    }
    break;
  case 15: 
    if(synchFlags & SYNCH_TO_MIDI) { // Force continue in ext sync
        synchContinue(SYNCH_SOURCE_MANUAL);
    }
    else if(synchBPM < 300)// Manual tempo inc
    {
      synchSetTempo(synchBPM+1);
      forceRefresh = 1;
    }
    break;
  }

  if(forceRefresh)
  {
    uiSetLeds(0,16,0);      
    if(synchFlags & SYNCH_TO_MIDI)
    {
      uiLeds[0] = uiLedBright;
      uiLeds[1] = !!(synchFlags & SYNCH_TO_MIDI) ? uiLedBright : uiLedMedium;                                                                                                                                                                                                                                                            
      
      uiLeds[13] = uiLedMedium;                                                                                                                                                                                                                                                            
      uiLeds[14] = uiLedMedium;                                                                                                                                                                                                                                                            
      uiLeds[15] = uiLedMedium;                                                                                                                                                                                                                                                            
      
    }
    else
    {    
#define BPM_METER(x,v) \
      ((abs(v-x) <= 4)? uiLedBright : \
      ((abs(v-x) <= 11)? uiLedMedium : \
      ((abs(v-x) <= 19)? uiLedDim : 0)))

        uiLeds[0] = uiLedMedium;
      uiLeds[1] = !!(synchFlags & SYNCH_SEND_CLOCK) ? uiLedBright : uiLedMedium;
      if(synchBPM <= 40) 
        uiLeds[3] = uiLedDim;
      else
        uiLeds[3] = BPM_METER(synchBPM,60);      
      uiLeds[4] = BPM_METER(synchBPM,80);
      uiLeds[5] = BPM_METER(synchBPM,100);
      uiLeds[6] = BPM_METER(synchBPM,120);
      uiLeds[7] = BPM_METER(synchBPM,140);
      uiLeds[8] = BPM_METER(synchBPM,160);
      uiLeds[9] = BPM_METER(synchBPM,180);
      uiLeds[10] = BPM_METER(synchBPM,200);
      if(synchBPM >= 240) 
        uiLeds[11] = uiLedDim;
      else
        uiLeds[11] = BPM_METER(synchBPM,220);
      uiLeds[13] = uiLedBright;
      uiLeds[14] = uiLedBright;
      uiLeds[15] = uiLedBright;      
    }
  } 
}

/////////////////////////////////////////////////////
// EDIT MIDI OUTPUT CHANNEL
void editMidiOutputChannel(char keyPress, byte forceRefresh)
{
  if(keyPress >= 0 && keyPress <= 15)
  {
    if(midiSendChannel != keyPress)
      arpStopNotes(millis(), NULL);
    midiSendChannel = keyPress;
    eepromSet(EEPROM_OUTPUT_CHAN, midiSendChannel);
    forceRefresh = 1;
  }
  if(forceRefresh)
  {
    uiClearLeds();
    uiSetLeds(0, 16, uiLedDim);
    uiLeds[midiSendChannel] = uiLedBright;
  }
}

/////////////////////////////////////////////////////
// EDIT MIDI INPUT  CHANNEL
void editMidiInputChannel(char keyPress, byte forceRefresh)
{
  if(keyPress >= 0 && keyPress <= 15)
  {
    if(midiReceiveChannel == keyPress)
    {
      midiReceiveChannel = MIDI_OMNI;
      eepromSet(EEPROM_INPUT_CHAN, MIDI_OMNI);
    }
    else
    {
      midiReceiveChannel = keyPress;
      eepromSet(EEPROM_INPUT_CHAN, midiReceiveChannel);
    }
    forceRefresh = 1;
    arpClear();
  }
  if(forceRefresh)
  {
    uiClearLeds();
    if(MIDI_OMNI == midiReceiveChannel)
      uiSetLeds(0, 16, uiLedBright);
    else
      uiLeds[midiReceiveChannel] = uiLedBright;
  }
}

/////////////////////////////////////////////////////
// EDIT NOTE TRANSPOSE
void editTranspose(char keyPress, byte forceRefresh)
{  
  // 0123456789012345
  // DDDOXXXXXXXXXXXX        
  if(keyPress >= 0 && keyPress <= 15)
  {
    if(editFlags & EDIT_FLAG_IS_HELD)
    {
      if(editFlags & EDIT_FLAG_IS_NEW) 
      {
        arpTransposeSequencePos = 0;
        arpTransposeSequenceLen = 0;
        arpTransposeSequenceMask = 0;
        editFlags &= ~EDIT_FLAG_IS_NEW;
      }
      
      if(arpTransposeSequenceLen < ARP_MAX_TRAN_SEQ - 1)      
      {
        arpTransposeSequenceMask |= (1<<keyPress);
        arpTransposeSequence[arpTransposeSequenceLen++] = keyPress - 3;
      }
    }
    else     
    {
      // pressing a note button while the transpose menu
      // button is not held will clear transpose sequence
      arpTransposeSequencePos = 0;
      arpTransposeSequenceLen = 0;      
      arpTransposeSequenceMask = 0;
      arpTranspose = keyPress - 3;
      arpFlags |= ARP_FLAG_REBUILD;
    }   
    forceRefresh = 1;
  }

  if(forceRefresh)
  {
    
    uiSetLeds(0, 16, uiLedDim);

    arpTransposeSequenceMask |= (1<<3);
    unsigned int m=1;
    for(byte i=0; i<16; ++i) 
    {
      if(arpTransposeSequenceMask & m) 
      {
        uiLeds[i] = uiLedMedium;
      }
      m<<=1;
    }

    if(arpTranspose >= -3 && arpTranspose < 13)
      uiLeds[arpTranspose + 3] = uiLedBright;
  }
}

/////////////////////////////////////////////////////
// FORCE TO SCALE TYPE
void editForceToScaleType(char keyPress, byte forceRefresh)
{  
  if(keyPress >= 0)
  {
    switch(keyPress)
    {
      case 0: arpForceToScaleMask |= ARP_SCALE_CHROMATIC; break;
      case 1: arpForceToScaleMask &= ~ARP_SCALE_CHROMATIC; arpForceToScaleMask |= ARP_SCALE_IONIAN; break;
      case 2: arpForceToScaleMask &= ~ARP_SCALE_CHROMATIC; arpForceToScaleMask |= ARP_SCALE_DORIAN; break;
      case 3: arpForceToScaleMask &= ~ARP_SCALE_CHROMATIC; arpForceToScaleMask |= ARP_SCALE_PHRYGIAN; break;
      case 4: arpForceToScaleMask &= ~ARP_SCALE_CHROMATIC; arpForceToScaleMask |= ARP_SCALE_LYDIAN; break;
      case 5: arpForceToScaleMask &= ~ARP_SCALE_CHROMATIC; arpForceToScaleMask |= ARP_SCALE_MIXOLYDIAN; break;
      case 6: arpForceToScaleMask &= ~ARP_SCALE_CHROMATIC; arpForceToScaleMask |= ARP_SCALE_AEOLIAN; break;
      case 7: arpForceToScaleMask &= ~ARP_SCALE_CHROMATIC; arpForceToScaleMask |= ARP_SCALE_LOCRIAN; break;
      
      case 11: arpForceToScaleMask &= ~ARP_SCALE_ADJUST_MASK; arpForceToScaleMask |= ARP_SCALE_ADJUST_SKIP; break;
      case 12: arpForceToScaleMask &= ~ARP_SCALE_ADJUST_MASK; arpForceToScaleMask |= ARP_SCALE_ADJUST_MUTE; break;
      case 13: arpForceToScaleMask &= ~ARP_SCALE_ADJUST_MASK; arpForceToScaleMask |= ARP_SCALE_ADJUST_FLAT; break;
      case 14: arpForceToScaleMask &= ~ARP_SCALE_ADJUST_MASK; arpForceToScaleMask |= ARP_SCALE_ADJUST_SHARP; break;
      case 15: arpForceToScaleMask &= ~ARP_SCALE_ADJUST_MASK; arpForceToScaleMask |= ARP_SCALE_ADJUST_TOGGLE; break;
    }
    arpFlags |= ARP_FLAG_REBUILD;
    forceRefresh = 1;
  }
  
  if(forceRefresh)
  {
    uiClearLeds();
    uiSetLeds(0, 8, uiLedDim);
    uiSetLeds(11, 5, uiLedDim);
    uiLeds[0] = uiLedMedium;
    uiLeds[14] = uiLedMedium;
    switch(arpForceToScaleMask & ARP_SCALE_CHROMATIC)
    {
      case ARP_SCALE_CHROMATIC:  uiLeds[0] = uiLedBright; break;
      case ARP_SCALE_IONIAN:     uiLeds[1] = uiLedBright; break;
      case ARP_SCALE_DORIAN:     uiLeds[2] = uiLedBright; break;
      case ARP_SCALE_PHRYGIAN:   uiLeds[3] = uiLedBright; break;
      case ARP_SCALE_LYDIAN:     uiLeds[4] = uiLedBright; break;
      case ARP_SCALE_MIXOLYDIAN: uiLeds[5] = uiLedBright; break;
      case ARP_SCALE_AEOLIAN:    uiLeds[6] = uiLedBright; break;
      case ARP_SCALE_LOCRIAN:    uiLeds[7] = uiLedBright; break;
    }    
    switch(arpForceToScaleMask & ARP_SCALE_ADJUST_MASK)
    {
      case ARP_SCALE_ADJUST_SKIP: uiLeds[11] = uiLedBright; break;
      case ARP_SCALE_ADJUST_MUTE: uiLeds[12] = uiLedBright; break;
      case ARP_SCALE_ADJUST_FLAT: uiLeds[13] = uiLedBright; break;
      case ARP_SCALE_ADJUST_SHARP: uiLeds[14] = uiLedBright; break;
      case ARP_SCALE_ADJUST_TOGGLE: uiLeds[15] = uiLedBright; break;
    }    
  }
}

/////////////////////////////////////////////////////
// FORCE TO SCALE ROOT NOTE
void editForceToScaleRoot(char keyPress, byte forceRefresh)
{  
  // 0123456789012345
  // DDDOXXXXXXXXXXXX        
  if(keyPress >= 0 && keyPress < 12)
  {
    arpForceToScaleRoot = keyPress;
    arpFlags |= ARP_FLAG_REBUILD;
    forceRefresh = 1;
  }

  if(forceRefresh)
  {
    uiClearLeds();
    uiSetLeds(0, 12, uiLedDim);
    uiLeds[1] = uiLedMedium;
    uiLeds[3] = uiLedMedium;
    uiLeds[6] = uiLedMedium;
    uiLeds[8] = uiLedMedium;
    uiLeds[10] = uiLedMedium;
    uiLeds[arpForceToScaleRoot] = uiLedBright;
  }
}

/////////////////////////////////////////////////////
// EDIT RUN
void editRun(unsigned long milliseconds)
{
  byte forceRefresh = (editFlags & EDIT_FLAG_FORCE_REFRESH);
  editFlags &= ~EDIT_FLAG_FORCE_REFRESH;

  // Capture any key pressed on the data entry keypad
  char dataKeyPress = uiDataKey;
  if(dataKeyPress != NO_VALUE)
  {
    // reset the timeout period after which the 
    // display will revert to the pattern view
    uiDataKey = NO_VALUE;
    editRevertTime = milliseconds + EDIT_REVERT_TIME;
  }

  // Capture any key pressed on the data entry keypad  
  char menuKeyPress = uiMenuKey;
  if(NO_VALUE == menuKeyPress)
  {
    // There is no key pressed, but previously we had a long hold
    // so we remain "locked" until a key is pressed
    if(EDIT_LONG_HOLD == editPressType)
      editPressType = EDIT_LONG_RELEASED;
  }
  else
  {
    uiMenuKey = NO_VALUE;
    if(menuKeyPress != editMode || EDIT_LONG_RELEASED == editPressType)
    {
      // change to a new edit mode, so 
      // screen needs to be refreshed
      editMode = menuKeyPress;
      editPressType = EDIT_PRESS;
      editLongHoldTime = 0;
      forceRefresh = 1;
    }
    editRevertTime = milliseconds + EDIT_REVERT_TIME;
  }

  // is any menu key currently held?
  if(uiLastMenuKey != NO_VALUE)
  {
    editFlags |= EDIT_FLAG_IS_HELD;      
    
    // set a time at which the "long hold" event happens
    if(!editLongHoldTime)
    {
      editLongHoldTime = milliseconds + uiLongHoldTime;
    }
    else if(milliseconds > editLongHoldTime)
    {
      if(editPressType < EDIT_LONG_PRESS)
      {
        editPressType = EDIT_LONG_PRESS; 
        forceRefresh = 1;
      }
      else
      {
        editPressType = EDIT_LONG_HOLD;      
        editLongHoldTime = (unsigned long)(-1);
        forceRefresh = 1;
      }
    }
  }
  else
  {
    editFlags &= ~EDIT_FLAG_IS_HELD;      
    editFlags |= EDIT_FLAG_IS_NEW;      
    editLongHoldTime = 0;
  }

  // check if we timed out user input and should revert
  // to pattern edit mode  
  if(editRevertTime > 0 && editRevertTime < milliseconds)
  {
    // revert back to pattern edit mode
    if(gPreferences & PREF_AUTOREVERT)
    {
      editMode = EDIT_MODE_PATTERN;
      editPressType = EDIT_NO_PRESS; 
    }
    forceRefresh = 1;
    editRevertTime = 0;
  }

  // run the current edit mode
  switch(editMode)
  {
  case EDIT_MODE_PATTERN:
    if(editPressType >= EDIT_LONG_HOLD)
      editPatternExt(dataKeyPress, forceRefresh);
    else
      editPattern(dataKeyPress, forceRefresh);
    break;    
  case EDIT_MODE_PATTERN_LENGTH:
    if(editPressType >= EDIT_LONG_HOLD)
      editPreferences(dataKeyPress, forceRefresh);
    else
      editPatternLength(dataKeyPress, forceRefresh);
    break;    
  case EDIT_MODE_ARP_TYPE:
    if(editPressType >= EDIT_LONG_HOLD)
      editArpOptions(dataKeyPress, forceRefresh);
    else
      editArpType(dataKeyPress, forceRefresh);
    break;        
  case EDIT_MODE_OCTAVE_SHIFT:
    if(editPressType >= EDIT_LONG_HOLD)
      editForceToScaleRoot(dataKeyPress, forceRefresh);
    else
      editOctaveShift(dataKeyPress, forceRefresh);
    break;
  case EDIT_MODE_OCTAVE_SPAN:
    if(editPressType >= EDIT_LONG_HOLD)
      editForceToScaleType(dataKeyPress, forceRefresh);
    else
      editOctaveSpan(dataKeyPress, forceRefresh);
    break;
  case EDIT_MODE_RATE:
    if(editPressType == EDIT_LONG_PRESS)
    {
      arpClear();
      midiPanic();
    }
    else
      editRate(dataKeyPress, forceRefresh);
    break;
  case EDIT_MODE_VELOCITY:
    editVelocity(dataKeyPress, forceRefresh);
    break;    
  case EDIT_MODE_GATE_LENGTH:
    editGateLength(dataKeyPress, forceRefresh);
    break;    
  case EDIT_MODE_INSERT:
    if(editPressType >= EDIT_LONG_HOLD)
      editManualChord(dataKeyPress, forceRefresh);
    else
      editInsertMode(dataKeyPress, forceRefresh);
    break;    
  case EDIT_MODE_TEMPO_SYNCH:
    if(editPressType >= EDIT_LONG_HOLD)
      editMidiOptions(dataKeyPress, forceRefresh);
    else
      editTempoSynch(dataKeyPress, forceRefresh);
    break;    
  case EDIT_MODE_CHANNEL:
    if(editPressType >= EDIT_LONG_HOLD)
      editMidiInputChannel(dataKeyPress, forceRefresh);
    else
      editMidiOutputChannel(dataKeyPress, forceRefresh);
    break;    
  case EDIT_MODE_TRANSPOSE:
    editTranspose(dataKeyPress, forceRefresh);
    break;        
  default:
    editPattern(dataKeyPress, forceRefresh);
    break;   
  }    
}    

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//
//
//
// HACK HEADER IO
//
//
//
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Class to manage pot inputs
class CPot 
{
  int value;
  enum { 
    TOLERANCE = 4,
    UNKNOWN = -1,
  };
public:  
  enum {
    MAX_CC = 127,
    TRANSPOSE,
    TEMPO,
    VELOCITY,
    PITCHBEND,
    GATELEN
  };    
  CPot() {
    reset();
  }
  void reset() {
    value = UNKNOWN;
  }    
  int endStops(int v)
  {
      if(v < TOLERANCE) 
        return 0;
      else if(v > 1023 - TOLERANCE) 
        return 1023;
      return v;
  }
  int centreDetent(int v)
  {
    const int range = 480;
    const int top_of_low_band = range;
    const int bottom_of_hi_band = 1023 - range;
    if(v < top_of_low_band)
      v = 512.0 * ((float)v/range);
    else if(v > bottom_of_hi_band) 
      v = 512 + 512.0 * (v-bottom_of_hi_band)/(float)range;
    else 
      v = 512;
     return constrain(v,0,1023);
  }
  void run(int pin, byte controller, unsigned long milliseconds) {
    int reading = analogRead(pin);
    if(value == UNKNOWN) {
      value = reading;
    }
    else if(abs(reading - value) > TOLERANCE) 
    {
      value = reading;
      int v;
      switch(controller) {
        case TRANSPOSE:          
          arpTranspose = 12 * (float)(centreDetent(value)-512.0)/511.0;
          arpFlags |= ARP_FLAG_REBUILD;
          editFlags |= EDIT_FLAG_FORCE_REFRESH;
          break;
        case TEMPO:
          v = endStops(value);
          v = 30 + 250.0 * (v/1023.0);
          synchSetTempo(v);
          editFlags |= EDIT_FLAG_FORCE_REFRESH;
          break;
        case VELOCITY:
          v = endStops(value);
          arpVelocity = v/8;
          arpVelocityMode = 1;
          editFlags |= EDIT_FLAG_FORCE_REFRESH;
          break;
        case GATELEN:
          v = endStops(value);
          if(1023 == v)
            arpGateLength = 0;
          else if(v < 10)
            arpGateLength = 1;
          else
            arpGateLength = 10 + (v*150.0)/1023.0;
          editFlags |= EDIT_FLAG_FORCE_REFRESH;
          break;
        case PITCHBEND:
          v = 16 * centreDetent(value);
          midiWrite(MIDI_MK_PITCHBEND, v&0x7F, (v>>7)&0x7F, 2, milliseconds);          
          break;
        default:
          v = endStops(value);
          if(controller > 0 && controller <= MAX_CC)
              midiWrite(MIDI_MK_CTRL_CHANGE, controller, v>>3, 2, milliseconds);
          break;            
      }      
    }
  }   
};

// Define three pot instances
CPot Pot1;
CPot Pot2;
CPot Pot3;

byte hhTime;   // stores divided ms just so we can check for ticks

#define P_HH_POT_PC0 14
#define P_HH_POT_PC4 18
#define P_HH_POT_PC5 19
#define P_HH_SW_PB3  11

////////////////////////////////////////////////////////////////////////////////
// Initialise hack header manager
void hhInit() 
{
  if(IS_HH_POTS) {
    pinMode(P_HH_SW_PB3,INPUT_PULLUP);
    pinMode(P_HH_POT_PC5,INPUT);
    pinMode(P_HH_POT_PC4,INPUT);
    pinMode(P_HH_POT_PC0,INPUT);
  }
  Pot1.reset();
  Pot2.reset();
  Pot3.reset();
  hhTime = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Run hack header manager
void hhRun(unsigned long milliseconds)
{      
  // enforce a minimum period of 16ms between I/O polls
  if((byte)(milliseconds>>4) == hhTime)
    return;
  hhTime = (byte)(milliseconds>>4); 
  arpFlags &= ~ARP_FLAG_MUTE;
  synchFlags &= ~SYNCH_HOLD_AT_ZERO;

  if(IS_HH_POTS)
  {
      switch(gPreferences & PREF_HHPOT_PC5)
      {
      case PREF_HHPOT_PC5_MOD:
         Pot1.run(5, 1, milliseconds);
         break;
      case PREF_HHPOT_PC5_TRANS:
         Pot1.run(5, CPot::TRANSPOSE, milliseconds);
         break;
      case PREF_HHPOT_PC5_CC:
         Pot1.run(5, HH_CC_PC5, milliseconds);
         break;
      }
      switch(gPreferences & PREF_HHPOT_PC4)
      {
      case PREF_HHPOT_PC4_VEL:
         Pot2.run(4, CPot::VELOCITY, milliseconds);
         break;
      case PREF_HHPOT_PC4_PB:
         Pot2.run(4, CPot::PITCHBEND, milliseconds);
         break;
      case PREF_HHPOT_PC4_CC:
         Pot2.run(4, HH_CC_PC4, milliseconds);
         break;
      }
      switch(gPreferences & PREF_HHPOT_PC0)
      {
      case PREF_HHPOT_PC0_TEMPO:
         Pot3.run(0, CPot::TEMPO, milliseconds);
         break;
      case PREF_HHPOT_PC0_GATE:
         Pot3.run(0, CPot::GATELEN, milliseconds);
         break;
      case PREF_HHPOT_PC0_CC:
         Pot3.run(0, HH_CC_PC0, milliseconds);
         break;
      }

      if(!!(gPreferences & PREF_HHSW_PB3)) {        
         if(!digitalRead(P_HH_SW_PB3)) {
           synchFlags |= SYNCH_HOLD_AT_ZERO|SYNCH_RESTART_ON_BEAT|SYNCH_ZERO_TICK_COUNT;           
         }           
      }
      else {
         if(!digitalRead(P_HH_SW_PB3))
           arpFlags |= ARP_FLAG_MUTE;
      }
  }    
}

////////////////////////////////////////////////////////////////////////////////
//
//
//
// HEARTBEAT
//
//
//
////////////////////////////////////////////////////////////////////////////////
#define P_HEARTBEAT        13
#define HEARTBEAT_PERIOD 500
unsigned long heartbeatNext;
byte heartbeatStatus;

////////////////////////////////////////////////////////////////////////////////
// HEARTBEAT INIT
void heartbeatInit()
{
  pinMode(P_HEARTBEAT, OUTPUT);     
  heartbeatNext = 0;
  heartbeatStatus = 0;
}

////////////////////////////////////////////////////////////////////////////////
// HEARTBEAT RUN
void heartbeatRun(unsigned long milliseconds)
{
  if(milliseconds > heartbeatNext)
  {
    digitalWrite(P_HEARTBEAT, heartbeatStatus);
    heartbeatStatus = !heartbeatStatus;
    heartbeatNext = milliseconds + HEARTBEAT_PERIOD;
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//
// SETUP
//
//
////////////////////////////////////////////////////////////////////////////////
void setup() {                
 
  midiInit();
  arpInit();
  heartbeatInit();
  editInit(); 

  // initialise the basic UI last, since this will 
  // start up the refresh interrupt
  cli();
  synchInit();
  uiInit();     
  sei();  
  
  // init hack header
  hhInit();

  // Load user preferences  
  prefsInit();
  
  // pressing hold switch at startup shows UI version
  uiShowVersion();

  // reset default EEPROM settings
  if(uiMenuKey == UI_KEY_C1 || (eepromGet(EEPROM_MAGIC_COOKIE) != EEPROM_MAGIC_COOKIE_VALUE))
  {
    midiSendChannel = 0;
    midiReceiveChannel = MIDI_OMNI;
    midiOptions = MIDI_OPTS_DEFAULT_VALUE;
    synchFlags = 0;
    gPreferences = 
      PREF_AUTOREVERT | 
      PREF_LONGPRESS2 | 
      PREF_LEDPROFILE2;
    arpOptions = 
      ARP_OPT_SKIPONREST;
  
    eepromSet(EEPROM_OUTPUT_CHAN, midiSendChannel);
    eepromSet(EEPROM_INPUT_CHAN, midiReceiveChannel);
    eepromSet(EEPROM_MIDI_OPTS, MIDI_OPTS_DEFAULT_VALUE);
    eepromSet(EEPROM_SYNCH_SOURCE,0);
    eepromSet(EEPROM_SYNCH_SEND,0);  
    prefsSave();
    prefsApply();
    arpOptionsSave();
    arpOptionsApply();
    eepromSet(EEPROM_MAGIC_COOKIE,EEPROM_MAGIC_COOKIE_VALUE);  
    
    uiSetLeds(0, 16, uiLedBright);
    delay(1000);
    editFlags |= EDIT_FLAG_FORCE_REFRESH;
  }  
}

////////////////////////////////////////////////////////////////////////////////
//
//
// LOOP
//
//
////////////////////////////////////////////////////////////////////////////////
void loop() 
{
  unsigned long milliseconds = millis();
  synchRun(milliseconds);
  arpRun(milliseconds);
  heartbeatRun(milliseconds);
  uiRun(milliseconds);
  editRun(milliseconds);   
  hhRun(milliseconds);
}

//EOF

