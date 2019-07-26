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
//      hotchk155/2013-18 //
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
//    4.0   Feb2015  Release A4
//    5.0   Jun2017  Release A5
//    5.1   18Jul17  Hack header fix
//    5.2   11Mar18  Fix timing glitch on aux sync
//    5.3+  May18    BETA release with CVTab support
//
#define VERSION_HI  5
#define VERSION_LO  7

//
// INCLUDE FILES
//
#include <avr/interrupt.h>  
#include <avr/io.h>
#include <EEPROM.h>
#include <Wire.h>

// Midi CC numbers to associate with hack header inputs
#define HH_CC_PC0             16
#define HH_CC_PC4             17
#define HH_CC_PC5             18

// The controller number used for Velocity CC mode
#define VELOCITY_CC           41 // Korg Volca FM
#define LOCAL_ON_CC          122 


// Hack header pulse clock config
#define SYNCH_CLOCK_PULSE_WIDTH    10
#define SYNCH_CLOCK_MIN_PERIOD     20
#define SYNCH_CLOCK_INVERT         0

// I2C ADDRESSES FOR DAUGHTER BOARDS
#define DAC_ADDR    0b1100000
#define EEPROM_ADDR 0b1010000

//
// PREFERENCES WORD
//
enum {
  PREF_HACKHEADER=       (unsigned int)0b1111111100000000,

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
  
  PREF_HH_8THCLOCK=     (unsigned int)0b1000000000000000, // ext clock is 8th notes
  PREF_HH_CVTAB_ACC=    (unsigned int)0b0100000000000000, // cvtab clock out is accent
  PREF_HH_CVTAB_HZVOLT= (unsigned int)0b0010000000000000, // cvtab uses hz/volt scaling
  PREF_HH_CVCAL=        (unsigned int)0b0000000100000000, // cvtab is in calibration mode
      
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

// HACK HEADER MODES
enum {
  HH_MODE_NONE,
  HH_MODE_CTRLTAB,
  HH_MODE_SYNCTAB,
  HH_MODE_CVTAB,
  HH_MODE_MEMOTAB
};

enum {
  HH_GATE_CLOSE,
  HH_GATE_OPEN,
  HH_GATE_RETRIG
};
#define P_HH_CVTAB_GATE 14

#define HH_CVCAL_CC_SCALE   70
#define HH_CVCAL_CC_OFS     71
#define HH_CVCAL_CC_SET     72
#define HH_CVCAL_CC_SAVE    73

// The preferences word
unsigned int gPreferences;

// Hack header mode
byte hhMode;

// CV TAB calibration data
char hhCVCalScale;
char hhCVCalOfs;

// CV TAB pitch glide info
long hhDACCurrent;
long hhDACTarget;
long hhDACIncrement;
byte hhGlideActive;

// Hack header function prototypes
void hhSetCV(long note, byte glide);
void hhSetGate(byte state);
void hhCVCalSave();

// Forward declare the UI refresh flag
#define EDIT_FLAG_FORCE_REFRESH 0x01
#define EDIT_FLAG_IS_HELD       0x02    // means that a menu button is held down
#define EDIT_FLAG_IS_NEW        0x04    
#define EDIT_FLAG_1             0x10    // these flags are used for some special button logic
#define EDIT_FLAG_2             0x20

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
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// DEFINE THE PATCH CONTENT
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define ARP_PATCH_VERSION   0xA0   // identifies patch version in EEPROM

// sizes of arrays
#define ARP_MAX_CHORD     12      // max notes in a chord
#define ARP_MAX_PATTERN   16      // max steps in a rhythmic pattern
#define ARP_MAX_TRAN_SEQ  16      // max steps in the transpose sequence

// bit flags for the pattern
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

// structure to store status of the arp as a "patch"
typedef struct {
  byte ver;
  int synchPlayRate;                    // ratio of ticks per arp note
  byte arpType;                         // arpeggio type
  char arpOctaveShift;                  // octave transpose
  byte arpOctaveSpan;                   // number of octaves to span with the arpeggio
  byte arpInsertMode;                   // additional note insertion mode
  byte arpVelocityMode;                 // (0 = original, 1 = override)
  byte arpVelocity;                     // velocity 
  byte arpAccentVelocity;               // accent velocity
  byte arpGateLength;                   // gate length (0 = tie notes, 1-127 = fraction of beat)
  char arpTranspose;                    // up/down transpose
  char arpForceToScaleRoot;             // Defines the root note of the scale (0 = C)
  unsigned int arpForceToScaleMask;     // Force to scale interval mask (defines the scale)
  unsigned int arpChord[ARP_MAX_CHORD]; // Chord notes
  int arpChordLength;                   // number of notes in the chord
  byte arpPattern[ARP_MAX_PATTERN];     
  byte arpPatternLength;                // user-defined pattern length (1-16)
  char arpTransposeSequence[ARP_MAX_TRAN_SEQ];
  byte arpTransposeSequenceLen;  
  unsigned int arpManualChord; // Manual chord selected by the user
} ARP_PATCH;

// the active patch
ARP_PATCH _P;

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
#define UI_HOLD_AS_SHIFT 0x10

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
byte uiShowVersion()
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
    return 1;
  }
  return 0;
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
      else if(!(uiHoldType & UI_HOLD_HELD) && !(uiHoldType & UI_HOLD_AS_SHIFT) && (milliseconds > uiHoldPressedTime + uiLongHoldTime))
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
        if(!(uiHoldType & UI_HOLD_HELD) && !(uiHoldType & UI_HOLD_AS_SHIFT))
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
        uiHoldType &= ~UI_HOLD_AS_SHIFT;
        uiHoldPressedTime = milliseconds;
      }    

  if(uiHoldType & UI_HOLD_LOCKED)
    digitalWrite(P_UI_HOLD_LED, !!(uiFlashHold & 0x80));
  else
    digitalWrite(P_UI_HOLD_LED, !!(uiHoldType & UI_HOLD_CHORD));   
}

void uiSetHold() {
  uiHoldType |= UI_HOLD_CHORD;
  uiHoldType &= ~UI_HOLD_PRESSED;
  uiHoldType &= ~UI_HOLD_HELD;
  uiHoldType &= ~UI_HOLD_AS_SHIFT;
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
  EEPROM_ARPOPTIONS1,
  EEPROM_MIDI_OPTS2,
  EEPROM_HH_MODE
};
#define EEPROM_MAGIC_COOKIE_VALUE  0x24

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
byte midiOptions2;

// midi options
#define MIDI_OPTS_SEND_CHMSG       0x01
#define MIDI_OPTS_PASS_INPUT_NOTES 0x02
#define MIDI_OPTS_PASS_INPUT_CHMSG 0x04
#define MIDI_OPTS_SYNCH_INPUT      0x08
#define MIDI_OPTS_SYNCH_AUX        0x10
#define MIDI_OPTS_FILTER_CHMODE    0x20
#define MIDI_OPTS_VOLCAFM_VEL      0x40
#define MIDI_OPTS_LOCAL_OFF        0x80
#define MIDI_OPTS2_USE_NOTE_OFF     0x01

#define MIDI_OPTS_MAX_VALUE        0xFF
#define MIDI_OPTS_DEFAULT_VALUE    (MIDI_OPTS_SEND_CHMSG|MIDI_OPTS_SYNCH_INPUT|MIDI_OPTS_SYNCH_AUX)

#define MIDI_OPTS2_MAX_VALUE        0x01
#define MIDI_OPTS2_DEFAULT_VALUE    0

// macros
#define MIDI_IS_NOTE_ON(msg) ((msg & 0xf0) == 0x90)
#define MIDI_IS_NOTE_OFF(msg) ((msg & 0xf0) == 0x80)
#define MIDI_MK_NOTE_ON (0x90 | midiSendChannel)
#define MIDI_MK_NOTE_OFF (0x80 | midiSendChannel)
#define MIDI_MK_CTRL_CHANGE (0xB0 | midiSendChannel)
#define MIDI_MK_PITCHBEND   (0xE0 | midiSendChannel)

// realtime synch messages
#define MIDI_SYNCH_TICK     0xf8
#define MIDI_SYNCH_START    0xfa
#define MIDI_SYNCH_CONTINUE 0xfb
#define MIDI_SYNCH_STOP     0xfc

#define MIDI_OMNI           0xff

extern void midiLocalOff(byte local_off);

////////////////////////////////////////////////////////////////////////////////
// MIDI INIT
void midiInit()
{
  // init the serial port
  Serial.begin(31250);
//  Serial.begin(9600);
  Serial.flush();

  midiInRunningStatus = 0;
  midiOutRunningStatus = 0;
  midiNumParams = 0;
  midiParamIndex = 0;
  midiSendChannel = eepromGet(EEPROM_OUTPUT_CHAN, 0, 15, 0);
  midiReceiveChannel = eepromGet(EEPROM_INPUT_CHAN, 0, 15, MIDI_OMNI);
  midiOptions = eepromGet(EEPROM_MIDI_OPTS, 0, MIDI_OPTS_MAX_VALUE, MIDI_OPTS_DEFAULT_VALUE);
  midiOptions2 = eepromGet(EEPROM_MIDI_OPTS2, 0, MIDI_OPTS2_MAX_VALUE, MIDI_OPTS2_DEFAULT_VALUE);
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
byte midiRead(unsigned long milliseconds, byte passThru, byte isMidiLockout)
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
      case 0xC0: //  Patch change  1  instrument #   
      case 0xD0: //  Channel Pressure  1  pressure  
        midiNumParams = 1;
        break;    
      case 0xA0: //  Aftertouch  2  key  touch  
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
            if(!!(midiOptions & MIDI_OPTS_PASS_INPUT_NOTES) || isMidiLockout) {
              midiWrite(midiInRunningStatus, midiParams[0], midiParams[1], midiNumParams, milliseconds);                
            }
            return midiInRunningStatus; // return to the arp engine
          case 0xB0: // CC
            if(!!(midiOptions & MIDI_OPTS_FILTER_CHMODE)) {
              if(midiParams[0] >= 120) {
                // break from switch - ignore message
                break;
              }
            }

            if(hhMode == HH_MODE_CVTAB) {
              if(gPreferences & PREF_HH_CVCAL) {
                 switch(midiParams[0]) {
                  case HH_CVCAL_CC_SCALE:
                    hhCVCalScale = (midiParams[1]&0x7F)-64;
                    break;
                  case HH_CVCAL_CC_OFS:
                    hhCVCalOfs = (midiParams[1]&0x7F)-64;
                    break;
                  case HH_CVCAL_CC_SET:
                    hhSetCV(midiParams[1],0);
                    break;
                  case HH_CVCAL_CC_SAVE:
                    gPreferences &= ~PREF_HH_CVCAL;
                    hhCVCalSave();
                    prefsSave();
                    break;
                 }
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
    midiWrite(MIDI_MK_NOTE_OFF, i, 0, 2, millis());    
}

////////////////////////////////////////////////////////////////////////////////
// CLEAR RUNNING STATUS
void midiClearRunningStatus()
{
  midiOutRunningStatus = 0;
}

////////////////////////////////////////////////////////////////////////////////
// SET LOCAL OFF
void midiLocalOff(byte local_off)
{
  midiOutRunningStatus = 0;
  for(int i=0;i<16;++i) {
    midiWrite(0xB0 | i, LOCAL_ON_CC, local_off? 0: 127, 2, millis());    
  }
}

////////////////////////////////////////////////////////////////////////////////
// MIDI NOTE OFF
// allows specific use of MIDI NOTE OFF 0x80 rather than zero velocity 
// NOTE ON (which is usually used with running status to save bandwidth)
void midiNoteOff(byte note, unsigned long milliseconds) {
  midiWrite(
    (midiOptions2 & MIDI_OPTS2_USE_NOTE_OFF)?MIDI_MK_NOTE_OFF:MIDI_MK_NOTE_ON, 
    note, 0, 2, milliseconds);  
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

volatile byte synchAuxEvents[8];               // buffer to store incoming sync events from the aux port
volatile byte synchAuxEventsHead;
volatile byte synchAuxEventsTail;
byte synchClockOutPin;
byte synchTickToPulseRatio;
volatile byte syncStepAdvance;

#define SYNCH_HH_EXT_CLOCK  (0xFF)             // special sendState value meaning external clock in use


// Define pins used for the hack header
#define P_SYNCH_HH_DETECT                19
#define P_SYNCH_HH_CLKOUT                18  
#define P_SYNCH_HH_CLKIN                 14
#define P_CVTAB_HH_CLKOUT                11  
#if SYNCH_HH_CLOCK_ACTIVELOW    
  #define SYNCH_HH_CLOCK_ON           {if(synchClockOutPin && !(gPreferences & PREF_HH_CVTAB_ACC)) digitalWrite(synchClockOutPin, HIGH);}
  #define SYNCH_HH_CLOCK_OFF          {if(synchClockOutPin && !(gPreferences & PREF_HH_CVTAB_ACC)) digitalWrite(synchClockOutPin, LOW);}
#else
  #define SYNCH_HH_CLOCK_OFF          {if(synchClockOutPin && !(gPreferences & PREF_HH_CVTAB_ACC)) digitalWrite(synchClockOutPin, LOW);}
  #define SYNCH_HH_CLOCK_ON           {if(synchClockOutPin && !(gPreferences & PREF_HH_CVTAB_ACC)) digitalWrite(synchClockOutPin, HIGH);}
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
    if(!(synchTickCount % _P.synchPlayRate))//ready for next step?
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
    
    if(!(synchTickCount % TICKS_PER_QUARTER_NOTE)) {      
      synchPulseClockTickCount = 0;
      synchFlags |= SYNCH_BEAT;
    }    
  }

  if(synchFlags & SYNCH_SEND_CLOCK)
    synchTicksToSend++;
    
  if(hhMode == HH_MODE_SYNCTAB || hhMode == HH_MODE_CVTAB)
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
  synchPulseClockTickCount = 0;
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
// PLACE A SYNC EVENT INTO AUX EVENTS QUEUE 
// Events from the aux port are converted into MIDI equivalent message
// and queued up in a circular buffer
void synchAuxEvent(byte event) {
  byte n = (synchAuxEventsHead + 1)&7;
  if(n != synchAuxEventsTail) {
    synchAuxEvents[synchAuxEventsHead] = event;
    synchAuxEventsHead = n;
  }
}

//////////////////////////////////////////////////////////////////////////
//
// synchReset_ISR
// Called at start of bar
// 
//////////////////////////////////////////////////////////////////////////
void synchReset_ISR()
{
  synchAuxEvent(MIDI_SYNCH_START);
}

//////////////////////////////////////////////////////////////////////////
//
// synchTick_ISR
// Called on midi synch
// 
//////////////////////////////////////////////////////////////////////////
void synchTick_ISR()
{
  synchAuxEvent(MIDI_SYNCH_TICK);
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
    if((hhMode == HH_MODE_SYNCTAB) && (synchFlags & SYNCH_TO_MIDI)) {
        ++syncStepAdvance;
    }    
    else {
      unsigned long ms = millis();
      if(synchLastPulseClockTime && synchLastPulseClockTime < ms)
        synchThisPulseClockPeriod = ms - synchLastPulseClockTime;
      else      
        synchFlags |= SYNCH_ZERO_TICK_COUNT; // synch the beat LED
      synchLastPulseClockTime = ms;
    }
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
  synchAuxEventsHead = 0;
  synchAuxEventsTail = 0;
  
  // by default don't synch
  if(eepromGet(EEPROM_SYNCH_SOURCE,0,1,0)) 
    synchFlags |= SYNCH_TO_MIDI;

  // by default do not send synch
  if(eepromGet(EEPROM_SYNCH_SEND,0,1,0))
    synchFlags |= SYNCH_SEND_CLOCK;
  
  synchTicksToSend = 0;
  synchStartSource = SYNCH_SOURCE_NONE;  
  
  // set default play rate
  _P.synchPlayRate = SYNCH_RATE_16;

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
  synchClockOutPin = 0;
  syncStepAdvance = 0;
  synchTickToPulseRatio = (gPreferences & PREF_HH_8THCLOCK)? 12:6; // sixteenths or eigths    
  if(hhMode == HH_MODE_SYNCTAB) {
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
    synchClockOutPin = P_SYNCH_HH_CLKOUT;
  }
  else if(hhMode == HH_MODE_CVTAB) {
    pinMode(P_CVTAB_HH_CLKOUT, OUTPUT);    
    synchClockOutPin = P_CVTAB_HH_CLKOUT;
  }
  interrupts();
}

////////////////////////////////////////////////////////////////////////////////
// SYNCH RUN
void synchRun(unsigned long milliseconds)
{
  byte auxRunning = !!(PIND & DBIT_SYNCH_RUN); // see if aux port is running
  if(auxRunning && !(synchFlags & SYNCH_AUX_RUNNING)) { // transition STOP->RUN
    synchFlags |= SYNCH_AUX_RUNNING;
    synchAuxEvent(MIDI_SYNCH_CONTINUE); // stick continue event in queue
  }
  else if(!auxRunning && !!(synchFlags & SYNCH_AUX_RUNNING)) {// transition RUN->STOP
    synchFlags &= ~SYNCH_AUX_RUNNING;
    synchAuxEvent(MIDI_SYNCH_STOP); // stick stop event in queue
  }

  // else check if we are using internal clock
  if(!(synchFlags & SYNCH_TO_MIDI))
  { 
    if(synchThisPulseClockPeriod) // have we got a clock period measurement?
    {      
      synchSetInternalTickPeriod((float)synchThisPulseClockPeriod / synchTickToPulseRatio);  // infer bpm
      synchThisPulseClockPeriod = 0;
      synchNextInternalTick = milliseconds;  // tick immediately
      while(synchInternalTicksSinceLastPulseClock++ < synchTickToPulseRatio)
        synchTick(SYNCH_SOURCE_INTERNAL); // make up any missed ticks (when tempo is being increased)
      synchInternalTicksSinceLastPulseClock = 0;
    }
    
    if(synchClockSendState == SYNCH_HH_EXT_CLOCK && 
      synchInternalTicksSinceLastPulseClock >= synchTickToPulseRatio)
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
  else if(syncStepAdvance) {
    --syncStepAdvance;
    if(synchFlags & SYNCH_RESTART_ON_BEAT) {
      synchPlayIndex = 0;
    }
    else {
      synchPlayIndex++;
    }
    synchFlags |= SYNCH_PLAY_ADVANCE;
    synchFlags &= ~SYNCH_RESTART_ON_BEAT;    
    synchFlags |= SYNCH_BEAT;
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
      if(synchPulseClockTickCount>=synchTickToPulseRatio)
      {
        SYNCH_HH_CLOCK_ON;
        synchPulseClockTickCount -= synchTickToPulseRatio;
        synchClockSendState = 1;
        synchTickToPulseRatio = (gPreferences & PREF_HH_8THCLOCK)? 12:6; // sixteenths or eigths            
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
// PROCESS ALL THE PENDING EVENTS IN THE AUX EVENT QUEUE
void synchAuxRead()
{                 
  while(synchAuxEventsTail != synchAuxEventsHead) {
    if(!!(synchFlags & SYNCH_TO_MIDI) && !!(midiOptions & MIDI_OPTS_SYNCH_AUX))
    {
      switch(synchAuxEvents[synchAuxEventsTail])
      {
      case MIDI_SYNCH_TICK:
        synchTick(SYNCH_SOURCE_AUX);
        break;            
      case MIDI_SYNCH_START:
        synchStart(SYNCH_SOURCE_AUX);
        break;
      case MIDI_SYNCH_CONTINUE:
        synchContinue(SYNCH_SOURCE_AUX);
        break;
      case MIDI_SYNCH_STOP:
        synchStop(SYNCH_SOURCE_AUX);
        break;
      }
    }
    synchAuxEventsTail = (synchAuxEventsTail + 1)&7;
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
//#define ARP_MAX_CHORD 12
#define ARP_MAKE_NOTE(note, vel) ((((unsigned int)vel)<<8)|(note))
#define ARP_GET_NOTE(x) ((x)&0x7f)
#define ARP_GET_VELOCITY(x) (((x)>>8)&0x7f)
#define ARP_MAX_SEQUENCE 60
#define ARP_NOTE_HELD 0x8000
#define ARP_PLAY_THRU 0x8000

char arpChordRootNote;                      // root note of the chord
int arpNotesHeld;                           // number of notes physically held
unsigned int arpSequence[ARP_MAX_SEQUENCE]; // generated sequence of notes
int arpSequenceLength;                      // number of notes in the sequence
int arpSequenceIndex;                       // index of playing note in sequence
byte arpPatternIndex;                       // position in the pattern (for display)
byte arpRefresh;                            // whether the pattern index is changed
byte arpPlayingNotes[16];                   // which notes are currently playing
unsigned long arpStopNoteTime;              // when playing notes time out
byte arpTransposeSequencePos;               // position in the transpose sequence
unsigned int arpTransposeSequenceMask;      // used to indicate transpose status on the LEDs
byte arpLastVelocityCC;                     // the last CC value sent out for velocity

enum {
  ARP_FLAG_REBUILD = 0x01,
  ARP_FLAG_MUTE    = 0x02
};

// ARP STATUS FLAGS
byte arpFlags;                              // status flags - see above
unsigned int arpOptions;

enum {
                                      //012345678901234567
  ARP_OPT_MIDITRANSPOSE   = (unsigned)0b1000000000000000, // Hold button secondary function
  ARP_OPT_SKIPONREST      = (unsigned)0b0010000000000000, // Whether rests are skipped or held
  ARP_OPTS_MASK           = (unsigned)0b1010000000000000,
};

enum {
  ARP_SHOW_PATN = 0,
  ARP_SHOW_ACCENT,
  ARP_SHOW_GLIDE,
  ARP_SHOW_TIE,
  ARP_SHOW_OCTUP,
  ARP_SHOW_OCTDN,
  ARP_SHOW_4THDN,
  ARP_SHOW_PLAYTHRU
};
byte arpShowLayer;

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
// ARP RESET
void arpReset() {
  arpSequenceLength = 0;
  arpChordRootNote = -1;
  arpSequenceIndex = 0;
  arpTransposeSequencePos = 0;
  arpTransposeSequenceMask = 0;
}

////////////////////////////////////////////////////////////////////////////////
// ARP INIT
void arpInit()
{
  int i;

  arpNotesHeld = 0;
  arpRefresh = 0;
  arpFlags = 0;
  arpLastVelocityCC = 0xFF;
  arpShowLayer = ARP_SHOW_PATN;
  
  _P.arpType = ARP_TYPE_UP;
  _P.arpOctaveShift = 0;
  _P.arpOctaveSpan = 1;
  _P.arpInsertMode = ARP_INSERT_OFF;
  _P.arpVelocity = 100;
  _P.arpAccentVelocity = 127;
  _P.arpVelocityMode = 1;
  _P.arpChordLength = 0;
  _P.arpPatternLength = 16;
  _P.arpGateLength = 100;
  _P.arpTranspose = 0;
  _P.arpForceToScaleRoot=0;
  _P.arpForceToScaleMask=ARP_SCALE_CHROMATIC|ARP_SCALE_ADJUST_SHARP;
  _P.arpManualChord = 0;
  _P.arpTransposeSequenceLen = 0;
  arpOptionsLoad();
  
  // the pattern starts with all beats on
  for(i=0;i<16;++i)
    _P.arpPattern[i] = ARP_PATN_PLAY;

  // no notes playing
  for(i=0;i<16;++i)
    arpPlayingNotes[i] = 0;    

  arpReset();
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
    midiWrite(MIDI_MK_NOTE_ON, note, velocity, 2, milliseconds);          
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
  byte gateOff = 0;
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
            midiNoteOff(note, milliseconds);
            arpPlayingNotes[i] &= ~m;
            gateOff = 1;
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
  _P.arpChordLength = 0;
  _P.arpManualChord = 0;
  editFlags |= EDIT_FLAG_FORCE_REFRESH;
  arpFlags |= ARP_FLAG_REBUILD;
}

////////////////////////////////////////////////////////////////////////////////
// COPY CHORD - RETURN LOWEST NOTE
char arpCopyChord(int *dest)
{
  char m =-1;
  int i;
  for(i=0; i<_P.arpChordLength; ++i)
  {
    dest[i] = _P.arpChord[i];
    if(m == -1 || _P.arpChord[i] < m)
      m = _P.arpChord[i];
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
    for(int i=0; i<_P.arpChordLength-1; ++i)
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
 return _P.arpChord[0];
}

////////////////////////////////////////////////////////////////////////////////
// RANDOMIZE CHORD
void arpRandomizeChord(int *dest)
{
  int i,j;

  // clear destination buffer
  for(i=0; i<_P.arpChordLength; ++i)
    dest[i] = 0;

  // loop through the source chord
  for(i=0; i<_P.arpChordLength; ++i)
  {
    // loop until we find a place to 
    // put this note in dest buffer
    for(;;)
    {
      // look for a place
      j = random(_P.arpChordLength);
      if(!dest[j])
      {
        // its empty, so we can use it
        dest[j] = _P.arpChord[i];
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
  if(!_P.arpChordLength)
    return;

  // sort the chord info if needed
  int chord[ARP_MAX_CHORD];
  if(_P.arpType == ARP_TYPE_UP || _P.arpType == ARP_TYPE_DOWN || _P.arpType == ARP_TYPE_UP_DOWN)
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
    _P.arpForceToScaleMask ^= ARP_SCALE_TOGGLE_FLAG;
    byte adjustToggle = !!(_P.arpForceToScaleMask & ARP_SCALE_TOGGLE_FLAG);
    
    // this loop is for the octave span
    int octaveCount;
    for(octaveCount = 0; 
      octaveCount < _P.arpOctaveSpan && tempSequenceLength < ARP_MAX_SEQUENCE; 
      ++octaveCount)
    {
      // Set up depending on arp type
      int transpose;
      int chordIndex;
      int lastChordIndex;
      int chordIndexDelta;      
      switch(_P.arpType)
      {
      case ARP_TYPE_RANDOM:
        arpRandomizeChord(chord);
        // fall thru
      case ARP_TYPE_UP:
      case ARP_TYPE_MANUAL:
      case ARP_TYPE_POLYGATE:
        chordIndex = 0;
        lastChordIndex = _P.arpChordLength - 1;
        transpose = _P.arpTranspose + 12 * (_P.arpOctaveShift + octaveCount);    
        chordIndexDelta = 1;
        nextPass = 0;
        break;          

      case ARP_TYPE_DOWN:
        chordIndex = _P.arpChordLength - 1;
        lastChordIndex = 0;
        transpose = _P.arpTranspose + 12 * (_P.arpOctaveShift + _P.arpOctaveSpan - octaveCount - 1);    
        chordIndexDelta = -1;
        nextPass = 0;
        break;          

      case ARP_TYPE_UP_DOWN:        
        if(nextPass == 1)
        {
          // going up we can play all the notes
          chordIndex = 0;
          lastChordIndex = _P.arpChordLength - 1;
          chordIndexDelta = 1;
          transpose = _P.arpTranspose + 12 * (_P.arpOctaveShift + octaveCount);    
          if(octaveCount == _P.arpOctaveSpan - 1)
            nextPass = 2;
        }
        else
        {
          // GOING DOWN!
          // Is the range just one octave?
          if(_P.arpOctaveSpan == 1)
          {
            // On the way down we don't play top or bottom notes of the chord
            chordIndex = _P.arpChordLength - 2;
            lastChordIndex = 1;
            nextPass = 0;
          }
          // are we on the top octave of the descent?
          else if(octaveCount == 0)
          {
            // the top note is skipped, the bottom note can be played
            chordIndex = _P.arpChordLength - 2;
            lastChordIndex = 0;
          }
          // are we on the bottom octave of the descent?
          else if(octaveCount == _P.arpOctaveSpan - 1)
          {
            // top note can be played but bottom note is not
            chordIndex = _P.arpChordLength - 1;
            lastChordIndex = 1;

            // this the the last octave to play
            nextPass = 0;
          }
          else
          {
            // this is not first or last octave of the descent, so there
            // is no need to skip any of the notes
            chordIndex = _P.arpChordLength - 1;
            lastChordIndex = 0;
          }
          transpose = _P.arpTranspose + 12 * (_P.arpOctaveShift + _P.arpOctaveSpan - octaveCount - 1);    
          chordIndexDelta = -1;
        }
        break;
      }        

      // Write notes from the chord into the arpeggio sequence
      while(chordIndex >= 0 && 
        chordIndex < _P.arpChordLength && 
        tempSequenceLength < ARP_MAX_SEQUENCE)
      {
        // fetch the current note
        int note = ARP_GET_NOTE(chord[chordIndex]);
        byte velocity = ARP_GET_VELOCITY(chord[chordIndex]);
        byte skipNote = 0;

        // transpose as needed
        note += transpose;

        // force to scale
        int scaleNote = note - _P.arpForceToScaleRoot;
        while(scaleNote<0)
          scaleNote+=12;
        if(!(_P.arpForceToScaleMask & ((int)0x0800>>(scaleNote % 12))))
        {
          switch(_P.arpForceToScaleMask & ARP_SCALE_ADJUST_MASK)
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
  if(_P.arpType == ARP_TYPE_POLYGATE)
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
    switch(_P.arpInsertMode)
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
  byte midiLockout = (!!(uiHoldType & UI_HOLD_LOCKED) &&  !(arpOptions & ARP_OPT_MIDITRANSPOSE));
  
  // we may have multiple notes to read
  for(;;)
  {
    
    // read the MIDI port
    byte msg = midiRead(milliseconds, 0, midiLockout);      
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
            _P.arpTranspose = note - arpChordRootNote;                        
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
        for(i=0;i<_P.arpChordLength;++i)
        {        
          if(ARP_GET_NOTE(_P.arpChord[i])== note)
            noteIndexInChord = i;
          if(_P.arpChord[i] & ARP_NOTE_HELD)
            arpNotesHeld++;
        }
  
        // is the note already part of the current chord?
        if(noteIndexInChord >= 0 && arpNotesHeld)
        {
          // Mark the key as held. There is no change to the arpeggio
          if(!(_P.arpChord[noteIndexInChord] & ARP_NOTE_HELD))
          {        
            _P.arpChord[noteIndexInChord] |= ARP_NOTE_HELD;
            arpNotesHeld++;
          }
        }
        else 
        {
          // if its the first note of a new chord then
          // we need to restart play
          if(!arpNotesHeld)
          {
            _P.arpChordLength = 0;
            if(!!(uiHoldType & UI_HOLD_CHORD))
              synchFlags |= SYNCH_RESTART_ON_BEAT; // wait till next beat before restarting
            else
              synchRestartSequence();
          }
  
          // insert the new note into the chord                   
          if(_P.arpChordLength < ARP_MAX_CHORD-1)
          {        
            _P.arpChord[_P.arpChordLength] = ARP_MAKE_NOTE(note,velocity);
            _P.arpChord[_P.arpChordLength] |= ARP_NOTE_HELD;
            _P.arpChordLength++;
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
      for(i=0; i<_P.arpChordLength; ++i)
      {
        // did we find the released key in the chord?
        if(ARP_GET_NOTE(_P.arpChord[i]) == note)
        {
          _P.arpChord[i] &= ~ARP_NOTE_HELD;
          noteIndexInChord = i;
        }
        else if(_P.arpChord[i] & ARP_NOTE_HELD)
        {
          arpNotesHeld++;
        }
      }

      // should the note be removed from the chord?
      if(!(uiHoldType & UI_HOLD_CHORD) && noteIndexInChord >= 0)
      {     

        // shift higher notes down one position
        // to remove the released note
        for(i = noteIndexInChord;i < _P.arpChordLength-1; ++i)
          _P.arpChord[i] = _P.arpChord[i+1];

        // rebuild the sequence
        --_P.arpChordLength;
        arpFlags |= ARP_FLAG_REBUILD;
      }
    }
  }

  // check if the hold switch is released while
  // there are notes being held
  if(!(uiHoldType & UI_HOLD_CHORD) && !arpNotesHeld && _P.arpChordLength)
    arpClear();
}

void arpSetManualChord()
{
  _P.arpChordLength = 0;
  arpNotesHeld = 0;  
  unsigned int m=1;
  for(int i=0; i<16; ++i) {
     if((_P.arpManualChord & m) && (_P.arpChordLength < ARP_MAX_CHORD-1))
     {        
       _P.arpChord[_P.arpChordLength] = ARP_MAKE_NOTE(48+i,127);
       _P.arpChordLength++;
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
  
  // read events from aux port
  synchAuxRead();

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
  if(!!(synchFlags & SYNCH_PLAY_ADVANCE) && arpSequenceLength && _P.arpPatternLength)
  {                 
    // get the index into the pattern
    arpPatternIndex = synchPlayIndex % _P.arpPatternLength;
    
    // Check if we need to transpose
    if(!arpPatternIndex && _P.arpTransposeSequenceLen) {
      _P.arpTranspose = _P.arpTransposeSequence[arpTransposeSequencePos];
      arpBuildSequence();                
      midiClearRunningStatus();
      if(++arpTransposeSequencePos >= _P.arpTransposeSequenceLen) {
        arpTransposeSequencePos = 0;
      }      
      editFlags |= EDIT_FLAG_FORCE_REFRESH;
    }

    if(hhMode == HH_MODE_CVTAB) {
      hhSetAccent(_P.arpPattern[arpPatternIndex] & ARP_PATN_ACCENT);
    }
 
    // check there is a note (not a rest at this) point in the pattern
    if((_P.arpPattern[arpPatternIndex] & ARP_PATN_PLAY) || (arpOptions & ARP_OPT_SKIPONREST))
    {
      byte glide = 0;
      if(_P.arpPattern[arpPatternIndex] & ARP_PATN_TIE)
        glide = 2;
      else if(_P.arpPattern[arpPatternIndex] & ARP_PATN_GLIDE)
        glide = 1;
        
      // Keep the sequence index within range      
      if(arpSequenceIndex >= arpSequenceLength)
        arpSequenceIndex = 0;
      
      // Loop to action play-through flag
      byte playThru = (_P.arpPattern[arpPatternIndex] & ARP_PATN_PLAYTHRU)? 2:0;      
      byte note = 0;
      do {      
        if(playThru == 2) {
          playThru = 1;
        }
        else {
          // check play thru flag
          playThru = !!(arpSequence[arpSequenceIndex] & ARP_PLAY_THRU);
        }
        
        // Play the note if applicable
        if(_P.arpPattern[arpPatternIndex] & ARP_PATN_PLAY)
        {
          note = ARP_GET_NOTE(arpSequence[arpSequenceIndex]);
          if((_P.arpPattern[arpPatternIndex] & ARP_PATN_OCTDN) && (note > 12))
            note-=12;
          else if((_P.arpPattern[arpPatternIndex] & ARP_PATN_OCTAVE) && (note <= 115))
            note+=12;
          else if((_P.arpPattern[arpPatternIndex] & ARP_PATN_4TH) && (note >= 5))
            note-=5;
          // determine note velocity
          byte velocity;          
          if(arpFlags & ARP_FLAG_MUTE) {
            velocity = 0;
          }
          else if(_P.arpPattern[arpPatternIndex] & ARP_PATN_ACCENT) {
            velocity = _P.arpAccentVelocity;
          }
          else {
            velocity = _P.arpVelocityMode? _P.arpVelocity : ARP_GET_VELOCITY(arpSequence[arpSequenceIndex]);        
          }
    
          // start the note playing
          if(note > 0)
          {
            arpStartNote(note, velocity, milliseconds, noteSet);
          }
        }
        
        // next note
        ++arpSequenceIndex;
      } while(playThru && arpSequenceIndex < arpSequenceLength);

      if(note > 0)
      { 
        if(hhMode == HH_MODE_CVTAB) {
          if(hhGlideActive) {
            hhSetCV(note, 1);
            hhSetGate(HH_GATE_OPEN);        
          }
          else {
            hhSetCV(note, 0);
            hhSetGate(HH_GATE_RETRIG);          
          }
        }
        
        // if the previous note is still playing when a new one is played
        // then stop it (should be the case only for "tie" mode)
        arpStopNotes(milliseconds, noteSet);
      }

      // check if we need to "glide"
      if(glide == 2) 
      {
          // tie          
          arpStopNoteTime = 0;
          hhGlideActive = 1;
      }
      else if(glide == 1) 
      {
         // full step
         arpStopNoteTime = milliseconds + synchStepPeriod;
         hhGlideActive = 0;
      }
      else if(_P.arpGateLength)
      {              
        // work out the gate length for this note
        arpStopNoteTime = milliseconds + (synchStepPeriod * _P.arpGateLength) / 150;
        hhGlideActive = 0;
      }
      else
      {
        // note till play till the next one starts
        arpStopNoteTime = 0;               
        hhGlideActive = 0;
      }
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
    if(hhMode == HH_MODE_CVTAB) {
      hhSetGate(HH_GATE_CLOSE);
    }
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
          _P.arpTranspose = 12 * (float)(centreDetent(value)-512.0)/511.0;
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
          _P.arpVelocity = v/8;
          _P.arpVelocityMode = 1;
          editFlags |= EDIT_FLAG_FORCE_REFRESH;
          break;
        case GATELEN:
          v = endStops(value);
          if(1023 == v)
            _P.arpGateLength = 0;
          else if(v < 10)
            _P.arpGateLength = 1;
          else
            _P.arpGateLength = 10 + (v*150.0)/1023.0;
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



/////////////////////////////////////////////////////
byte hhReadMemory(int addr, byte *dest, int len) 
{

  // set the start address
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(addr>>8);
  Wire.write(addr&0xFF);
  Wire.endTransmission();
  
  // Arduino Wire library can read a maximum
  // of 32 bytes at a time, so we need to read
  // multiple blocks  
  while(len > 0) {  

    // get next block or all remaining bytes if less
    int blockSize = len;
    if(blockSize>32) {
      blockSize = 32;
    }      
    len-=blockSize;
    if(Wire.requestFrom(EEPROM_ADDR, blockSize) != blockSize) {
      return 0;
    }

    // copy to destination
    while(blockSize-- > 0) {
      *dest++ = Wire.read();
    }  
  }
  return 1;
}

/////////////////////////////////////////////////////
// Address must be on a 32-byte boundary, ie (addr & ~31) == 0
byte hhWriteMemory(int addr, byte *src, int len) {    
    // while there are more bytes to send
    while(len > 0) {

      // since the arduino Wire library has a 32 byte buffer size
      // we will need 2 write cycles to fill a 32 byte page on the
      // EEPROM (since 2 byte write address must also be sent)
      // Therefore we'll send each 32 byte page as two 16 bit 
      // writes
      Wire.beginTransmission(EEPROM_ADDR);
      Wire.write(addr>>8);
      Wire.write(addr&0xFF);

      int blockSize = len;
      if(blockSize > 16) {
        blockSize = 16;
      }
      addr+=blockSize;
      len-=blockSize;
      for(int i=0; i<blockSize; ++i) {
        Wire.write(*src++);
      }
      Wire.endTransmission();      
      
      // wait for the write to complete
      for(;;) {
        Wire.beginTransmission(EEPROM_ADDR);
        Wire.write(0);
        Wire.write(0);
        if(2 != Wire.endTransmission()) {
          break;
        }
      }      
    }      
}

#define PATCH_ADDR(i) (((int)(i))<<9)
byte hhPatchQuery(byte which) {
  byte ver=0;
  return hhReadMemory(PATCH_ADDR(which), &ver, 1) && (ver==ARP_PATCH_VERSION);
}

void hhSetDAC(int dac) {
      Wire.beginTransmission(DAC_ADDR); 
      Wire.write((dac>>8)&0xF); 
      Wire.write((byte)dac); 
      Wire.endTransmission();         
}


long hhNote2DAC_HzVolt(long note) {
  // use a hard coded lookup table to get the
  // the DAC value for note in top octave
  long dac;
  if(note == 72)
    dac = 4000; // we can just about manage a C6!
  else switch((byte)note % 12) {  
    case 0: dac = 2000; break;
    case 1: dac = 2119; break;
    case 2: dac = 2245; break;
    case 3: dac = 2378; break;
    case 4: dac = 2520; break;
    case 5: dac = 2670; break;
    case 6: dac = 2828; break;
    case 7: dac = 2997; break;
    case 8: dac = 3175; break;
    case 9: dac = 3364; break;
    case 10: dac = 3564; break;
    case 11: dac = 3775; break; 
  }

  // transpose to the requested octave by 
  // right shifting
  byte octave = ((byte)note)/12;
  if(octave > 5) octave = 5;
  dac >>= (5-octave);
  dac = ((dac * (4096 + hhCVCalScale))/4096) + hhCVCalOfs;

  while(dac>4095) {
    dac/=2;
  }
  return dac;
}
long hhNote2DAC_VOct(long note) {
      long dac = (((note-12) * 500)/12);
      dac = ((dac * (4096 + hhCVCalScale))/4096) + hhCVCalOfs;
      while(dac<0) dac+=500;
      while(dac>4095) dac-=500;
      return dac;  
}
void hhSetCV(long note, byte glide) {
      long dac = (gPreferences & PREF_HH_CVTAB_HZVOLT)? 
        hhNote2DAC_HzVolt(note) : 
        hhNote2DAC_VOct(note);

      hhDACTarget = dac<<16;
      hhDACIncrement = (hhDACTarget - hhDACCurrent)/((long)synchStepPeriod);        
      
      if(!glide || !hhDACIncrement) {
        // immediately set the requested CV
        hhSetDAC(dac);
        hhDACCurrent = hhDACTarget;      
        hhDACIncrement = 0;
      }
}

void hhSetGate(byte state) {
  switch(state) {
    case HH_GATE_CLOSE:
      digitalWrite(P_HH_CVTAB_GATE,LOW);      
      break;
    case HH_GATE_RETRIG:
      digitalWrite(P_HH_CVTAB_GATE,LOW);      
      delay(1);
    case HH_GATE_OPEN:
      digitalWrite(P_HH_CVTAB_GATE,HIGH);      
      break;
   }
}
void hhSetAccent(byte state) {
  if((hhMode == HH_MODE_CVTAB) && (gPreferences & PREF_HH_CVTAB_ACC)) {
    digitalWrite(P_CVTAB_HH_CLKOUT, state? HIGH:LOW);
  }
}

#define HH_CAL_ADDR     0x1FF7 // top 8 bytes of slot for patch 15
#define HH_CAL_COOKIE   0x12
void hhCVCalSave() {
    byte data[3] = {
      HH_CAL_COOKIE,
      hhCVCalScale,
      hhCVCalOfs
    };
    if(hhWriteMemory(HH_CAL_ADDR, data, sizeof(data))) {
      uiSetLeds(0, 16, uiLedBright);
      delay(1000);
      editFlags |= EDIT_FLAG_FORCE_REFRESH;
    }
}

void hhCVCalLoad() {
    byte data[3] = {0};
    if(hhReadMemory(HH_CAL_ADDR, data, sizeof(data))) {
      if(data[0] == HH_CAL_COOKIE) {
        hhCVCalScale = (char)data[1];
        hhCVCalOfs = (char)data[2];
      }
    }
}

////////////////////////////////////////////////////////////////////////////////
// Initialise hack header manager
void hhInit() 
{
  if(hhMode == HH_MODE_CTRLTAB) {
    pinMode(P_HH_SW_PB3,INPUT_PULLUP);
    pinMode(P_HH_POT_PC5,INPUT);
    pinMode(P_HH_POT_PC4,INPUT);
    pinMode(P_HH_POT_PC0,INPUT);
  }
  Pot1.reset();
  Pot2.reset();
  Pot3.reset();
  hhTime = 0;

  hhCVCalScale = 0;
  hhCVCalOfs = 0;

  hhDACCurrent = 0;
  hhDACTarget = 0;
  hhDACIncrement = 0;
  hhGlideActive = 0;
  
  if(hhMode == HH_MODE_CVTAB) {
    Wire.begin();
    Wire.beginTransmission(DAC_ADDR); 
    Wire.write(0b10011001); // buffered Vref, powered up, 2x
    Wire.endTransmission();       
    pinMode(P_HH_CVTAB_GATE,OUTPUT);    
    digitalWrite(P_HH_CVTAB_GATE,LOW);    

    hhCVCalLoad();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Run hack header manager
void hhRun(unsigned long milliseconds)
{      
  if(hhMode == HH_MODE_CVTAB) {
    if((byte)milliseconds == hhTime)
      return;
    hhTime = (byte)(milliseconds);
    if(!hhDACIncrement) {
      // do nothing
    }
    else {
      // run CV glide, stopping when target CV is reached
      hhDACCurrent += hhDACIncrement;
      if(hhDACIncrement > 0) {
        if(hhDACCurrent >= hhDACTarget) {
          hhDACCurrent = hhDACTarget;
          hhDACIncrement = 0;
        }
      }
      else
      {
        if(hhDACCurrent <= hhDACTarget) {
          hhDACCurrent = hhDACTarget;
          hhDACIncrement = 0;
        }
      }               
      hhSetDAC(hhDACCurrent>>16);
    }
  }
  else if(hhMode == HH_MODE_CTRLTAB) {

    // enforce a minimum period of 16ms between I/O polls
    if((byte)(milliseconds>>4) == hhTime)
      return;
    hhTime = (byte)(milliseconds>>4); 
    arpFlags &= ~ARP_FLAG_MUTE;
    synchFlags &= ~SYNCH_HOLD_AT_ZERO;
  
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
  EDIT_MODE_TRANSPOSE        = UI_KEY_C4,

  // shifted "virtual" menu buttons
  EDIT_MODE_LOAD_PATCH       = 0x40,
  EDIT_MODE_SAVE_PATCH,
  EDIT_MODE_CLEAR_PATCH
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
byte editBlinkState;

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
  editBlinkState = 0;
}

////////////////////////////////////////////////////////////////////////////////
// EDIT PATTERN
void editPattern(char keyPress, byte forceRefresh)
{
  static const byte extBit[]={
    ARP_PATN_PLAY,
    ARP_PATN_PLAYTHRU,
    ARP_PATN_4TH,
    ARP_PATN_OCTDN,
    ARP_PATN_OCTAVE,
    ARP_PATN_TIE,
    ARP_PATN_GLIDE,
    ARP_PATN_ACCENT
  };
  if(editFlags & EDIT_FLAG_IS_HELD) { // PATN button is down now. 

    if(!(editFlags & EDIT_FLAG_1)) {
      forceRefresh = 1;          
      editFlags |= EDIT_FLAG_1;       
    }
    if(keyPress >= 8 && keyPress < 16) { // is a layer button pressed?      
      forceRefresh = 1;          
      arpShowLayer = keyPress - 8;      
    }    
  }
  else {

    if(editFlags & EDIT_FLAG_1) {
      forceRefresh = 1; 
      editFlags &= ~EDIT_FLAG_1;
    }    
    if(keyPress != NO_VALUE)
    {
      _P.arpPattern[keyPress] = (_P.arpPattern[keyPress] ^ extBit[arpShowLayer]);
      forceRefresh = 1;
    }
  }

  
  if(forceRefresh || arpRefresh)
  {   
    if(editFlags & EDIT_FLAG_1) { 
      uiClearLeds();
      uiSetLeds(8, 8, uiLedMedium);
      uiLeds[8+arpShowLayer] = uiLedBright;
    }
    else if(arpShowLayer == ARP_SHOW_PATN) {
      for(int i=0; i<16; ++i) {
        uiLeds[i] = (_P.arpPattern[i] & ARP_PATN_PLAY) ? uiLedMedium : 0;
      }
    }
    else {
      for(int i=0; i<16; ++i) {
        if((_P.arpPattern[i] & ARP_PATN_PLAY)) {
            uiLeds[i] = (_P.arpPattern[i] & extBit[arpShowLayer]) ? uiLedMedium : uiLedDim;
        }
        else {
            uiLeds[i] = (_P.arpPattern[i] & extBit[arpShowLayer]) ? uiLedDim : 0;          
        }
      }
    }

    // only display the play position if we have a sequence
    if(arpSequenceLength) {
      uiLeds[arpPatternIndex] = uiLedBright;
    }
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
    _P.arpPatternLength = keyPress + 1;
    forceRefresh = 1;
  }

  if(forceRefresh)
  {    
    uiClearLeds();
    uiSetLeds(0, _P.arpPatternLength, uiLedDim);
    uiLeds[_P.arpPatternLength-1] = uiLedBright;
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
    _P.arpType = keyPress;
    arpFlags |= ARP_FLAG_REBUILD;
    forceRefresh = 1;
    break;
  case 12: 
    for(i = 0;i<16;++i) {
      _P.arpPattern[i] &= ARP_PATN_PLAY;
      if(_P.arpPattern[i] & ARP_PATN_PLAY) {
        switch(random(8)) {
          case 3: _P.arpPattern[i] |= ARP_PATN_ACCENT; break;
          case 4: _P.arpPattern[i] |= ARP_PATN_TIE; break;
          case 5: _P.arpPattern[i] |= ARP_PATN_GLIDE; break;
          case 6: _P.arpPattern[i] |= ARP_PATN_OCTAVE; break;
          case 7: _P.arpPattern[i] |= ARP_PATN_GLIDE|ARP_PATN_OCTAVE; break;
        }
      }
    }
    break;
  case 13: 
    for(i = 0;i<16;++i) {
      switch(random(5)) {
        case 0: case 1: _P.arpPattern[i] = 0; break;
        case 4: _P.arpPattern[i] = ARP_PATN_PLAY|ARP_PATN_ACCENT; break;
        default: _P.arpPattern[i] = ARP_PATN_PLAY; break;
      }
    }
    break;
  case 14:
    for(i = 0;i<16;++i) _P.arpPattern[i] = 0;
    _P.arpPatternLength = 16;
    break;
  case 15:
    for(i = 0;i<16;++i) _P.arpPattern[i] = ARP_PATN_PLAY;
    _P.arpPatternLength = 16;
    break;
  } 

  if(forceRefresh)
  {
    uiClearLeds();
    uiSetLeds(0, 6, uiLedMedium);
    uiLeds[_P.arpType] = uiLedBright;
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
    if(b & ARP_OPTS_MASK)
    {
       arpOptions^=b;
       arpOptionsSave();
       arpOptionsApply();
       forceRefresh = 1;
    } 
  }

  if(forceRefresh)
  {
    uiSetLeds(ARP_OPTS_MASK, arpOptions);
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
    _P.arpOctaveShift = keyPress - 3;
    arpFlags |= ARP_FLAG_REBUILD;
    forceRefresh = 1;
  }

  if(forceRefresh)
  {
    uiClearLeds();
    uiSetLeds(0, 7, uiLedDim);
    uiLeds[3] = uiLedMedium;
    uiLeds[3 + _P.arpOctaveShift] = uiLedBright;
  }
}

/////////////////////////////////////////////////////
// EDIT OCTAVE SPAN
void editOctaveSpan(char keyPress, byte forceRefresh)
{
  if(keyPress >= 0 && keyPress <= 3)
  {
    _P.arpOctaveSpan = keyPress + 1;
    arpFlags |= ARP_FLAG_REBUILD;
    forceRefresh = 1;
  }

  if(forceRefresh)
  {
    uiClearLeds();
    uiSetLeds(0, 4, uiLedDim);
    uiLeds[_P.arpOctaveSpan - 1] = uiLedBright;
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
    _P.synchPlayRate = rates[keyPress];
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
      if(_P.synchPlayRate == rates[i]) 
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
  if(keyPress == 0 && !_P.arpVelocity && _P.arpVelocityMode)
  {    
    _P.arpVelocityMode = 0;
    forceRefresh = 1;
  }
  else if(keyPress >= 0 && keyPress <= 15)
  {    
    _P.arpVelocity = vel[keyPress];
    _P.arpVelocityMode = 1;
    forceRefresh = 1;
  }

  if(forceRefresh)
  {
    uiClearLeds();
    if(!_P.arpVelocityMode) 
    {
      // original velocity
      uiLeds[0] = uiLedBright;
      uiLeds[15] = uiLedBright;
    }
    else
    {
      for(int i=0; i<16; ++i)
      {
        if(_P.arpVelocity <= vel[i]) {
          uiLeds[i] = uiLedBright;
          break;
        }
        uiLeds[i] = uiLedMedium;
      }
    }
  }
}
void editAccentVelocity(char keyPress, byte forceRefresh)
{
  byte vel[16] = {0,9,17,26,34,43,51,60,68,77,85,94,102,111,119,127};    
  if(keyPress >= 0 && keyPress <= 15)
  {    
    _P.arpAccentVelocity = vel[keyPress];
    forceRefresh = 1;
  }

  if(forceRefresh)
  {
    uiClearLeds();
    for(int i=15; i>=0; --i)
    {
      if(_P.arpAccentVelocity >= vel[i]) {
        uiLeds[i] = uiLedBright;
        break;
      }
      uiLeds[i] = uiLedMedium;
    }
  }
}

/////////////////////////////////////////////////////
// EDIT GATE LENGTH
void editGateLength(char keyPress, byte forceRefresh)
{
  if(keyPress >= 0 && keyPress <= 14)
  {    
    _P.arpGateLength = 10*(keyPress + 1);
    forceRefresh = 1;
  }
  else if(keyPress == 15)
  {
    _P.arpGateLength = 0;
    forceRefresh = 1;
  }
  if(forceRefresh)
  {
    uiClearLeds();
    if(_P.arpGateLength > 0)
    {      
      uiSetLeds(0, _P.arpGateLength/10, uiLedMedium);
      uiLeds[_P.arpGateLength/10 - 1] = uiLedBright;
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
  else if(7 == keyPress)
  {    
    midiOptions ^= MIDI_OPTS_LOCAL_OFF;
    eepromSet(EEPROM_MIDI_OPTS, midiOptions);    
    midiLocalOff(midiOptions & MIDI_OPTS_LOCAL_OFF);    
    forceRefresh = 1;
  }
  else if(8 == keyPress)
  {    
    midiOptions2 ^= MIDI_OPTS2_USE_NOTE_OFF;
    eepromSet(EEPROM_MIDI_OPTS2, midiOptions2);    
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
    uiLeds[7] = !!(midiOptions&MIDI_OPTS_LOCAL_OFF)? uiLedBright : uiLedDim;    
    uiLeds[8] = !!(midiOptions2&MIDI_OPTS2_USE_NOTE_OFF)? uiLedBright : uiLedDim;        
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
    _P.arpInsertMode = keyPress;
    arpFlags |= ARP_FLAG_REBUILD;
    forceRefresh = 1;
    break;
  case 10:
    _P.arpChordLength=2+random(3);
    for(i=0; i<_P.arpChordLength; ++i)
    {
      for(;;)
      {
        note = 48+random(12); 
        for(j = 0; j<i; ++j)
        {
          if(ARP_GET_NOTE(_P.arpChord[j]) == note)
            break;
        }
        if(j>=i)
          break;
      }           
      _P.arpChord[i] = ARP_MAKE_NOTE(note,64+random(64));
    }
    newChord = 1;
    break;
  case 11: // MIN7
    _P.arpChord[0] = ARP_MAKE_NOTE(48,127);       
    _P.arpChord[1] = ARP_MAKE_NOTE(51,127); 
    _P.arpChord[2] = ARP_MAKE_NOTE(55,127);
    _P.arpChord[3] = ARP_MAKE_NOTE(58,127);
    _P.arpChordLength = 4;
    newChord = 1;
    break;
  case 12: // MAJ7
    _P.arpChord[0] = ARP_MAKE_NOTE(48,127);       
    _P.arpChord[1] = ARP_MAKE_NOTE(52,127); 
    _P.arpChord[2] = ARP_MAKE_NOTE(55,127);
    _P.arpChord[3] = ARP_MAKE_NOTE(59,127);
    _P.arpChordLength = 4;
    newChord = 1;
    break;
  case 13: // DOM7
    _P.arpChord[0] = ARP_MAKE_NOTE(48,127);       
    _P.arpChord[1] = ARP_MAKE_NOTE(52,127); 
    _P.arpChord[2] = ARP_MAKE_NOTE(55,127);
    _P.arpChord[3] = ARP_MAKE_NOTE(58,127);
    _P.arpChordLength = 4;
    newChord = 1;
    break;
  case 14: // MIN
    _P.arpChord[0] = ARP_MAKE_NOTE(48,127);       
    _P.arpChord[1] = ARP_MAKE_NOTE(51,127);       
    _P.arpChord[2] = ARP_MAKE_NOTE(55,127);       
    _P.arpChordLength = 3;
    newChord = 1;
    break;
  case 15: // MAJ
    _P.arpChord[0] = ARP_MAKE_NOTE(48,127);       
    _P.arpChord[1] = ARP_MAKE_NOTE(52,127);       
    _P.arpChord[2] = ARP_MAKE_NOTE(55,127);       
    _P.arpChordLength = 3;
    newChord = 1;
    break;
  }

  if(newChord) {
    _P.arpManualChord = 0;
    arpFlags |= ARP_FLAG_REBUILD;
    uiHoldType |= UI_HOLD_CHORD;
  }
  if(forceRefresh)
  {
    uiClearLeds();
    uiSetLeds(0, 5, uiLedDim);
    uiLeds[_P.arpInsertMode] = uiLedBright;
    uiSetLeds(10, 6, uiLedMedium);
  }
}

/////////////////////////////////////////////////////
// MANUAL CHORD INPUT
void editManualChord(char keyPress, byte forceRefresh) 
{
  
  if(keyPress >= 0 && keyPress < 16) {
    _P.arpManualChord ^= (1<<keyPress);
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
          uiLeds[i] = (_P.arpManualChord & m) ? uiLedBright:uiLedMedium;
          break;
        default:
          uiLeds[i] = (_P.arpManualChord & m) ? uiLedBright:uiLedDim;
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
        _P.arpTransposeSequenceLen = 0;
        arpTransposeSequenceMask = 0;
        editFlags &= ~EDIT_FLAG_IS_NEW;
      }
      
      if(_P.arpTransposeSequenceLen < ARP_MAX_TRAN_SEQ - 1)      
      {
        arpTransposeSequenceMask |= (1<<keyPress);
        _P.arpTransposeSequence[_P.arpTransposeSequenceLen++] = keyPress - 3;
      }
    }
    else     
    {
      // pressing a note button while the transpose menu
      // button is not held will clear transpose sequence
      arpTransposeSequencePos = 0;
      _P.arpTransposeSequenceLen = 0;      
      arpTransposeSequenceMask = 0;
      _P.arpTranspose = keyPress - 3;
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

    if(_P.arpTranspose >= -3 && _P.arpTranspose < 13)
      uiLeds[_P.arpTranspose + 3] = uiLedBright;
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
      case 0: _P.arpForceToScaleMask |= ARP_SCALE_CHROMATIC; break;
      case 1: _P.arpForceToScaleMask &= ~ARP_SCALE_CHROMATIC; _P.arpForceToScaleMask |= ARP_SCALE_IONIAN; break;
      case 2: _P.arpForceToScaleMask &= ~ARP_SCALE_CHROMATIC; _P.arpForceToScaleMask |= ARP_SCALE_DORIAN; break;
      case 3: _P.arpForceToScaleMask &= ~ARP_SCALE_CHROMATIC; _P.arpForceToScaleMask |= ARP_SCALE_PHRYGIAN; break;
      case 4: _P.arpForceToScaleMask &= ~ARP_SCALE_CHROMATIC; _P.arpForceToScaleMask |= ARP_SCALE_LYDIAN; break;
      case 5: _P.arpForceToScaleMask &= ~ARP_SCALE_CHROMATIC; _P.arpForceToScaleMask |= ARP_SCALE_MIXOLYDIAN; break;
      case 6: _P.arpForceToScaleMask &= ~ARP_SCALE_CHROMATIC; _P.arpForceToScaleMask |= ARP_SCALE_AEOLIAN; break;
      case 7: _P.arpForceToScaleMask &= ~ARP_SCALE_CHROMATIC; _P.arpForceToScaleMask |= ARP_SCALE_LOCRIAN; break;
      
      case 11: _P.arpForceToScaleMask &= ~ARP_SCALE_ADJUST_MASK; _P.arpForceToScaleMask |= ARP_SCALE_ADJUST_SKIP; break;
      case 12: _P.arpForceToScaleMask &= ~ARP_SCALE_ADJUST_MASK; _P.arpForceToScaleMask |= ARP_SCALE_ADJUST_MUTE; break;
      case 13: _P.arpForceToScaleMask &= ~ARP_SCALE_ADJUST_MASK; _P.arpForceToScaleMask |= ARP_SCALE_ADJUST_FLAT; break;
      case 14: _P.arpForceToScaleMask &= ~ARP_SCALE_ADJUST_MASK; _P.arpForceToScaleMask |= ARP_SCALE_ADJUST_SHARP; break;
      case 15: _P.arpForceToScaleMask &= ~ARP_SCALE_ADJUST_MASK; _P.arpForceToScaleMask |= ARP_SCALE_ADJUST_TOGGLE; break;
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
    switch(_P.arpForceToScaleMask & ARP_SCALE_CHROMATIC)
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
    switch(_P.arpForceToScaleMask & ARP_SCALE_ADJUST_MASK)
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
    _P.arpForceToScaleRoot = keyPress;
    arpFlags |= ARP_FLAG_REBUILD;
    forceRefresh = 1;
  }

  if(forceRefresh)
  {
    uiClearLeds();
    uiSetLeds(0, 12, uiLedMedium);
    uiLeds[1] = uiLedDim;
    uiLeds[3] = uiLedDim;
    uiLeds[6] = uiLedDim;
    uiLeds[8] = uiLedDim;
    uiLeds[10] = uiLedDim;
    uiLeds[_P.arpForceToScaleRoot] = uiLedBright;
  }
}

/////////////////////////////////////////////////////
// LOAD/SAVE PATCHES
void editPatchAction(byte editMode, char keyPress, byte blinkState, byte forceRefresh)
{  
  byte ver;
  if(keyPress >= 0 && keyPress < 16)
  {
    byte ver;
    switch(editMode) {
    case EDIT_MODE_LOAD_PATCH:
      if(hhPatchQuery(keyPress)) {
        hhReadMemory(PATCH_ADDR(keyPress), (byte*)&_P, sizeof _P);
        arpReset();
        uiSetHold();
        editMode = EDIT_MODE_PATTERN;
        editPressType = EDIT_NO_PRESS;
        editFlags = EDIT_FLAG_FORCE_REFRESH;      
        arpFlags |= ARP_FLAG_REBUILD;
        synchFlags |= SYNCH_RESTART_ON_BEAT; 
      }
      break;      
    case EDIT_MODE_SAVE_PATCH:
      _P.ver = ARP_PATCH_VERSION;
      hhWriteMemory(PATCH_ADDR(keyPress), (byte*)&_P, sizeof _P);
      forceRefresh = 1;
      break;
    case EDIT_MODE_CLEAR_PATCH:
      ver=0;
      hhWriteMemory(PATCH_ADDR(keyPress), &ver, 1);
      forceRefresh = 1;
      break;      
    }
  }

  if(forceRefresh)
  {
    for(int i=0; i<16; ++i) {
      uiLeds[i] = hhPatchQuery(i)? uiLedMedium : uiLedDim;    
    }
  }
  else if(blinkState&1) {
    byte phase = !!(blinkState & 0xF0);
    for(int i=0; i<16; ++i) { 
      uiLeds[i] = (uiLeds[i] == uiLedDim)? uiLedDim : phase? uiLedBright : uiLedMedium;
    }
  }
}

/////////////////////////////////////////////////////
// EDIT RUN
void editRun(unsigned long milliseconds)
{
  byte forceRefresh = (editFlags & EDIT_FLAG_FORCE_REFRESH);
  editFlags &= ~EDIT_FLAG_FORCE_REFRESH;

  // blink state allows menu handlers to blink LEDs. 
  // bits 7-4 come from ms counter and bit 1 is set
  // each time 32ms has passed
  byte blinkState = (milliseconds&0xF0);
  if((blinkState ^ editBlinkState)&0xF0) {
    editBlinkState = milliseconds;
    blinkState |= 1;
  }

  // Capture any key pressed on the data entry keypad
  char dataKeyPress = uiDataKey;
  if(dataKeyPress != NO_VALUE)
  {
    // reset the timeout period after which the 
    // display will revert to the pattern view
    uiDataKey = NO_VALUE;
    editRevertTime = milliseconds + EDIT_REVERT_TIME;
  }

  // Capture any key pressed on the menu keypad  
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
    
    // check if HOLD is pressed
    if((uiHoldType & (UI_HOLD_PRESSED|UI_HOLD_HELD)) == UI_HOLD_PRESSED)
    {
      if(hhMode == HH_MODE_CVTAB || hhMode == HH_MODE_MEMOTAB) {
        // the hold button was pressed at the same time that a menu button was 
        // pressed, so the normal hold button action is cancelled and the hold
        // button can access shifted menu functions...      
        uiHoldType |= UI_HOLD_AS_SHIFT;      
        switch(menuKeyPress) {
          case EDIT_MODE_PATTERN:
            menuKeyPress = EDIT_MODE_LOAD_PATCH;
            break;
          case EDIT_MODE_PATTERN_LENGTH:
            menuKeyPress = EDIT_MODE_SAVE_PATCH;
            break;
          case EDIT_MODE_ARP_TYPE:
            menuKeyPress = EDIT_MODE_CLEAR_PATCH;
            break;
          default:
            menuKeyPress = 0;
            break;
        }
      }
    }   
     
    if(((menuKeyPress != NO_VALUE) && (menuKeyPress != editMode)) || 
      (EDIT_LONG_RELEASED == editPressType))
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
  if(editRevertTime > 0 && editRevertTime <= milliseconds)
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
    if(editPressType >= EDIT_LONG_HOLD)
      editAccentVelocity(dataKeyPress, forceRefresh);
    else
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
  case EDIT_MODE_LOAD_PATCH:
  case EDIT_MODE_SAVE_PATCH:
  case EDIT_MODE_CLEAR_PATCH:
    editPatchAction(editMode, dataKeyPress, blinkState, forceRefresh);
    break;
  default:
    editPattern(dataKeyPress, forceRefresh);
    break;   
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

  // load hack header mode
  hhMode = eepromGet(EEPROM_HH_MODE);   

  // Load user preferences  
  prefsInit();
  
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
  
  // Apply prefs
  prefsApply();
  
  // pressing hold switch at startup shows UI version
  if(uiShowVersion()) {
    void(*reboot)() = 0;      
    byte do_reboot= 0;
    
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
      eepromSet(EEPROM_MIDI_OPTS2, MIDI_OPTS2_DEFAULT_VALUE);
      eepromSet(EEPROM_SYNCH_SOURCE,0);
      eepromSet(EEPROM_SYNCH_SEND,0);  
      eepromSet(EEPROM_HH_MODE, HH_MODE_NONE); 
      prefsSave();
      arpOptionsSave();
      eepromSet(EEPROM_MAGIC_COOKIE,EEPROM_MAGIC_COOKIE_VALUE);  
      
      uiSetLeds(0, 16, uiLedBright);
      delay(1000);
      reboot();
    }  
    
    // pressing a data key when version is displayed allows hack header
    // mode to be changed
    if(uiDataKey >= HH_MODE_NONE && uiDataKey <= HH_MODE_MEMOTAB) {
      hhMode = uiDataKey;
      gPreferences &= ~PREF_HACKHEADER;
      eepromSet(EEPROM_HH_MODE, hhMode); 
    }
    if(uiDataKey != NO_VALUE) {    
      uiSetLeds(0, 16, 0);
      uiSetLeds(0, 5, uiLedMedium);
      uiSetLeds(hhMode, 1, uiLedBright);
      delay(1000);
      reboot();
    }
  }

  // init hack header
  hhInit();
  
  if(midiOptions & MIDI_OPTS_LOCAL_OFF) {
    midiLocalOff(true);
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
