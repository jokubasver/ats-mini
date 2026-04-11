// CW (Morse code) decoder
//
// Decodes CW from an audio signal sampled on GPIO11/IO11.
// Requires hardware modification: connect audio IC pin 8 to ESP32 IO11
// through a lowpass RC filter to remove Class-D PWM switching noise.
//
// Note: GPIO11 is on ADC2 of the ESP32-S3. ADC2 may produce unreliable
// readings when Wi-Fi is active; occasional decode errors are expected
// in that case.

#include "Common.h"
#include "CW.h"

// Number of ADC samples taken per cwTickTime() call
#define CW_NUM_SAMPLES   16

// Morse binary tree table size (covers up to 5-element codes)
#define MORSE_TREE_SIZE  64

// CW decoder state machine states
#define CW_IDLE   0   // Waiting for signal
#define CW_MARK   1   // Signal is present (key down / tone detected)
#define CW_SPACE  2   // Signal is absent (key up, timing the gap)

// Signal envelope thresholds (out of a 0..2047 amplitude range).
// Increase ON threshold if noise triggers false decoding;
// decrease it if weak signals are missed.
#define CW_THRESHOLD_ON   200  // Envelope level to declare mark active
#define CW_THRESHOLD_OFF   80  // Envelope level to declare mark inactive

// Timing parameters
#define CW_MIN_MARK_MS    10   // Minimum mark duration — rejects short noise spikes
#define CW_SILENCE_MS   3000   // Silence duration after which decoder resets to idle

// Initial dit-length estimate in milliseconds (15 WPM ≈ 80 ms/dit)
#define CW_DEFAULT_DIT_MS  80

// Morse code binary tree
// Navigate from the root (index 1) using:
//   dit → index * 2
//   dah → index * 2 + 1
// The character at the resulting index is the decoded letter/digit.
// This table covers the standard ITU Morse alphabet (A-Z), digits (0-9)
// and a handful of common punctuation characters.
static const char morseTable[MORSE_TREE_SIZE] =
{
  0,    // 0: unused
  0,    // 1: root
  'E',  // 2: .
  'T',  // 3: -
  'I',  // 4: ..
  'A',  // 5: .-
  'N',  // 6: -.
  'M',  // 7: --
  'S',  // 8: ...
  'U',  // 9: ..-
  'R',  // 10: .-.
  'W',  // 11: .--
  'D',  // 12: -..
  'K',  // 13: -.-
  'G',  // 14: --.
  'O',  // 15: ---
  'H',  // 16: ....
  'V',  // 17: ...-
  'F',  // 18: ..-.
  0,    // 19: ..-- (Ü)
  'L',  // 20: .-..
  0,    // 21: .-.- (Ä)
  'P',  // 22: .--.
  'J',  // 23: .---
  'B',  // 24: -...
  'X',  // 25: -..-
  'C',  // 26: -.-.
  'Y',  // 27: -.--
  'Z',  // 28: --..
  'Q',  // 29: --.-
  0,    // 30: ---. (Ö)
  0,    // 31: ---- (CH)
  '5',  // 32: .....
  '4',  // 33: ....-
  0,    // 34: ...-. (&)
  '3',  // 35: ...--
  0,    // 36: ..-..
  0,    // 37: ..-.-
  0,    // 38: ..--. (?)
  '2',  // 39: ..---
  0,    // 40: .-...
  0,    // 41: .-..-
  '+',  // 42: .-.-. (AR)
  0,    // 43: .-.- (extended)
  0,    // 44: .--..
  0,    // 45: .--.-
  0,    // 46: .---.
  '1',  // 47: .----
  '6',  // 48: -....
  '=',  // 49: -...- (BT)
  '/',  // 50: -..-.
  0,    // 51: -..--
  0,    // 52: -.-..
  0,    // 53: -.-.- (KA)
  0,    // 54: -.--. (?)
  0,    // 55: -.---
  '7',  // 56: --...
  0,    // 57: --..-
  0,    // 58: --.-.
  0,    // 59: --.-- (Ñ)
  '8',  // 60: ---..
  0,    // 61: ---.-
  '9',  // 62: ----.
  '0',  // 63: -----
};

// -----------------------------------------------------------------------
// State variables
// -----------------------------------------------------------------------

static uint8_t  cwState = CW_IDLE;
static uint32_t cwStateMs = 0;
static bool     cwLetterDecoded = false;
static uint8_t  cwCode = 1;               // Morse tree index (1 = root)
static uint32_t cwDitLen = CW_DEFAULT_DIT_MS;  // Adaptive dit estimate (ms)

// Hysteresis state for mark/space detection
static bool     cwMarkActive = false;

// Long-term DC bias of the ADC input (12-bit, nominally 2048)
static int32_t  cwDcBias = 2048;

// Decoded text ring buffer
static char    cwText[CW_TEXT_LEN + 1] = "";
static uint8_t cwTextLen = 0;

// -----------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------

// Append a character to the text buffer.  When full, drop the oldest word.
static void cwAddChar(char c)
{
  if(cwTextLen < CW_TEXT_LEN)
  {
    cwText[cwTextLen++] = c;
    cwText[cwTextLen]   = '\0';
    return;
  }

  // Buffer full — remove characters up to and including the first space
  int i = 0;
  while(i < cwTextLen && cwText[i] != ' ') i++;
  if(i < cwTextLen)
  {
    memmove(cwText, cwText + i + 1, cwTextLen - i);
    cwTextLen -= i + 1;
  }
  else
  {
    // No space found — drop the first half of the buffer
    cwTextLen /= 2;
    memmove(cwText, cwText + cwTextLen, cwTextLen + 1);
  }

  if(cwTextLen < CW_TEXT_LEN)
  {
    cwText[cwTextLen++] = c;
    cwText[cwTextLen]   = '\0';
  }
}

// Look up the current Morse tree index, emit the character and reset
static bool cwDecodeChar(void)
{
  bool emitted = false;
  if(cwCode >= 2 && cwCode < MORSE_TREE_SIZE)
  {
    char c = morseTable[cwCode];
    if(c)
    {
      cwAddChar(c);
      emitted = true;
    }
  }
  cwCode = 1;  // Reset to root
  return emitted;
}

// Take CW_NUM_SAMPLES ADC readings, update the DC bias estimate, and
// return the mean absolute deviation from DC as the signal envelope.
static uint16_t cwSampleLevel(void)
{
  int32_t sum    = 0;
  int32_t absSum = 0;

  for(int i = 0; i < CW_NUM_SAMPLES; i++)
  {
    int32_t s = analogRead(CW_ADC_PIN);
    sum += s;
    int32_t diff = s - cwDcBias;
    absSum += (diff < 0 ? -diff : diff);
  }

  // Slowly track DC bias (EMA, time constant ≈ 64 calls).
  // Compute error without intermediate integer division to reduce quantization.
  cwDcBias += ((sum - cwDcBias * CW_NUM_SAMPLES) / CW_NUM_SAMPLES) >> 6;

  return (uint16_t)(absSum / CW_NUM_SAMPLES);
}

// -----------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------

void cwInit(void)
{
  // Set full 3.3 V range on the CW ADC pin
  analogSetPinAttenuation(CW_ADC_PIN, ADC_11db);

  cwState        = CW_IDLE;
  cwStateMs      = millis();
  cwCode         = 1;
  cwDitLen       = CW_DEFAULT_DIT_MS;
  cwMarkActive   = false;
  cwDcBias       = 2048;
  cwTextLen      = 0;
  cwText[0]      = '\0';
  cwLetterDecoded = false;
}

// Called every main-loop iteration.  Returns true when the decoded text
// buffer changes so the caller can request a screen redraw.
bool cwTickTime(void)
{
  uint32_t now     = millis();
  bool     changed = false;

  // Sample ADC and apply hysteresis to produce a clean mark/space signal
  uint16_t level = cwSampleLevel();
  if(!cwMarkActive && level > CW_THRESHOLD_ON)  cwMarkActive = true;
  if( cwMarkActive && level < CW_THRESHOLD_OFF) cwMarkActive = false;

  bool isMark = cwMarkActive;

  switch(cwState)
  {
    // -----------------------------------------------------------------
    case CW_IDLE:
      if(isMark)
      {
        cwState        = CW_MARK;
        cwStateMs      = now;
        cwCode         = 1;
        cwLetterDecoded = false;
      }
      break;

    // -----------------------------------------------------------------
    case CW_MARK:
      if(!isMark)
      {
        uint32_t dur = now - cwStateMs;

        if(dur >= (uint32_t)CW_MIN_MARK_MS && cwCode > 0)
        {
          if(dur < cwDitLen * 2)
          {
            // Dit — navigate to left child
            if(cwCode * 2 < MORSE_TREE_SIZE) cwCode *= 2;
            else                             cwCode  = 0;  // Overflow: too many elements

            // Update dit-length estimate (EMA α ≈ 0.25)
            cwDitLen = (cwDitLen * 3 + dur) / 4;
          }
          else
          {
            // Dah — navigate to right child
            if(cwCode * 2 + 1 < MORSE_TREE_SIZE) cwCode = cwCode * 2 + 1;
            else                                  cwCode = 0;  // Overflow

            // Estimate dit from dah duration (standard ratio = 3:1)
            cwDitLen = (cwDitLen * 3 + dur / 3) / 4;
          }

          // Clamp to [20 ms, 500 ms]  ≈  60 WPM … 2.4 WPM
          if(cwDitLen < 20)  cwDitLen = 20;
          if(cwDitLen > 500) cwDitLen = 500;
        }

        cwState   = CW_SPACE;
        cwStateMs = now;
      }
      break;

    // -----------------------------------------------------------------
    case CW_SPACE:
      if(isMark)
      {
        // New mark — start accumulating the next element
        cwState        = CW_MARK;
        cwStateMs      = now;
        cwLetterDecoded = false;
      }
      else
      {
        uint32_t spaceDur = now - cwStateMs;

        // Letter gap: space duration > 2 dits
        if(!cwLetterDecoded && spaceDur > cwDitLen * 2)
        {
          changed |= cwDecodeChar();
          cwLetterDecoded = true;
        }

        // Word gap: space duration > 5 dits
        if(cwLetterDecoded && spaceDur > cwDitLen * 5)
        {
          if(cwTextLen > 0 && cwText[cwTextLen - 1] != ' ')
          {
            cwAddChar(' ');
            changed = true;
          }
          cwState   = CW_IDLE;
          cwStateMs = now;
        }

        // Long silence: decode any pending character and return to idle
        if(spaceDur > CW_SILENCE_MS)
        {
          if(!cwLetterDecoded && cwCode > 1)
            changed |= cwDecodeChar();

          cwState   = CW_IDLE;
          cwStateMs = now;
          cwCode    = 1;
        }
      }
      break;
  }

  return changed;
}

const char *getCwText(void)
{
  return cwText;
}

void clearCwText(void)
{
  cwTextLen      = 0;
  cwText[0]      = '\0';
  cwState        = CW_IDLE;
  cwStateMs      = millis();
  cwCode         = 1;
  cwLetterDecoded = false;
}
