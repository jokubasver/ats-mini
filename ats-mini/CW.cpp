// CW (Morse code) decoder
//
// Decodes CW from an audio signal sampled on GPIO11/IO11.
// Uses the Goertzel algorithm for frequency-selective tone detection.
// Six Goertzel bins run in parallel covering 400–900 Hz so that any
// typical SSB CW sidetone is detected regardless of the exact dial
// offset, giving ±300 Hz of tolerance around the 700 Hz centre.
//
// Hardware modification: connect NS4160 amplifier IC pin 8 (OUTP) to
// ESP32 IO11 through a lowpass RC filter (1 kΩ series + 100 nF to GND).
// The filter cutoff at ~1.6 kHz passes the audio while attenuating the
// Class-D PWM carrier.
//
// GPIO11 is on ADC2 of the ESP32-S3.  Unlike the original ESP32, ADC2
// on the ESP32-S3 is fully independent of Wi-Fi and may be used freely.

#include "Common.h"
#include "CW.h"
#include <math.h>

// Morse binary tree table size (covers up to 5-element codes)
#define MORSE_TREE_SIZE  64

// CW decoder state machine states
#define CW_IDLE   0   // Waiting for signal
#define CW_MARK   1   // Signal is present (key down / tone detected)
#define CW_SPACE  2   // Signal is absent (key up, timing the gap)

// -----------------------------------------------------------------------
// Goertzel tone detector parameters
// -----------------------------------------------------------------------
//
// Samples the ADC at CW_SAMPLE_RATE Hz and feeds blocks of CW_GOERTZEL_N
// samples into CW_NUM_BINS parallel Goertzel filters, each tuned to a
// different bin within the typical SSB CW audio passband.
//
// Bin spacing = fs / N = 4000 / 40 = 100 Hz
// Bins k = 4…9 correspond to 400, 500, 600, 700, 800, 900 Hz.
// This covers the full range of CW sidetones encountered when operating
// USB/LSB without a precisely set carrier offset.
//
// A mark is declared when any single bin's magnitude² exceeds the ON
// threshold; a space when all bins fall below the OFF threshold.
//
// Each block of 40 samples at 4000 Hz covers 10 ms — well below the
// shortest dit at any practical CW speed.

#define CW_SAMPLE_RATE      4000   // ADC sample rate in Hz
#define CW_SAMPLE_US         250   // Sample interval in µs (= 1 000 000 / CW_SAMPLE_RATE)
#define CW_GOERTZEL_N         40   // Samples per Goertzel block
#define CW_GOERTZEL_K_MIN      4   // Lowest bin  → k * fs / N = 4 * 4000 / 40 = 400 Hz
#define CW_GOERTZEL_K_MAX      9   // Highest bin → k * fs / N = 9 * 4000 / 40 = 900 Hz
#define CW_NUM_BINS            (CW_GOERTZEL_K_MAX - CW_GOERTZEL_K_MIN + 1)  // 6 bins

// Maximum number of ADC samples to read per cwTickTime() call.
// 80 samples = 20 ms at 4 kHz — enough to process one full Goertzel block
// per main-loop iteration without blocking the loop for more than ~2.4 ms.
#define CW_MAX_BATCH  80

// Goertzel magnitude² thresholds for mark/space detection.
// The Goertzel magnitude² for a pure sine at a bin frequency with
// ADC amplitude A (12-bit, 0–4095) is approximately (N/2 * A)² = (20*A)².
// So CW_THRESHOLD_ON²  =  800² =   640 000 corresponds to A ≈ 40 counts (≈ 32 mV peak).
//    CW_THRESHOLD_OFF² =  320² =   102 400 corresponds to A ≈ 16 counts (≈ 13 mV peak).
// These thresholds apply to each individual bin; a mark is declared when
// ANY bin exceeds CW_THRESHOLD_ON2.
// The ON threshold is set slightly lower than 1000² to improve sensitivity
// for tones that fall between bin centres (worst-case inter-bin response is
// ~64 % of peak magnitude²).
// Increase CW_THRESHOLD_ON if noise triggers false decoding;
// decrease it if weak signals are missed.
#define CW_THRESHOLD_ON2   (800.0f * 800.0f)  // Magnitude² to declare tone present
#define CW_THRESHOLD_OFF2  (320.0f * 320.0f)  // Magnitude² to declare tone absent

// Timing parameters
#define CW_MIN_MARK_MS    10   // Minimum mark duration — rejects short noise spikes
#define CW_SILENCE_MS   3000   // Silence duration after which decoder resets to idle
#define CW_MIN_DIT_MS     20   // Minimum adaptive dit length (≈ 60 WPM)
#define CW_MAX_DIT_MS    500   // Maximum adaptive dit length (≈ 2.4 WPM)

// Initial dit-length estimate in milliseconds (15 WPM ≈ 80 ms/dit)
#define CW_DEFAULT_DIT_MS   80

// After this many milliseconds of continuous silence the dit-length estimate
// is reset to the default so that a speed change is not penalised by a stale
// estimate from a previous transmission.
#define CW_DIT_RESET_MS  10000

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

// Goertzel filter state (one entry per bin)
static float    cwGQ1[CW_NUM_BINS];     // s[n-1] for each bin
static float    cwGQ2[CW_NUM_BINS];     // s[n-2] for each bin
static int      cwGCount = 0;           // Sample count in current block
static float    cwGCoeff[CW_NUM_BINS];  // 2 * cos(2π * k / N) for each bin, computed in cwInit()

// Long-term DC bias of the ADC input (12-bit, nominally 2048)
static int32_t  cwDcBias = 2048;

// Decoded text ring buffer
static char    cwText[CW_TEXT_LEN + 1] = "";
static uint8_t cwTextLen = 0;

// -----------------------------------------------------------------------
// ADC sampling — done inline in cwTickTime() to avoid FreeRTOS task pressure.
//
// cwTickTime() is called every main-loop iteration.  It calculates how many
// 4 kHz sample periods have elapsed since the last call and reads that many
// ADC samples directly via analogRead(), up to CW_MAX_BATCH samples.
// This keeps all ADC work on the Arduino loop task and avoids any interaction
// with the FreeRTOS scheduler or watchdog timer.
// -----------------------------------------------------------------------

static uint32_t cwLastSampleUs = 0;  // micros() timestamp of last sample batch

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

// -----------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------

void cwInit(void)
{
  // Precompute Goertzel coefficients: 2 * cos(2π * k / N) for each bin
  for(int i = 0; i < CW_NUM_BINS; i++)
  {
    int k = CW_GOERTZEL_K_MIN + i;
    cwGCoeff[i] = 2.0f * cosf(2.0f * (float)M_PI * k / (float)CW_GOERTZEL_N);
    cwGQ1[i]    = 0.0f;
    cwGQ2[i]    = 0.0f;
  }

  cwState        = CW_IDLE;
  cwStateMs      = millis();
  cwCode         = 1;
  cwDitLen       = CW_DEFAULT_DIT_MS;
  cwMarkActive   = false;
  cwDcBias       = 2048;
  cwGCount       = 0;
  cwTextLen      = 0;
  cwText[0]      = '\0';
  cwLetterDecoded = false;

  // Warm up the ADC: configure attenuation and discard the first conversion,
  // which often returns an out-of-range value on ESP32-S3.
  analogSetPinAttenuation(CW_ADC_PIN, ADC_11db);
  analogRead(CW_ADC_PIN);

  // Capture the current time so that cwTickTime() starts sampling immediately.
  cwLastSampleUs = micros();
}

// Called every main-loop iteration.  Returns true when the decoded text
// buffer changes so the caller can request a screen redraw.
bool cwTickTime(void)
{
  uint32_t now     = millis();
  bool     changed = false;

  // -----------------------------------------------------------------------
  // Read ADC samples inline — no FreeRTOS task, no hardware timer, no ISR.
  //
  // Calculate how many 4 kHz sample periods (250 µs each) have elapsed since
  // the last batch, read that many samples directly via analogRead(), then
  // advance cwLastSampleUs by exactly that many periods so that jitter in
  // the main-loop call rate does not accumulate error.
  //
  // The batch is capped at CW_MAX_BATCH (80 samples = 20 ms) so that even
  // after a long display refresh the loop does not block for more than ~2 ms
  // of ADC reads.  Dropped samples cannot be recovered, but missing a few
  // samples during screen updates has no audible or functional impact on CW
  // decoding — dits and dahs are 20–500 ms long.
  // -----------------------------------------------------------------------
  uint32_t nowUs    = micros();
  uint32_t elapsedUs = nowUs - cwLastSampleUs;
  uint32_t numSamples = elapsedUs / CW_SAMPLE_US;
  if(numSamples > CW_MAX_BATCH) numSamples = CW_MAX_BATCH;

  for(uint32_t s = 0; s < numSamples; s++)
  {
    int32_t raw = (int32_t)analogRead(CW_ADC_PIN);

    // Remove slowly-tracked DC offset.
    // α = 1/256 gives a time constant of ~256 samples = 64 ms at 4 kHz,
    // long enough to track slow DC drift without following the audio signal.
    cwDcBias += (raw - cwDcBias) >> 8;
    float x = (float)(raw - cwDcBias);

    // Goertzel IIR iteration for each bin: s[n] = x[n] + coeff * s[n-1] - s[n-2]
    for(int i = 0; i < CW_NUM_BINS; i++)
    {
      float q0  = x + cwGCoeff[i] * cwGQ1[i] - cwGQ2[i];
      cwGQ2[i]  = cwGQ1[i];
      cwGQ1[i]  = q0;
    }

    if(++cwGCount >= CW_GOERTZEL_N)
    {
      // Block complete.  Find the maximum magnitude² across all bins.
      // |X[k]|² = s²[N-1] + s²[N-2] - coeff * s[N-1] * s[N-2]
      // Compare against squared thresholds to avoid sqrtf().
      float maxMag2 = 0.0f;
      for(int i = 0; i < CW_NUM_BINS; i++)
      {
        float mag2 = cwGQ1[i]*cwGQ1[i] + cwGQ2[i]*cwGQ2[i]
                     - cwGCoeff[i] * cwGQ1[i] * cwGQ2[i];
        if(mag2 > maxMag2) maxMag2 = mag2;
        cwGQ1[i] = cwGQ2[i] = 0.0f;
      }
      cwGCount = 0;

      // Update mark/space with hysteresis using magnitude² thresholds.
      // A mark fires when ANY bin exceeds the ON threshold.
      if(!cwMarkActive && maxMag2 > CW_THRESHOLD_ON2)  cwMarkActive = true;
      if( cwMarkActive && maxMag2 < CW_THRESHOLD_OFF2) cwMarkActive = false;
    }
  }

  // Advance the sample timestamp by exactly the number of samples consumed
  // so that fractional periods carry forward to the next call.
  cwLastSampleUs += numSamples * CW_SAMPLE_US;

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
      else if((now - cwStateMs) > CW_DIT_RESET_MS)
      {
        // Long silence: reset the adaptive dit-length estimate so that a
        // new transmission at a different speed gets a fair start.
        cwDitLen  = CW_DEFAULT_DIT_MS;
        cwStateMs = now;  // Restart the idle timer to avoid repeated resets
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

            // Update dit-length estimate (EMA α ≈ 0.125 — slower than 0.25
            // to keep the estimate stable for a consistent sender)
            cwDitLen = (cwDitLen * 7 + dur) / 8;
          }
          else
          {
            // Dah — navigate to right child
            if(cwCode * 2 + 1 < MORSE_TREE_SIZE) cwCode = cwCode * 2 + 1;
            else                                  cwCode = 0;  // Overflow

            // Estimate dit from dah duration (standard ratio = 3:1)
            cwDitLen = (cwDitLen * 7 + dur / 3) / 8;
          }

          // Clamp to [CW_MIN_DIT_MS, CW_MAX_DIT_MS]
          if(cwDitLen < CW_MIN_DIT_MS) cwDitLen = CW_MIN_DIT_MS;
          if(cwDitLen > CW_MAX_DIT_MS) cwDitLen = CW_MAX_DIT_MS;
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
