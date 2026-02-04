#include <PDM.h>
#include <math.h>
#include <stdint.h>

#define pdmDataBufferSize 4096
uint16_t pdmDataBuffer[pdmDataBufferSize];
uint32_t sampleFreq;

AP3_PDM myPDM;

// Targets
static const float NOTE_C3 = 130.813f;
static const float NOTE_G3 = 196.000f;
static const float NOTE_D4 = 293.665f;

// Per-note absolute thresholds (tune these)
static const float ABS_C3 =  80000.0f;   // C3 fundamental often weaker
static const float ABS_G3 = 250000.0f;
static const float ABS_D4 = 250000.0f;

// Dominance rules (used when C3 not present)
static const float DOMINANCE_THRESH = 0.55f;
static const float SECOND_RATIO     = 1.35f;

// Debounce any-note state
static const uint8_t HOLD_FRAMES    = 2;
static const uint8_t RELEASE_FRAMES = 6;

// Note lock hysteresis (prevents bouncing)
static const uint8_t NOTE_LOCK_FRAMES = 12;

// Debug
#define DEBUG 0

// Debounce state (for "anything detected?")
struct NoteState {
  uint8_t attack;
  uint8_t release;
  bool on;
};

static void initNoteState(struct NoteState* s)
{
  s->attack = 0;
  s->release = 0;
  s->on = false;
}

static bool update_on(struct NoteState* s, bool hit)
{
  if (hit)
  {
    if (s->attack < HOLD_FRAMES) s->attack++;
    if (s->attack >= HOLD_FRAMES) s->on = true;
    s->release = RELEASE_FRAMES;
  }
  else
  {
    s->attack = 0;
    if (s->on)
    {
      if (s->release > 0) s->release--;
      if (s->release == 0) s->on = false;
    }
  }
  return s->on;
}

// Goertzel at exact frequency (no integer bin rounding)
static float goertzel_power_exact(const int16_t* x, uint32_t N, float targetHz, float fs)
{
  float w = 2.0f * (float)M_PI * targetHz / fs;
  float cosine = cosf(w);
  float coeff  = 2.0f * cosine;

  float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f;
  for (uint32_t i = 0; i < N; i++)
  {
    q0 = coeff * q1 - q2 + (float)x[i];
    q2 = q1;
    q1 = q0;
  }
  return q1*q1 + q2*q2 - q1*q2*coeff;
}

static void computeSampleFreq()
{
  uint32_t PDMClk = 0, MClkDiv = 0;

  switch (myPDM.getClockDivider())
  {
    case AM_HAL_PDM_MCLKDIV_4: MClkDiv = 4; break;
    case AM_HAL_PDM_MCLKDIV_3: MClkDiv = 3; break;
    case AM_HAL_PDM_MCLKDIV_2: MClkDiv = 2; break;
    case AM_HAL_PDM_MCLKDIV_1: MClkDiv = 1; break;
    default: MClkDiv = 0; break;
  }

  switch (myPDM.getClockSpeed())
  {
    case AM_HAL_PDM_CLK_12MHZ:    PDMClk = 12000000; break;
    case AM_HAL_PDM_CLK_6MHZ:     PDMClk = 6000000;  break;
    case AM_HAL_PDM_CLK_3MHZ:     PDMClk = 3000000;  break;
    case AM_HAL_PDM_CLK_1_5MHZ:   PDMClk = 1500000;  break;
    case AM_HAL_PDM_CLK_750KHZ:   PDMClk = 750000;   break;
    case AM_HAL_PDM_CLK_375KHZ:   PDMClk = 375000;   break;
    case AM_HAL_PDM_CLK_187KHZ:   PDMClk = 187000;   break;
    default: PDMClk = 0; break;
  }

  sampleFreq = (PDMClk / (MClkDiv * 2 * myPDM.getDecimationRate()));
}

// Use plain ints instead of enums to dodge Arduino auto-prototype problems
static const uint8_t NOTE_NONE = 0;
static const uint8_t NOTE_C3_ID = 1;
static const uint8_t NOTE_G3_ID = 2;
static const uint8_t NOTE_D4_ID = 3;

static struct NoteState anyNote;
static uint8_t lockedNote = NOTE_NONE;
static uint8_t lockTimer = 0;
static uint8_t lastPrinted = NOTE_NONE;

void setup()
{
  Serial.begin(115200);

  if (!myPDM.begin())
  {
    Serial.println("PDM Init failed.");
    while (1) ;
  }

  computeSampleFreq();
  initNoteState(&anyNote);
}

void loop()
{
  if (!myPDM.available())
  {
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    return;
  }

  myPDM.getData(pdmDataBuffer, pdmDataBufferSize);
  int16_t* x = (int16_t*)pdmDataBuffer;

  // Remove DC
  int32_t sum = 0;
  for (uint32_t i = 0; i < pdmDataBufferSize; i++) sum += x[i];
  int16_t mean = (int16_t)(sum / (int32_t)pdmDataBufferSize);
  for (uint32_t i = 0; i < pdmDataBufferSize; i++) x[i] -= mean;

  float fs = (float)sampleFreq;

  float pC3 = goertzel_power_exact(x, pdmDataBufferSize, NOTE_C3, fs);
  float pG3 = goertzel_power_exact(x, pdmDataBufferSize, NOTE_G3, fs);
  float pD4 = goertzel_power_exact(x, pdmDataBufferSize, NOTE_D4, fs);

  bool presC3 = (pC3 > ABS_C3);
  bool presG3 = (pG3 > ABS_G3);
  bool presD4 = (pD4 > ABS_D4);

  // Candidate selection:
  // - If C3 present, force C3 (prevents harmonics from stealing it)
  // - Else choose dominant strongest note among G3/D4 (using 3-note dominance)
  uint8_t candidate = NOTE_NONE;

  if (presC3)
  {
    candidate = NOTE_C3_ID;
  }
  else
  {
    // Find strongest and second strongest among the three
    float p1 = pC3; uint8_t best = NOTE_C3_ID;
    float p2 = pG3; uint8_t mid  = NOTE_G3_ID;
    float p3 = pD4; uint8_t low  = NOTE_D4_ID;

    if (p1 < p2) { float t=p1; p1=p2; p2=t; uint8_t nt=best; best=mid; mid=nt; }
    if (p2 < p3) { float t=p2; p2=p3; p3=t; uint8_t nt=mid;  mid=low;  low=nt; }
    if (p1 < p2) { float t=p1; p1=p2; p2=t; uint8_t nt=best; best=mid; mid=nt; }

    float sum3 = pC3 + pG3 + pD4 + 1.0f;
    float dom = p1 / sum3;

    bool okAbs = false;
    if (best == NOTE_G3_ID) okAbs = presG3;
    else if (best == NOTE_D4_ID) okAbs = presD4;
    else okAbs = false; // ignore C3 here; presC3 was false

    bool ok = okAbs && (dom > DOMINANCE_THRESH) && (p1 > SECOND_RATIO * p2);
    if (ok) candidate = best;
  }

#if DEBUG
  Serial.print("pC3="); Serial.print(pC3);
  Serial.print(" pG3="); Serial.print(pG3);
  Serial.print(" pD4="); Serial.print(pD4);
  Serial.print(" | presC3="); Serial.print(presC3);
  Serial.print(" presG3="); Serial.print(presG3);
  Serial.print(" presD4="); Serial.print(presD4);
  Serial.print(" | cand="); Serial.println(candidate);
#endif

  bool on = update_on(&anyNote, candidate != NOTE_NONE);

  // Locking: once a note is chosen, keep it for NOTE_LOCK_FRAMES
  if (on && candidate != NOTE_NONE)
  {
    if (lockedNote == NOTE_NONE || lockTimer == 0)
    {
      lockedNote = candidate;
      lockTimer = NOTE_LOCK_FRAMES;
    }
    else
    {
      if (candidate == lockedNote)
      {
        lockTimer = NOTE_LOCK_FRAMES; // refresh
      }
      // else: ignore candidate until timer expires
    }
  }
  else
  {
    lockedNote = NOTE_NONE;
    lockTimer = 0;
  }

  if (lockTimer > 0) lockTimer--;

  uint8_t toPrint = (on ? lockedNote : NOTE_NONE);

  // Print only on change
  if (toPrint != lastPrinted)
  {
    lastPrinted = toPrint;

    if (toPrint == NOTE_C3_ID) Serial.println("C3 detected");
    else if (toPrint == NOTE_G3_ID) Serial.println("G3 detected");
    else if (toPrint == NOTE_D4_ID) Serial.println("D4 detected");
    // NOTE_NONE prints nothing
  }

  am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
}
