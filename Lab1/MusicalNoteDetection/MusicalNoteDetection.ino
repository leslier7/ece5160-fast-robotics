/* Author: Nathan Seidle
  Created: July 24, 2019

  Modified by Robbie Leslie 2026

  This example demonstrates how to use the pulse density microphone (PDM) on Artemis boards.
  This library and example are heavily based on the Apollo3 pdm_fft example.
*/

/* 
// This file is subject to the terms and conditions defined in
// file 'LICENSE.md', which is part of this source code package.
*/

//Global variables needed for PDM library
#define pdmDataBufferSize 4096 //Default is array of 4096 * 32bit
uint16_t pdmDataBuffer[pdmDataBufferSize];

//Global variables needed for the FFT in this sketch
float g_fPDMTimeDomain[pdmDataBufferSize * 2];
float g_fPDMFrequencyDomain[pdmDataBufferSize * 2];
float g_fPDMMagnitudes[pdmDataBufferSize * 2];
uint32_t sampleFreq;

//Enable these defines for additional debug printing
#define PRINT_PDM_DATA 0
#define PRINT_FFT_DATA 0

// Target note frequencies (Hz)
static const float NOTE_C3 = 130.813f;
static const float NOTE_D3 = 146.832f;
static const float NOTE_E3 = 164.814f;

// How wide (in FFT bins) to search around the target frequency.
// 2–5 is typical. Larger = more tolerant, but more false positives.
static const uint32_t NOTE_BIN_RADIUS = 3;

// How far above noise floor a peak must be to count as "present".
static const float PRESENCE_RATIO = 6.0f;  // tweak 3–15 depending on environment


// Clamp helper
static inline uint32_t clamp_u32(int32_t v, uint32_t lo, uint32_t hi)
{
  if (v < (int32_t)lo) return lo;
  if (v > (int32_t)hi) return hi;
  return (uint32_t)v;
}

#include <PDM.h> //Include PDM library included with the Aruino_Apollo3 core
AP3_PDM myPDM;   //Create instance of PDM class

//Math library needed for FFT
#include <arm_math.h>

void setup()
{
  Serial.begin(115200);
  Serial.println("SparkFun PDM Example");

  if (myPDM.begin() == false) // Turn on PDM with default settings, start interrupts
  {
    Serial.println("PDM Init failed. Are you sure these pins are PDM capable?");
    while (1)
      ;
  }
  Serial.println("PDM Initialized");

  printPDMConfig();
}

void loop()
{
  if (myPDM.available())
  {
    myPDM.getData(pdmDataBuffer, pdmDataBufferSize);

    //printLoudest();
    printNotes();
  }

  // Go to Deep Sleep until the PDM ISR or other ISR wakes us.
  am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
}

// AI CODE BEGIN

// Compute a simple noise floor estimate from the magnitudes.
// Skips DC and very low bins where bias/rumble lives.
static float estimate_noise_floor(const float* mags, uint32_t nBins)
{
  const uint32_t start = 5;             // skip DC..low bins
  const uint32_t end   = nBins / 2;     // only up to Nyquist
  float sum = 0.0f;
  uint32_t count = 0;

  for (uint32_t i = start; i < end; i++)
  {
    sum += mags[i];
    count++;
  }

  // Mean magnitude as a rough noise floor
  return (count > 0) ? (sum / (float)count) : 0.0f;
}

// Find the peak magnitude in a band around target frequency.
// Returns peak magnitude, and optionally the bin of that peak.
static float peak_in_band(const float* mags,
                          float targetHz,
                          uint32_t* outPeakBin)
{
  const float binHz = (float)sampleFreq / (float)pdmDataBufferSize;

  // Convert frequency to nearest FFT bin
  int32_t centerBin = (int32_t)(targetHz / binHz + 0.5f);

  // Limit search to valid bins (1..N/2-1)
  int32_t lo = centerBin - (int32_t)NOTE_BIN_RADIUS;
  int32_t hi = centerBin + (int32_t)NOTE_BIN_RADIUS;

  uint32_t nHalf = pdmDataBufferSize / 2;
  uint32_t loU = clamp_u32(lo, 1, nHalf - 1);
  uint32_t hiU = clamp_u32(hi, 1, nHalf - 1);

  float peak = 0.0f;
  uint32_t peakBin = loU;

  for (uint32_t b = loU; b <= hiU; b++)
  {
    float v = mags[b];
    if (v > peak)
    {
      peak = v;
      peakBin = b;
    }
  }

  if (outPeakBin) *outPeakBin = peakBin;
  return peak;
}

// AI CODE END

// Find three Notes (C3, D, E) and prints them to the screen
void printNotes()
{
  int16_t *pi16PDMData = (int16_t *)pdmDataBuffer;

  // 1) Copy samples into complex FFT input (real, imag=0)
  for (uint32_t i = 0; i < pdmDataBufferSize; i++)
  {
    g_fPDMTimeDomain[2 * i]     = (float)pi16PDMData[i];
    g_fPDMTimeDomain[2 * i + 1] = 0.0f;
  }

  // Optional but recommended: remove DC offset to avoid bin0 dominance
  // Compute mean and subtract from real parts only
  float mean = 0.0f;
  for (uint32_t i = 0; i < pdmDataBufferSize; i++)
    mean += g_fPDMTimeDomain[2 * i];
  mean /= (float)pdmDataBufferSize;
  for (uint32_t i = 0; i < pdmDataBufferSize; i++)
    g_fPDMTimeDomain[2 * i] -= mean;

  // 2) FFT -> magnitudes
  arm_cfft_radix4_instance_f32 S;
  arm_cfft_radix4_init_f32(&S, pdmDataBufferSize, 0, 1);
  arm_cfft_radix4_f32(&S, g_fPDMTimeDomain);
  arm_cmplx_mag_f32(g_fPDMTimeDomain, g_fPDMMagnitudes, pdmDataBufferSize);

  // 3) Estimate noise floor and check note bands
  float noise = estimate_noise_floor(g_fPDMMagnitudes, pdmDataBufferSize);

  uint32_t binC, binD, binE;
  float peakC = peak_in_band(g_fPDMMagnitudes, NOTE_C3, &binC);
  float peakD = peak_in_band(g_fPDMMagnitudes, NOTE_D3, &binD);
  float peakE = peak_in_band(g_fPDMMagnitudes, NOTE_E3, &binE);

  bool hasC = (noise > 0.0f) && (peakC > PRESENCE_RATIO * noise);
  bool hasD = (noise > 0.0f) && (peakD > PRESENCE_RATIO * noise);
  bool hasE = (noise > 0.0f) && (peakE > PRESENCE_RATIO * noise);

  const float binHz = (float)sampleFreq / (float)pdmDataBufferSize;

  // 4) Print results
  Serial.print("Noise floor: ");
  Serial.print(noise, 2);   // 2 decimal places
  Serial.print(" | ");


  if (hasC) Serial.printf("C3(%.1fHz) ", binC * binHz);
  if (hasD) Serial.printf("D3(%.1fHz) ", binD * binHz);
  if (hasE) Serial.printf("E3(%.1fHz) ", binE * binHz);

  if (!hasC && !hasD && !hasE) Serial.printf("No target notes");

  Serial.printf("\n");
}

//*****************************************************************************
//
// Analyze and print frequency data.
//
//*****************************************************************************
void printLoudest(void)
{
  float fMaxValue;
  uint32_t ui32MaxIndex;
  int16_t *pi16PDMData = (int16_t *)pdmDataBuffer;
  uint32_t ui32LoudestFrequency;

  //
  // Convert the PDM samples to floats, and arrange them in the format
  // required by the FFT function.
  //
  for (uint32_t i = 0; i < pdmDataBufferSize; i++)
  {
    if (PRINT_PDM_DATA)
    {
      Serial.printf("%d\n", pi16PDMData[i]);
    }

    g_fPDMTimeDomain[2 * i] = pi16PDMData[i] / 1.0;
    g_fPDMTimeDomain[2 * i + 1] = 0.0;
  }

  if (PRINT_PDM_DATA)
  {
    Serial.printf("END\n");
  }

  //
  // Perform the FFT.
  //
  arm_cfft_radix4_instance_f32 S;
  arm_cfft_radix4_init_f32(&S, pdmDataBufferSize, 0, 1);
  arm_cfft_radix4_f32(&S, g_fPDMTimeDomain);
  arm_cmplx_mag_f32(g_fPDMTimeDomain, g_fPDMMagnitudes, pdmDataBufferSize);

  if (PRINT_FFT_DATA)
  {
    for (uint32_t i = 0; i < pdmDataBufferSize / 2; i++)
    {
      Serial.printf("%f\n", g_fPDMMagnitudes[i]);
    }

    Serial.printf("END\n");
  }

  //
  // Find the frequency bin with the largest magnitude.
  //
  arm_max_f32(g_fPDMMagnitudes, pdmDataBufferSize / 2, &fMaxValue, &ui32MaxIndex);

  ui32LoudestFrequency = (sampleFreq * ui32MaxIndex) / pdmDataBufferSize;

  if (PRINT_FFT_DATA)
  {
    Serial.printf("Loudest frequency bin: %d\n", ui32MaxIndex);
  }

  Serial.printf("Loudest frequency: %d         \n", ui32LoudestFrequency);
}

//*****************************************************************************
//
// Print PDM configuration data.
//
//*****************************************************************************
void printPDMConfig(void)
{
  uint32_t PDMClk;
  uint32_t MClkDiv;
  float frequencyUnits;

  //
  // Read the config structure to figure out what our internal clock is set
  // to.
  //
  switch (myPDM.getClockDivider())
  {
  case AM_HAL_PDM_MCLKDIV_4:
    MClkDiv = 4;
    break;
  case AM_HAL_PDM_MCLKDIV_3:
    MClkDiv = 3;
    break;
  case AM_HAL_PDM_MCLKDIV_2:
    MClkDiv = 2;
    break;
  case AM_HAL_PDM_MCLKDIV_1:
    MClkDiv = 1;
    break;

  default:
    MClkDiv = 0;
  }

  switch (myPDM.getClockSpeed())
  {
  case AM_HAL_PDM_CLK_12MHZ:
    PDMClk = 12000000;
    break;
  case AM_HAL_PDM_CLK_6MHZ:
    PDMClk = 6000000;
    break;
  case AM_HAL_PDM_CLK_3MHZ:
    PDMClk = 3000000;
    break;
  case AM_HAL_PDM_CLK_1_5MHZ:
    PDMClk = 1500000;
    break;
  case AM_HAL_PDM_CLK_750KHZ:
    PDMClk = 750000;
    break;
  case AM_HAL_PDM_CLK_375KHZ:
    PDMClk = 375000;
    break;
  case AM_HAL_PDM_CLK_187KHZ:
    PDMClk = 187000;
    break;

  default:
    PDMClk = 0;
  }

  //
  // Record the effective sample frequency. We'll need it later to print the
  // loudest frequency from the sample.
  //
  sampleFreq = (PDMClk / (MClkDiv * 2 * myPDM.getDecimationRate()));

  frequencyUnits = (float)sampleFreq / (float)pdmDataBufferSize;

  Serial.printf("Settings:\n");
  Serial.printf("PDM Clock (Hz):         %12d\n", PDMClk);
  Serial.printf("Decimation Rate:        %12d\n", myPDM.getDecimationRate());
  Serial.printf("Effective Sample Freq.: %12d\n", sampleFreq);
  Serial.printf("FFT Length:             %12d\n\n", pdmDataBufferSize);
  Serial.printf("FFT Resolution: %15.3f Hz\n", frequencyUnits);
}
