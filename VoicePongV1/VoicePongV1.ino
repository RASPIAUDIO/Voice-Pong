#include "Arduino.h"
#include "WiFi.h"
#include "museS3.h"
#include <TFT_eSPI.h>

// ---------- I2S + FFT Includes ----------
#include <arduinoFFT.h>
#include "ESP_I2S.h"
//#include "ES8388.h"   // for ES8388 codec on MuseS3

// ================== GAME SETTINGS ==================
#define SCREEN_WIDTH   320
#define SCREEN_HEIGHT  240

#define PADDLE_WIDTH   10
#define PADDLE_HEIGHT  40
#define PADDLE_MARGIN  5   // Distance from left/right screen edge

// Ball size
#define BALL_SIZE      8

// Original base speeds
#define BALL_SPEED_X   0.6f
#define BALL_SPEED_Y   0.4f

// ================== GLOBAL OBJECTS ==================
TFT_eSPI tft = TFT_eSPI();   // Shared TFT display

// ----- Paddles -----
static int paddle1Y, paddle2Y;
static int oldPaddle1Y, oldPaddle2Y;

// ----- Ball position & speed -----
static float ballX, ballY;
static float oldBallX, oldBallY;
static float ballVX, ballVY;

// ----- Scoring -----
static int score1 = 0;
static int score2 = 0;

// ----- Speed Increase Control -----
unsigned long lastScoreTime     = 0;     // When last point was scored
unsigned long nextSpeedIncrease = 5000; // Increase speed after 10s
bool pointJustScored            = false;

// ============ I2S + FFT SETTINGS ============
#define SAMPLE_RATE     8000
#define FFT_SAMPLES     1024       // Must be power of 2
#define AMP_THRESHOLD   10000.0    // If the max amplitude is below this, we consider "no signal"

// We'll focus on 80..200 Hz
// Inverted: higher freq => top, lower freq => bottom
static const double FREQ_MIN = 200.0;
static const double FREQ_MAX = 400.0;

// Stereo buffer => 2 × FFT_SAMPLES
static int16_t audioBuffer[FFT_SAMPLES * 2];

// Arrays for Left & Right channels
static double vRealL[FFT_SAMPLES];
static double vImagL[FFT_SAMPLES];

static double vRealR[FFT_SAMPLES];
static double vImagR[FFT_SAMPLES];

ArduinoFFT<double> FFTLeft(vRealL, vImagL, FFT_SAMPLES, SAMPLE_RATE);
ArduinoFFT<double> FFTRight(vRealR, vImagR, FFT_SAMPLES, SAMPLE_RATE);

// ----- I2S / Codec -----
I2SClass  i2s;
ES8388    es;

// ================== FORWARD DECLARATIONS ==================
void drawCenterLine();
void drawScores();
void erasePaddlesAndBall();
void drawPaddlesAndBall(bool initialDraw = false);
void onPointScored();

// FFT helper to read audio and get left/right frequencies
void updateFrequencies(double &freqLeft, double &freqRight);

// Maps frequency [FREQ_MIN..FREQ_MAX] to [0..(SCREEN_HEIGHT-PADDLE_HEIGHT)]
// with inversion (higher freq => top, lower freq => bottom)
int mapFreqToPaddle(double freq);

// ================== SETUP ==================
void setup() 
{
  Serial.begin(115200);
  Serial.println("\n=== Pong + FFT (Modified) ===");

  // ---------- TFT Setup ----------
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);

  // ---------- I2S + Codec Setup ----------
  pinMode(GPIO_PA_EN, OUTPUT);
  digitalWrite(GPIO_PA_EN, HIGH);

  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, 
                 I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("Failed to init I2S!");
    while (true) { delay(100); }
  }

  // Initialize ES8388 codec
  while (!es.begin(IIC_DATA, IIC_CLK)) {
    Serial.println("Retrying ES8388 init...");
    delay(1000);
  }
  es.volume(ES8388::ES_MAIN, 100);
  es.volume(ES8388::ES_OUT1, 100);
  es.mute(ES8388::ES_OUT1, false);
  es.mute(ES8388::ES_MAIN, false);
  es.microphone_volume(90);
  es.ALC(false);

  // ---------- Initialize Paddles ----------
  paddle1Y = (SCREEN_HEIGHT - PADDLE_HEIGHT) / 2;
  paddle2Y = (SCREEN_HEIGHT - PADDLE_HEIGHT) / 2;
  oldPaddle1Y = paddle1Y;
  oldPaddle2Y = paddle2Y;

  // ---------- Initialize Ball ----------
  ballX = (float)SCREEN_WIDTH / 2.0f;
  ballY = (float)SCREEN_HEIGHT / 2.0f;
  oldBallX = ballX;
  oldBallY = ballY;

  // *** Make the ball 5× faster at start ***
  ballVX = 10.0f * BALL_SPEED_X;
  ballVY = 10.0f * BALL_SPEED_Y;

  score1 = 0;
  score2 = 0;
  lastScoreTime = millis();

  // ---------- Draw initial elements ----------
  drawCenterLine();
  drawScores();
  drawPaddlesAndBall(true);

  Serial.println("Setup complete.");
}

// ================== MAIN LOOP ==================
void loop()
{
  // 1) Get updated frequencies
  double freqLeft = 0.0;
  double freqRight = 0.0;
  updateFrequencies(freqLeft, freqRight);

  // 2) Update paddle positions
  oldPaddle1Y = paddle1Y;
  oldPaddle2Y = paddle2Y;

  // If freq == 0 => no valid signal => paddle goes to middle
  if (freqLeft >= 10 && freqLeft <= 5000) {
    paddle1Y = mapFreqToPaddle(freqLeft);
  } else {
    paddle1Y = (SCREEN_HEIGHT - PADDLE_HEIGHT) / 2;
  }

  if (freqRight >= 10 && freqRight <= 5000) {
    paddle2Y = mapFreqToPaddle(freqRight);
  } else {
    paddle2Y = (SCREEN_HEIGHT - PADDLE_HEIGHT) / 2;
  }

  // 3) Update ball position
  oldBallX = ballX;
  oldBallY = ballY;

  ballX += ballVX;
  ballY += ballVY;

  int intBallX = (int)ballX;
  int intBallY = (int)ballY;

  // 4) Check top/bottom collisions
  if (ballY <= 0.0f || ballY >= (SCREEN_HEIGHT - BALL_SIZE)) {
    ballVY = -ballVY;
  }

  // 5) Check collisions with left paddle
  if (intBallX <= (PADDLE_MARGIN + PADDLE_WIDTH)) {
    if ((intBallY + BALL_SIZE >= paddle1Y) && (intBallY <= paddle1Y + PADDLE_HEIGHT)) {
      ballVX = -ballVX;
      // Add slight random vertical variation
      ballVY += random(-1, 2) * 0.1f;
    } else {
      // Player 2 scores
      score2++;
      onPointScored();
    }
  }

  // 6) Check collisions with right paddle
  if (intBallX >= (SCREEN_WIDTH - PADDLE_MARGIN - PADDLE_WIDTH - BALL_SIZE)) {
    if ((intBallY + BALL_SIZE >= paddle2Y) && (intBallY <= paddle2Y + PADDLE_HEIGHT)) {
      ballVX = -ballVX;
      ballVY += random(-1, 2) * 0.1f;
    } else {
      // Player 1 scores
      score1++;
      onPointScored();
    }
  }

  // 7) Increase speed if 10s pass with no score
  unsigned long now = millis();
  if (!pointJustScored && (now - lastScoreTime > nextSpeedIncrease)) {
    ballVX *= 1.2f;
    ballVY *= 1.2f;
    nextSpeedIncrease += 10000;
  }

  // 8) Erase old and draw new
  erasePaddlesAndBall();
  drawPaddlesAndBall();

  // Let it breathe a bit
  // (Though capturing 1024 samples @8kHz already ~128ms)
  // You can experiment with this delay
  //delay(20);
}

// ================== PONG HELPER FUNCTIONS ==================
void onPointScored()
{
  pointJustScored = true;

  // Reset ball to center
  ballX = (float)SCREEN_WIDTH / 2.0f;
  ballY = (float)SCREEN_HEIGHT / 2.0f;

  // Random new direction
  // Keep the same "5x base speed" logic or revert to normal?
  // If you want it ALWAYS to start 5x faster, do so here:
  ballVX = (random(0,2) == 0) ? (10.0f * BALL_SPEED_X) : (-10.0f * BALL_SPEED_X);
  ballVY = (random(0,2) == 0) ? (10.0f * BALL_SPEED_Y) : (-10.0f * BALL_SPEED_Y);

  // Update scoreboard
  drawScores();

  // Reset speed logic
  lastScoreTime     = millis();
  nextSpeedIncrease = 10000;
  pointJustScored   = false;
}

void drawCenterLine()
{
  for (int y = 0; y < SCREEN_HEIGHT; y += 10) {
    tft.drawFastVLine(SCREEN_WIDTH / 2, y, 5, TFT_WHITE);
  }
}

void drawScores()
{
  tft.fillRect(0, 0, SCREEN_WIDTH, 20, TFT_BLACK);
  
  tft.setCursor(SCREEN_WIDTH / 2 - 60, 0);
  tft.print(score1);

  tft.setCursor(SCREEN_WIDTH / 2 + 50, 0);
  tft.print(score2);

  drawCenterLine();
}

void erasePaddlesAndBall()
{
  tft.fillRect(PADDLE_MARGIN, oldPaddle1Y, PADDLE_WIDTH, PADDLE_HEIGHT, TFT_BLACK);
  tft.fillRect(SCREEN_WIDTH - PADDLE_MARGIN - PADDLE_WIDTH, oldPaddle2Y, PADDLE_WIDTH, PADDLE_HEIGHT, TFT_BLACK);
  tft.fillRect((int)oldBallX, (int)oldBallY, BALL_SIZE, BALL_SIZE, TFT_BLACK);
}

void drawPaddlesAndBall(bool initialDraw)
{
  tft.fillRect(PADDLE_MARGIN, paddle1Y, PADDLE_WIDTH, PADDLE_HEIGHT, TFT_GREEN);
  tft.fillRect(SCREEN_WIDTH - PADDLE_MARGIN - PADDLE_WIDTH, paddle2Y, PADDLE_WIDTH, PADDLE_HEIGHT, TFT_YELLOW);
  tft.fillRect((int)ballX, (int)ballY, BALL_SIZE, BALL_SIZE, TFT_WHITE);

  if (initialDraw) {
    drawScores();
  }
}

// ================== FFT HELPER FUNCTIONS ==================
void updateFrequencies(double &freqLeft, double &freqRight)
{
  // We need 2×FFT_SAMPLES = 2048 reads
  const int totalSamples = FFT_SAMPLES * 2;
  int samplesCollected   = 0;

  while (samplesCollected < totalSamples) {
    int sampleVal = i2s.read();  // 16-bit audio sample
    if (sampleVal == -1) {
      // No data available yet
      delayMicroseconds(50);
      continue;
    }
    audioBuffer[samplesCollected++] = (int16_t)sampleVal;
  }

  // Separate out left & right channels
  for (int i = 0; i < FFT_SAMPLES; i++) {
    int16_t leftSample  = audioBuffer[2*i];
    int16_t rightSample = audioBuffer[2*i + 1];

    vRealL[i] = (double)leftSample;
    vImagL[i] = 0.0;

    vRealR[i] = (double)rightSample;
    vImagR[i] = 0.0;
  }

  // ----- Left Channel -----
  FFTLeft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFTLeft.compute(FFTDirection::Forward);
  FFTLeft.complexToMagnitude();

  double maxAmpLeft = 0.0;
  for (int i = 0; i < FFT_SAMPLES/2; i++) {
    if (vRealL[i] > maxAmpLeft) {
      maxAmpLeft = vRealL[i];
    }
  }
  if (maxAmpLeft > AMP_THRESHOLD) {
    freqLeft = FFTLeft.majorPeak();
  } else {
    freqLeft = 0.0;
  }

  // ----- Right Channel -----
  FFTRight.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFTRight.compute(FFTDirection::Forward);
  FFTRight.complexToMagnitude();

  double maxAmpRight = 0.0;
  for (int i = 0; i < FFT_SAMPLES/2; i++) {
    if (vRealR[i] > maxAmpRight) {
      maxAmpRight = vRealR[i];
    }
  }
  if (maxAmpRight > AMP_THRESHOLD) {
    freqRight = FFTRight.majorPeak();
  } else {
    freqRight = 0.0;
  }

  // Debug printing
  Serial.printf("[L] Amp=%.1f Freq=%.1f  |  [R] Amp=%.1f Freq=%.1f\n", 
                maxAmpLeft, freqLeft, maxAmpRight, freqRight);
}

// Inverse mapping: freq=FREQ_MIN => bottom, freq=FREQ_MAX => top
int mapFreqToPaddle(double freq)
{
  // Clamp to [FREQ_MIN..FREQ_MAX]
  if (freq < FREQ_MIN) freq = FREQ_MIN;
  if (freq > FREQ_MAX) freq = FREQ_MAX;

  // Normalized in [0..1], but invert so high freq => top
  //   ratio=1 means bottom, ratio=0 means top.
  double ratio = (freq - FREQ_MIN) / (FREQ_MAX - FREQ_MIN);  // 0..1
  double inverted = 1.0 - ratio;                            // invert

  // Map 0..1 => 0..(SCREEN_HEIGHT - PADDLE_HEIGHT)
  int pos = (int)(inverted * (SCREEN_HEIGHT - PADDLE_HEIGHT));
  return pos;
}
