#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "joy_rmt_rc.h"
#include "RunningMedian.h"

#define SHUNT_MOHM 100
#define MAX_AVG_CURRENT 5000
#define MAX_SPIKE_CURRENT 10000

#define LED_STATUS 32
#define BUZZER 27
#define ONE_WIRE 4

#define M1_SENSE 34
#define M2_SENSE 35

#define M1_S1 14
#define M1_S2 12
#define M2_S1 13
#define M2_S2 5

#define M1_A_LO 18
#define M1_A_HI 15
#define M1_B_LO 21
#define M1_B_HI 19

#define M2_A_LO 23
#define M2_A_HI 22
#define M2_B_LO 26
#define M2_B_HI 25

#define deadzoneMinCH1 1470
#define deadzoneMaxCH1 1530
#define minCH1 950
#define maxCH1 2050


#define deadzoneMinCH2 1470
#define deadzoneMaxCH2 1530
#define minCH2 950
#define maxCH2 2050

#define STATE_WAIT_NEUTRAL 0
#define STATE_DEFAULT 1
#define STATE_ERROR -1



#define turningMod 0


#define FOREACH_INPUT(INPUT) \
  INPUT(NONE)                \
  INPUT(RC)                  \
  INPUT(VOLTAGE)

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

enum INPUT_MODE
{
  FOREACH_INPUT(GENERATE_ENUM)
};

Adafruit_NeoPixel pixels(1, LED_STATUS, NEO_GRB);

const int M1_A_CHANNEL = 0;
const int M1_B_CHANNEL = 1;
const int M2_A_CHANNEL = 2;
const int M2_B_CHANNEL = 3;

RunningMedian m1 = RunningMedian(50);
RunningMedian m2 = RunningMedian(50);
long m1_last = 0;
long m2_last = 0;
int state = STATE_WAIT_NEUTRAL;

// Configure RMT peripheral for each RC channel
RingbufHandle_t ringbuffers[axis_count];

int mode = 0;

void check(bool value, char *output)
{
  if (value)
  {
    printf(output);
  }
}

void setup()
{

  Serial.begin(115200);

  pinMode(ONE_WIRE, INPUT);

  pinMode(M1_S1, INPUT);
  pinMode(M1_S2, INPUT);

  pinMode(M2_S1, INPUT_PULLUP);
  pinMode(M2_S2, INPUT);

  pinMode(BUZZER, OUTPUT);

  pinMode(M1_A_LO, OUTPUT);
  pinMode(M1_A_HI, OUTPUT);
  pinMode(M1_B_LO, OUTPUT);
  pinMode(M1_B_HI, OUTPUT);
  pinMode(M2_A_LO, OUTPUT);
  pinMode(M2_A_HI, OUTPUT);
  pinMode(M2_B_LO, OUTPUT);
  pinMode(M2_B_HI, OUTPUT);

  yield();
  mode = 0; //digitalRead(M2_S2);

  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(0, 10, 0));
  pixels.show();
  Serial.println("Start done");

  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);

  pixels.setPixelColor(0, pixels.Color(100, 0, 0));
  pixels.show();

  const int freq = 2000;
  const int resolution = 8;

  ledcSetup(M1_A_CHANNEL, freq, resolution);
  ledcAttachPin(M1_A_HI, M1_A_CHANNEL);
  ledcSetup(M1_B_CHANNEL, freq, resolution);
  ledcAttachPin(M1_B_HI, M1_B_CHANNEL);

  ledcSetup(M2_A_CHANNEL, freq, resolution);
  ledcAttachPin(M2_A_HI, M2_A_CHANNEL);
  ledcSetup(M2_B_CHANNEL, freq, resolution);
  ledcAttachPin(M2_B_HI, M2_B_CHANNEL);

  esp_err_t er = ESP_OK;
  for (int i = 0; i < axis_count; i++)
  {
    rmt_config_t rmt_rx_config = {};
    rmt_rx_config.rmt_mode = RMT_MODE_RX;
    rmt_rx_config.channel = rc_channels[i].channel;
    rmt_rx_config.clk_div = rmt_clock_divider;
    rmt_rx_config.gpio_num = rc_channels[i].pin;
    rmt_rx_config.mem_block_num = 1;
    rmt_rx_config.rx_config.filter_en = true;
    rmt_rx_config.rx_config.filter_ticks_thresh = rmt_filter_threshold;
    rmt_rx_config.rx_config.idle_threshold = rmt_idle_threshold;

    er = rmt_config(&rmt_rx_config);
    check(ESP_OK != er, "ERROR: rmt_config failed\n");

    er = rmt_driver_install(rc_channels[i].channel, 1024, 0);
    check(ESP_OK != er, "ERROR: rmt_driver_install failed\n");

    er = rmt_get_ringbuf_handle(rc_channels[i].channel, &(ringbuffers[i]));
    check(ESP_OK != er, "ERROR: rmt_get_ringbuf_handle failed\n");
    check(NULL == ringbuffers[i], "ERROR: rmt_get_ringbuf_handle gave a NULL handle\n");

    er = rmt_rx_start(rc_channels[i].channel, true);
    check(ESP_OK != er, "ERROR: rmt_rx_start failed\n");
  }

  pixels.setPixelColor(0, pixels.Color(0, 0, 100));
  pixels.show();
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void joy_rmt_rc_read_task()
{

  // Read loop
  size_t length = 0;
  rmt_item32_t *items = NULL;
  uint32_t duration;
  for (int i = 0; i < axis_count; i++)
  {
    duration = 0;
    items = (rmt_item32_t *)xRingbufferReceive(ringbuffers[i], &length, 1);
    if (NULL != items)
    {
      // Convert length from number of bytes to number of entries
      length /= sizeof(rmt_item32_t);

      // Read high period of most recent data point
      rmt_item32_t recent = items[length - 1];
      if (0 != recent.level0)
      {
        duration = recent.duration0;
      }
      else if (0 != recent.level1)
      {
        duration = recent.duration1;
      }

      // Return memory to ring buffer
      vRingbufferReturnItem(ringbuffers[i], (void *)items);

      // Clamp between min & max
      if (duration < rc_receive_min)
      {
        continue;
      }
      else if (duration > rc_receive_max)
      {
        continue;
      }

      if (i == 0)
      {
        m1.add(duration);
        m1_last = millis();
      }
      if (i == 1)
      {
        m2.add(duration);
        m2_last = millis();
      }
    }
  }
}

int waitforNeutral()
{

  float m1v = m1.getAverage();
  float m2v = m2.getAverage();

  int ok = 0;
  if (m1v > deadzoneMinCH1 && m1v < deadzoneMaxCH1)
  {
    Serial.println("Channel 1 neutral");
    ok++;
  }
  if (m2v > deadzoneMinCH2 && m2v < deadzoneMaxCH2)
  {
    Serial.println("Channel 2 neutral");
    ok++;
  }

  Serial.print("Waiting for channels to be neutral");
  Serial.print(m1v);
  Serial.print(" ");
  Serial.print(m2v);
  Serial.println(ok);

  if (ok == 0)
  {
    pixels.setPixelColor(0, pixels.Color(200, 0, 0));
    pixels.show();
  }
  if (ok == 1)
  {
    pixels.setPixelColor(0, pixels.Color(100, 25, 25));
    pixels.show();
  }
  if (ok == 2)
  {
    pixels.setPixelColor(0, pixels.Color(0, 25, 0));
    int buzzerDelay = 50+mode*200;
    digitalWrite(BUZZER, HIGH);
    delay(buzzerDelay);
    digitalWrite(BUZZER, LOW);
    delay(buzzerDelay);
    digitalWrite(BUZZER, HIGH);
    delay(buzzerDelay);
    digitalWrite(BUZZER, LOW);
    return 1;
  }

  return 0;
}

float handleMotor(int motor, float ratio, int high_channel_a_ledc, int low_pin_a, int high_channel_b_ledc, int low_pin_b)
{
  if (ratio > 0)
  {
    int forward = mapfloat(ratio, 0, 1, 0, 255);
    digitalWrite(low_pin_a, LOW);
    digitalWrite(low_pin_b, HIGH);
    ledcWrite(high_channel_a_ledc, forward);
    ledcWrite(high_channel_b_ledc, 0);

    return forward;
  }
  else
  {
    int reverse = mapfloat(ratio, -1, 0, 255, 0);
    digitalWrite(low_pin_a, HIGH);
    digitalWrite(low_pin_b, LOW);
    ledcWrite(high_channel_a_ledc, 0);
    ledcWrite(high_channel_b_ledc, reverse);
    return reverse;
  }
}

uint32_t calculateCurrentSpike(uint32_t current, float powerByte){
  if(powerByte < 1){
    return 0;
  }
  return current * 255/powerByte;
}

void defaultMode(float ch1, float ch2){
  float power1 = 0;
  float power2 = 0;
  if (mode == 0)
  {
    power1 = handleMotor(1, ch1, M1_A_CHANNEL, M1_A_LO, M1_B_CHANNEL, M1_B_LO);
    power2 = handleMotor(2, ch2, M2_A_CHANNEL, M2_A_LO, M2_B_CHANNEL, M2_B_LO);
  }
  else if(mode == 1)
  {
    float brake = 1-abs(ch2);
    float brakeFactor = brake * turningMod + (1 - turningMod);

    float left = ch1;
    float right = ch1;
    if (ch2 < 0)
    {
      left = left * brakeFactor;
    }
    else
    {
      right = right * brakeFactor;
    }
    power1 = handleMotor(1, left, M1_A_CHANNEL, M1_A_LO, M1_B_CHANNEL, M1_B_LO);
    power2 = handleMotor(2, right, M2_A_CHANNEL, M2_A_LO, M2_B_CHANNEL, M2_B_LO);
  }

  uint32_t cur1_ma = analogReadMilliVolts(M1_SENSE)*1000 /50 /SHUNT_MOHM;
  uint32_t cur2_ma = analogReadMilliVolts(M2_SENSE)*1000 /50 /SHUNT_MOHM;
  
  uint32_t cur1_spike_ma = calculateCurrentSpike(cur1_ma, power1);
  uint32_t cur2_spike_ma = calculateCurrentSpike(cur2_ma, power2);

  if(cur1_ma > MAX_AVG_CURRENT){
    state = STATE_ERROR;
    Serial.print("M1 reached current limit average");
    Serial.println(cur1_ma);
    return;
  }

  if(cur2_ma > MAX_AVG_CURRENT){
    state = STATE_ERROR;
    Serial.print("M2 reached current limit average");
    Serial.println(cur2_ma);
    return;
  }

  if(cur1_spike_ma > MAX_SPIKE_CURRENT){
    state = STATE_ERROR;
    Serial.print("M1 reached current limit average");
    Serial.println(cur1_spike_ma);
    return;
  }

  if(cur2_spike_ma > MAX_SPIKE_CURRENT){
    state = STATE_ERROR;
    Serial.print("M2 reached current limit average");
    Serial.println(cur2_spike_ma);
    return;
  }
  pixels.setPixelColor(0, pixels.Color(0, power1, power2));
  Serial.print("power1 ");
  Serial.print(power1);
  Serial.print(" cur1Ma:");
  Serial.print(cur1_ma);
  Serial.print(" cur1Maspike:");
  Serial.print(cur1_spike_ma);
  Serial.print(" power2 ");
  Serial.print(power2);
  Serial.print(" cur2Ma:");
  Serial.print(cur2_ma);
  Serial.print(" cur2Maspike:");
  Serial.print(cur2_spike_ma);
  Serial.println();
}


void loop()
{
  joy_rmt_rc_read_task();
  if (state == STATE_WAIT_NEUTRAL)
  {
    state = waitforNeutral();
    return;
  }

  long curTime = millis();
  if (curTime - m1_last > 200)
  {
    state = STATE_ERROR;
  }
  if (curTime - m2_last > 200)
  {
    state = STATE_ERROR;
  }

  float m1Raw = m1.getAverage();
  float m2Raw = m2.getAverage();

  float ch1 = 0;
  float ch2 = 0;
  if (m1Raw < deadzoneMinCH1 || m1Raw > deadzoneMaxCH1)
  {
    ch1 = mapfloat(m1Raw, minCH1, maxCH1, -1, 1);
    ch1 = constrain(ch1, -1, 1);
  }
  if (m2Raw < deadzoneMinCH2 || m2Raw > deadzoneMaxCH2)
  {
    ch2 = mapfloat(m2Raw, minCH2, maxCH2, -1, 1);
    ch2 = constrain(ch2, -1, 1);
  }
  
  if(state == STATE_ERROR){
    handleMotor(1, 0, M1_A_CHANNEL, M1_A_LO, M1_B_CHANNEL, M1_B_LO);
    handleMotor(2, 0, M2_A_CHANNEL, M2_A_LO, M2_B_CHANNEL, M2_B_LO);
    digitalWrite(BUZZER, HIGH);
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();

    delay(100);
  }
  if(state == STATE_DEFAULT){
    defaultMode(ch1, ch2);
  }
  pixels.show();
}