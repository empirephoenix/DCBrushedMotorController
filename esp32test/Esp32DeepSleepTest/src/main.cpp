#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "joy_rmt_rc.h"
#include "RunningMedian.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// pin setup
#define LED_STATUS 32
#define ONE_WIRE_BUS 35

#define M1_SENSE 34

#define IN1 14
#define IN2 12

#define M1_A_LO 22
#define M1_A_HI 23
#define M1_B_LO 18
#define M1_B_HI 19

#define freq 6000
#define resolution 8

// Configuration
#define SHUNT_MOHM 2
#define MAX_AVG_CURRENT 30000
#define MAX_SPIKE_CURRENT 60000
#define NULL_PASS_DELAY 50

#define mode 0
#define turningMod 0

#define deadzoneMinCH1 1480
#define deadzoneMaxCH1 1520
#define minCH1 950
#define maxCH1 2050

#define minTEMPC -30
#define maxTEMPC 60

// constants
#define STATE_WAIT_NEUTRAL 0
#define STATE_DEFAULT 1
#define STATE_ERROR -1

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

Adafruit_NeoPixel pixels(1, LED_STATUS, NEO_GRBW);

float lastPower;
float nullPass;

const int M1_A_CHANNEL = 0;
const int M1_B_CHANNEL = 1;

RunningMedian m1 = RunningMedian(50);
long m1_last = 0;
  int state = STATE_WAIT_NEUTRAL;

// Configure RMT peripheral for each RC channel
RingbufHandle_t ringbuffers[axis_count];

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


void check(bool value, char *output)
{
  if (value)
  {
    printf(output);
  }
}

void setup()
{
  lastPower = 0;
  Serial.begin(115200);

  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);

  pinMode(M1_A_LO, OUTPUT);
  pinMode(M1_A_HI, OUTPUT);
  pinMode(M1_B_LO, OUTPUT);
  pinMode(M1_B_HI, OUTPUT);
  
  sensors.begin();
  yield();
  
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(0, 10, 0));
  pixels.show();
  Serial.println("Start done");

  pixels.setPixelColor(0, pixels.Color(100, 0, 0));
  pixels.show();


  ledcSetup(M1_A_CHANNEL, freq, resolution);
  ledcAttachPin(M1_A_HI, M1_A_CHANNEL);
  ledcSetup(M1_B_CHANNEL, freq, resolution);
  ledcAttachPin(M1_B_HI, M1_B_CHANNEL);

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
    }
  }
}

int waitforNeutral(float tempC)
{

  float m1v = m1.getAverage();

  int ok = 0;
  boolean neutral = m1v > deadzoneMinCH1 && m1v < deadzoneMaxCH1;
  boolean tempReady = ok == 1 && tempC < minTEMPC || tempC > maxTEMPC;

  Serial.print("Channel is at ");
  Serial.print(m1v);
  Serial.print(" and temperature ");
  Serial.print(tempC);
  Serial.println("°C");
    

  if (neutral && tempReady)
  {
    Serial.print("Precheck ok");
    pixels.setPixelColor(0, pixels.Color(100, 100, 100));
    pixels.show();
    return STATE_DEFAULT;   
  }

  if (!tempReady){
    Serial.print("Invalid Temperature");
    pixels.setPixelColor(0, pixels.Color(100, 100, 100));
  }
  if (!neutral) {
    Serial.print("Not neutral");
      pixels.setPixelColor(0, pixels.Color(200, 0, 0));
  }
      
  pixels.show();
  return STATE_WAIT_NEUTRAL;  
}

float handleMotor(float ratio, int high_channel_a_ledc, int low_pin_a, int high_channel_b_ledc, int low_pin_b)
{
  if (nullPass > millis())
  {
    return 0;
  }

  float oldPower = lastPower;
  lastPower = ratio;
  Serial.print(oldPower);
  Serial.print(" ");
  Serial.println(ratio);
  if (oldPower <= 0 && ratio > 0)
  {
    digitalWrite(low_pin_a, LOW);
    digitalWrite(low_pin_b, LOW);
    ledcWrite(high_channel_a_ledc, 0);
    ledcWrite(high_channel_b_ledc, 0);
    Serial.print("Null pass delay");
    nullPass = millis() + NULL_PASS_DELAY;
    return 0;
  }
  if (oldPower >= 0 && ratio < 0)
  {
    digitalWrite(low_pin_a, LOW);
    digitalWrite(low_pin_b, LOW);
    ledcWrite(high_channel_a_ledc, 0);
    ledcWrite(high_channel_b_ledc, 0);
    Serial.println("Null pass delay");
    nullPass = millis() + NULL_PASS_DELAY;
    return 0;
  }

  if (ratio > 0)
  {
    int forward = mapfloat(ratio, 0, 1, 0, 254);
    digitalWrite(low_pin_a, LOW);
    digitalWrite(low_pin_b, HIGH);
    ledcWrite(high_channel_a_ledc, forward);
    ledcWrite(high_channel_b_ledc, 0);

    return forward;
  }
  else
  {
    int reverse = mapfloat(ratio, -1, 0, 254, 0);
    digitalWrite(low_pin_a, HIGH);
    digitalWrite(low_pin_b, LOW);
    ledcWrite(high_channel_a_ledc, 0);
    ledcWrite(high_channel_b_ledc, reverse);
    return reverse;
  }
}

uint32_t calculateCurrentSpike(uint32_t current, float powerByte)
{
  if (powerByte < 5)
  {
    return 0;
  }
  return current * 255 / powerByte;
}

void defaultMode(float ch1)
{
  float power1 = 0;
  float power2 = 0;
#if (mode == 0)
  power1 = handleMotor(ch1, M1_A_CHANNEL, M1_A_LO, M1_B_CHANNEL, M1_B_LO);

#elif (mode == 1)
  float brake = 1 - abs(ch2);
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
#endif

  uint32_t cur1_ma = analogReadMilliVolts(M1_SENSE) * 1000 / 50 / SHUNT_MOHM;

  uint32_t cur1_spike_ma = calculateCurrentSpike(cur1_ma, power1);

  if (cur1_ma > MAX_AVG_CURRENT)
  {
    state = STATE_ERROR;
    Serial.print("M1 reached current limit average");
    Serial.println(cur1_ma);
    Serial.flush();
    return;
  }

  if (cur1_spike_ma > MAX_SPIKE_CURRENT)
  {
    state = STATE_ERROR;
    Serial.print("M1 reached current limit average");
    Serial.println(cur1_spike_ma);
    Serial.flush();
    return;
  }

  pixels.setPixelColor(0, pixels.Color(0, power1, power2));
  Serial.print("power1 ");
  Serial.print(power1);
  Serial.print(" cur1Ma:");
  Serial.print(cur1_ma);
  Serial.print(" cur1Maspike:");
  Serial.print(cur1_spike_ma);
}

void loop()
{
  sensors.requestTemperatures();
  joy_rmt_rc_read_task();
  float tempC = sensors.getTempCByIndex(0);

  if (state == STATE_WAIT_NEUTRAL || state == STATE_ERROR)
  {
    state = waitforNeutral(tempC);
    return;
  }

  long curTime = millis();
  //here
  m1_last = curTime;
  if (curTime - m1_last > 200)
  {
    Serial.println("Missing signal for m1, error");
    Serial.flush();
    state = STATE_ERROR;
  }

  if (tempC > maxTEMPC)
  {
    state = STATE_ERROR;
  }

  float m1Raw = m1.getAverage();

  float ch1 = 0;
  if (m1Raw < deadzoneMinCH1 || m1Raw > deadzoneMaxCH1)
  {
    ch1 = mapfloat(m1Raw, minCH1, maxCH1, -1, 1);
    ch1 = constrain(ch1, -1, 1);
  }
  Serial.print("Ch1 raw");
  Serial.println(m1Raw);


  if (state == STATE_ERROR)
  {
    handleMotor(0, M1_A_CHANNEL, M1_A_LO, M1_B_CHANNEL, M1_B_LO);

    pixels.setPixelColor(0, pixels.Color(255, 0, 0, 255));
    pixels.show();
    delay(100);
    pixels.setPixelColor(0, pixels.Color(255, 0, 0, 255));
    pixels.show();
    delay(100);
    yield();
  }
  if (state == STATE_DEFAULT)
  {
    defaultMode(ch1);
  }
  pixels.show();
}