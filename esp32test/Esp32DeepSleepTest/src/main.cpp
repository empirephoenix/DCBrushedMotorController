#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "joy_rmt_rc.h"
#include "RunningMedian.h"

#define LED_STATUS 32
#define BUZZER 27
#define ONE_WIRE 4

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

#define deadzone 50

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

// Configure RMT peripheral for each RC channel
RingbufHandle_t ringbuffers[axis_count];


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

  pinMode(M2_S1, INPUT);
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
  if (m1v > rc_receive_mid - deadzone && m1v < rc_receive_mid + deadzone)
  {
    Serial.println("Channel 1 neutral");
    ok++;
  }
  if (m2v > rc_receive_mid - deadzone && m2v < rc_receive_mid + deadzone)
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
    digitalWrite(BUZZER, HIGH);
    delay(50);
    digitalWrite(BUZZER, LOW);
    delay(50);
    digitalWrite(BUZZER, HIGH);
    delay(50);
    digitalWrite(BUZZER, LOW);
    return 1;
  }
  
  return 0;
}

float handleMotor(int motor, float duration, int high_channel_a_ledc, int low_pin_a, int high_channel_b_ledc, int low_pin_b)
{
  if (isnan(duration))
  {
    // freewheel active
    return 0;
  }
  float cleanDuration = constrain(duration, 1000, 2000);
  if (cleanDuration > 1450 && cleanDuration < 1550)
  {
    return 0;
  }
  if (cleanDuration > 1500)
  {
    int forward = map(cleanDuration, 1550, 2000, 0, 255);
    Serial.print(motor);
    Serial.print("+");
    Serial.println(forward);
    digitalWrite(low_pin_a, LOW);
    digitalWrite(low_pin_b, HIGH);
    ledcWrite(high_channel_a_ledc, forward);
    ledcWrite(high_channel_b_ledc, 0);

    return forward;
  }
  else
  {
    int reverse = map(cleanDuration, 1000, 1450, 255, 0);
    Serial.print(motor);
    Serial.print("-");
    Serial.println(reverse);
    digitalWrite(low_pin_a, HIGH);
    digitalWrite(low_pin_b, LOW);
    ledcWrite(high_channel_a_ledc, 0);
    ledcWrite(high_channel_b_ledc, reverse);
    return reverse;
  }
}

int state = 0;
void loop()
{
  Serial.println("loop");
  joy_rmt_rc_read_task();
  if (state == 0)
  {
    state = waitforNeutral();
    return;
  }



  long curTime = millis();
  if (curTime - m1_last > 200)
  {
    state = 3;
    m1.clear();
  }
  if (curTime - m2_last > 200)
  {
    state = 3;
    m2.clear();
  }

  float power1 = handleMotor(1, m1.getAverage(), M1_A_CHANNEL, M1_A_LO, M1_B_CHANNEL, M1_B_LO);
  float power2 = handleMotor(2,m2.getAverage(), M2_A_CHANNEL, M2_A_LO, M2_B_CHANNEL, M2_B_LO);

  pixels.setPixelColor(0, pixels.Color(0, power1, power2));
  if (state == 3)
  {
    digitalWrite(BUZZER, HIGH);
    pixels.setPixelColor(0, pixels.Color(255, 0,0));
    pixels.show();
  }
  
  pixels.show();
}