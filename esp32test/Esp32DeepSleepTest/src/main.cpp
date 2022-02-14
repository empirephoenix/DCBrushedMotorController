#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "joy_rmt_rc.h"

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

Adafruit_NeoPixel pixels(1, LED_STATUS, NEO_GRB);

const int M1_A_CHANNEL = 0;

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

  const int freq = 2000;

  const int resolution = 8;

  ledcSetup(M1_A_CHANNEL, freq, resolution);
  ledcAttachPin(M1_B_HI, M1_A_CHANNEL);


}


void check(bool value, char* output)
{
  if (value)
  {
    printf(output);
  }
}

void joy_rmt_rc_read_task()
{
  esp_err_t er = ESP_OK;


  // Configure RMT peripheral for each RC channel
  RingbufHandle_t ringbuffers[axis_count];

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

  // Read loop
  size_t length = 0;
  rmt_item32_t *items = NULL;
  uint32_t duration;
  while(true)
  {
    for (int i = 0; i < axis_count; i++)
    {
      duration = 0;
      items = (rmt_item32_t *)xRingbufferReceive(ringbuffers[i], &length, portMAX_DELAY);
      if (NULL != items)
      {
        // Convert length from number of bytes to number of entries
        length /= sizeof(rmt_item32_t);

        // Read high period of most recent data point
        rmt_item32_t recent = items[length-1];
        if (0 != recent.level0)
        {
          duration = recent.duration0;
        }
        else if (0 != recent.level1)
        {
          duration = recent.duration1;
        }

        // Return memory to ring buffer
        vRingbufferReturnItem(ringbuffers[i], (void*)items);

        // Clamp between min & max
        if (duration < rc_receive_min)
        {
          duration = rc_receive_min;
        }
        else if (duration > rc_receive_max)
        {
          duration = rc_receive_max;
        }

        // Convert range from (rc_receive_min, rc_receive_max) to (-1, 1)
        Serial.print(i);
        Serial.print(":");
        Serial.println(-1.0 + 2.0*((float)duration-rc_receive_min)/(rc_receive_max-rc_receive_min));
      }
      else
      {
        // No data? Failsafe to center.
        Serial.print(i);
        Serial.print(":");
        Serial.println(0.0);
      }
    }

    Serial.print("Tasktime");
    Serial.println(xTaskGetTickCount());


  }
}

void loop()
{

  joy_rmt_rc_read_task();
  //int pwmin = pulseIn(M1_S1, HIGH, 200000);
  //int pwm_clean = constrain(pwmin, 1000, 2000);
  //int out_duty_cycle = map(pwm_clean,1000,2000,20,5000);
  //Serial.println(out_duty_cycle);
  //ledcWrite(M1_A_CHANNEL, out_duty_cycle);
  //digitalWrite(M1_A_LO, HIGH);
}