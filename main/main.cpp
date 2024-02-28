/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "ui/ui.h"


static const int RXPin = 10, TXPin = 11, sclPin = 12, sdaPin = 13;
static const uint32_t GPSBaud = 9600;


class LGFX : public lgfx::LGFX_Device 
{
  lgfx::Panel_ST7796  _panel_instance;  // ST7796UI
  lgfx::Bus_Parallel8 _bus_instance;    // MCU8080 8B
  lgfx::Light_PWM     _light_instance;
  lgfx::Touch_FT5x06  _touch_instance;

public:
  LGFX(void) 
  {
    {
      auto cfg = _bus_instance.config();
      cfg.freq_write = 40000000;
      cfg.pin_wr = 47;
      cfg.pin_rd = -1;
      cfg.pin_rs = 0;

      // LCD data interface, 8bit MCU (8080)
      cfg.pin_d0 = 9;
      cfg.pin_d1 = 46;
      cfg.pin_d2 = 3;
      cfg.pin_d3 = 8;
      cfg.pin_d4 = 18;
      cfg.pin_d5 = 17;
      cfg.pin_d6 = 16;
      cfg.pin_d7 = 15;

      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    {
      auto cfg = _panel_instance.config();

      cfg.pin_cs = -1;
      cfg.pin_rst = 4;
      cfg.pin_busy = -1;

      /*cfg.memory_width = 320;
      cfg.memory_height = 480;*/
      cfg.panel_width       = 320;
      cfg.panel_height      = 480;
      cfg.offset_x          = 0;
      cfg.offset_y          = 0;
      cfg.offset_rotation   = 0;
      cfg.dummy_read_pixel  = 8;
      cfg.dummy_read_bits   = 1;
      cfg.readable          = false;
      cfg.invert            = true;
      cfg.rgb_order         = false;
      cfg.dlen_16bit        = false;
      cfg.bus_shared        = false;

      _panel_instance.config(cfg);
    }

    {
      auto cfg = _light_instance.config();

      cfg.pin_bl  = 45;
      cfg.invert  = false;
      cfg.freq    = 44100;
      cfg.pwm_channel = 7;

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }

    {
      auto cfg = _touch_instance.config();

      cfg.x_min         = 0;
      cfg.x_max         = 319;
      cfg.y_min         = 0;
      cfg.y_max         = 479;
      cfg.pin_int    = 7;  
      cfg.bus_shared = true; 
      cfg.offset_rotation = 0;

      cfg.i2c_port      = 1;
      cfg.i2c_addr      = 0x38;
      cfg.pin_sda       = 6;
      cfg.pin_scl       = 5;
      cfg.freq          = 400000;

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }

    setPanel(&_panel_instance);
  }
};

static LGFX tft;


/*Change to your screen resolution*/
static const uint32_t screenWidth = 320;
static const uint32_t screenHeight = 480;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.writePixels((lgfx::rgb565_t *)&color_p->full, w * h);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  uint16_t x, y;
  if (tft.getTouch(&x, &y)) {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = x;
    data->point.y = y;

  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void ui_reset() {
  /*
  lv_slider_set_value(ui_SpeedSlider, 0, LV_ANIM_OFF);
  lv_obj_clear_flag(ui_SpeedSlider, LV_OBJ_FLAG_CLICKABLE);
  lv_label_set_text(ui_SpeedLabel, "0");
  lv_label_set_text(ui_SpeedLabel2, "0");
  lv_label_set_text(ui_SatellitesLabel, "0");
  lv_label_set_text(ui_CoordinatesLabel, "0\n0");
  lv_label_set_text(ui_AltitudeLabel, "0.00");
  */
}


void initDisplay() {
  tft.begin();
  tft.setRotation(2);
  tft.setBrightness(255);
  tft.fillScreen(TFT_BLACK);
}

//static lv_timer_t * timer_datetime;
//static void timer_datetime_callback(lv_timer_t * timer);

SemaphoreHandle_t xGuiSemaphore;

#define LV_TICK_PERIOD_MS 1

static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}

void ui_event_alarm_on(lv_event_t * e);
void ui_event_alarm_off(lv_event_t * e);


extern "C" void app_main(void)
{
  printf("Hello world!\n");
  xGuiSemaphore = xSemaphoreCreateMutex();
  //--
  initDisplay();
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);

  //Initialize the display
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);

  //Change the following line to your display resolution
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  //Initialize the (dummy) input device driver
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  /* Create and start a periodic timer interrupt to call lv_tick_inc */
  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &lv_tick_task,
      .name = "periodic_gui"
  };
  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

  ui_init();
  ui_reset();

  lv_obj_add_event_cb(ui_ButtonAlarmOn, ui_event_alarm_on, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(ui_ButtonAlarmOff, ui_event_alarm_off, LV_EVENT_CLICKED, NULL);

  while (1) {
      //Delay 1 tick (assumes FreeRTOS tick is 10ms
      vTaskDelay(pdMS_TO_TICKS(10));

      //Try to take the semaphore, call lvgl related function on success 
      if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
          lv_task_handler();
          xSemaphoreGive(xGuiSemaphore);
      }
  }

  vTaskDelete(NULL);
}

///////////////////// FUNCTIONS ////////////////////
void ui_event_alarm_on(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        printf("LV_EVENT_CLICKED AlarmOn\n");
    }
}

void ui_event_alarm_off(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        printf("LV_EVENT_CLICKED AlarmOff\n");
    }
}

/*
static void timer_datetime_callback(lv_timer_t * timer)
{   
    time_t now;
    struct tm datetimeinfo;
    time(&now);
    localtime_r(&now, &datetimeinfo);
    lv_timer_handler();
    lgfx::v1::delay(10);
    //ui_lblTime
    char    time_buf[256];
    strftime(time_buf, sizeof (time_buf), "%m-%d-%Y %H:%M:%S", &datetimeinfo);
    //printf("date time:%s\n", time_buf);

    lv_label_set_text(ui_lblTime, time_buf);
    lv_timer_ready(timer_datetime);    
}
*/