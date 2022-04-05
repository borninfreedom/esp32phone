#include <Arduino.h>
//#include "./includes/oled.h"
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <lv_examples.h>
#include "esp32-hal-cpu.h"
// extern Adafruit_SH1106G display;
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <string>

TFT_eSPI tft = TFT_eSPI(); /* TFT instance */
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

U8G2_SH1106_128X64_NONAME_2_SW_I2C u8g2(U8G2_R0, 22, 21);

#if USE_LV_LOG != 0
/* Serial debugging */
void my_print(lv_log_level_t level, const char *file, uint32_t line, const char *dsc)
{

  Serial.printf("%s@%d->%s\r\n", file, line, dsc);
  Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors(&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
bool my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  uint16_t touchX, touchY;

  bool touched = tft.getTouch(&touchX, &touchY, 600);

  if (!touched)
  {
    data->state = LV_INDEV_STATE_REL;
  }
  else
  {
    data->state = LV_INDEV_STATE_PR;

    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;

    Serial.print("Data x");
    Serial.println(touchX);

    Serial.print("Data y");
    Serial.println(touchY);
  }

  return false; /*Return `false` because we are not buffering and no more data to read*/
}

TaskHandle_t Task_Display;
TaskHandle_t Task_OLED;

char count = 0;
void setup()
{
  // put your setup code here, to run once:
  // Serial.begin(9600);
  // //testdrawcircle();
  // display.begin(i2c_Address,true);
  // testdrawbitmap(logo16_glcd_bmp, LOGO16_GLCD_HEIGHT, LOGO16_GLCD_WIDTH);
  Serial.begin(115200); /* prepare for possible serial debug */
  setCpuFrequencyMhz(240);
  lv_init();

#if USE_LV_LOG != 0
  lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

  tft.begin();        /* TFT init */
  tft.setRotation(1); /* Landscape orientation */

  uint16_t calData[5] = {275, 3620, 264, 3532, 1};
  tft.setTouch(calData);

  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

  /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 480;
  disp_drv.ver_res = 320;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the (dummy) input device driver*/
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  u8g2.begin();

  lv_demo_benchmark();
  // lv_demo_music();
  //  lv_example_get_started_1();

  xTaskCreatePinnedToCore(task_display, "Task_Display", 10000, NULL, 1, &Task_Display, 0);
  delay(500);
  // xTaskCreatePinnedToCore(task_oled, "Task_OLED", 10000, NULL, 1, &Task_OLED, 1);
  // delay(500);
}

void task_display(void *pvParameters)
{
  for (;;)
  {
    lv_task_handler();
    delay(5);
  }
}

void task_oled(void *pvParameters)
{
  for (;;)
  {
    u8g2.firstPage();
    do
    {
      u8g2.setFont(u8g2_font_ncenB14_tr);
      std::string s = std::to_string(count);
      const char *ss = s.c_str();
      u8g2.drawStr(50, 24, ss);
      count++;
      if (count % 100 == 0)
      {
        count = 0;
      }
      delay(5);
    } while (u8g2.nextPage());
  }
}

void loop()
{
  // put your main code here, to run repeatedly:
  // testdrawcircle();
  // Serial.println("hello");
  // delay(1000);
  // Serial.println(getCpuFrequencyMhz());
  // delay(1000);
  // lv_task_handler();
  // delay(5);
  // Serial.println(getCpuFrequencyMhz());
  // Serial.println(xPortGetCoreID());

  //在loop里面，oled屏幕的刷新速率略快
  // u8g2.firstPage();
  //   do
  //   {
  //     u8g2.setFont(u8g2_font_ncenB14_tr);
  //     std::string s = std::to_string(count);
  //     const char *ss = s.c_str();
  //     u8g2.drawStr(50, 24, ss);
  //     count++;
  //     if (count % 100 == 0)
  //     {
  //       count = 0;
  //     }
  //     delay(5);
  //   } while (u8g2.nextPage());
}
