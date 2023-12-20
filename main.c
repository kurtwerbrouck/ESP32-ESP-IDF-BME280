// Visual code IDF I2C BME280 and Oled 18/06/2023
// Kurt Werbrouck

#include <string.h>
#include <stdio.h>
#include "stdlib.h"
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "mqtt_client.h"

#include "malloc.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/apps/sntp.h"
#include "driver/ledc.h"
#include <sys/time.h>

#include "ssd1366.h"
#include "font8x8_basic.h"

#include <esp_task_wdt.h>

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

#define LED GPIO_NUM_2

uint8_t *data_rd;

#define I2C_MASTER_SDA_IO       GPIO_NUM_21
#define I2C_MASTER_SCL_IO       GPIO_NUM_22
#define I2C_FREQ_HZ             100000
#define I2C_PORT_NUM            I2C_NUM_0
#define I2C_TX_BUF_DISABLE      0
#define I2C_RX_BUF_DISABLE      0

portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED; 

const int CONNECTED_BIT       = BIT0;
const int SNTPreaddone        = BIT10;
const int BME280readdone      = BIT11;

#define TAG_BME280  "Bme280 i2c"
#define TAG_TIME    "Time"

TaskHandle_t BME280_Th        = NULL;
TaskHandle_t Run_Blink        = NULL;

struct BME280_Data
{
  float BME280_Temp;
  float BME280_Pres;
  float BME280_Hum;
};
struct BME280_Data BME280_DATA;

struct Oled_Data {
  char* Text;
  uint8_t page;
  uint8_t begin;
  uint8_t end;
  TaskHandle_t taskh;
};

struct Oled_Data OLED_DATA_BME280[5] ={ {(char*) TAG_BME280,  0x00, 0x00, 0x10, 0},
                                        {(char*) "Temp:         C",     0x03, 0x00, 0x10, 0},
                                        {(char*) "Pres:         ph",     0x05, 0x00, 0x10, 0},
                                        {(char*) "Hum :         %",     0x07, 0x00, 0x10, 0},
                                        {(char*) "XXXX",      0x05, 0x00, 0x10, 0},
};

void i2c_master_init(gpio_num_t I2C_SDA,gpio_num_t I2C_SCL,int I2C_CHANNEL) 
{
  i2c_config_t i2c_config;
  i2c_config.mode 				      = I2C_MODE_MASTER;
  i2c_config.sda_io_num 		    = I2C_SDA;//I2C_MASTER_SDA_IO;
  i2c_config.scl_io_num 		    = I2C_SCL;//I2C_MASTER_SCL_IO;
  i2c_config.sda_pullup_en 		  = GPIO_PULLUP_ENABLE;
  i2c_config.scl_pullup_en 		  = GPIO_PULLUP_ENABLE;
  i2c_config.master.clk_speed 	= I2C_FREQ_HZ;
  i2c_config.clk_flags          = 0;

  //i2c_param_config(I2C_NUM_0, &i2c_config);
  //i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

  i2c_param_config(I2C_CHANNEL, &i2c_config);
  i2c_driver_install(I2C_CHANNEL, I2C_MODE_MASTER, 0, 0, 0);

}

static esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd,size_t size)
{
  if (size == 0) { return ESP_OK; }
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (i2c_addr << 1 ) | 0x0, I2C_MASTER_ACK);
   i2c_master_write_byte(cmd, i2c_reg, I2C_MASTER_ACK);
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | 0x1, I2C_MASTER_ACK); //readbit
   if( size > 1) { i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK); }
   i2c_master_read_byte(cmd, data_rd + size - 1  , I2C_MASTER_NACK);
   i2c_master_stop(cmd);
   esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
   i2c_cmd_link_delete(cmd);
   return ret;
}

static esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t data_rd)
{ 
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   //i2c_master_write_byte(cmd, (i2c_addr << 1 ) | 0x00, I2C_MASTER_ACK);
   i2c_master_write_byte(cmd, (i2c_addr << 1 ) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
   i2c_master_write_byte(cmd, i2c_reg, I2C_MASTER_ACK);
   i2c_master_write_byte(cmd, data_rd, I2C_MASTER_ACK); //readbit
   i2c_master_stop(cmd);
   esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);
   i2c_cmd_link_delete(cmd);
  return ret;  
}

void ssd1306_init() 
{
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
	i2c_master_write_byte(cmd, 0x14, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); // reverse left-right mapping
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); // reverse up-bottom mapping

	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);

	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		ESP_LOGI("TAG_I2C","OLED configured successfully\n");
	} else {
		ESP_LOGI("TAG_I2C","OLED configuration failed. code: 0x%.2X\n", espRc);
	}
	i2c_cmd_link_delete(cmd);
}

void task_ssd1306_display_clear() 
{
	esp_err_t espRc;
	i2c_cmd_handle_t cmd;

	uint8_t zero[128] = {}; //for (int j=0;j<128;j++) { zero[j] = 0x00; }

	for (uint8_t i = 0; i < 8; i++) {
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
		i2c_master_write_byte(cmd, 0xB0 | i, true);
		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
		i2c_master_write_byte(cmd, 0x00, true );
		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
		i2c_master_write_byte(cmd, 0x10, true);

		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
		i2c_master_write(cmd, zero, 128, true);

		i2c_master_stop(cmd);
		espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000/portTICK_PERIOD_MS);
	
		if (espRc == ESP_OK) {
			//ESP_LOGE(tag, "clearing %x",i);
		} else {
			//ESP_LOGE(tag, "Scroll command HOR L R failed. code: 0x%.2X", espRc);
		}

		i2c_cmd_link_delete(cmd);
	}
}

void task_ssd1306_contrast() 
{
	i2c_cmd_handle_t cmd;
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
  i2c_master_write_byte(cmd, OLED_CMD_SET_CONTRAST, true);
  i2c_master_write_byte(cmd, 125, true); // contrast value
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(1/portTICK_PERIOD_MS);
}

void task_ssd1306_display_text(void *arg_text) 
{
  struct Oled_Data *Oled_Datap   = (struct Oled_Data*)arg_text;

  char *text              = (char*) Oled_Datap->Text;
  uint8_t text_len        = strlen(text);

	i2c_cmd_handle_t cmd;

	uint8_t cur_page = Oled_Datap->page; //OLED_DATA[1].page; // was 0

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, 0x00 | Oled_Datap->begin, true); // 0x00 reset column
	i2c_master_write_byte(cmd, 0x10 | Oled_Datap->end, true); // 0x10
	i2c_master_write_byte(cmd, 0xB0 | cur_page, true); // reset page

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	for (uint8_t i = 0; i < text_len; i++) 
    {
		if (text[i] == '\n') 
        {
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
			i2c_master_write_byte(cmd, 0x00, true); // reset column
			i2c_master_write_byte(cmd, 0x10, true);
			i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // increment page

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
		} else 
            {
                cmd = i2c_cmd_link_create();
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

                i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
                i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);

                i2c_master_stop(cmd);
                i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
                i2c_cmd_link_delete(cmd);
		    }
	}
	vTaskDelete(NULL);
}

void LedBlink_API(void * parameter)                                             // Blink Led
{
  ledc_timer_config_t ledc_timer;
  ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;       
  ledc_timer.duty_resolution  = LEDC_TIMER_13_BIT;
  ledc_timer.timer_num        = LEDC_TIMER_0;
  ledc_timer.freq_hz          = 10; 
  ledc_timer_config(&ledc_timer);
  ledc_channel_config_t ledc_channel;
  ledc_channel.gpio_num       = 2;
  ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
  ledc_channel.channel        = LEDC_CHANNEL_0;
  ledc_channel.intr_type      = LEDC_INTR_DISABLE;
  ledc_channel.timer_sel      = LEDC_TIMER_0;
  ledc_channel.duty           = 4095/2;
  ledc_channel.hpoint         = 0; 
  ledc_channel_config(&ledc_channel); 
  vTaskDelete(NULL);
}

void BME280_Read(void *vparameters)
{ 
  for(;;) {
  uint8_t val;                  /* pointer to pass the data from registers */
  unsigned char x;             /* return value van de i2c routines */

  int16_t msb,lsb,xlsb;         /* temp registers for temp - pressure */
 
  uint16_t dig_T1;              /* compensatie registers */
  int16_t dig_T2,dig_T3;        /* compensatie registers */
  
  int32_t adc_T;                /* adc temp value */
  int32_t t_fine;            
  double var1, var2,T;
  
  //float temp;                   /* divide variables */
 
  uint16_t dig_P1;
  int16_t dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7,dig_P8,dig_P9;
  int32_t adc_P;
  double p;

  double adc_H;
  double var_H;
  uint8_t dig_H1,dig_H3,H4H5;
  int16_t dig_H2,dig_H4,dig_H5,dig_H6;

  x = i2c_master_read_slave_reg(I2C_NUM_0,0X76,0x88,&(val),24);   // Compensatie registers - for temp and press -> hum different registers
/*
  Serial.printf("Compensatie Temp address : 0x88  Value : %02X\n",val);
  Serial.printf("Compensatie Temp address : 0x89  Value : %02X\n",*(&val+1));
  Serial.printf("Compensatie Temp address : 0x8A  Value : %02X\n",*(&val+2));
  Serial.printf("Compensatie Temp address : 0x8B  Value : %02X\n",*(&val+3));
  Serial.printf("Compensatie Temp address : 0x8C  Value : %02X\n",*(&val+4));
  Serial.printf("Compensatie Temp address : 0x8D  Value : %02X\n\n",*(&val+5));
*/
  dig_T1 = ( *(&val+1) << 8 & 0xFF00 ) | ( val & 0x00FF ) ;
  dig_T2 = ( *(&val+3) << 8 & 0xFF00 ) | ( *(&val+2) & 0x00FF ) ;
  dig_T3 = ( *(&val+5) << 8 & 0xFF00 ) | ( *(&val+4) & 0x00FF ) ;

  dig_P1 = ( *(&val+7) << 8 & 0xFF00 ) | ( *(&val+6) & 0x00FF ) ;
  dig_P2 = ( *(&val+9) << 8 & 0xFF00 ) | ( *(&val+8) & 0x00FF ) ;
  dig_P3 = ( *(&val+11) << 8 & 0xFF00 ) | ( *(&val+10) & 0x00FF ) ;
  dig_P4 = ( *(&val+13) << 8 & 0xFF00 ) | ( *(&val+12) & 0x00FF ) ;
  dig_P5 = ( *(&val+15) << 8 & 0xFF00 ) | ( *(&val+14) & 0x00FF ) ;
  dig_P6 = ( *(&val+17) << 8 & 0xFF00 ) | ( *(&val+16) & 0x00FF ) ;
  dig_P7 = ( *(&val+19) << 8 & 0xFF00 ) | ( *(&val+18) & 0x00FF ) ;
  dig_P8 = ( *(&val+21) << 8 & 0xFF00 ) | ( *(&val+20) & 0x00FF ) ;
  dig_P9 = ( *(&val+23) << 8 & 0xFF00 ) | ( *(&val+22) & 0x00FF ) ;

  x = i2c_master_read_slave_reg(I2C_NUM_0,0X76,0xA1,&(val),1);    // compensatie registers for hum
  
  dig_H1 =  ( val & 0xFF ) ;

  x = i2c_master_read_slave_reg(I2C_NUM_0,0X76,0xE1,&(val),7);    // compensatie registers for hum
  
  dig_H2 = ( *(&val+1) << 8 & 0xFF00 ) | ( val & 0x00FF ) ;
  dig_H3 =  *(&val+2) & 0xFF ;
  
  H4H5 = *(&val+4) & 0xFF ;
  
  dig_H4 = ( *(&val+3) << 4 & 0xFFF0 ) | ( *(&val+4) & 0x000F ) ;
  dig_H5 = ( *(&val+5) << 4 & 0xFFF0 ) | ( H4H5 >> 4 & 0x000F ) ;
  
  dig_H6 = ( *(&val+6) & 0xFF ) ;

  x = i2c_master_read_slave_reg(I2C_NUM_0,0X76,0xFA,&(val),1) ;     //Serial.printf("address : 0xFA  Value : %02X\n",(val));
  x = i2c_master_read_slave_reg(I2C_NUM_0,0X76,0xFB,&(val),1);      //Serial.printf("address : 0xFB  Value : %02X\n",(val));
  x = i2c_master_read_slave_reg(I2C_NUM_0,0X76,0xFC,&(val),1);      //Serial.printf("address : 0xFC  Value : %02X\n",(val));
 
  i2c_master_write_slave_reg(I2C_NUM_0,0X76,0xF4,0b10110101);        //Serial.printf("Write at address : 0xF4  Value : B10100001\n"); // register to start reading temp/pres
 
  i2c_master_write_slave_reg(I2C_NUM_0,0X76,0xF2,0b00000101);        //Serial.printf("Write at address : 0xF2  Value : B10100001\n"); // register to start reading hum
 
  x = i2c_master_read_slave_reg(I2C_NUM_0,0X76,0xD0,&(val),1);      //Serial.printf("address : 0xD0  Value : %02X\n",(val)); // check the writen setting
  x = i2c_master_read_slave_reg(I2C_NUM_0,0X76,0xF4,&(val),1);      //Serial.printf("address : 0xF4  Value : %02X\n",(val)); // check the writen setting
  x = i2c_master_read_slave_reg(I2C_NUM_0,0X76,0xF2,&(val),1);      //Serial.printf("address : 0xF2  Value : %02X\n",(val)); // check the writen setting
  
  x       = i2c_master_read_slave_reg(I2C_NUM_0,0X76,0xFA,&(val),3);    // read the temp
  msb     = val;                                                    //Serial.printf("temp msb : %02X\n",msb);
  lsb     = *(&val+1);                                              //Serial.printf("temp lsb : %02X\n",lsb);
  xlsb    = *(&val+2);                                              //Serial.printf("tempxlsb : %02X\n",xlsb);

  adc_T   = ( (msb << 16) | (lsb << 8) | xlsb )>>4;
  var1    = ((adc_T)/16384.0 - (dig_T1)/1024.0) * (dig_T2);
  var2    = (((adc_T)/131072.0 - (dig_T1)/8192.0) * ((adc_T)/131072.0 - (dig_T1)/8192.0)) * (dig_T3);
  t_fine  = var1 + var2;
  T       = (var1 + var2)/5120.0;

  BME280_DATA.BME280_Temp = (float) T; 
  //ESP_LOGI("TAG_BME280","Temp Value : %.2f °C.\n",BME280_DATA.BME280_Temp); 
   
  /***************** PRESSURE ********************/

  x = i2c_master_read_slave_reg(I2C_NUM_0,0X76,0xF7,&(val),3);    // read the pres
  msb = val;                                                      // ESP_LOGI ("TEST","pres msb : %02X",msb);
  lsb = *(&val+1);                                                // ESP_LOGI ("TEST","pres lsb : %02X",lsb);
  xlsb = *(&val+2);                                               // ESP_LOGI ("TEST","pres xlsb : %02X\n",xlsb);

  adc_P = ( (msb << 16) | (lsb << 8) | xlsb )>>4;
  
  var1 = (t_fine / 2.0) - 64000;
  var2 = var1 * var1 * (dig_P6) / 32768.0;
  var2 = var2 + var1 * (dig_P5) *2;
  var2 = (var2 / 4.0) + ((dig_P4) * 65536.0);
  var1 = ((dig_P3) * var1 * var1 / 524288.0 + (dig_P2) * var1) / 524228.0;
  var1 = (1.0 + var1 / 32768.0)* (dig_P1);
//  if (var1 == 0) { return 0; }
  p = 1048576.0 - adc_P;
  p = (p - (var2 / 4096.0)) * 6250 / var1;
  var1 = ( dig_P9 ) *  p * p /2147483648.0;
  var2 =  p * ( dig_P8 ) / 32768.0;
  p = p + ( var1 + var2 + ( dig_P7 )) / 16.0;

  BME280_DATA.BME280_Pres = (float) p / 100; 
  //ESP_LOGI("TAG_BME280","Pressure Value : %.2f hPa\n",BME280_DATA.BME280_Pres); 

  /*************** humidity ***************/

  x = i2c_master_read_slave_reg(I2C_NUM_0,0X76,0xFD,&(val),2);    // read the hum
  msb = val;                                                     // ESP_LOGI ("TEST","pres msb : %02X",msb);
  lsb = *(&val+1);                                               // ESP_LOGI ("TEST","pres lsb : %02X",lsb);
  
  adc_H = ( (msb << 8) | (lsb) );

  var_H = (t_fine) - 76800.0;
  var_H = (adc_H - ((dig_H4) * 64.0 + (dig_H5) / 16384.0 * var_H)) * ((dig_H2) / 65536.0 * (1.0 + (dig_H6) / 67108864.0 * var_H * (1.0 + (dig_H3) / 67108864.0 * var_H)));
  var_H = var_H * (1.0 - (dig_H1)* var_H / 524288.0);
  if (var_H > 100.0) var_H = 100.0;
    else if (var_H < 0.0) var_H = 0.0;

  BME280_DATA.BME280_Hum = (float) var_H;
  //ESP_LOGI("TAG_BME280","Humidity Value : %.2f rH\n",BME280_DATA.BME280_Hum);

  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
}

void app_main(void)
{
  ESP_LOGI("TAG_APP_MAIN","Setup runs on core : %d\n",xPortGetCoreID());
    
  i2c_master_init(I2C_MASTER_SDA_IO,I2C_MASTER_SCL_IO,I2C_NUM_0); //Init bme280 i2c
  i2c_master_init(GPIO_NUM_15,GPIO_NUM_4,I2C_NUM_1);               //Init sd1604 i2c

  ssd1306_init();
  task_ssd1306_display_clear();
  task_ssd1306_contrast();

  xTaskCreate(&LedBlink_API,"LedBlink_API",1024,NULL,10,&Run_Blink);
  
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  xTaskCreatePinnedToCore(&BME280_Read,"BME280_Read",2048,NULL,10,&BME280_Th,0);
  
  xTaskCreatePinnedToCore(&task_ssd1306_display_text,"ssd1306_display_text",2048,&OLED_DATA_BME280[0],2,NULL,1);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  xTaskCreatePinnedToCore(&task_ssd1306_display_text,"ssd1306_display_text",2048,&OLED_DATA_BME280[1],2,NULL,1);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  xTaskCreatePinnedToCore(&task_ssd1306_display_text,"ssd1306_display_text",2048,&OLED_DATA_BME280[2],2,NULL,1);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  xTaskCreatePinnedToCore(&task_ssd1306_display_text,"ssd1306_display_text",2048,&OLED_DATA_BME280[3],2,NULL,1);
  vTaskDelay(50 / portTICK_PERIOD_MS);

  for(;;) 
  { 
    char stringVal[10];
    sprintf(stringVal, "%2.2f",BME280_DATA.BME280_Temp);
    //dtostrf(BME280_DATA.BME280_Temp,2,2,stringVal);
    OLED_DATA_BME280[4].Text  = stringVal;
    OLED_DATA_BME280[4].page  = 0x03;
    OLED_DATA_BME280[4].begin = 0x00;
    OLED_DATA_BME280[4].end   = 0x14;
    xTaskCreatePinnedToCore(&task_ssd1306_display_text,"ssd1306_display_text",2048,&OLED_DATA_BME280[4],2,NULL,1);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    sprintf(stringVal, "%4.0f",BME280_DATA.BME280_Pres);
    //dtostrf(BME280_DATA.BME280_Pres,4,2,stringVal);
    OLED_DATA_BME280[4].Text  = stringVal;
    OLED_DATA_BME280[4].page  = 0x05;
    OLED_DATA_BME280[4].begin = 0x08;
    OLED_DATA_BME280[4].end   = 0x13;
    xTaskCreatePinnedToCore(&task_ssd1306_display_text,"ssd1306_display_text",2048,&OLED_DATA_BME280[4],2,NULL,1);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    sprintf(stringVal, "%2.2f",BME280_DATA.BME280_Hum);
    //dtostrf(BME280_DATA.BME280_Hum,2,2,stringVal);
    OLED_DATA_BME280[4].Text  = stringVal;
    OLED_DATA_BME280[4].page  = 0x07;
    OLED_DATA_BME280[4].begin = 0x00;
    OLED_DATA_BME280[4].end   = 0x14;
    xTaskCreatePinnedToCore(&task_ssd1306_display_text,"ssd1306_display_text",2048,&OLED_DATA_BME280[4],2,NULL,1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}   
