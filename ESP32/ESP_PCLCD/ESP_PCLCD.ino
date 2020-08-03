#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "BluetoothSerial.h"
#include <map>

#include <inttypes.h>

#include <stdio.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//StaticJsonDocument<700> jsonBuffer;
// StaticJsonDocument<N> allocates memory on the stack, it can be
// replaced by DynamicJsonDocument which allocates in the heap.
//

DynamicJsonDocument jsonBuffer(4096);
BluetoothSerial SerialBT;

/*================================================================================var================================================================================*/

/*==========debug==========*/
String TestJson = "{\"Msg\": \"OK,\", \"Result\": {\"root\": {\"sys\": [{\"id\": \"SCPUCLK\", \"label\": \"CPU Clock\", \"value\": \"3989\"}, {\"id\": \"SCPUUTI\", \"label\": \"CPU Utilization\", \"value\": \"5\"}, {\"id\": \"SMEMUTI\", \"label\": \"Memory Utilization\", \"value\": \"29\"}, {\"id\": \"SGPU1UTI\", \"label\": \"GPU Utilization\", \"value\": \"21\"}], \"temp\": [{\"id\": \"TCPU\", \"label\": \"CPU\", \"value\": \"54\"}, {\"id\": \"TGPU1DIO\", \"label\": \"GPU Diode\", \"value\": \"49\"}], \"volt\": [{\"id\": \"VCPU\", \"label\": \"CPU Core\", \"value\": \"1.002\"}, {\"id\": \"VGPU1\", \"label\": \"GPU Core\", \"value\": \"0.569\"}], \"pwr\": [{\"id\": \"PCPUPKG\", \"label\": \"CPU Package\", \"value\": \"23.40\"}, {\"id\": \"PBATTCHR\", \"label\": \"Battery Charge Rate\", \"value\": \"0.00\"}]}}}";
const uint8_t DBG_LED_PKG[] = {0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0x0a};
uint8_t ii = 0;
uint8_t jj = 0;

/*==========dat==========*/
std::map<String, String> all_data_map;
/*==========sys==========*/

const String ID_SYS_CPU_Clock = "SCPUCLK";
const String ID_SYS_CPU_Used = "SCPUUTI";
const String ID_SYS_Memory_Used = "SMEMUTI";
const String ID_SYS_GPU_Used = "SGPU1UTI";

uint8_t IsHave_CPU_Clock = 0;
uint8_t IsHave_CPU_Used = 0;
uint8_t IsHave_Memory_Used = 0;
uint8_t IsHave_GPU_Used = 0;

uint16_t CPU_Clock = 0;
int8_t CPU_Used = 0;
int8_t Memory_Used = 0;
int8_t GPU_Used = 0;

/*==========temp==========*/

const String ID_TEMP_Temp_CPU = "TCPU";
const String ID_TEMP_Temp_GPUDiode = "TGPU1DIO";

uint8_t IsHave_Temp_CPU = 0;
uint8_t IsHave_Temp_GPUDiode = 0;

int16_t Temp_CPU = 0;
int16_t Temp_GPUDiode = 0;

/*==========volt==========*/

const String ID_VOLT_Volt_CPU_Core = "VCPU";
const String ID_VOLT_Volt_GPU_Core = "VGPU1";

uint8_t IsHave_Volt_CPU_Core = 0;
uint8_t IsHave_Volt_GPU_Core = 0;

double Volt_CPU_Core = 0.0;
double Volt_GPU_Core = 0.0;

/*==========pwr==========*/

const String ID_PWR_Pwr_CPU_Package = "PCPUPKG";
const String ID_PWR_Pwr_Battery = "PBATTCHR";

uint8_t IsHave_Pwr_CPU_Package = 0;
uint8_t IsHave_Pwr_Battery = 0;

double Pwr_CPU_Package = 0.0;
double Pwr_Battery = 0.0;

/*==========rtos==========*/
volatile SemaphoreHandle_t procSemaphore;
volatile SemaphoreHandle_t LEDReadySemaphore;

/*==========JsonTake==========*/
uint8_t AllowRecv = 0;
String RawBuffer = "";
String StrJsonBuffer = "";
JsonObject obj;

String JsonTakeID;
String JsonTakeVal;

/*==========LED==========*/

const uint8_t BRIGHT_COMMAND[8] = {0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f};

const uint8_t DIGMAP[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71};
const uint8_t BarMap[] = {1, 3, 5};

const uint16_t freqLED[10] = {300, 500, 1000, 1500, 2200, 2500, 3300, 4000, 4300, 5000};
const uint16_t tempLED[10] = {20, 25, 30, 40, 50, 60, 80, 100, 110, 120};

uint8_t LED_Buffer[15] = {0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0x0a};

int8_t DispCounter = 0;
uint8_t LEDBarData[3] = {0};   //bar1,bar2,bar3
uint8_t LEDDig[4][7] = {{0}};  //{Dig1,Dig2,Dig3,Dig4}
uint8_t LEDDigSingal[4] = {0}; //Dig1,Dig2,Dig3,Dig4
uint8_t LEDXStu[3] = {0};      //Dig1,Dig2,Dig3

uint8_t Bright = 7;//0->7
uint8_t autoCounter = 1;

/*================================================================================func================================================================================*/

void TaskSerialGetJson(void *pvParameters);
void TaskDecodeJson(void *pvParameters);
void TaskDisplay(void *pvParameters);
void TaskKeyScan(void *pvParameters); 

uint8_t JsonBufTodata();

void dataToLEDDate();

void LED_DigWrite(uint8_t *x);
void LED_DigWrite(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
void Dot(uint8_t *x);
void Dot(uint8_t a, uint8_t b, uint8_t c, uint8_t d);

uint16_t inCodeLedBar(uint8_t b);
uint8_t inCodeLedBarVal(const uint16_t *sMap, uint16_t v);

void VoltCodex(uint8_t *Dst, double Res);
void pwrCodex(uint8_t *Dst, double Res);
void usedCodex(uint8_t *Dst, uint16_t Res);

void LedBarWrite(uint8_t loc, uint8_t b);
void LEDXwrite(uint8_t stu3, uint8_t FlagLED);

void LED_SendDate(uint8_t *buf);

/*================================================================================init================================================================================*/
void HWinit()
{
      pinMode(33, INPUT_PULLUP);
      pinMode(25, INPUT_PULLUP);
      pinMode(26, INPUT_PULLUP);
      pinMode(27, INPUT_PULLUP);
      pinMode(21, OUTPUT);
      pinMode(17, OUTPUT);
      pinMode(16, OUTPUT);
      pinMode(4, OUTPUT);
}

void PSerialinit()
{
      Serial1.begin(115200, SERIAL_8N1, 19, 22);
      Serial.begin(115200);
      Serial1.setRxBufferSize(1024);
      Serial.setRxBufferSize(1024);
}

/*================================================================================main================================================================================*/

void setup()
{
      HWinit();
      SerialBT.begin("ESP32"); //Bluetooth device name

      if (digitalRead(33) == 0)
      {
            digitalWrite(17, 1);
            while (1)
            {
                  pinMode(19, INPUT);
                  pinMode(22, INPUT);
                  delay(1);
            }
      }

      delay(100);
      PSerialinit();

      if (digitalRead(25) == 0)
      {
            digitalWrite(16, 1);
            while (1)
            {
                  if (Serial.available())
                        Serial1.write(Serial.read());
                  if (Serial1.available())
                        Serial.write(Serial1.read());
                  delay(1);
            }
      }

      for (uint8_t i = 0; i < 15; i++)
      {
            Serial1.write(DBG_LED_PKG[i]);
      }

      procSemaphore = xSemaphoreCreateBinary();
      LEDReadySemaphore = xSemaphoreCreateBinary(); //xSemaphoreCreateMutex();
      xSemaphoreGive(LEDReadySemaphore);

      xTaskCreatePinnedToCore(TaskSerialGetJson, "JsonGet", 4096, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
      xTaskCreatePinnedToCore(TaskDecodeJson, "JsonDecode", 4096, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
      xTaskCreatePinnedToCore(TaskDisplay, "Display", 1024, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
      xTaskCreatePinnedToCore(TaskKeyScan, "TaskKeyScan", 1024, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
}

void loop()
{
      // DeserializationError error = deserializeJson(jsonBuffer, TestJson);
      // if (error)
      // {
      //       Serial.println(F("Failed to read file, using default configuration"));
      //       Serial.println(error.c_str());
      // }
      // //JsonObject object = jsonBuffer.as<JsonObject>();
      // String res = jsonBuffer["Result"]["root"]["sys"][2];
      // Serial.println(res.c_str());


      vTaskDelay(10000);
}

/*================================================================================task================================================================================*/

void TaskSerialGetJson(void *pvParameters) // This is a task.
{
      char tempCh = 0;
      (void)pvParameters;
      for (;;) // A Task shall never return or exit.
      {
            if (Serial.available())
            {
                  tempCh = Serial.read();

                  if (tempCh == 0xa0)
                  {
                        AllowRecv = 1;
                  } //tempCh == '{'

                  if (AllowRecv == 1)
                  {
                        if (tempCh < 0x7f) // is tempCh in ASCII
                        {
                              RawBuffer += tempCh;
                        }

                        if (tempCh == '\n')
                        {
                              AllowRecv = 0;

                              StrJsonBuffer = RawBuffer;

                              RawBuffer = "";
                              xSemaphoreGive(procSemaphore);
                        } //tempCh == '\n'

                  } //AllowRecv == 1

            } //Serial1.available
            vTaskDelay(1);
      }
}

void TaskDecodeJson(void *pvParameters)
{
      (void)pvParameters;
      for (;;) // A Task shall never return or exit.
      {

            if (xSemaphoreTake(procSemaphore, 0) == pdTRUE)
            {
                  digitalWrite(4, 1);

                  // SerialBT.println(StrJsonBuffer.c_str());

                  DeserializationError error = deserializeJson(jsonBuffer, StrJsonBuffer);
                  if (error)
                  {
                        digitalWrite(4, 0);
                        digitalWrite(21, 1);
                        SerialBT.println(F("Failed to read file, using default configuration"));
                        SerialBT.println(error.c_str());
                        continue;
                  }
                  else
                  {

                        //if (xSemaphoreTake(LEDReadySemaphore, 500) == pdTRUE)
                        //{
                        JsonBufTodata();
                        dataToLEDDate();

                        xSemaphoreGive(LEDReadySemaphore);
                        //}
                        digitalWrite(21, 0);
                        digitalWrite(4, 0);
                  }
            } //xSemaphoreTake

            vTaskDelay(1);
      } //for
}

void TaskDisplay(void *pvParameters)
{
      (void)pvParameters;
      for (;;) // A Task shall never return or exit.
      {
            if (xSemaphoreTake(LEDReadySemaphore, 0) == pdTRUE)
            {
                  LED_DigWrite(LEDDig[0][DispCounter], LEDDig[1][DispCounter], LEDDig[2][DispCounter], LEDDig[3][DispCounter]);

                  LedBarWrite(BarMap[0], LEDBarData[0]);
                  LedBarWrite(BarMap[1], LEDBarData[1]);
                  LedBarWrite(BarMap[2], LEDBarData[2]);

                  uint8_t tempx = ((LEDXStu[0] ? 1 : 0) | ((LEDXStu[1] ? 1 : 0) << 1) | ((LEDXStu[2] ? 1 : 0) << 2)) & 0x07;

                  LEDXwrite(tempx, DispCounter);

                  if (Bright > 7)
                        Bright = 7;
                  if (Bright < 0)
                        Bright = 0;

                  LED_Buffer[13] = BRIGHT_COMMAND[Bright];

                  LED_SendDate(LED_Buffer);

                  if (autoCounter == 1)
                  {
                        DispCounter++;
                        if (DispCounter > 6)
                              DispCounter = 0;
                  }

                  //xSemaphoreGive(LEDReadySemaphore);

            } //xSemaphoreTake

            vTaskDelay(100);
      } //for
}

void TaskKeyScan(void *pvParameters)
{
      (void)pvParameters;
      for (;;) // A Task shall never return or exit.
      {
            if (digitalRead(33) == 0)
            { //bright
                  vTaskDelay(10);
                  if (digitalRead(33) == 0)
                  {
                        while (digitalRead(33) == 0)
                        {
                              vTaskDelay(1);
                        }
                        Bright++;
                        if (Bright > 7)
                              Bright = 0;
                  }
            }
            if (digitalRead(25) == 0)
            { //UP
                  if (digitalRead(25) == 0)
                  {
                        while (digitalRead(25) == 0)
                        {
                              vTaskDelay(1);
                        }
                        autoCounter = 0;
                        DispCounter++;
                        if (DispCounter > 6)
                              DispCounter = 0;
                  }
            }

            if (digitalRead(26) == 0)
            { //DOWN
                  if (digitalRead(26) == 0)
                  {
                        while (digitalRead(26) == 0)
                        {
                              vTaskDelay(1);
                        }
                        autoCounter = 0;
                        DispCounter--;
                        if (DispCounter < 0)
                              DispCounter = 6;
                  }
            }

            if (digitalRead(27) == 0)
            { //autoCounter
                  if (digitalRead(27) == 0)
                  {
                        while (digitalRead(27) == 0)
                        {
                              vTaskDelay(1);
                        }
                        autoCounter = 1;
                  }
            }

            vTaskDelay(1);
      }
}

/*================================================================================logic================================================================================*/

uint8_t JsonBufTodata()
{
      String res = jsonBuffer["Msg"];
      if (strncmp(res.c_str(), "OK", 2) == 0)
      {
            SerialBT.println("JSON_OK");
            obj = jsonBuffer.as<JsonObject>();

            //put Json res to HashMap

            //sys
            for (uint8_t i = 0; i < 4; i++)
            {
                  JsonTakeID = obj["Result"]["root"]["sys"][i]["id"].as<String>();
                  JsonTakeVal = obj["Result"]["root"]["sys"][i]["value"].as<String>();
                  all_data_map[JsonTakeID] = JsonTakeVal;
            }

            //temp
            for (uint8_t i = 0; i < 2; i++)
            {
                  JsonTakeID = obj["Result"]["root"]["temp"][i]["id"].as<String>();
                  JsonTakeVal = obj["Result"]["root"]["temp"][i]["value"].as<String>();
                  // SerialBT.println(i);
                  // SerialBT.println(JsonTakeID.c_str());
                  // SerialBT.println(JsonTakeVal.c_str());
                  all_data_map[JsonTakeID] = JsonTakeVal;
            }

            //volt
            for (uint8_t i = 0; i < 2; i++)
            {
                  JsonTakeID = obj["Result"]["root"]["volt"][i]["id"].as<String>();
                  JsonTakeVal = obj["Result"]["root"]["volt"][i]["value"].as<String>();
                  all_data_map[JsonTakeID] = JsonTakeVal;
            }

            //pwr
            for (uint8_t i = 0; i < 2; i++)
            {
                  JsonTakeID = obj["Result"]["root"]["pwr"][i]["id"].as<String>();
                  JsonTakeVal = obj["Result"]["root"]["pwr"][i]["value"].as<String>();
                  all_data_map[JsonTakeID] = JsonTakeVal;
            }

            //string to Number

            //=======================================sys
            IsHave_CPU_Clock = 0;
            IsHave_CPU_Used = 0;
            IsHave_Memory_Used = 0;
            IsHave_GPU_Used = 0;

            if (all_data_map[ID_SYS_CPU_Clock] != "")
            {
                  CPU_Clock = atoi(all_data_map[ID_SYS_CPU_Clock].c_str());
                  IsHave_CPU_Clock = 1;
            }

            if (all_data_map[ID_SYS_CPU_Used] != "")
            {
                  CPU_Used = atoi(all_data_map[ID_SYS_CPU_Used].c_str());
                  IsHave_CPU_Used = 1;
            }

            if (all_data_map[ID_SYS_Memory_Used] != "")
            {
                  Memory_Used = atoi(all_data_map[ID_SYS_Memory_Used].c_str());
                  IsHave_Memory_Used = 1;
            }

            if (all_data_map[ID_SYS_GPU_Used] != "")
            {
                  GPU_Used = atoi(all_data_map[ID_SYS_GPU_Used].c_str());
                  IsHave_GPU_Used = 1;
            }
            //=======================================temp
            IsHave_Temp_CPU = 0;
            IsHave_Temp_GPUDiode = 0;
            if (all_data_map[ID_TEMP_Temp_CPU] != "")
            {
                  Temp_CPU = atoi(all_data_map[ID_TEMP_Temp_CPU].c_str());
                  IsHave_Temp_CPU = 1;
            }

            if (all_data_map[ID_TEMP_Temp_GPUDiode] != "")
            {
                  Temp_GPUDiode = atoi(all_data_map[ID_TEMP_Temp_GPUDiode].c_str());
                  IsHave_Temp_GPUDiode = 1;
            }

            //=======================================volt
            IsHave_Volt_CPU_Core = 0;
            IsHave_Volt_GPU_Core = 0;
            if (all_data_map[ID_VOLT_Volt_CPU_Core] != "")
            {
                  Volt_CPU_Core = atof(all_data_map[ID_VOLT_Volt_CPU_Core].c_str());
                  IsHave_Volt_CPU_Core = 1;
            }

            if (all_data_map[ID_VOLT_Volt_GPU_Core] != "")
            {
                  Volt_GPU_Core = atof(all_data_map[ID_VOLT_Volt_GPU_Core].c_str());
                  IsHave_Volt_GPU_Core = 1;
            }

            //=======================================pwr
            IsHave_Pwr_CPU_Package = 0;
            IsHave_Pwr_Battery = 0;
            if (all_data_map[ID_PWR_Pwr_CPU_Package] != "")
            {
                  Pwr_CPU_Package = atof(all_data_map[ID_PWR_Pwr_CPU_Package].c_str());
                  IsHave_Pwr_CPU_Package = 1;
            }

            if (all_data_map[ID_PWR_Pwr_Battery] != "")
            {
                  Pwr_Battery = atof(all_data_map[ID_PWR_Pwr_Battery].c_str());
                  IsHave_Pwr_Battery = 1;
            }

      } //OK
      else
      {
            IsHave_Temp_CPU = 0;
            IsHave_Temp_GPUDiode = 0;
            IsHave_Volt_CPU_Core = 0;
            IsHave_Volt_GPU_Core = 0;
            IsHave_Pwr_CPU_Package = 0;
            IsHave_Pwr_Battery = 0;
            IsHave_CPU_Clock = 0;
            IsHave_CPU_Used = 0;
            IsHave_Memory_Used = 0;
            IsHave_GPU_Used = 0;
      } //ERR
      //SerialBT.println(res.c_str());
      return 1;
}

void dataToLEDDate()
{
      //LED  CPUTEMP
      if (IsHave_Temp_CPU == 1)
      {
            LEDXStu[0] = 1;
            LEDBarData[0] = inCodeLedBarVal(tempLED, Temp_CPU);
      }
      else
      {
            LEDXStu[0] = 0;
      }

      //LED  CPUFREQ
      if (IsHave_CPU_Clock == 1)
      {
            LEDXStu[1] = 1;
            LEDBarData[1] = inCodeLedBarVal(freqLED, CPU_Clock);
      }
      else
      {
            LEDXStu[1] = 0;
      }

      //LED  GPUTEMP
      if (IsHave_Temp_GPUDiode == 1)
      {
            LEDXStu[2] = 1;
            LEDBarData[2] = inCodeLedBarVal(tempLED, Temp_GPUDiode);
      }
      else
      {
            LEDXStu[2] = 0;
      }

      //GPU_Used
      if (IsHave_GPU_Used == 1)
      {
            usedCodex(LEDDigSingal, GPU_Used);
            for (uint8_t i = 0; i < 4; i++)
            {
                  LEDDig[i][0] = LEDDigSingal[i];
            }
      }
      else
      {
            for (uint8_t i = 0; i < 4; i++)
            {
                  LEDDig[i][0] = 0;
            }
      }

      //CPU_Used
      if (IsHave_CPU_Used == 1)
      {
            usedCodex(LEDDigSingal, CPU_Used);
            for (uint8_t i = 0; i < 4; i++)
            {
                  LEDDig[i][1] = LEDDigSingal[i];
            }
      }
      else
      {
            for (uint8_t i = 0; i < 4; i++)
            {
                  LEDDig[i][1] = 0;
            }
      }

      //Memory_Used
      if (IsHave_Memory_Used == 1)
      {
            usedCodex(LEDDigSingal, Memory_Used);
            for (uint8_t i = 0; i < 4; i++)
            {
                  LEDDig[i][2] = LEDDigSingal[i];
            }
      }
      else
      {
            for (uint8_t i = 0; i < 4; i++)
            {
                  LEDDig[i][2] = 0;
            }
      }

      //Volt_CPU_Core
      if (IsHave_Volt_CPU_Core == 1)
      {
            VoltCodex(LEDDigSingal, Volt_CPU_Core);
            for (uint8_t i = 0; i < 4; i++)
            {
                  LEDDig[i][3] = LEDDigSingal[i];
            }
      }
      else
      {
            for (uint8_t i = 0; i < 4; i++)
            {
                  LEDDig[i][3] = 0;
            }
      }

      //Volt_GPU_Core
      if (IsHave_Volt_GPU_Core == 1)
      {
            VoltCodex(LEDDigSingal, Volt_GPU_Core);
            for (uint8_t i = 0; i < 4; i++)
            {
                  LEDDig[i][4] = LEDDigSingal[i];
            }
      }
      else
      {
            for (uint8_t i = 0; i < 4; i++)
            {
                  LEDDig[i][4] = 0;
            }
      }

      //Pwr_CPU_Package
      if (IsHave_Pwr_CPU_Package == 1)
      {
            pwrCodex(LEDDigSingal, Pwr_CPU_Package);
            for (uint8_t i = 0; i < 4; i++)
            {
                  LEDDig[i][5] = LEDDigSingal[i];
            }
      }
      else
      {
            for (uint8_t i = 0; i < 4; i++)
            {
                  LEDDig[i][5] = 0;
            }
      }

      //Pwr_Battery
      if (IsHave_Pwr_Battery == 1)
      {
            pwrCodex(LEDDigSingal, abs(Pwr_Battery));
            for (uint8_t i = 0; i < 4; i++)
            {
                  LEDDig[i][6] = LEDDigSingal[i];
            }
      }
      else
      {
            for (uint8_t i = 0; i < 4; i++)
            {
                  LEDDig[i][6] = 0;
            }
      }
}

/*================================================================================LED_TOOLS================================================================================*/

void LED_DigWrite(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
      LED_Buffer[7] = a;
      LED_Buffer[8] = b;
      LED_Buffer[9] = c;
      LED_Buffer[10] = d;
}

void LED_DigWrite(uint8_t *x)
{
      for (uint8_t i = 0; i < 4; i++)
      {
            LED_Buffer[7 + i] = x[i];
      }
}

void Dot(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
      if (a != 0)
      {
            LED_Buffer[7] = LED_Buffer[7] | 0x80;
      }
      if (b != 0)
      {
            LED_Buffer[8] = LED_Buffer[8] | 0x80;
      }
      if (c != 0)
      {
            LED_Buffer[9] = LED_Buffer[9] | 0x80;
      }
      if (d != 0)
      {
            LED_Buffer[10] = LED_Buffer[10] | 0x80;
      }
}

void Dot(uint8_t *x)
{
      for (uint8_t i = 0; i < 4; i++)
      {
            if (x[i] != 0)
            {
                  LED_Buffer[7 + i] = LED_Buffer[7 + i] | 0x80;
            }
      }
}

uint16_t inCodeLedBar(uint8_t b)
{
      uint16_t tmp = 0;
      for (uint8_t i = 0; i < b; i++)
      {
            tmp |= 1U << i;
      }
      return tmp;
}

uint8_t inCodeLedBarVal(const uint16_t *sMap, uint16_t v)
{

      if (v > sMap[9])
      {
            return 10;
      }
      else if (v > sMap[8])
      {
            return 9;
      }
      else if (v > sMap[7])
      {
            return 8;
      }
      else if (v > sMap[6])
      {
            return 7;
      }
      else if (v > sMap[5])
      {
            return 6;
      }
      else if (v > sMap[4])
      {
            return 5;
      }
      else if (v > sMap[3])
      {
            return 4;
      }
      else if (v > sMap[2])
      {
            return 3;
      }
      else if (v > sMap[1])
      {
            return 2;
      }
      else if (v > sMap[0])
      {
            return 1;
      }
      else
      {
            return 0;
      }
      return 0;
}

void VoltCodex(uint8_t *Dst, double Res)
{ //x.xxx
      uint16_t temp16 = (int)(Res * 1000.0);

      uint8_t temp = temp16 % 10;
      Dst[3] = DIGMAP[temp];

      temp = (temp16 / 10) % 10;
      Dst[2] = DIGMAP[temp];

      temp = (temp16 / 100) % 10;
      Dst[1] = DIGMAP[temp];

      temp = (temp16 / 1000) % 10;
      Dst[0] = DIGMAP[temp] | 0x80;
}

void pwrCodex(uint8_t *Dst, double Res)
{ //xxx.x
      uint16_t temp16 = (int)(Res * 10.0);

      uint8_t temp = temp16 % 10;
      Dst[3] = DIGMAP[temp];

      temp = (temp16 / 10) % 10;
      Dst[2] = DIGMAP[temp] | 0x80;

      temp = (temp16 / 100) % 10;
      Dst[1] = DIGMAP[temp];

      temp = (temp16 / 1000) % 10;
      Dst[0] = DIGMAP[temp];
}

void usedCodex(uint8_t *Dst, uint16_t Res)
{ //xxxx
      uint8_t temp = Res % 10;
      Dst[3] = DIGMAP[temp];
      temp = (Res / 10) % 10;
      Dst[2] = DIGMAP[temp];
      temp = (Res / 100) % 10;
      Dst[1] = DIGMAP[temp];
      temp = (Res / 1000) % 10;
      Dst[0] = DIGMAP[temp];
}

void LedBarWrite(uint8_t loc, uint8_t b)
{
      if (b > 10)
            b = 10;
      uint16_t tmp = inCodeLedBar(b);
      uint8_t LSB8 = tmp & 0xff;
      uint8_t MSB2 = (tmp >> 8) & 0x03;

      LED_Buffer[loc] = LSB8;
      LED_Buffer[loc + 1] = MSB2;
}

void LEDXwrite(uint8_t stu3, uint8_t FlagLED)
{
      uint8_t tmp = stu3 & 0x07;

      if (FlagLED > 6)
            FlagLED = 6;

      uint16_t tmpBuf16 = 1U << FlagLED;
      tmpBuf16 = tmpBuf16 << 3;
      tmpBuf16 += tmp;

      uint8_t LSB8 = tmpBuf16 & 0xff;
      uint8_t MSB2 = (tmpBuf16 >> 8) & 0x03;

      LED_Buffer[11] = LSB8;
      LED_Buffer[12] = MSB2;
}

void LED_SendDate(uint8_t *buf)
{

      for (uint8_t i = 0; i < 15; i++)
      {
            Serial1.write(buf[i]);
      }
}

/*

  {"Msg": "Fail,", "Result": "NULL"}
  {"Msg": "OK,", "Result": {"root": {"sys": [{"id": "SCPUCLK", "label": "CPU Clock", "value": "3191"}, {"id": "SMEMCLK", "label": "Memory Clock", "value": "1330"}]}}}
  {"Msg": "OK,", "Result": {"root": {"sys": [{"id": "SCPUCLK", "label": "CPU Clock", "value": "3989"}, {"id": "SCPUUTI", "label": "CPU Utilization", "value": "5"}, {"id": "SMEMUTI", "label": "Memory Utilization", "value": "29"}, {"id": "SGPU1UTI", "label": "GPU Utilization", "value": "21"}], "temp": [{"id": "TCPU", "label": "CPU", "value": "54"}, {"id": "TGPU1DIO", "label": "GPU Diode", "value": "49"}], "volt": [{"id": "VCPU", "label": "CPU Core", "value": "1.002"}, {"id": "VGPU1", "label": "GPU Core", "value": "0.569"}], "pwr": [{"id": "PCPUPKG", "label": "CPU Package", "value": "23.40"}, {"id": "PBATTCHR", "label": "Battery Charge Rate", "value": "0.00"}]}}}
*/
