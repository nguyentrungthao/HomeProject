/**
 * @file DWIN.h
 * @author Trung Thao (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-04-21
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef DWIN_H
#define DWIN_H

#include <ctype.h>
#include <stdint.h>

#include <Arduino.h>
#include <HardwareSerial.h>
#include <FS.h>
#include <list>

#define CMD_HEAD1 0x5A
#define CMD_HEAD2 0xA5
#define CMD_WRITE 0x82
#define CMD_READ 0x83
#define CMD_TOUCH 0x83

#define DWIN_DEFAULT_BAUD_RATE 115200
#define ARDUINO_RX_PIN 10
#define ARDUINO_TX_PIN 11

struct TouchFrame_t
{
  uint16_t u16VPaddress;
  uint16_t u16KeyValue;
};
struct PendingRequest_t
{
  uint8_t u8cmd;
  uint16_t u16VPaddress;
  QueueHandle_t xQueueGetResponeData;
};

class DWIN
{
public:
  typedef void (*hmiListener)(String address, uint16_t lastBytes, String message, String response);

  DWIN(HardwareSerial &port, uint8_t receivePin = ARDUINO_RX_PIN, uint8_t transmitPin = ARDUINO_TX_PIN, long baud = DWIN_DEFAULT_BAUD_RATE, uint16_t sizeLeaseQueue = 10);

  void begin(uint32_t u32StackDepthReceive = 4096, BaseType_t xCoreID = tskNO_AFFINITY);
  void setupTouchCallBack(QueueHandle_t *pxTouchQueue, uint16_t sizeOfQueue);
  void setupTouchCallBack(hmiListener callBackTouch);

  void echoEnabled(bool enabled);
  void crcEnabled(bool enabled);

  esp_err_t sendArray(const uint8_t *dwinSendArray, uint8_t arraySize, uint8_t *pu8OutData = NULL, uint16_t u16OutDataSize = 0, uint16_t u16TimeOutInSecond = 1000);

  esp_err_t setVPWord(uint16_t address, uint16_t data, uint8_t *pu8OutData = NULL, uint16_t u16OutDataSize = 0, uint16_t u16TimeOutInSecond = 1000);
  esp_err_t readVPWord(uint16_t address, uint8_t numWords, uint8_t *pu8OutData = NULL, uint16_t u16OutDataSize = 0, uint16_t u16TimeOutInSecond = 1000);
  esp_err_t setVPByte(uint16_t address, uint8_t data, uint8_t *pu8OutData = NULL, uint16_t u16OutDataSize = 0, uint16_t u16TimeOutInSecond = 1000);

  // Get Version
  double getHWVersion();
  // get GUI software version
  double getGUISoftVersion();
  // restart HMI
  esp_err_t restartHMI();

  // set Particular Page
  esp_err_t setPage(uint8_t pageID);
  // get Current Page ID
  uint8_t getPage();

  // set LCD Brightness
  esp_err_t setBrightness(uint8_t pConstrast);
  // set LCD Brightness
  uint8_t getBrightness();

  // Play a sound
  esp_err_t playSound(uint8_t soundID);
  // beep Buzzer
  esp_err_t beepHMI(int time);

  // set the hardware RTC The first two digits of the year are automatically added
  esp_err_t setRTC(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
  // upHMI_DATE the software RTC The first two digits of the year are automatically added
  esp_err_t setRTCSOFT(uint8_t year, uint8_t month, uint8_t day, uint8_t weekday, uint8_t hour, uint8_t minute, uint8_t second);

  // set Data on VP Address
  esp_err_t setText(uint16_t address, String textData);
  // get Data on VP Address
  String getText(uint16_t vpAddress, uint8_t length);

  esp_err_t setTextColor(uint16_t spAddress, uint16_t spOffset, uint16_t color, uint8_t *pu8OutData = NULL, uint16_t u16OutDataSize = 0, uint16_t u16TimeOutInSecond = 1000);

  // esp_err_t setInt16Value(uint16_t vpAddress, int16_t value);
  // esp_err_t setInt32Value(uint16_t vpAddress, int32_t value);
  // esp_err_t setUint16Value(uint16_t vpAddress, uint16_t value);
  // esp_err_t setUint32Value(uint16_t vpAddress, uint32_t value);
  // esp_err_t setFloatValue(long vpAddress, float fValue);
  // int16_t getInt16Value(uint16_t vpAddress);
  // int32_t getInt32Value(uint16_t vpAddress);
  // uint16_t getUint16Value(uint16_t vpAddress);
  // uint32_t getUint32Value(uint16_t vpAddress);
  // float getFloatValue(uint16_t vpAddress);

  esp_err_t setGraphYCentral(uint16_t spAddr, int value);
  esp_err_t setGraphVDCentral(uint16_t spAddr, int value);
  esp_err_t setGraphMulY(uint16_t spAddr, int value);
  esp_err_t setGraphColor(uint16_t spAddr, int value);
  esp_err_t setGraphRightToLeft(uint16_t spAddr);
  esp_err_t setGraphLeftToRight(uint16_t spAddr);
  esp_err_t sendGraphValue(uint8_t channel, uint16_t value);
  esp_err_t sendGraphValue(uint16_t channel, const uint16_t *values, uint8_t valueCount);
  esp_err_t resetGraph(uint8_t channel);

  // Chỉ trả về False khi xảy ra lỗi trong quá trình cập nhật
  // Các trường hợp không tìm thấy thư mục hoặc file cập nhật
  // Tức là không có bản cập nhật mới sẽ trả về true
  esp_err_t updateHMI(fs::FS &filesystem, const char *dirPath);

private:
  HardwareSerial *_dwinSerial;
  uint8_t _rxPin, _txPin;
  long _baudrate;
  bool _isSoft; // Is serial interface software
  long _baud;   // DWIN HMI Baud rate
  bool _echo;   // Response Command Show
  bool _crc;
  int8_t u8CountFalse;
  hmiListener listenerCallback;
  QueueHandle_t xQueueTouch;
  QueueHandle_t xQueueCommandReadWrite;
  QueueHandle_t xQueueResponeDataEvent;
  SemaphoreHandle_t xMutexPendingRequest;

  static TaskHandle_t xTaskDWINHandle;

  bool isFirmwareFile(String &fileName);
  void flushSerial();
  void clearSerial();
  uint16_t u16CalculateCRCModbus(uint8_t *data, size_t length);

  static void xTaskReceiveUartEvent(void *ptr);
  void xHandle();
  static void vTriggerTaskReceiveFromUartEvent();
  uint16_t u6CalcuSizeOfResponeBuffer(const uint8_t *sendBuffer, uint16_t u16SizeOfBuffer);
};
#endif // DWIN_H
