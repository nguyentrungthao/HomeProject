#include "triacDimming.h"
#include <Arduino.h>

gpio_num_t triac::acdet = -1;
// uint8_t triac::u8Frequence = 0;
uint16_t triac::u16TriacHighLimit = 9600; // mặc định ở tần số 50Hz 
static triac* triac_set[5] = {};
static uint8_t num_triac = 0;

// phục vụ cho việc nhận dạng tần số
#define triacSAMPLES_TO_AVERAGE 10
struct FrequenceMeasure_t {
  volatile uint32_t previousInterruptMicros;
  volatile uint32_t sumOfIntervals;
  volatile uint32_t counter;
};

triac::triac(gpio_num_t pin, timer_group_t grp, timer_idx_t idx) {
  this->pin = pin;
  this->grp = grp;
  this->idx = idx;
}

void triac::configACDETPIN(gpio_num_t acdet_pin) {
  triac::acdet = acdet_pin;
  vACDetFrequenceMeasure(triac::acdet);
  
  attachInterruptArg(triac::acdet, acdet_intr_handler_in_isr, NULL, FALLING);
}
void triac::vACDetFrequenceMeasure(gpio_num_t acdet) {
  volatile FrequenceMeasure_t xFrequenceMeasure = {};
  attachInterruptArg(acdet, acdet_frequence_measure_callback, (void*)&xFrequenceMeasure, FALLING);
  do {
    delay(120);
  } while (xFrequenceMeasure.counter < triacSAMPLES_TO_AVERAGE);
  detachInterrupt(acdet);

  uint32_t timeBetweenInterrupts = xFrequenceMeasure.sumOfIntervals / triacSAMPLES_TO_AVERAGE;
  if (timeBetweenInterrupts > 9800) {
    // triac::u8Frequence = 50;
    triac::u16TriacHighLimit = 9600; // biểu thị cho tần số 50Hz 
  } else {
    // triac::u8Frequence = 60;
    triac::u16TriacHighLimit = 7500; // biểu thị cho tần số 60Hz 
  }
}
void IRAM_ATTR triac::acdet_frequence_measure_callback(void* ptr) {
  unsigned long currentMicros = micros();
  volatile FrequenceMeasure_t* pxFrequenceMeasure = (FrequenceMeasure_t*)ptr;
  if (pxFrequenceMeasure->previousInterruptMicros != 0 && pxFrequenceMeasure->counter < triacSAMPLES_TO_AVERAGE) {
    pxFrequenceMeasure->sumOfIntervals = pxFrequenceMeasure->sumOfIntervals + (currentMicros - pxFrequenceMeasure->previousInterruptMicros);
    pxFrequenceMeasure->counter++;
  }
  pxFrequenceMeasure->previousInterruptMicros = currentMicros;
}


void triac::init(void) {
  pinMode(this->pin, OUTPUT);
  digitalWrite(this->pin, 0);

  timer_config_t triac1_tim_conf = {
    .alarm_en = TIMER_ALARM_EN,
    .counter_en = TIMER_PAUSE,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = TIMER_AUTORELOAD_EN,
    .divider = 80,
  };
  timer_init(this->grp, this->idx, &triac1_tim_conf);
  timer_set_counter_value(this->grp, this->idx, 0);
  timer_isr_callback_add(this->grp, this->idx, timer_triac_intr_handler, (void*)this, ESP_INTR_FLAG_IRAM);
  timer_enable_intr(this->grp, this->idx);

  triac_set[num_triac++] = this;
  xTaskCreate(TimerTimoutTask, "TimeOutTask", 2048, this, configMAX_PRIORITIES - 1, &TimerTimoutTaskHandle);
}


void triac::SetTimeOverFlow(uint16_t timeOverFlow) {
  if (timeOverFlow > u16TriacHighLimit) timeOverFlow = u16TriacHighLimit;
  if (timeOverFlow < TRIAC_LOW_LIMIT) timeOverFlow = TRIAC_LOW_LIMIT;
  this->timeOverFlow = timeOverFlow;
}


void triac::TurnOnTriac() {
  this->RunStatus = true;
  digitalWrite(this->pin, 0);
}
void triac::TurnOffTriac() {
  this->RunStatus = false;
  digitalWrite(this->pin, 0);
}

void IRAM_ATTR triac::acdet_intr_handler_in_isr(void* arg) {
  if (digitalRead(triac::acdet)) return;

  for (uint8_t i = 0; i < num_triac; i++) {
    triac* ptriac = triac_set[i];

    ptriac->dis_timer = false;
    digitalWrite(ptriac->pin, 0);
    if (ptriac->RunStatus == true && ptriac->timeOverFlow < u16TriacHighLimit) {
      timer_group_set_alarm_value_in_isr(ptriac->grp, ptriac->idx, ptriac->timeOverFlow);
      timer_start(ptriac->grp, ptriac->idx);
    }
  }
}

void triac::acdet_intr_handler_in_task() {
  if (digitalRead(triac::acdet)) return;

  for (uint8_t i = 0; i < num_triac; i++) {
    triac* ptriac = triac_set[i];

    ptriac->dis_timer = false;
    digitalWrite(ptriac->pin, 0);
    if (ptriac->RunStatus == true && ptriac->timeOverFlow < u16TriacHighLimit) {
      timer_set_alarm_value(ptriac->grp, ptriac->idx, ptriac->timeOverFlow);
      timer_start(ptriac->grp, ptriac->idx);
    }
  }
}

bool IRAM_ATTR triac::timer_triac_intr_handler(void* arg) {
  if (arg == NULL) return false;
  triac* ptriac = (triac*)arg;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(ptriac->TimerTimoutTaskHandle, 0x01, eSetBits, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

  return false;
}

void triac::TimerTimoutTask(void* ptr) {
  if (ptr == NULL) return;
  triac* pTriac = (triac*)ptr;
  uint32_t notifyNum;
  while (1) {
    xTaskNotifyWait(pdFALSE, pdTRUE, &notifyNum, portMAX_DELAY);
    timer_pause(pTriac->grp, pTriac->idx);
    gpio_set_level(pTriac->pin, 1);
    delayMicroseconds(triacHOLD_TIME);
    gpio_set_level(pTriac->pin, 0);
  }
}