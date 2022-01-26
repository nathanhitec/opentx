/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include <inttypes.h>
#include "definitions.h"
#include "opentx_constants.h"
#include "board_common.h"
#include "hal.h"

#if defined(ROTARY_ENCODER_NAVIGATION)
// Rotary Encoder driver
void rotaryEncoderInit();
void rotaryEncoderCheck();
#endif

#define FLASHSIZE                       0x80000
#define BOOTLOADER_SIZE                 0x8000
#define FIRMWARE_ADDRESS                0x08000000

#define LUA_MEM_MAX                     (0)    // max allowed memory usage for complete Lua  (in bytes), 0 means unlimited

#if defined(STM32F4)
  #define PERI1_FREQUENCY               42000000
  #define PERI2_FREQUENCY               84000000
#else
  #define PERI1_FREQUENCY               60000000
  #define PERI2_FREQUENCY               30000000
#endif

#define TIMER_MULT_APB1                 2
#define TIMER_MULT_APB2                 2

extern uint16_t sessionTimer;

// Board driver
void boardInit();
void boardOff();

// Timers driver
void init2MhzTimer();
void init5msTimer();


void initPiUART();
void sendRCChannelsOverMavlink(uint16_t* channel_data);
void sendHeartbeat();
void sendPulse();
void initPulse();
void MavlinkSendBuffer(const uint8_t * data, uint16_t size);
// PCBREV driver
enum {
  // X7
  PCBREV_X7_STD = 0,
  PCBREV_X7_40 = 1,
};

// SD driver
#define BLOCK_SIZE                      512 /* Block Size in Bytes */
#if !defined(SIMU) || defined(SIMU_DISKIO)
uint32_t sdIsHC();
uint32_t sdGetSpeed();
#define SD_IS_HC()                      (sdIsHC())
#define SD_GET_SPEED()                  (sdGetSpeed())
#define SD_GET_FREE_BLOCKNR()           (sdGetFreeSectors())
#else
#define SD_IS_HC()                      (0)
#define SD_GET_SPEED()                  (0)
#endif
#define __disk_read                     disk_read
#define __disk_write                    disk_write
#if defined(SIMU)
  #if !defined(SIMU_DISKIO)
    #define sdInit()
    #define sdDone()
  #endif
  #define sdMount()
  #define SD_CARD_PRESENT()               true
#else
void sdInit();
void sdMount();
void sdDone();
void sdPoll10ms();
uint32_t sdMounted();
#define SD_CARD_PRESENT()               ((SD_GPIO_PRESENT_GPIO->IDR & SD_GPIO_PRESENT_GPIO_PIN) == 1)
#endif

// Flash Write driver
#define FLASH_PAGESIZE 256
void unlockFlash();
void lockFlash();
void flashWrite(uint32_t * address, const uint32_t * buffer);
uint32_t isFirmwareStart(const uint8_t * buffer);
uint32_t isBootloaderStart(const uint8_t * buffer);

#define EXTERNAL_MODULE_ON()            EXTERNAL_MODULE_PWR_ON() 

#define INTERNAL_MODULE_ON()          false

#define INTERNAL_MODULE_OFF()         true

#if defined(EXTMODULE_USART)
#define EXTERNAL_MODULE_OFF()         extmoduleStop()
#else
#define EXTERNAL_MODULE_OFF()         EXTERNAL_MODULE_PWR_OFF()
#endif

#define IS_INTERNAL_MODULE_ON()         (GPIO_ReadInputDataBit(INTMODULE_PWR_GPIO, INTMODULE_PWR_GPIO_PIN) == Bit_SET)

void extmoduleSerialStart();
void extmoduleInvertedSerialStart(uint32_t baudrate);
void extmoduleSendBuffer(const uint8_t * data, uint8_t size);
void extmoduleSendNextFrame();
void extmoduleSendInvertedByte(uint8_t byte);

// Trainer driver
#define SLAVE_MODE()                    (g_model.trainerData.mode == TRAINER_MODE_SLAVE)

#define TRAINER_SELECT_SLAVE()         (GPIO_ReadInputDataBit(GPIOE, SWITCHES_GPIO_PIN_E) == Bit_RESET)


#if defined(TRAINER_DETECT_GPIO)
  // Trainer detect is a switch on the jack
  #define TRAINER_CONNECTED()          (GPIO_ReadInputDataBit(TRAINER_DETECT_GPIO, TRAINER_DETECT_GPIO_PIN) == Bit_RESET)
#elif defined(PCBXLITES)
  // Trainer is on the same connector than Headphones
  enum JackState
  {
    SPEAKER_ACTIVE,
    HEADPHONE_ACTIVE,
    TRAINER_ACTIVE,
  };
  extern uint8_t jackState;
  #define TRAINER_CONNECTED()           (jackState == TRAINER_ACTIVE)
#elif defined(PCBXLITE)
  // No Tainer jack on Taranis X-Lite
  #define TRAINER_CONNECTED()           false
#else
  // Trainer detect catches PPM, detection would use more CPU
  #define TRAINER_CONNECTED()           true
#endif

#if defined(TRAINER_GPIO)
  void init_trainer_ppm();
  void stop_trainer_ppm();
  void init_trainer_capture();
  void stop_trainer_capture();
#else
  #define init_trainer_ppm()
  #define stop_trainer_ppm()
  #define init_trainer_capture()
  #define stop_trainer_capture()
#endif
#if defined(TRAINER_MODULE_CPPM)
  void init_trainer_module_cppm();
  void stop_trainer_module_cppm();
#else
  #define init_trainer_module_cppm()
  #define stop_trainer_module_cppm()
#endif
#if defined(TRAINER_MODULE_SBUS)
  void init_trainer_module_sbus();
  void stop_trainer_module_sbus();
#else
  #define init_trainer_module_sbus()
  #define stop_trainer_module_sbus()
#endif

#if defined(INTMODULE_HEARTBEAT_GPIO)
void init_intmodule_heartbeat();
void stop_intmodule_heartbeat();
void check_intmodule_heartbeat();
#else
#define init_intmodule_heartbeat()
#define stop_intmodule_heartbeat()
#define check_intmodule_heartbeat()
#endif

void check_telemetry_exti();

// SBUS
int sbusGetByte(uint8_t * byte);

// Keys driver
enum EnumKeys
{
#if defined(KEYS_GPIO_REG_MENU)
  KEY_MENU,
#endif

  KEY_EXIT,
  KEY_ENTER,

#if defined(KEYS_GPIO_REG_PAGE)
  KEY_PAGE,
#endif


#if defined(KEYS_GPIO_REG_PLUS)
  KEY_PLUS,
  KEY_MINUS,
#endif

  KEY_COUNT,
  KEY_MAX = KEY_COUNT - 1,
  TRM_BASE,
  TRM_LH_DWN = TRM_BASE,
  TRM_LH_UP,
  TRM_LV_DWN,
  TRM_LV_UP,
  TRM_RV_DWN,
  TRM_RV_UP,
  TRM_RH_DWN,
  TRM_RH_UP,
  TRM_LAST = TRM_RH_UP,
  
  NUM_KEYS
};


  #define KEY_UP                        KEY_PLUS
  #define KEY_DOWN                      KEY_MINUS
  #define KEY_RIGHT                     KEY_MINUS
  #define KEY_LEFT                      KEY_PLUS


#if defined(KEYS_GPIO_PIN_SHIFT)
#define IS_SHIFT_KEY(index)             (index == KEY_SHIFT)
#if defined(SIMU)
#define IS_SHIFT_PRESSED()              (readKeys() & (1 << KEY_SHIFT))
#else
#define IS_SHIFT_PRESSED()              (~KEYS_GPIO_REG_SHIFT & KEYS_GPIO_PIN_SHIFT)
#endif
#else
#define IS_SHIFT_KEY(index)             (false)
#define IS_SHIFT_PRESSED()              (false)
#endif

enum EnumSwitches
{
  SW_SA,
  SW_SB,
  SW_SC,
  SW_SD,
  SW_SE
};

  #define IS_3POS(x)                      ((x) == SW_SB || (x) == SW_SC)

enum EnumSwitchesPositions
{
  SW_SA0,
  SW_SA1,
  SW_SA2,
  SW_SB0,
  SW_SB1,
  SW_SB2,
  SW_SC0,
  SW_SC1,
  SW_SC2,
  SW_SD0,
  SW_SD1,
  SW_SD2,
  SW_SE0,
  SW_SE1,
  SW_SE2,
  NUM_SWITCHES_POSITIONS
};


  #define NUM_SWITCHES                  5
  #define STORAGE_NUM_SWITCHES          5

  //TODO: might have to change switch, pots and sliders config
  //#define DEFAULT_SWITCH_CONFIG         (SWITCH_2POS << 8) + (SWITCH_2POS << 6) + (SWITCH_3POS << 4) + (SWITCH_3POS << 2) + (SWITCH_2POS << 0);
  //#define DEFAULT_POTS_CONFIG           (POT_WITH_DETENT << 0) + (POT_WITH_DETENT << 2);
  //#define DEFAULT_SLIDERS_CONFIG        (SLIDER_WITH_DETENT << 1) + (SLIDER_WITH_DETENT << 0)

#define STORAGE_NUM_SWITCHES_POSITIONS  (15)

void keysInit();
uint32_t switchState(uint8_t index);
uint32_t readKeys();
uint32_t readTrims();
#define TRIMS_PRESSED()                 (readTrims())
#define KEYS_PRESSED()                  (readKeys())

// WDT driver
#define WDG_DURATION                      500 /*ms*/
#if !defined(WATCHDOG) || defined(SIMU)
  #define WDG_ENABLE(x)
  #define WDG_RESET()
#else
  #define WDG_ENABLE(x)                 watchdogInit(x)
  #define WDG_RESET()                   IWDG->KR = 0xAAAA

#endif
void watchdogInit(unsigned int duration);
#define WAS_RESET_BY_SOFTWARE()             (RCC->CSR & RCC_CSR_SFTRSTF)
#define WAS_RESET_BY_WATCHDOG()             (RCC->CSR & (RCC_CSR_WDGRSTF | RCC_CSR_WWDGRSTF))
#define WAS_RESET_BY_WATCHDOG_OR_SOFTWARE() (RCC->CSR & (RCC_CSR_WDGRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_SFTRSTF))

// ADC driver
enum Analogs {
  STICK1,
  STICK2,
  STICK3,
  STICK4,
  POT_FIRST,
  POT1 = POT_FIRST,
  POT_LAST = POT1,
  SLIDER1,
  SLIDER2,
  TX_VOLTAGE,
  TX_RTC_VOLTAGE,
  NUM_ANALOGS
};


#define NUM_POTS                      1
#define NUM_SLIDERS                   2

#define STORAGE_NUM_POTS              1
#define STORAGE_NUM_SLIDERS           2

#define NUM_XPOTS                     0//STORAGE_NUM_POTS

#define NUM_MOUSE_ANALOGS             0
#define STORAGE_NUM_MOUSE_ANALOGS     0

#define NUM_TRIMS                     4
#define NUM_TRIMS_KEYS                4*2


#define STICKS_PWM_ENABLED()          false

PACK(typedef struct {
  uint8_t pcbrev:2;
  uint8_t sticksPwmDisabled:1;
  uint8_t pxx2Enabled:1;
}) HardwareOptions;

extern HardwareOptions hardwareOptions;


  #define IS_PXX2_INTERNAL_ENABLED()            (false)
  #define IS_PXX1_INTERNAL_ENABLED()            (false)


enum CalibratedAnalogs {
  CALIBRATED_STICK1,
  CALIBRATED_STICK2,
  CALIBRATED_STICK3,
  CALIBRATED_STICK4,
  CALIBRATED_POT_FIRST,
  CALIBRATED_POT_LAST = CALIBRATED_POT_FIRST + NUM_POTS - 1,
  CALIBRATED_SLIDER_FIRST,
  CALIBRATED_SLIDER_LAST = CALIBRATED_SLIDER_FIRST + NUM_SLIDERS - 1,
  NUM_CALIBRATED_ANALOGS
};


#define IS_POT(x)                     ((x)>=POT_FIRST && (x)<=POT_LAST)
#define IS_SLIDER(x)                  ((x)>POT_LAST && (x)<TX_VOLTAGE)

extern uint16_t adcValues[NUM_ANALOGS];

//TODO: might have to change battery driver config
// Battery driver
  // NI-MH 9.6V
  #define BATTERY_WARN                  87 // 8.7V
  #define BATTERY_MIN                   85 // 8.5V
  #define BATTERY_MAX                   115 // 11.5V
  #define BATT_SCALE                    150

#if defined(__cplusplus) && !defined(SIMU)
extern "C" {
#endif

// Power driver
#define SOFT_PWR_CTRL
void pwrInit();
uint32_t pwrCheck();
void pwrOn();
void pwrOff();
bool pwrPressed();
#if defined(PWR_BUTTON_PRESS)
#define STARTUP_ANIMATION
uint32_t pwrPressedDuration();
#endif
void pwrResetHandler();
#define pwrForcePressed()   false

#if defined(SIMU)
#define UNEXPECTED_SHUTDOWN()           false
#else
#define UNEXPECTED_SHUTDOWN()           (WAS_RESET_BY_WATCHDOG() || g_eeGeneral.unexpectedShutdown)
#endif

// Backlight driver
void backlightInit();
void backlightDisable();
#define BACKLIGHT_DISABLE()             backlightDisable()
#define BACKLIGHT_FORCED_ON             101
uint8_t isBacklightEnabled();
#if !defined(__cplusplus)
  #define backlightEnable(...)
#elif defined(PCBX9E) || defined(PCBX9DP)
  void backlightEnable(uint8_t level = 0, uint8_t color = 0);
  #define BACKLIGHT_ENABLE()            backlightEnable(currentBacklightBright, g_eeGeneral.backlightColor)
#else
  void backlightEnable(uint8_t level = 0);
  #define BACKLIGHT_ENABLE()            backlightEnable(currentBacklightBright)
#endif

#if !defined(SIMU)
  void usbJoystickUpdate();
#endif

  #define USB_NAME                     "Hitec GCSv2"
  #define USB_MANUFACTURER             'H', 'i', 't', 'e', 'c', ' ', ' ', ' '  /* 8 bytes */
  #define USB_PRODUCT                  'G', 'C', 'S', 'V', '2', ' ', ' ', ' '  /* 8 Bytes */


#if defined(__cplusplus) && !defined(SIMU)
}
#endif

// I2C driver: EEPROM + Audio Volume
#define EEPROM_SIZE                   (32*1024)

void i2cInit();
void eepromReadBlock(uint8_t * buffer, size_t address, size_t size);
void eepromStartWrite(uint8_t * buffer, size_t address, size_t size);
uint8_t eepromIsTransferComplete();

// Debug driver
void debugPutc(const char c);

// Telemetry driver
void telemetryPortInit(uint32_t baudrate, uint8_t mode);
void telemetryPortSetDirectionInput();
void telemetryPortSetDirectionOutput();
void sportSendByte(uint8_t byte);
void sportSendByteLoop(uint8_t byte);
void sportStopSendByteLoop();
void sportSendBuffer(const uint8_t * buffer, uint32_t count);
bool telemetryGetByte(uint8_t * byte);
void telemetryClearFifo();
extern uint32_t telemetryErrors;

// soft-serial
void telemetryPortInvertedInit(uint32_t baudrate);


  #define HAS_SPORT_UPDATE_CONNECTOR()  false


// Sport update driver
#if defined(SPORT_UPDATE_PWR_GPIO)
void sportUpdateInit();
void sportUpdatePowerOn();
void sportUpdatePowerOff();
void sportUpdatePowerInit();
#define SPORT_UPDATE_POWER_ON()         sportUpdatePowerOn()
#define SPORT_UPDATE_POWER_OFF()        sportUpdatePowerOff()
#define SPORT_UPDATE_POWER_INIT()       sportUpdatePowerInit()
#define IS_SPORT_UPDATE_POWER_ON()      (GPIO_ReadInputDataBit(SPORT_UPDATE_PWR_GPIO, SPORT_UPDATE_PWR_GPIO_PIN) == Bit_SET)
#else
#define sportUpdateInit()
#define SPORT_UPDATE_POWER_ON()
#define SPORT_UPDATE_POWER_OFF()
#define SPORT_UPDATE_POWER_INIT()
#define IS_SPORT_UPDATE_POWER_ON()      (false)
#endif

// Audio driver
void audioInit() ;
void audioEnd() ;
void dacStart();
void dacStop();
void setSampleRate(uint32_t frequency);
#define VOLUME_LEVEL_MAX  23
#define VOLUME_LEVEL_DEF  12
#if !defined(SOFTWARE_VOLUME)
void setScaledVolume(uint8_t volume);
void setVolume(uint8_t volume);
int32_t getVolume();
#endif
#if defined(AUDIO_SPEAKER_ENABLE_GPIO)
void initSpeakerEnable();
void enableSpeaker();
void disableSpeaker();
#else
static inline void initSpeakerEnable() { }
static inline void enableSpeaker() { }
static inline void disableSpeaker() { }
#endif
#if defined(HEADPHONE_TRAINER_SWITCH_GPIO)
void initHeadphoneTrainerSwitch();
void enableHeadphone();
void enableTrainer();
#else
static inline void initHeadphoneTrainerSwitch() { }
static inline void enableHeadphone() { }
static inline void enableTrainer() { }
#endif
#if defined(JACK_DETECT_GPIO)
void initJackDetect();
bool isJackPlugged();
#endif
void audioConsumeCurrentBuffer();
#define audioDisableIrq()               __disable_irq()
#define audioEnableIrq()                __enable_irq()

// Haptic driver
void hapticInit();
void hapticOff();
#if defined(HAPTIC_PWM)
  void hapticOn(uint32_t pwmPercent);
#else
  void hapticOn();
#endif

// Aux serial port driver
#if defined(AUX_SERIAL_GPIO)
#define DEBUG_BAUDRATE                  115200
#define LUA_DEFAULT_BAUDRATE            115200
#define AUX_SERIAL
extern uint8_t auxSerialMode;
#if defined __cplusplus
void auxSerialSetup(unsigned int baudrate, bool dma, uint16_t length = USART_WordLength_8b, uint16_t parity = USART_Parity_No, uint16_t stop = USART_StopBits_1);
#endif
void auxSerialInit(unsigned int mode, unsigned int protocol);
void auxSerialPutc(char c);
#define auxSerialTelemetryInit(protocol) auxSerialInit(UART_MODE_TELEMETRY, protocol)
void auxSerialSbusInit();
void auxSerialStop();
#define AUX_SERIAL_POWER_ON()
#define AUX_SERIAL_POWER_OFF()
#endif


// BT driver
#define BLUETOOTH_BOOTLOADER_BAUDRATE   230400
#define BLUETOOTH_DEFAULT_BAUDRATE      115200
#if defined(PCBX9E)
#define BLUETOOTH_FACTORY_BAUDRATE      9600
#else
#define BLUETOOTH_FACTORY_BAUDRATE      57600
#endif
#define BT_TX_FIFO_SIZE    64
#define BT_RX_FIFO_SIZE    256
void bluetoothInit(uint32_t baudrate, bool enable);
void bluetoothWriteWakeup();
uint8_t bluetoothIsWriting();
void bluetoothDisable();

#if defined(BLUETOOTH_PROBE) && !defined(SIMU)
  extern volatile uint8_t btChipPresent;
  #define IS_BLUETOOTH_CHIP_PRESENT()     (btChipPresent)
#else
  #define IS_BLUETOOTH_CHIP_PRESENT()     (false)
#endif

// USB Charger
#if defined(USB_CHARGER)
void usbChargerInit();
bool usbChargerLed();
#endif

// LED driver
void ledInit();
void ledOff();
void ledRed();
void ledGreen();
void ledBlue();

// LCD driver
#define LCD_W                           212
#define LCD_H                           64
#define LCD_DEPTH                       4
#define LCD_CONTRAST_MIN                0
#define LCD_CONTRAST_MAX                45
#define LCD_CONTRAST_DEFAULT            25
#define IS_LCD_RESET_NEEDED()           (!WAS_RESET_BY_WATCHDOG_OR_SOFTWARE())


void lcdInit();
void lcdInitFinish();
void lcdOff();
void baroInit();


//used for x9e hijack, blank function defs
void toplcdInit();
void toplcdOff();
void toplcdRefreshStart();
void toplcdRefreshEnd();
void setTopFirstTimer(int32_t value);
void setTopSecondTimer(uint32_t value);
void setTopRssi(uint32_t rssi);
void setTopBatteryState(int state, uint8_t blinking);
void setTopBatteryValue(uint32_t volts);

// TODO lcdRefreshWait() stub in simpgmspace and remove LCD_DUAL_BUFFER
#if defined(LCD_DMA) && !defined(LCD_DUAL_BUFFER) && !defined(SIMU)
void lcdRefreshWait();
#else
#define lcdRefreshWait()
#endif
#if defined(PCBX9D) || defined(SIMU) || !defined(__cplusplus)
void lcdRefresh();
#else
void lcdRefresh(bool wait=true); // TODO uint8_t wait to simplify this
#endif
void lcdSetRefVolt(unsigned char val);
void lcdSetContrast();

// Top LCD driver
#if defined(TOPLCD_GPIO)
void toplcdInit();
void toplcdOff();
void toplcdRefreshStart();
void toplcdRefreshEnd();
void setTopFirstTimer(int32_t value);
void setTopSecondTimer(uint32_t value);
void setTopRssi(uint32_t rssi);
void setTopBatteryState(int state, uint8_t blinking);
void setTopBatteryValue(uint32_t volts);
#endif

#define USART_FLAG_ERRORS (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)

#if defined(__cplusplus)
#include "fifo.h"
#include "dmafifo.h"

#define TELEMETRY_FIFO_SIZE             64

extern Fifo<uint8_t, TELEMETRY_FIFO_SIZE> telemetryFifo;
typedef DMAFifo<32> AuxSerialRxFifo;
extern AuxSerialRxFifo auxSerialRxFifo;
#endif


#endif // _BOARD_H_