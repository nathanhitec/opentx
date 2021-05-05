
if(PCBREV STREQUAL GCSV2)
  set(PWR_BUTTON "SWITCH" CACHE STRING "Pwr button type (PRESS/SWITCH)")
  set(STICKS "STANDARD" CACHE STRING "Sticks type (STANDARD/HORUS)")
  set(CPU_TYPE STM32F4)
  set(CPU_TYPE_FULL "STM32F407xE" CACHE STRING "Chip type (STM32F407xE/STM32F27xE)")
  set(LINKER_SCRIPT targets/hitec/stm32f4_flash.ld)
  add_definitions(-DSTM32F40_41xxx)
  set(EXTERNAL_MODULE YES)

  set(LUA no) #TODO: should I do this?
  set(LUA_EXPORT lua_export_tx12) #LUA_EXPORT reference

  set(FLAVOUR gcsv2) #FLAVOUR Should be ok, seems to only be used for text

  add_definitions(-DPCBX7) #TODO: referenced in 25 files

  add_definitions(-DRADIO_TX12) #TODO: referenced in 26 files

  set(NAVIGATION_TYPE x7)#TODO: Might need to implement own nav? Really don't want to

  add_definitions(-DMANUFACTURER_RADIOMASTER)#should be ok, only used for text

  add_definitions(-DPCBTARANIS) #TODO: referenced in 68 files
  add_definitions(-DPCBX7 -DSOFTWARE_VOLUME) #TODO: referenced in 7 files

  #TODO: is this ok for eeporm variant?
  add_definitions(-DEEPROM_VARIANT=0x4002)

  add_definitions(-DPWR_BUTTON_${PWR_BUTTON})


  add_definitions(-DAUDIO -DVOICE -DRTCLOCK)
  set(USB_CHARGER YES)

  set(FIRMWARE_TARGET_SRC
    ${FIRMWARE_TARGET_SRC}
    startup_stm32f40_41xxx.s
    )

  set(GUI_DIR 128x64)

  #TODO: need to see if this correct navigation type
  set(NAVIGATION_TYPE x9d)

  #TODO: Do custom bitmaps for hitec need to be made?
  set(BITMAPS_TARGET 9x_bitmaps)
  set(FONTS_TARGET 9x_fonts_1bit)

  set(LCD_DRIVER lcd_driver_spi.cpp)
  set(GVAR_SCREEN model_gvars.cpp)
  set(STATUS_LEDS YES)
  set(ROTARY_ENCODER YES)
else()
   message(FATAL_ERROR "Unknown PCBREV: '${PCBREV}' expecting GCSV2")
endif()


if(ROTARY_ENCODER)
  set(TARGET_SRC
    ${TARGET_SRC}
    ../common/arm/stm32/rotary_encoder_driver.cpp
    )
endif()

if(USB_CHARGER)
  set(TARGET_SRC ${TARGET_SRC} usb_charger_driver.cpp)
  add_definitions(-DUSB_CHARGER)
endif()

set(SRC
  ${SRC}
  io/frsky_firmware_update.cpp
  io/multi_firmware_update.cpp
  )

set(HSE_VALUE 12000000)
set(SDCARD YES)
set(EEPROM EEPROM_RLC)
set(TARGET_DIR hitec)

set(GUI_SRC
  ${GUI_SRC}
  model_input_edit.cpp
  model_mix_edit.cpp
  model_display.cpp
  radio_diagkeys.cpp
  radio_diaganas.cpp
  view_channels.cpp
  view_telemetry.cpp
  view_about.cpp
  bmp.cpp
  )

if(STATUS_LEDS)
  set(LED_DRIVER led_driver.cpp)
endif()


set(TARGET_SRC
  ${TARGET_SRC}
  ${LED_DRIVER}
  backlight_driver.cpp
  trainer_driver.cpp
  pi_driver.cpp
  debug_driver.cpp
  ../common/arm/stm32/timers_driver.cpp
  ../common/arm/stm32/audio_dac_driver.cpp
  ../common/arm/stm32/adc_driver.cpp
  ../common/arm/stm32/mixer_scheduler_driver.cpp
  )

set(TARGET_SRC
    ${TARGET_SRC}
    ../common/arm/stm32/sticks_pwm_driver.cpp
    )

set(FIRMWARE_TARGET_SRC
  ${FIRMWARE_TARGET_SRC}
  ${LCD_DRIVER}
  board.cpp
  i2c_driver.cpp
  ../common/arm/stm32/pwr_driver.cpp
  ../common/arm/loadboot.cpp
  )


if(EXTERNAL_MODULE)
  set(TARGET_SRC
    ${TARGET_SRC}
    extmodule_driver.cpp
  )
  add_definitions(-DHARDWARE_EXTERNAL_MODULE)
endif()

set(RADIO_DEPENDENCIES ${RADIO_DEPENDENCIES} ${BITMAPS_TARGET})
