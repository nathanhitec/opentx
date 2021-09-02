
set(EXTERNAL_MODULE YES)
set(SPLASH DEFAULT)
option(AUTOUPDATE "Auto update internal chips from SD" OFF)
option(PXX1 "PXX1 protocol support" OFF)
option(PXX2 "PXX2 protocol support" OFF)
option(AFHDS3 "AFHDS3 TX Module" OFF)
option(GHOST "Ghost TX Module" OFF)
option(INTERNAL_MODULE_PPM "Support for PPM internal module" OFF)
option(LCD_DUAL_BUFFER "Dual LCD Buffer" OFF)

if(PCBREV STREQUAL GCSV2)
  set(PWR_BUTTON SWITCH)
  set(STICKS STANDARD)

  #hijacking x9e model definitions for now
  set(LUA_EXPORT lua_export_x9e) 
  set(FLAVOUR gcsv2)
  add_definitions(-DPCBX9E -DPCBX9 -DGCSV2)
 
   #TODO: is this the correct eeporm variant?
  add_definitions(-DEEPROM_VARIANT=0x8000)
  add_definitions(-DPWR_BUTTON_${PWR_BUTTON})

   set(CPU_TYPE STM32F4)
  set(CPU_TYPE_FULL "STM32F407xE" CACHE STRING "CPU Type (STM32F407xE/STM32F27xE)")
  set(USB_CHARGER YES)
 

  set(GUI_DIR 212x64)
  set(NAVIGATION_TYPE x9d) 

  set(BITMAPS_TARGET taranis_bitmaps)
  set(FONTS_TARGET taranis_fonts)

  set(LCD_DRIVER lcd_driver_spi.cpp)
  set(GVAR_SCREEN model_gvars.cpp)
  set(STATUS_LEDS YES)
  

  set(ROTARY_ENCODER YES)
  add_definitions(-DMANUFACTURER_HITEC)
  add_definitions(-DSOFTWARE_VOLUME)

else()
   message(FATAL_ERROR "Unknown PCBREV: '${PCBREV}' expecting GCSV2")
endif()

    
if(CPU_TYPE_FULL STREQUAL STM32F407xE)
  add_definitions(-DSTM32F40_41xxx)
  set(LINKER_SCRIPT targets/hitec/stm32f407_flash.ld)
  set(FIRMWARE_TARGET_SRC
    ${FIRMWARE_TARGET_SRC}
    top_lcd_driver.cpp
    startup_stm32f40_41xxx.s
    )
elseif(CPU_TYPE_FULL STREQUAL STM32F427xE)
  add_definitions(-DSTM32F427_437xx)
  set(LINKER_SCRIPT targets/hitec/stm32f427_flash.ld)
  set(FIRMWARE_TARGET_SRC
    ${FIRMWARE_TARGET_SRC}
    top_lcd_driver.cpp
    startup_stm32f42_43xxx.s
    )
else()
    message(FATAL_ERROR "Please provide CPU_TYPE_FULL of STM32F407xE or STM32F427xE")
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

set(HSE_VALUE 12000000)
set(SDCARD YES)
set(EEPROM EEPROM_RLC)
set(TARGET_DIR hitec)
set(PPM_LIMITS_SYMETRICAL YES)


add_definitions(-DPCBTARANIS)
add_definitions(-DAUDIO -DVOICE -DRTCLOCK)
add_definitions(-DSBUS_TRAINER)


set(SRC
  ${SRC}
  io/frsky_firmware_update.cpp
  io/multi_firmware_update.cpp
  )


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

set(SRC
  ${SRC}
  sbus.cpp
  )

  

set(TARGET_SRC
  ${TARGET_SRC}
  ${LED_DRIVER}
  trainer_driver.cpp
  backlight_driver.cpp
  pi_driver.cpp
  debug_driver.cpp
  ../common/arm/stm32/timers_driver.cpp
  ../common/arm/stm32/audio_dac_driver.cpp
  ../common/arm/stm32/adc_driver.cpp
  ../common/arm/stm32/mixer_scheduler_driver.cpp
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
