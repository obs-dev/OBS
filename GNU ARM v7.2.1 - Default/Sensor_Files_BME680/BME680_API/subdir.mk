################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sensor_Files_BME680/BME680_API/bme68x.c 

OBJS += \
./Sensor_Files_BME680/BME680_API/bme68x.o 

C_DEPS += \
./Sensor_Files_BME680/BME680_API/bme68x.d 


# Each subdirectory must supply rules for building sources it contributes
Sensor_Files_BME680/BME680_API/bme68x.o: ../Sensor_Files_BME680/BME680_API/bme68x.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DBGM220PC22WGA=1' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DMBEDTLS_CONFIG_FILE=<mbedtls_config.h>' '-DMBEDTLS_PSA_CRYPTO_CONFIG_FILE=<psa_crypto_config.h>' '-DNVM3_DEFAULT_MAX_OBJECT_SIZE=1400' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' -I"D:\Microart\NEW\config" -I"D:\Microart\NEW" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/Device/SiliconLabs/BGM22/Include" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//app/common/util/app_assert" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//app/common/util/app_log" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/common/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//protocol/bluetooth/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/bootloader" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/bootloader/api" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/Include" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/crypto/sl_component/sl_cryptoacc_library/include" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/service/device_init/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/common/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/emlib/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/service/hfxo_manager/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/driver/i2cspm/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/service/iostream/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/driver/leddrv/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/crypto/mbedtls/include" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/crypto/sl_component/sl_mbedtls_support/config" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/crypto/mbedtls/library" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/crypto/sl_component/sl_alt/include" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/crypto/sl_component/sl_mbedtls_support/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/service/mpu/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/nvm3/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//app/bluetooth/common/ota_dfu" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/service/power_manager/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/crypto/sl_component/sl_psa_driver/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/common" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ble" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ieee802154" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/zwave" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/mfm" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/chip/efr32/efr32xg2x" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions/efr32xg22" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_pti" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/service/ram_interrupt_vector_init/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/crypto/sl_component/se_manager/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/crypto/sl_component/se_manager/src" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//util/silicon_labs/silabs_core/memory_manager" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/common/toolchain/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/service/system/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/service/sleeptimer/inc" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/crypto/sl_component/sl_protocol_crypto/src" -I"D:/Simplicity Studio 5/developer/sdks/gecko_sdk_suite/v3.1//platform/service/udelay/inc" -I"D:\Microart\NEW\autogen" -Os -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -c -fmessage-length=0 -MMD -MP -MF"Sensor_Files_BME680/BME680_API/bme68x.d" -MT"Sensor_Files_BME680/BME680_API/bme68x.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

