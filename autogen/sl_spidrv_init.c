#include "spidrv.h"
#include "sl_spidrv_instances.h"
#include "em_assert.h"


#include "sl_spidrv_mySPI_config.h"

SPIDRV_HandleData_t sl_spidrv_mySPI_handle_data;
SPIDRV_Handle_t sl_spidrv_mySPI_handle = &sl_spidrv_mySPI_handle_data;

SPIDRV_Init_t sl_spidrv_init_mySPI = {
  .port = SL_SPIDRV_MYSPI_PERIPHERAL,
#if defined(_USART_ROUTELOC0_MASK)
  .portLocationTx = SL_SPIDRV_MYSPI_TX_LOC,
  .portLocationRx = SL_SPIDRV_MYSPI_RX_LOC,
  .portLocationClk = SL_SPIDRV_MYSPI_CLK_LOC,
#if defined(SL_SPIDRV_MYSPI_CS_LOC)
  .portLocationCs = SL_SPIDRV_MYSPI_CS_LOC,
#endif
#elif defined(_GPIO_USART_ROUTEEN_MASK)
  .portTx = SL_SPIDRV_MYSPI_TX_PORT,
  .portRx = SL_SPIDRV_MYSPI_RX_PORT,
  .portClk = SL_SPIDRV_MYSPI_CLK_PORT,
#if defined(SL_SPIDRV_MYSPI_CS_PORT)
  .portCs = SL_SPIDRV_MYSPI_CS_PORT,
#endif
  .pinTx = SL_SPIDRV_MYSPI_TX_PIN,
  .pinRx = SL_SPIDRV_MYSPI_RX_PIN,
  .pinClk = SL_SPIDRV_MYSPI_CLK_PIN,
#if defined(SL_SPIDRV_MYSPI_CS_PIN)
  .pinCs = SL_SPIDRV_MYSPI_CS_PIN,
#endif
#else
  .portLocation = SL_SPIDRV_MYSPI_ROUTE_LOC,
#endif
  .bitRate = SL_SPIDRV_MYSPI_BITRATE,
  .frameLength = SL_SPIDRV_MYSPI_FRAME_LENGTH,
  .dummyTxValue = 0,
  .type = SL_SPIDRV_MYSPI_TYPE,
  .bitOrder = SL_SPIDRV_MYSPI_BIT_ORDER,
  .clockMode = SL_SPIDRV_MYSPI_CLOCK_MODE,
  .csControl = SL_SPIDRV_MYSPI_CS_CONTROL,
  .slaveStartMode = SL_SPIDRV_MYSPI_SLAVE_START_MODE,
};

void sl_spidrv_init_instances(void) {
  SPIDRV_Init(sl_spidrv_mySPI_handle, &sl_spidrv_init_mySPI);
}
