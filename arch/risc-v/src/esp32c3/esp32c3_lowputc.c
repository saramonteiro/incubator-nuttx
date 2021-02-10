/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_lowputc.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "hardware/esp32c3_uart.h"
#include "esp32c3_clockconfig.h"
#include "hardware/esp32c3_system.h"
#include "riscv_arch.h"
#include "chip.h"
#include "esp32c3_lowputc.h"
#include "esp32c3_config.h"
#include "hardware/esp32c3_soc.h"
#include "esp32c3_gpio.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)

static const struct esp32c3_uart_s g_console_config =
{
  .base = REG_UART_BASE(0),
  .id = 0,
  .irq = ESP32C3_IRQ_UART0,
  .baud = CONFIG_UART0_BAUD,
  .bits = CONFIG_UART0_BITS,
  .parity = CONFIG_UART0_PARITY,
  .stop_b2 =  CONFIG_UART0_2STOP,
  .int_pri = 1,
  .txpin = CONFIG_ESP32C3_UART0_TXPIN,
  .txsig = U0TXD_OUT_IDX,
  .rxpin = CONFIG_ESP32C3_UART0_RXPIN,
  .rxsig = U0RXD_IN_IDX,  
};

# elif defined(CONFIG_UART1_SERIAL_CONSOLE)

static const struct esp32c3_uart_s g_uart1_config =
{
  .base = REG_UART_BASE(1),
  .id = 1,
  .irq = ESP32C3_IRQ_UART1,
  .baud = CONFIG_UART1_BAUD,
  .bits = CONFIG_UART1_BITS,
  .parity = CONFIG_UART1_PARITY,
  .stop_b2 =  CONFIG_UART1_2STOP,
  .int_pri = 1,
  .txpin = CONFIG_ESP32C3_UART1_TXPIN,
  .txsig = U1TXD_OUT_IDX,
  .rxpin = CONFIG_ESP32C3_UART1_RXPIN,
  .rxsig = U1RXD_IN_IDX,
};
#endif /* CONFIG_UART0_SERIAL_CONSOLE */
#endif /* HAVE_SERIAL_CONSOLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_lowputc_reset_core
 *   Reset both TX and RX core
 ****************************************************************************/

void esp32c3_lowputc_reset_cores(const struct esp32c3_uart_s *conf)
{
  uint32_t set_bit = 1 << UART_RST_CORE_S;
  modifyreg32(UART_CLK_CONF_REG(conf->id), UART_RST_CORE_M, set_bit);
  modifyreg32(UART_CLK_CONF_REG(conf->id), UART_RST_CORE_M, 0);
}

/****************************************************************************
 * Name: esp32c3_lowputc_rst_tx
 *   Reset TX core
 ****************************************************************************/

void esp32c3_lowputc_rst_tx(const struct esp32c3_uart_s *conf)
{
  uint32_t set_bit = 1 << UART_TX_RST_CORE_S;
  modifyreg32(UART_CLK_CONF_REG(conf->id), UART_TX_RST_CORE_M, set_bit);
  modifyreg32(UART_CLK_CONF_REG(conf->id), UART_TX_RST_CORE_M, 0);
}

/****************************************************************************
 * Name: esp32c3_lowputc_rst_rx
 *   Reset RX core
 ****************************************************************************/

void esp32c3_lowputc_rst_rx(const struct esp32c3_uart_s *conf)
{
  uint32_t set_bit = 1 << UART_RX_RST_CORE_S;
  modifyreg32(UART_CLK_CONF_REG(conf->id), UART_RX_RST_CORE_M, set_bit);
  modifyreg32(UART_CLK_CONF_REG(conf->id), UART_RX_RST_CORE_M, 0);
}


/****************************************************************************
 * Name: esp32c3_lowputc_enable_sclk
 *    Enable clock for whole core
 ****************************************************************************/

void esp32c3_lowputc_enable_sclk(const struct esp32c3_uart_s *conf)
{
  modifyreg32(UART_CLK_CONF_REG(conf->id), UART_SCLK_EN_M,
              1 << UART_SCLK_EN_S);
  modifyreg32(UART_CLK_CONF_REG(conf->id), UART_RX_SCLK_EN_M,
              1 << UART_RX_SCLK_EN_S);
  modifyreg32(UART_CLK_CONF_REG(conf->id), UART_TX_SCLK_EN_M,
              1 << UART_TX_SCLK_EN_S);
}

/****************************************************************************
 * Name: esp32c3_lowputc_disable_sclk
 *    Disable clock for whole core
 ****************************************************************************/

void esp32c3_lowputc_disable_sclk(const struct esp32c3_uart_s *conf)
{
  modifyreg32(UART_CLK_CONF_REG(conf->id), UART_SCLK_EN_M, 0);
  modifyreg32(UART_CLK_CONF_REG(conf->id), UART_RX_SCLK_EN_M, 0);
  modifyreg32(UART_CLK_CONF_REG(conf->id), UART_TX_SCLK_EN_M, 0);
}

/****************************************************************************
 * Name: esp32c3_lowputc_set_sclk
 *    Set a source clock for UART
 *    APB_CLK  = 1  80 MHz
 *    CLK_8    = 2  8 MHz
 *    XTAL_CLK = 3
 ****************************************************************************/

void esp32c3_lowputc_set_sclk(const struct esp32c3_uart_s *conf, enum
                              uart_sclk source)
{
  uint32_t clk = (uint32_t)source << UART_SCLK_SEL_S;
  modifyreg32(UART_CLK_CONF_REG(conf->id), UART_SCLK_SEL_M, clk);
}

/****************************************************************************
 * Name: esp32c3_lowputc_get_sclk
 *    Get the source clock for UART
 ****************************************************************************/

uint32_t esp32c3_lowputc_get_sclk(const struct esp32c3_uart_s * conf)
{
  uint32_t clk_conf_reg;
  uint32_t ret = -ENODATA;
  clk_conf_reg   = getreg32(UART_CLK_CONF_REG(conf->id));
  clk_conf_reg  &= UART_SCLK_SEL_M;
  clk_conf_reg >>= UART_SCLK_SEL_S;
  switch (clk_conf_reg)
    {
      case 1:
        //ret = esp32c3_clk_apb_freq();
        ret = APB_CLK_FREQ;
        break;
      case 2:
        ret = RTC_CLK_FREQ;
        break;
      case 3:
        ret = XTAL_CLK_FREQ;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: esp32c3_lowputc_baud
 *    Set the baud rate
 ****************************************************************************/

void esp32c3_lowputc_baud(const struct esp32c3_uart_s * conf)
{
  const int sclk_div = 1;
  uint32_t sclk_freq = esp32c3_lowputc_get_sclk(conf);
  uint32_t clk_div = ((sclk_freq) << 4) / conf->baud;
  uint32_t int_part = clk_div >> 4;
  uint32_t frag_part = clk_div &  0xf;

  /* The baud rate configuration register is divided into
   * an integer part and a fractional part.
   */

  modifyreg32(UART_CLKDIV_REG(conf->id), UART_CLKDIV_M, int_part);
  modifyreg32(UART_CLKDIV_REG(conf->id), UART_CLKDIV_FRAG_M,
                              frag_part << UART_CLKDIV_FRAG_S);
  modifyreg32(UART_CLK_CONF_REG(conf->id), UART_SCLK_DIV_NUM_M,
                                (sclk_div - 1) << UART_SCLK_DIV_NUM_S);
}

/****************************************************************************
 * Name: esp32c3_lowputc_normal_mode
 *    Set the UART to operate in normal mode
 ****************************************************************************/

void esp32c3_lowputc_normal_mode(const struct esp32c3_uart_s * conf)
{
  /* Disable RS485 mode */

  modifyreg32(UART_RS485_CONF_REG(conf->id), UART_RS485_EN_M, 0);
  modifyreg32(UART_RS485_CONF_REG(conf->id), UART_RS485TX_RX_EN_M, 0);
  modifyreg32(UART_RS485_CONF_REG(conf->id), UART_RS485RXBY_TX_EN_M, 0);

  /* Disable IRDA mode */

  modifyreg32(UART_CONF0_REG(conf->id), UART_IRDA_EN_M, 0);
}

/****************************************************************************
 * Name: esp32c3_lowputc_parity
 *    Set the parity
 ****************************************************************************/

void esp32c3_lowputc_parity(const struct esp32c3_uart_s * conf)
{
  if (conf->parity == UART_PARITY_DISABLE)
    {
      modifyreg32(UART_CONF0_REG(conf->id), UART_PARITY_EN_M, 0);
    }
  else
    {
      modifyreg32(UART_CONF0_REG(conf->id), UART_PARITY_M,
                  ((conf->parity & 0x1) << UART_PARITY_S));
      modifyreg32(UART_CONF0_REG(conf->id), UART_PARITY_EN_M,
                                 1 << UART_PARITY_EN_S);
    }
}

/****************************************************************************
 * Name: esp32c3_lowputc_data_length
 *    Set the data length
 ****************************************************************************/

int esp32c3_lowputc_data_length(const struct esp32c3_uart_s * conf)
{
  int ret = OK;
  uint32_t length = (conf->bits - 5);

  /* If it is the allowed range */

  if (length >= UART_DATA_5_BITS && length <= UART_DATA_8_BITS)
    {
      modifyreg32(UART_CONF0_REG(conf->id), UART_BIT_NUM_M,
                    length << UART_BIT_NUM_S);
    }
  else
    {
      ret = -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: esp32c3_lowputc_stop_length
 *    Set the stop length
 ****************************************************************************/

void esp32c3_lowputc_stop_length(const struct esp32c3_uart_s * conf)
{
  if (conf->stop_b2 == 0)
    {
      modifyreg32(UART_CONF0_REG(conf->id), UART_STOP_BIT_NUM_M,
                    UART_STOP_BITS_1 << UART_STOP_BIT_NUM_S);
    }
  else
    {
      modifyreg32(UART_CONF0_REG(conf->id), UART_STOP_BIT_NUM_M,
                    UART_STOP_BITS_2 << UART_STOP_BIT_NUM_S);
    }
}

/****************************************************************************
 * Name: esp32c3_lowputc_set_tx_idle_time
 *    Set the idle time between transfers
 ****************************************************************************/

void esp32c3_lowputc_set_tx_idle_time(const struct esp32c3_uart_s *
                                      conf, uint32_t time)
{
  time = time << UART_TX_IDLE_NUM_S;
  time = time & UART_TX_IDLE_NUM_M; /* Just in case value overloads */
  modifyreg32(UART_IDLE_CONF_REG(conf->id), UART_TX_IDLE_NUM_M,
              time);
}

/****************************************************************************
 * Name: esp32c3_lowputc_send_byte
 *    Send one byte
 ****************************************************************************/

void esp32c3_lowputc_send_byte(const struct esp32c3_uart_s * conf,
                               char byte)
{
  putreg32((uint32_t) byte, UART_FIFO_REG(conf->id));
}

/****************************************************************************
 * Name: esp32c3_lowputc_is_tx_fifo_full
 *    Verifies if TX FIFO is full
 ****************************************************************************/

bool esp32c3_lowputc_is_tx_fifo_full(const struct esp32c3_uart_s *
                                     conf)
{
  uint32_t reg;
  reg = getreg32(UART_STATUS_REG(conf->id));
  reg = reg >> UART_TXFIFO_CNT_S;
  reg = reg & UART_TXFIFO_CNT_V;
  if (reg < (UART_TX_FIFO_SIZE -1))
    {
      return false;
    }
  else
    {
      return true;
    }
}

/****************************************************************************
 * Name: esp32c3_lowputc_rst_peripheral
 *    Reset peripheral using System reg
 ****************************************************************************/

void esp32c3_lowputc_rst_peripheral(const struct esp32c3_uart_s *
                                     conf)
{
  if (conf->id == 0)
    {
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_UART_RST_M, SYSTEM_UART_RST_M);
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_UART_RST_M, 0);
    }
  else
    {
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_UART1_RST_M, SYSTEM_UART1_RST_M);
      modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_UART1_RST_M, 0);
    }

}

/****************************************************************************
 * Name: esp32c3_lowputc_rst_txfifo
 *    Reset tx fifo
 ****************************************************************************/

void esp32c3_lowputc_rst_txfifo(const struct esp32c3_uart_s *
                                     conf)
{
  modifyreg32(UART_CONF0_REG(conf->id), UART_TXFIFO_RST_M,
                             UART_TXFIFO_RST_M);
  modifyreg32(UART_CONF0_REG(conf->id), UART_TXFIFO_RST_M, 0);
}

/****************************************************************************
 * Name: esp32c3_lowputc_rst_rxfifo
 *    Reset rx fifo
 ****************************************************************************/

void esp32c3_lowputc_rst_rxfifo(const struct esp32c3_uart_s *
                                     conf)
{
  modifyreg32(UART_CONF0_REG(conf->id), UART_RXFIFO_RST_M,
                             UART_RXFIFO_RST_M);
  modifyreg32(UART_CONF0_REG(conf->id), UART_RXFIFO_RST_M, 0);
}


/****************************************************************************
 * Name: disable_all_uart_int
 *
 * Description:
 *   Disable all uart interrupts
 ****************************************************************************/

void disable_all_uart_int(const struct esp32c3_uart_s *priv, uint32_t *
                          current_status)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (current_status != NULL)
    {
      /* save current status */

      *current_status = getreg32(UART_INT_ENA_REG(priv->id));
    }

  /* disable all uart int */

  putreg32(0, UART_INT_ENA_REG(priv->id));

  /* clear all ints */

  putreg32(0xFFFFFFFF, UART_INT_CLR_REG(priv->id));

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: restore_all_uart_int
 *
 * Description:
 *   Restore all uart interrupts
 ****************************************************************************/

void restore_all_uart_int(const struct esp32c3_uart_s *priv, uint32_t* last_status)
{
  /* restore the previous behaviour */

  putreg32(*last_status, UART_INT_ENA_REG(priv->id));
}

// /****************************************************************************
//  * Name: esp32c3_lowputc_config_pins
//  *
//  * Description:
//  *   Configure uart pins
//  ****************************************************************************/

// void esp32c3_lowputc_config_pins(struct esp32c3_uart_s *priv)
// {
//   /* Configure the pins */

//   esp32c3_configgpio(priv->txpin, OUTPUT_FUNCTION_1);
//   gpio_matrix_out(priv->txpin, priv->txsig, 0, 0);

//   esp32c3_configgpio(priv->rxpin, INPUT_FUNCTION_1);
//   gpio_matrix_in(priv->rxpin, priv->rxsig, 0);
// }

// /****************************************************************************
//  * Name: esp32c3_lowputc_restore_pins
//  *
//  * Description:
//  *   Configure pins back to normal
//  ****************************************************************************/

// void esp32c3_lowputc_config_pins(struct esp32c3_uart_s *priv)
// {
//   /* Configure the pins */

//   esp32c3_configgpio(priv->txpin, INPUT);
//   gpio_matrix_out(priv->txpin, MATRIX_DETACH_OUT_SIG, false, false);

//   esp32c3_configgpio(priv->rxpin, INPUT);
//   gpio_matrix_in(priv->rxpin, MATRIX_DETACH_IN_LOW_PIN, false);
// }

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE

  /* Wait until the TX FIFO has space to insert new char */

  while (esp32c3_lowputc_is_tx_fifo_full(&g_console_config));

  /* Then send the character */

  esp32c3_lowputc_send_byte(&g_console_config, ch);

#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: esp32c3_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void esp32c3_lowsetup(void)
{
  /* Enable and configure the selected console device */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  ets_printf("Baud rate: %d\n", g_console_config.baud);

  /* Initialize UART module */

//  esp32c3_lowputc_rst_peripheral(&g_console_config);

  /* Reset both cores */

//  esp32c3_lowputc_reset_cores(&g_console_config);

  /* Configure Clock */

//  esp32c3_lowputc_set_sclk(&g_console_config, APB_CLK);

  /* Reset TX FIFO */

//  esp32c3_lowputc_rst_txfifo(&g_console_config);

  /* Reset RX FIFO */

//  esp32c3_lowputc_rst_rxfifo(&g_console_config);

  /* Configure the UART Baud Rate */

  esp32c3_lowputc_baud(&g_console_config);

  /* Set a mode */

  esp32c3_lowputc_normal_mode(&g_console_config);

  /* Parity */

  esp32c3_lowputc_parity(&g_console_config);

  /* Data Frame size */

  esp32c3_lowputc_data_length(&g_console_config);

  /* Stop bit */

  esp32c3_lowputc_stop_length(&g_console_config);

  /* Disable uart ints */

  //disable_all_uart_int(&g_console_config, NULL);

  /* No Tx idle interval */

  esp32c3_lowputc_set_tx_idle_time(&g_console_config, 0);

  /* Reset TX and RX cores */

  // esp32c3_lowputc_rst_tx(&g_console_config);
  // esp32c3_lowputc_rst_rx(&g_console_config);

  /* Set pins */

  //esp32c3_lowputc_config_pins(&g_console_config);

  /* Enable cores */

  esp32c3_lowputc_enable_sclk(&g_console_config);

#endif /* HAVE_SERIAL_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
}
