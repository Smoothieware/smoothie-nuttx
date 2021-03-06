/****************************************************************************
 * arch/arm/src/xmc4/xmc4_spi.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_XMC4_XMC4_SPI_H
#define __ARCH_ARM_SRC_XMC4_XMC4_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip/xmc4_spi.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

struct  spi_dev_s;
enum    spi_dev_e;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/************************************************************************************
 * Name: xmc4_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameter:
 *   bus number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ************************************************************************************/

FAR struct spi_dev_s *xmc4_spibus_initialize(int bus);

/************************************************************************************
 * Name:  xmc4_spi[n]select, xmc4_spi[n]status, and xmc4_spi[n]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   including xmc4_spibus_initialize()) are provided by common Kinetis logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in xmc4_board_initialize() to configure SPI chip select
 *      pins.
 *   2. Provide xmc4_spi[n]select() and xmc4_spi[n]status() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      xmc4_spi[n]cmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in the way
 *      your board is configured.
 *   3. Add a call to xmc4_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by xmc4_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

#ifdef CONFIG_XMC4_SPI0
void  xmc4_spi0select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
uint8_t xmc4_spi0status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
int xmc4_spi0cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif
#ifdef CONFIG_XMC4_SPI1
void  xmc4_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
uint8_t xmc4_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
int xmc4_spi1cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif
#ifdef CONFIG_XMC4_SPI2
void  xmc4_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
uint8_t xmc4_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
int xmc4_spi2cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif

/****************************************************************************
 * Name: ssp_flush
 *
 * Description:
 *   Flush and discard any words left in the RX fifo.  This can be called
 *   from spi[n]select after a device is deselected (if you worry about such
 *   things).
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#if defined(CONFIG_XMC4_SPI0) || defined(CONFIG_XMC4_SPI1) || defined(CONFIG_XMC4_SPI2)
struct spi_dev_s;
void spi_flush(FAR struct spi_dev_s *dev);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_XMC4_XMC4_SPI_H */
