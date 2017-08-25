/****************************************************************************
 * config/smoothie-v2minialpha/src/lpc43_mmcsd.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "lpc43_gpio.h"
#include "lpc43_sdmmc.h"

#include "smoothie-v2minialpha.h"

#ifdef CONFIG_LPC43_SDMMC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure holds static information unique to one MMCSD peripheral */

struct lpc43_mmcsd_state_s
{
  struct sdio_dev_s *mmcsd;   /* R/W device handle */
  bool initialized;           /* TRUE: MMCSD block driver is initialized */
  bool inserted;              /* TRUE: card is inserted */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* MMCSD device state */

static struct lpc43_mmcsd_state_s g_mmcsd;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

bool lpc43_cardinserted(int slotno);

/****************************************************************************
 * Name: lpc43_mmcsd_cardetect_int
 *
 * Description:
 *   Card detect interrupt handler
 *
 * TODO: Any way to automatically moun/unmount filesystem based on card
 * detect status?  Yes... send a message or signal to an application.
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_HAVECARDDETECT
static int lpc43_mmcsd_cardetect_int(int irq, void *regs, FAR void *arg)
{
  bool inserted;

  /* Get the state of the GPIO pin */

  inserted = lpc43_cardinserted(0);

  /* Has the card detect state changed? */

  if (inserted != g_mmcsd.inserted)
    {
      /* Yes... remember that new state and inform the MMCSD driver */

      g_mmcsd.inserted = inserted;

      /* Report the new state to the SDIO driver */

      sdio_mediachange(g_mmcsd.mmcsd, inserted);
    }

   return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_mmcsd_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int lpc43_mmcsd_initialize(void)
{
  int ret;
  finfo("Initializing SDIO\n");

  /* Have we already initialized? */

  if (!g_mmcsd.initialized)
    {
      /* Mount the SDIO-based MMC/SD block driver */
      /* First, get an instance of the SDIO interface */

      g_mmcsd.mmcsd = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);
      if (!g_mmcsd.mmcsd)
        {
          ferr("ERROR: Failed to initialize SDIO\n");
          return -ENODEV;
        }

      /* Now bind the SDIO interface to the MMC/SD driver */

      ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, g_mmcsd.mmcsd);
      if (ret != OK)
        {
          ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
          return ret;
        }

#ifdef CONFIG_MMCSD_HAVECARDDETECT
      /* Initialize card-detect GPIO.  There is no write-protection GPIO. */

      lpc43_gpio_config(GPIO_SD_CD);

      /* Configure card detect interrupts */

      (void)irq_attach(GPIO_SD_CD_IRQ, lpc43_mmcsd_cardetect_int, NULL);
      g_mmcsd.inserted = lpc43_cardinserted(0);
#else
      g_mmcsd.inserted = true; /* An assumption? */
#endif
      /* Then inform the MMCSD driver if there is or is not a card in the slot. */

      sdio_mediachange(g_mmcsd.mmcsd, g_mmcsd.inserted);

      /* Now we are initialized */

      g_mmcsd.initialized = true;

      /* Enable card detect interrupts */

#ifdef CONFIG_MMCSD_HAVECARDDETECT
      //lpc43_gpioirqenable(GPIO_SD_CD_IRQ);
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: lpc43_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected MMCSD slot
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_HAVECARDDETECT
bool lpc43_cardinserted(int slotno)
{
  bool removed;

  /* Get the state of the GPIO pin */

  removed = lpc43_gpio_read(GPIO_SD_CD);
  finfo("Slot %d inserted: %s\n", slotno, removed ? "NO" : "YES");

  return !removed;
}
#endif

/****************************************************************************
 * Name: lpc43_writeprotected
 *
 * Description:
 *   Check if a card is inserted into the selected MMCSD slot
 *
 ****************************************************************************/

bool lpc43_writeprotected(int slotno)
{
  /* There are no write protect pins */

  return false;
}
#else
#  define nsh_spifi_initialize() (OK)
#endif /* CONFIG_LPC43_SDMMC */

