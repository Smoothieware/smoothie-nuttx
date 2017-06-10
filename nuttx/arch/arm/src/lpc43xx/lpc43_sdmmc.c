/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_sdmmc.c
 *
 *   Copyright (C) 2017 Alan Carvalho de Assis. All rights reserved.
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 * This code is based on arch/arm/src/lpc17xx/lpc17_sdcard.c
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

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/mmcsd.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"

#include "lpc43_cgu.h"
#include "lpc43_ccu.h"
#include "lpc43_gpio.h"
#include "lpc43_sdmmc.h"

#include "chip/lpc43_pinconfig.h"

#ifdef CONFIG_LPC43_SDMMC

#define CONFIG_LPC43_SDMMC_REGDEBUG 1

#define mcinfo _info
#define mcerr _info

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MCI_DMADES0_OWN         (1UL << 31)
#define MCI_DMADES0_CH          (1 << 4)
#define MCI_DMADES0_FS          (1 << 3)
#define MCI_DMADES0_LD          (1 << 2)
#define MCI_DMADES0_DIC         (1 << 1)

/* Configuration ************************************************************/
/* Required system configuration options:
 *
 *   CONFIG_SCHED_WORKQUEUE -- Callback support requires work queue support.
 *
 * Driver-specific configuration options:
 *
 *   CONFIG_SDIO_MUXBUS - Setting this configuration enables some locking
 *     APIs to manage concurrent accesses on the SD card bus.  This is not
 *     needed for the simple case of a single SD card, for example.
 *   CONFIG_SDIO_DMA - Enable SD card DMA.  This is a marginally optional.
 *     For most usages, SD accesses will cause data overruns if used without DMA.
 *     NOTE the above system DMA configuration options.
 *   CONFIG_SDIO_WIDTH_D1_ONLY - This may be selected to force the driver
 *     operate with only a single data line (the default is to use all
 *     4 SD data lines).
 *   CONFIG_DEBUG_MEMCARD_* - Enables some very low-level debug output
 *     This also requires CONFIG_DEBUG_FS and CONFIG_DEBUG_INFO
 */

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

/* Clock Division */

#define LPC43_CLKDIV_INIT         6   /* Divide by 510 = 400KHz */
#define SDCARD_CLOCK_MMCXFR       102 /* SDCARD_MMCXFR_CLKDIV   */
#define SDCARD_CLOCK_SDWIDEXFR    5   /* SDCARD_SDXFR_CLKDIV    */
#define SDCARD_CLOCK_SDXFR        102 /* SDCARD_SDXFR_CLKDIV    */

/* Timing */

#define SDCARD_CMDTIMEOUT         (10000)
#define SDCARD_LONGTIMEOUT        (0x7fffffff)

/* Type of Card Bus Size */

#define SDCARD_BUS_D1             0
#define SDCARD_BUS_D4             1
#define SDCARD_BUS_D8             0x100

/* Data transfer interrupt mask bits */

#define SDCARD_RECV_MASK     (SDMMC_INT_DCRC | SDMMC_INT_RCRC | SDMMC_INT_DRTO | \
                              SDMMC_INT_RTO | SDMMC_INT_EBE | SDMMC_INT_RXDR | \
                              SDMMC_INT_SBE)

#define SDCARD_SEND_MASK     (SDMMC_INT_DCRC | SDMMC_INT_RCRC | SDMMC_INT_DRTO | \
                              SDMMC_INT_RTO | SDMMC_INT_EBE | SDMMC_INT_TXDR | \
                              SDMMC_INT_DTO | SDMMC_INT_SBE)

#define SDCARD_DMARECV_MASK  (SDCARD_MASK0_DCRCFAILIE | SDCARD_MASK0_DTIMEOUTIE | \
                              SDCARD_MASK0_DATAENDIE | SDCARD_MASK0_RXOVERRIE | \
                              SDCARD_MASK0_STBITERRIE)
#define SDCARD_DMASEND_MASK  (SDCARD_MASK0_DCRCFAILIE | SDCARD_MASK0_DTIMEOUTIE | \
                              SDCARD_MASK0_DATAENDIE | SDCARD_MASK0_TXUNDERRIE | \
                              SDCARD_MASK0_STBITERRIE)

/* Event waiting interrupt mask bits */

#define SDCARD_INT_ERROR     (SDMMC_INT_RE | SDMMC_INT_RCRC | SDMMC_INT_DCRC | \
                              SDMMC_INT_RTO | SDMMC_INT_DRTO | SDMMC_INT_HTO | \
                              SDMMC_INT_FRUN | SDMMC_INT_HLE | SDMMC_INT_SBE | \
                              SDMMC_INT_EBE)

#define SDCARD_CMDDONE_STA   (SDMMC_INT_CDONE)
#define SDCARD_RESPDONE_STA  (0)

#define SDCARD_CMDDONE_MASK  (SDMMC_INT_CDONE)
#define SDCARD_RESPDONE_MASK (SDMMC_INT_RTO | SDMMC_INT_RCRC | SDMMC_INT_CDONE)
#define SDCARD_XFRDONE_MASK  (SDMMC_INT_DTO)

#define SDCARD_CMDDONE_ICR   (SDMMC_INT_CDONE)
#define SDCARD_RESPDONE_ICR  (SDMMC_INT_RTO | SDMMC_INT_RCRC | SDMMC_INT_CDONE)

#define SDCARD_XFRDONE_ICR   (SDMMC_INT_DTO | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                              SDMMC_INT_FRUN | SDMMC_INT_SBE)

#define SDCARD_WAITALL_ICR   (SDCARD_CMDDONE_ICR | SDCARD_RESPDONE_ICR | \
                              SDCARD_XFRDONE_ICR)

/* Let's wait until we have both SD card transfer complete and DMA complete. */

#define SDCARD_XFRDONE_FLAG  (1)
#define SDCARD_DMADONE_FLAG  (2)
#define SDCARD_ALLDONE       (3)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct {
        volatile uint32_t des0;                                         /*!< Control and status */
        volatile uint32_t des1;                                         /*!< Buffer size(s) */
        volatile uint32_t des2;                                         /*!< Buffer address pointer 1 */
        volatile uint32_t des3;                                         /*!< Buffer address pointer 2 */
} sdmmc_dma_t;

sdmmc_dma_t mci_dma_dd[2];

/* This structure defines the state of the LPC43XX SD card interface */

struct lpc43_dev_s
{
  struct sdio_dev_s  dev;        /* Standard, base SD card interface */

  /* LPC43XX-specific extensions */
  /* Event support */

  sem_t              waitsem;    /* Implements event waiting */
  sdio_eventset_t    waitevents; /* Set of events to be waited for */
  uint32_t           waitmask;   /* Interrupt enables for event waiting */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  WDOG_ID            waitwdog;   /* Watchdog that handles event timeouts */

  /* Callback support */

  sdio_statset_t     cdstatus;   /* Card status */
  sdio_eventset_t    cbevents;   /* Set of events to be cause callbacks */
  worker_t           callback;   /* Registered callback function */
  void              *cbarg;      /* Registered callback argument */
  struct work_s      cbwork;     /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t          *buffer;     /* Address of current R/W buffer */
  size_t             remaining;  /* Number of bytes remaining in the transfer */
  uint32_t           xfrmask;    /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  bool               widebus;    /* Required for DMA support */
#ifdef CONFIG_SDIO_DMA
  bool               dmamode;    /* true: DMA mode transfer */
#endif
};

/* Register logging support */

#ifdef CONFIG_DEBUG_MEMCARD_INFO
struct lpc43_sdcard_regs_s
{
  uint8_t  pwr;
  uint16_t clkcr;
  uint16_t dctrl;
  uint32_t dtimer;
  uint32_t dlen;
  uint32_t dcount;
  uint32_t sta;
  uint32_t mask;
  uint32_t fifocnt;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_LPC43_SDMMC_REGDEBUG
static uint32_t lpc43_getreg(uint32_t addr);
static void lpc43_putreg(uint32_t val, uint32_t addr);
static void lpc43_checksetup(void);
#else
# define lpc43_getreg(addr)      getreg32(addr)
# define lpc43_putreg(val,addr)  putreg32(val,addr)
# define lpc43_checksetup()
#endif

/* Low-level helpers ********************************************************/

static void lpc43_takesem(struct lpc43_dev_s *priv);
#define     lpc43_givesem(priv) (sem_post(&priv->waitsem))
static inline void lpc43_setclock(uint32_t clkdiv);
static inline void lpc43_settype(uint32_t ctype);
static inline void lpc43_sdcard_clock(bool enable);
static int lpc43_ciu_sendcmd(uint32_t cmd, uint32_t arg);
static void lpc43_configwaitints(struct lpc43_dev_s *priv, uint32_t waitmask,
              sdio_eventset_t waitevents, sdio_eventset_t wkupevents);
static void lpc43_configxfrints(struct lpc43_dev_s *priv, uint32_t xfrmask);
static void lpc43_setpwrctrl(uint32_t pwrctrl);
static inline uint32_t lpc43_getpwrctrl(void);

/* Data Transfer Helpers ****************************************************/

static uint8_t lpc43_log2(uint16_t value);
static void lpc43_eventtimeout(int argc, uint32_t arg);
static void lpc43_endwait(struct lpc43_dev_s *priv, sdio_eventset_t wkupevent);
static void lpc43_endtransfer(struct lpc43_dev_s *priv, sdio_eventset_t wkupevent);

/* Interrupt Handling *******************************************************/

static int  lpc43_interrupt(int irq, void *context, FAR void *arg);

/* SD Card Interface Methods ************************************************/

/* Mutual exclusion */

#ifdef CONFIG_SDIO_MUXBUS
static int lpc43_lock(FAR struct sdio_dev_s *dev, bool lock);
#endif

/* Initialization/setup */

static void lpc43_reset(FAR struct sdio_dev_s *dev);
static sdio_capset_t lpc43_capabilities(FAR struct sdio_dev_s *dev);
static uint8_t lpc43_status(FAR struct sdio_dev_s *dev);
static void lpc43_widebus(FAR struct sdio_dev_s *dev, bool enable);
static void lpc43_clock(FAR struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int  lpc43_attach(FAR struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  lpc43_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t arg);
static int  lpc43_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
              size_t nbytes);
static int  lpc43_sendsetup(FAR struct sdio_dev_s *dev,
              FAR const uint8_t *buffer, uint32_t nbytes);
static int  lpc43_cancel(FAR struct sdio_dev_s *dev);

static int  lpc43_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd);
static int  lpc43_recvshortcrc(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  lpc43_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  lpc43_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  lpc43_recvnotimpl(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rnotimpl);

/* EVENT handler */

static void lpc43_waitenable(FAR struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static sdio_eventset_t
            lpc43_eventwait(FAR struct sdio_dev_s *dev, uint32_t timeout);
static void lpc43_callbackenable(FAR struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static int  lpc43_registercallback(FAR struct sdio_dev_s *dev,
              worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_SDIO_DMA
static int  lpc43_dmarecvsetup(FAR struct sdio_dev_s *dev,
              FAR uint8_t *buffer, size_t buflen);
static int  lpc43_dmasendsetup(FAR struct sdio_dev_s *dev,
              FAR const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void lpc43_callback(void *arg);

/****************************************************************************
 * Name: lpc43_getreg
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   addr - The register address to read
 *
 * Returned Value:
 *   The value read from the register
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_SDMMC_REGDEBUG
static uint32_t lpc43_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval   = 0;
  static uint32_t count    = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              mcinfo("...\n");
            }

          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          mcinfo("[repeats %d more times]\n", count-3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  mcinfo("%08x->%08x\n", addr, val);
  return val;
}
#endif


/****************************************************************************
 * Name: lpc43_putreg
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   val - The value to write to the register
 *   addr - The register address to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_SDMMC_REGDEBUG
static void lpc43_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  mcinfo("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif


/****************************************************************************
 * Private Data
 ****************************************************************************/

struct lpc43_dev_s g_scard_dev =
{
  .dev =
  {
#ifdef CONFIG_SDIO_MUXBUS
    .lock             = lpc43_lock,
#endif
    .capabilities     = lpc43_capabilities,
    .reset            = lpc43_reset,
    .status           = lpc43_status,
    .widebus          = lpc43_widebus,
    .clock            = lpc43_clock,
    .attach           = lpc43_attach,
    .sendcmd          = lpc43_sendcmd,
    .recvsetup        = lpc43_recvsetup,
    .sendsetup        = lpc43_sendsetup,
    .cancel           = lpc43_cancel,
    .waitresponse     = lpc43_waitresponse,
    .recvR1           = lpc43_recvshortcrc,
    .recvR2           = lpc43_recvlong,
    .recvR3           = lpc43_recvshort,
    .recvR4           = lpc43_recvnotimpl,
    .recvR5           = lpc43_recvnotimpl,
    .recvR6           = lpc43_recvshortcrc,
    .recvR7           = lpc43_recvshort,
    .waitenable       = lpc43_waitenable,
    .eventwait        = lpc43_eventwait,
    .callbackenable   = lpc43_callbackenable,
    .registercallback = lpc43_registercallback,
#ifdef CONFIG_SDIO_DMA
    .dmarecvsetup     = lpc43_dmarecvsetup,
    .dmasendsetup     = lpc43_dmasendsetup,
#endif
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Low-level Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: lpc43_takesem
 *
 * Description:
 *   Take the wait semaphore (handling false alarm wakeups due to the receipt
 *   of signals).
 *
 * Input Parameters:
 *   dev - Instance of the SD card device driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_takesem(struct lpc43_dev_s *priv)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&priv->waitsem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: lpc43_setclock
 *
 * Description:
 *   Define the new clock frequency
 *
 * Input Parameters:
 *   clkdiv - A new division value to generate the needed frequency.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void lpc43_setclock(uint32_t clkdiv)
{
  mcinfo("Entry!\n");

  /* Disable the clock before setting frequency */

  lpc43_sdcard_clock(false);

  /* Use the Divider0 */

  lpc43_putreg(SDMMC_CLKSRC_CLKDIV0, LPC43_SDMMC_CLKSRC);

  /* Inform CIU */

  lpc43_ciu_sendcmd(SDMMC_CMD_UPDCLOCK | SDMMC_CMD_WAITPREV, 0);

  /* Set Divider0 to desired value */

  lpc43_putreg(clkdiv & SDMMC_CLKDIV0_MASK, LPC43_SDMMC_CLKDIV);

  /* Inform CIU */

  lpc43_ciu_sendcmd(SDMMC_CMD_UPDCLOCK | SDMMC_CMD_WAITPREV, 0);

  /* Enable the clock */

  lpc43_sdcard_clock(true);

  /* Inform CIU */

  lpc43_ciu_sendcmd(SDMMC_CMD_UPDCLOCK | SDMMC_CMD_WAITPREV, 0);
}

/****************************************************************************
 * Name: lpc43_settype
 *
 * Description: Define the Bus Size of SDCard (1, 4 or 8-bit)
 *   
 * Input Parameters:
 *   ctype - A new CTYPE (Card Type Register) value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void lpc43_settype(uint32_t ctype)
{
  mcinfo("Entry!\n");
  lpc43_putreg(ctype, LPC43_SDMMC_CTYPE);
}

/****************************************************************************
 * Name: lpc43_sdcard_clock
 *
 * Description: Enable/Disable the SDCard clock
 *   
 * Input Parameters:
 *   enable - False = clock disabled; True = clock enabled.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void lpc43_sdcard_clock(bool enable)
{
  if (enable)
    {
      lpc43_putreg(SDMMC_CLKENA_ENABLE, LPC43_SDMMC_CLKENA);
    }
  else
    {
      lpc43_putreg(0, LPC43_SDMMC_CLKENA);
    }
}

/****************************************************************************
 * Name: lpc43_ciu_sendcmd
 *
 * Description:
 *   Function to send command to Card interface unit (CIU)
 *
 * Input Parameters:
 *   cmd - The command to be executed
 *   arg - The argument to use with the command.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/


static int lpc43_ciu_sendcmd(uint32_t cmd, uint32_t arg)
{
  volatile int32_t tmo = SDCARD_CMDTIMEOUT;

  mcinfo("Entry!\n");

  /* set command arg reg */

  lpc43_putreg(arg, LPC43_SDMMC_CMDARG);
  lpc43_putreg(SDMMC_CMD_STARTCMD | cmd, LPC43_SDMMC_CMD);

  /* poll until command is accepted by the CIU */

  while (--tmo && (lpc43_getreg(LPC43_SDMMC_CMD) & SDMMC_CMD_STARTCMD));

  return (tmo < 1) ? 1 : 0;
}

/****************************************************************************
 * Name: lpc43_configwaitints
 *
 * Description:
 *   Enable/disable SD card interrupts needed to suport the wait function
 *
 * Input Parameters:
 *   priv       - A reference to the SD card device state structure
 *   waitmask   - The set of bits in the SD card MASK register to set
 *   waitevents - Waited for events
 *   wkupevent  - Wake-up events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_configwaitints(struct lpc43_dev_s *priv, uint32_t waitmask,
                                 sdio_eventset_t waitevents,
                                 sdio_eventset_t wkupevent)
{
  irqstate_t flags;

  mcinfo("Entry!\n");

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags = enter_critical_section();
  priv->waitevents = waitevents;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = waitmask;
  priv->xfrmask    = waitmask;

  lpc43_putreg(priv->xfrmask | priv->waitmask, LPC43_SDMMC_INTMASK);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc43_configxfrints
 *
 * Description:
 *   Enable SD card interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the SD card device state structure
 *   xfrmask - The set of bits in the SD card MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_configxfrints(struct lpc43_dev_s *priv, uint32_t xfrmask)
{
  mcinfo("Entry!\n");

  irqstate_t flags;
  flags = enter_critical_section();
  priv->xfrmask = xfrmask;
  lpc43_putreg(priv->xfrmask | priv->waitmask, LPC43_SDMMC_INTMASK);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc43_setpwrctrl
 *
 * Description:
 *   Change the PWRCTRL field of the SD card POWER register to turn the SD card
 *   ON or OFF
 *
 * Input Parameters:
 *   clkcr - A new PWRCTRL setting
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_setpwrctrl(uint32_t pwrctrl)
{
  mcinfo("Entry!\n");
}

/****************************************************************************
 * Name: lpc43_getpwrctrl
 *
 * Description:
 *   Return the current value of the  the PWRCTRL field of the SD card P
 *   register.  This function can be used to see if the SD card is powered ON
 *   or OFF
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current value of the  the PWRCTRL field of the SD card PWR register.
 *
 ****************************************************************************/

static inline uint32_t lpc43_getpwrctrl(void)
{
  mcinfo("Entry!\n");
  return 0;
}

/****************************************************************************
 * Data Transfer Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_log2
 *
 * Description:
 *   Take (approximate) log base 2 of the provided number (Only works if the
 *   provided number is a power of 2).
 *
 ****************************************************************************/

static uint8_t lpc43_log2(uint16_t value)
{
  uint8_t log2 = 0;

  mcinfo("Entry!\n");

  /* 0000 0000 0000 0001 -> return 0,
   * 0000 0000 0000 001x -> return 1,
   * 0000 0000 0000 01xx -> return 2,
   * 0000 0000 0000 1xxx -> return 3,
   * ...
   * 1xxx xxxx xxxx xxxx -> return 15,
   */

  DEBUGASSERT(value > 0);
  while (value != 1)
    {
      value >>= 1;
      log2++;
    }

  return log2;
}

/****************************************************************************
 * Name: lpc43_eventtimeout
 *
 * Description:
 *   The watchdog timeout setup when the event wait start has expired without
 *   any other waited-for event occurring.
 *
 * Input Parameters:
 *   argc   - The number of arguments (should be 1)
 *   arg    - The argument (state structure reference cast to uint32_t)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void lpc43_eventtimeout(int argc, uint32_t arg)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)arg;

  mcinfo("Entry!\n");

  /* There is always race conditions with timer expirations. */

  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0 || priv->wkupevent != 0);

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. wake up any waiting threads */

      lpc43_endwait(priv, SDIOWAIT_TIMEOUT);
      mcerr("ERROR: Timeout: remaining: %d\n", priv->remaining);
    }
}

/****************************************************************************
 * Name: lpc43_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv      - An instance of the SD card device interface
 *   wkupevent - The event that caused the wait to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void lpc43_endwait(struct lpc43_dev_s *priv, sdio_eventset_t wkupevent)
{
  mcinfo("Entry!\n");

  /* Cancel the watchdog timeout */

  (void)wd_cancel(priv->waitwdog);

  /* Disable event-related interrupts */

  lpc43_configwaitints(priv, 0, 0, wkupevent);

  /* Wake up the waiting thread */

  lpc43_givesem(priv);
}

/****************************************************************************
 * Name: lpc43_endtransfer
 *
 * Description:
 *   Terminate a transfer with the provided status.  This function is called
 *   only from the SD card interrupt handler when end-of-transfer conditions
 *   are detected.
 *
 * Input Parameters:
 *   priv   - An instance of the SD card device interface
 *   wkupevent - The event that caused the transfer to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void lpc43_endtransfer(struct lpc43_dev_s *priv, sdio_eventset_t wkupevent)
{
  mcinfo("Entry!\n");

  /* Disable all transfer related interrupts */

  lpc43_configxfrints(priv, 0);

  /* Clearing pending interrupt status on all transfer related interrupts */

  lpc43_putreg(priv->waitmask, LPC43_SDMMC_RINTSTS);

  /* Mark the transfer finished */

  priv->remaining = 0;

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      lpc43_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Interrrupt Handling
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_interrupt
 *
 * Description:
 *   SD card interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the SD card device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int lpc43_interrupt(int irq, void *context, FAR void *arg)
{
  struct lpc43_dev_s *priv = &g_scard_dev;
  uint32_t enabled;
  uint32_t pending;

  /* Loop while there are pending interrupts.  Check the SD card status
   * register.  Mask out all bits that don't correspond to enabled
   * interrupts.  (This depends on the fact that bits are ordered
   * the same in both the STA and MASK register).  If there are non-zero
   * bits remaining, then we have work to do here.
   */

  while ((enabled = lpc43_getreg(LPC43_SDMMC_RINTSTS) & lpc43_getreg(LPC43_SDMMC_INTMASK)) != 0)
    {
      /* Handle in progress, interrupt driven data transfers ****************/
      pending  = enabled & priv->xfrmask;

      if (pending != 0)
        {
          /* Handle data end events */

          if ((pending & SDMMC_INT_DTO) != 0 || (pending & SDMMC_INT_TXDR) != 0)
            {
              /* Finish the transfer */

              lpc43_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
            }

          /* Handle data block send/receive CRC failure */

          else if ((pending & SDMMC_INT_DCRC) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data block CRC failure, remaining: %d\n", priv->remaining);
              lpc43_putreg(SDMMC_INT_DCRC, LPC43_SDMMC_RINTSTS);
              lpc43_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle data timeout error */

          else if ((pending & SDMMC_INT_DRTO) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data timeout, remaining: %d\n", priv->remaining);
              lpc43_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT);
            }

          /* Handle RX FIFO overrun error */

          else if ((pending & SDMMC_INT_FRUN) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: RX FIFO overrun, remaining: %d\n", priv->remaining);
              lpc43_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle TX FIFO underrun error */

          else if ((pending & SDMMC_INT_FRUN) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: TX FIFO underrun, remaining: %d\n", priv->remaining);
              lpc43_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle start bit error */

          else if ((pending & SDMMC_INT_SBE) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Start bit, remaining: %d\n", priv->remaining);
              lpc43_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }
        }

      /* Handle wait events *************************************************/

      pending  = enabled & priv->waitmask;
      if (pending != 0)
        {
          /* Is this a response completion event? */

          if ((pending & SDMMC_INT_DTO) != 0)
            {
              /* Yes.. Is their a thread waiting for response done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  lpc43_putreg(SDCARD_RESPDONE_ICR | SDCARD_CMDDONE_ICR, LPC43_SDMMC_RINTSTS);
                  lpc43_endwait(priv, SDIOWAIT_RESPONSEDONE);
                }
            }

          /* Is this a command completion event? */

          if ((pending & SDMMC_INT_CDONE) != 0)
            {
              /* Yes.. Is their a thread waiting for command done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  lpc43_putreg(SDCARD_CMDDONE_ICR, LPC43_SDMMC_RINTSTS);
                  lpc43_endwait(priv, SDIOWAIT_CMDDONE);
                }
            }
        }
    }
  return OK;
}

/****************************************************************************
 * SD card Interface Methods
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_lock
 *
 * Description:
 *   Locks the bus. Function calls low-level multiplexed bus routines to
 *   resolve bus requests and acknowledgment issues.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   lock   - TRUE to lock, FALSE to unlock.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_MUXBUS
static int lpc43_lock(FAR struct sdio_dev_s *dev, bool lock)
{
  /* Single SD card instance so there is only one possibility.  The multiplex
   * bus is part of board support package.
   */

  lpc43_muxbus_sdio_lock(lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: lpc43_reset
 *
 * Description:
 *   Reset the SD card controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_reset(FAR struct sdio_dev_s *dev)
{
  FAR struct lpc43_dev_s *priv = (FAR struct lpc43_dev_s *)dev;
  irqstate_t flags;
  uint32_t regval;

  mcinfo("Entry!\n");

  flags = enter_critical_section();

  /* Software Reset */

  lpc43_putreg(SDMMC_BMOD_SWR, LPC43_SDMMC_BMOD);

  /* Reset all blocks */

  lpc43_putreg(SDMMC_CTRL_CNTLRRESET | SDMMC_CTRL_FIFORESET | SDMMC_CTRL_DMARESET, LPC43_SDMMC_CTRL);

  while ((regval = lpc43_getreg(LPC43_SDMMC_CTRL)) &
         (SDMMC_CTRL_CNTLRRESET | SDMMC_CTRL_FIFORESET | SDMMC_CTRL_DMARESET));

  /* Reset data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */

  wd_cancel(priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;      /* Address of current R/W buffer */
  priv->remaining  = 0;      /* Number of bytes remaining in the transfer */
  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  priv->widebus    = true;  /* Required for DMA support */

  regval = 0;

#ifdef CONFIG_SDIO_DMA
  priv->dmamode    = false;  /* true: DMA mode transfer */

  /* Use the Internal DMA */

  regval = SDMMC_CTRL_INTDMA;
#endif

  /* Enable interrupts */

  regval |= SDMMC_CTRL_INTENABLE;
  lpc43_putreg(regval, LPC43_SDMMC_CTRL);

  /* Disable Interrupts */

  lpc43_putreg(0, LPC43_SDMMC_INTMASK);

  /* Clear to Interrupts */

  lpc43_putreg(0xffffffff, LPC43_SDMMC_RINTSTS);

  /* Define MAX Timeout */

  lpc43_putreg(SDCARD_LONGTIMEOUT, LPC43_SDMMC_TMOUT);

  regval = 16 | (15 << SDMMC_FIFOTH_RXWMARK_SHIFT);
  lpc43_putreg(regval, LPC43_SDMMC_FIFOTH);

  /* Enable internal DMA, burst size of 4, fixed burst */

  regval  = SDMMC_BMOD_DE;
  regval |= SDMMC_BMOD_PBL_4XFRS;
  regval |= ((4) << SDMMC_BMOD_DSL_SHIFT) & SDMMC_BMOD_DSL_MASK;
  lpc43_putreg(regval, LPC43_SDMMC_BMOD);

  /* Disable clock to CIU (needs latch) */

  lpc43_putreg(0, LPC43_SDMMC_CLKENA);
  lpc43_putreg(0, LPC43_SDMMC_CLKSRC);

  leave_critical_section(flags);

}

/****************************************************************************
 * Name: lpc43_capabilities
 *
 * Description:
 *   Get capabilities (and limitations) of the SDIO driver (optional)
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see SDIO_CAPS_* defines)
 *
 ****************************************************************************/

static sdio_capset_t lpc43_capabilities(FAR struct sdio_dev_s *dev)
{
  sdio_capset_t caps = 0;

  caps |= SDIO_CAPS_DMABEFOREWRITE;

#ifdef CONFIG_LPC43_SDMMC_WIDTH_D1_ONLY
  caps |= SDIO_CAPS_1BIT_ONLY;
#endif
#ifdef CONFIG_SDIO_DMA
  caps |= SDIO_CAPS_DMASUPPORTED;
#endif

  return caps;
}


/****************************************************************************
 * Name: lpc43_status
 *
 * Description:
 *   Get SD card status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see lpc43_status_* defines)
 *
 ****************************************************************************/

static sdio_statset_t lpc43_status(FAR struct sdio_dev_s *dev)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;

  mcinfo("Entry!\n");

  return priv->cdstatus;
}

/****************************************************************************
 * Name: lpc43_widebus
 *
 * Description:
 *   Called after change in Bus width has been selected (via ACMD6).  Most
 *   controllers will need to perform some special operations to work
 *   correctly in the new bus mode.
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *   wide - true: wide bus (4-bit) bus mode enabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_widebus(FAR struct sdio_dev_s *dev, bool wide)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;

  mcinfo("Entry!\n");

  priv->widebus = wide;
}

/****************************************************************************
 * Name: lpc43_clock
 *
 * Description:
 *   Enable/disable SD card clocking
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *   rate - Specifies the clocking to use (see enum sdio_clock_e)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  uint8_t clkdiv;
  uint8_t ctype;
  bool enabled = false;

  switch (rate)
    {
      /* Disable clocking (with default ID mode divisor) */

      default:
      case CLOCK_SDIO_DISABLED:
        clkdiv = LPC43_CLKDIV_INIT;
        ctype  = SDCARD_BUS_D1;
        enabled = false;
        return;
        break;

      /* Enable in initial ID mode clocking (<400KHz) */

      case CLOCK_IDMODE:
        clkdiv = LPC43_CLKDIV_INIT;
        ctype = SDCARD_BUS_D1;
        enabled = true;
        break;

      /* Enable in MMC normal operation clocking */

      case CLOCK_MMC_TRANSFER:
        clkdiv = SDCARD_CLOCK_MMCXFR;
        ctype  = SDCARD_BUS_D1;
        enabled = true;
        break;

      /* SD normal operation clocking (wide 4-bit mode) */

      case CLOCK_SD_TRANSFER_4BIT:
#ifndef CONFIG_SDIO_WIDTH_D1_ONLY
        clkdiv = SDCARD_CLOCK_SDWIDEXFR;
        ctype  = SDCARD_BUS_D4;
        enabled = true;
        break;
#endif

      /* SD normal operation clocking (narrow 1-bit mode) */

      case CLOCK_SD_TRANSFER_1BIT:
        clkdiv = SDCARD_CLOCK_SDXFR;
        ctype  = SDCARD_BUS_D1;
        enabled = true;
        break;
    }

  /* Setup the type of card bus wide */

  lpc43_settype(ctype);

  /* Set the new clock frequency division */

  lpc43_setclock(clkdiv);

  /* Enable disable clock */

  lpc43_sdcard_clock(enabled);
}

/****************************************************************************
 * Name: lpc43_attach
 *
 * Description:
 *   Attach and prepare interrupts
 *
 * Input Parameters:
 *   dev - An instance of the SD card device interface
 *
 * Returned Value:
 *   OK on success; A negated errno on failure.
 *
 ****************************************************************************/

static int lpc43_attach(FAR struct sdio_dev_s *dev)
{
  int ret;
  uint32_t regval;

  mcinfo("Entry!\n");

  /* Attach the SD card interrupt handler */

  ret = irq_attach(LPC43M4_IRQ_SDIO, lpc43_interrupt, NULL);
  if (ret == OK)
    {

      /* Disable all interrupts at the SD card controller and clear static
       * interrupt flags
       */

      lpc43_putreg(SDMMC_INT_RESET, LPC43_SDMMC_INTMASK);
      lpc43_putreg(SDMMC_INT_ALL  , LPC43_SDMMC_RINTSTS);

      /* Enable Interrupts to happen when the INTMASK is activated */

      regval  = lpc43_getreg(LPC43_SDMMC_CTRL);
      regval |= SDMMC_CTRL_INTENABLE;
      lpc43_putreg(regval, LPC43_SDMMC_CTRL);

      /* Enable SD card interrupts at the NVIC.  They can now be enabled at
       * the SD card controller as needed.
       */

      up_enable_irq(LPC43M4_IRQ_SDIO);
    }

  return ret;
}

/****************************************************************************
 * Name: lpc43_sendcmd
 *
 * Description:
 *   Send the SD card command
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *   cmd  - The command to send (32-bits, encoded)
 *   arg  - 32-bit argument required with some commands
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int lpc43_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t arg)
{
  uint32_t regval = 0;
  uint32_t cmdidx;

  mcinfo("Entry!\n");

  /* The CMD0 needs the SENDINIT CMD */

  if (cmd == 0)
    {
      regval |= SDMMC_CMD_SENDINIT;
    }

  /* Is this a Read/Write Transfer Command ? */

  if ((cmd & MMCSD_WRDATAXFR) == MMCSD_WRDATAXFR)
    {
      regval |= SDMMC_CMD_DATAXFREXPTD | SDMMC_CMD_WRITE | SDMMC_CMD_WAITPREV;
    }
  else if ((cmd & MMCSD_RDDATAXFR) == MMCSD_RDDATAXFR)
         {
           regval |= SDMMC_CMD_DATAXFREXPTD | SDMMC_CMD_WAITPREV;
         }

  /* Set WAITRESP bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      regval |= SDMMC_CMD_NORESPONSE;
      break;

    case MMCSD_R1B_RESPONSE:
      regval |= SDMMC_CMD_WAITPREV;
    case MMCSD_R1_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      regval |= SDMMC_CMD_SHORTRESPONSE;
      break;

    case MMCSD_R2_RESPONSE:
      regval |= SDMMC_CMD_LONGRESPONSE;
      break;
    }

  /* Set the command index */

  cmdidx  = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval |= cmdidx;

  mcinfo("cmd: %08x arg: %08x regval: %08x\n", cmd, arg, regval);

  /* Write the SD card CMD */

  lpc43_putreg(SDCARD_RESPDONE_ICR | SDCARD_CMDDONE_ICR, LPC43_SDMMC_RINTSTS);
  lpc43_ciu_sendcmd(regval, arg);

  return OK;
}

/****************************************************************************
 * Name: lpc43_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card in non-DMA
 *   (interrupt driven mode).  This method will do whatever controller setup
 *   is necessary.  This would be called for SD memory just BEFORE sending
 *   CMD13 (SEND_STATUS), CMD17 (READ_SINGLE_BLOCK), CMD18
 *   (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), etc.  Normally, SDCARD_WAITEVENT
 *   will be called to receive the indication that the transfer is complete.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   buffer - Address of the buffer in which to receive the data
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc43_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
                           size_t nbytes)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  uint32_t blocksize, bytecnt;
  uint32_t regval;

  mcinfo("Entry!\n");

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Save the destination buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
#ifdef CONFIG_SDIO_DMA
  priv->dmamode   = false;
#endif

  /* Then set up the SD card data path */

  blocksize = 64;
  bytecnt   = 512;

  lpc43_putreg(blocksize, LPC43_SDMMC_BLKSIZ);
  lpc43_putreg(bytecnt, LPC43_SDMMC_BYTCNT);

  /* And enable interrupts */

  lpc43_configxfrints(priv, SDCARD_RECV_MASK);

  return OK;
}

/****************************************************************************
 * Name: lpc43_sendsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card.  This method
 *   will do whatever controller setup is necessary.  This would be called
 *   for SD memory just AFTER sending CMD24 (WRITE_BLOCK), CMD25
 *   (WRITE_MULTIPLE_BLOCK), ... and before SDCARD_SENDDATA is called.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   buffer - Address of the buffer containing the data to send
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc43_sendsetup(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer,
                           size_t nbytes)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  uint32_t blocksize;

  mcinfo("Entry!\n");

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
#ifdef CONFIG_SDIO_DMA
  priv->dmamode   = false;
#endif

  return OK;
}

/****************************************************************************
 * Name: lpc43_cancel
 *
 * Description:
 *   Cancel the data transfer setup of SDCARD_RECVSETUP, SDCARD_SENDSETUP,
 *   SDCARD_DMARECVSETUP or SDCARD_DMASENDSETUP.  This must be called to cancel
 *   the data transfer setup if, for some reason, you cannot perform the
 *   transfer.
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc43_cancel(FAR struct sdio_dev_s *dev)
{
  mcinfo("Entry!\n");

  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;

  /* Disable all transfer- and event- related interrupts */

  lpc43_configxfrints(priv, 0);
  lpc43_configwaitints(priv, 0, 0, 0);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */

  lpc43_putreg(SDCARD_WAITALL_ICR, LPC43_SDMMC_RINTSTS);

  /* Cancel any watchdog timeout */

  (void)wd_cancel(priv->waitwdog);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_SDIO_DMA
  if (priv->dmamode)
    {
      /* Make sure that the DMA is stopped (it will be stopped automatically
       * on normal transfers, but not necessarily when the transfer terminates
       * on an error condition.
       */

      //lpc43_dmastop(priv->dma);
    }
#endif

  /* Mark no transfer in progress */

  priv->remaining = 0;

  return OK;
}

/****************************************************************************
 * Name: lpc43_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *   cmd  - The command that was sent.  See 32-bit command definitions above.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc43_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd)
{
  mcinfo("Entry!\n");

  int32_t timeout;
  uint32_t events;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      events  = SDCARD_CMDDONE_STA;
      timeout = SDCARD_CMDTIMEOUT;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R6_RESPONSE:
      events  = SDCARD_RESPDONE_STA;
      timeout = SDCARD_LONGTIMEOUT;
      break;

    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
      return -ENOSYS;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      events  = SDCARD_RESPDONE_STA;
      timeout = SDCARD_CMDTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  events |= SDCARD_CMDDONE_STA;

  mcinfo("cmd: %08x events: %08x STATUS: %08x RINTSTS: %08x\n",
               cmd, events, lpc43_getreg(LPC43_SDMMC_STATUS), lpc43_getreg(LPC43_SDMMC_RINTSTS));

  /* Any interrupt error? */

  if (lpc43_getreg(LPC43_SDMMC_RINTSTS) & SDCARD_INT_ERROR)
    {
      mcerr("Error interruption!\n");
      return -ETIMEDOUT;
    }

  if (cmd == 0x451)
    {
      events = 0;
    }

  /* Then wait for the response (or timeout) */

  while ((lpc43_getreg(LPC43_SDMMC_RINTSTS) & events) != events)
    {
      if (--timeout <= 0)
        {
          mcerr("ERROR: Timeout cmd: %08x events: %08x STA: %08x RINTSTS: %08x\n",
               cmd, events, lpc43_getreg(LPC43_SDMMC_STATUS), lpc43_getreg(LPC43_SDMMC_RINTSTS));

          return -ETIMEDOUT;
        }
    }

  if ((lpc43_getreg(LPC43_SDMMC_STATUS) & SDMMC_STATUS_FIFOCOUNT_MASK) > 0)
    {
      mcinfo("There is data on FIFO!\n");
    }

  lpc43_putreg(SDCARD_CMDDONE_ICR, LPC43_SDMMC_RINTSTS);
  return OK;
}

/****************************************************************************
 * Name: lpc43_recvRx
 *
 * Description:
 *   Receive response to SD card command.  Only the critical payload is
 *   returned -- that is 32 bits for 48 bit status and 128 bits for 136 bit
 *   status.  The driver implementation should verify the correctness of
 *   the remaining, non-returned bits (CRCs, CMD index, etc.).
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   Rx - Buffer in which to receive the response
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a faiure to obtain the requested reponse (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intacta and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int lpc43_recvshortcrc(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *rshort)
{
#ifdef CONFIG_DEBUG_FEATURES
  uint32_t respcmd;
#endif
  uint32_t regval;

  int ret = OK;

  mcinfo("Entry! CMD = %08x\n", cmd);

  /* R1  Command response (48-bit)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Command index (0-63)
   *     39:8      bit31  - bit0   32-bit card status
   *     7:1       bit6   - bit0   CRC7
   *     0         1               End bit
   *
   * R1b Identical to R1 with the additional busy signaling via the data
   *     line.
   *
   * R6  Published RCA Response (48-bit, SD card only)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Command index (0-63)
   *     39:8      bit31  - bit0   32-bit Argument Field, consisting of:
   *                               [31:16] New published RCA of card
   *                               [15:0]  Card status bits {23,22,19,12:0}
   *     7:1       bit6   - bit0   CRC7
   *     0         1               End bit
   */


#ifdef CONFIG_DEBUG_FEATURES
  if (!rshort)
    {
      mcerr("ERROR: rshort=NULL\n");
      ret = -EINVAL;
    }

  /* Check that this is the correct response to this command */

  else if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1B_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R6_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = lpc43_getreg(LPC43_SDMMC_RINTSTS);
      if ((regval & SDMMC_INT_RTO) != 0)
        {
          mcerr("ERROR: Command timeout: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & SDMMC_INT_RCRC) != 0)
        {
          mcerr("ERROR: CRC failure: %08x\n", regval);
          ret = -EIO;
        }
    }

  /* Clear all pending message completion events and return the R1/R6 response */

  lpc43_putreg(SDCARD_RESPDONE_ICR | SDCARD_CMDDONE_ICR, LPC43_SDMMC_RINTSTS);
  *rshort = lpc43_getreg(LPC43_SDMMC_RESP0);
  mcinfo("CRC = %08x\n", *rshort);

  return ret;
}

static int lpc43_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t rlong[4])
{
  uint32_t regval;
  int ret = OK;

  mcinfo("Entry!\n");


  /* R2  CID, CSD register (136-bit)
   *     135       0               Start bit
   *     134       0               Transmission bit (0=from card)
   *     133:128   bit5   - bit0   Reserved
   *     127:1     bit127 - bit1   127-bit CID or CSD register
   *                               (including internal CRC)
   *     0         1               End bit
   */

#ifdef CONFIG_DEBUG_FEATURES
  /* Check that R1 is the correct response to this command */

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R2_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = lpc43_getreg(LPC43_SDMMC_RINTSTS);
      if (regval & SDMMC_INT_RTO)
        {
          mcerr("ERROR: Timeout STA: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & SDMMC_INT_RCRC)
        {
          mcerr("ERROR: CRC fail STA: %08x\n", regval);
          ret = -EIO;
        }
    }

  /* Return the long response */

  lpc43_putreg(SDCARD_RESPDONE_ICR | SDCARD_CMDDONE_ICR, LPC43_SDMMC_RINTSTS);
  if (rlong)
    {
      rlong[0] = lpc43_getreg(LPC43_SDMMC_RESP3);
      rlong[1] = lpc43_getreg(LPC43_SDMMC_RESP2);
      rlong[2] = lpc43_getreg(LPC43_SDMMC_RESP1);
      rlong[3] = lpc43_getreg(LPC43_SDMMC_RESP0);
    }

  return ret;
}

static int lpc43_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *rshort)
{
  uint32_t regval;
  int ret = OK;

  mcinfo("Entry!\n");

  /* R3  OCR (48-bit)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Reserved
   *     39:8      bit31  - bit0   32-bit OCR register
   *     7:1       bit6   - bit0   Reserved
   *     0         1               End bit
   */

  /* Check that this is the correct response to this command */

#ifdef CONFIG_DEBUG_FEATURES
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout occurred (Apparently a CRC error can terminate
       * a good response)
       */

      regval = lpc43_getreg(LPC43_SDMMC_RINTSTS);
      if (regval & SDMMC_INT_RTO)
        {
          mcerr("ERROR: Timeout STA: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
    }

  lpc43_putreg(SDCARD_RESPDONE_ICR | SDCARD_CMDDONE_ICR, LPC43_SDMMC_RINTSTS);
  if (rshort)
    {
      *rshort = lpc43_getreg(LPC43_SDMMC_RESP0);
    }

  return ret;
}

/* MMC responses not supported */

static int lpc43_recvnotimpl(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *rnotimpl)
{
  mcinfo("Entry!\n");

  lpc43_putreg(SDCARD_RESPDONE_ICR | SDCARD_CMDDONE_ICR, LPC43_SDMMC_RINTSTS);

  return -ENOSYS;
}

/****************************************************************************
 * Name: lpc43_waitenable
 *
 * Description:
 *   Enable/disable of a set of SD card wait events.  This is part of the
 *   the SDCARD_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling lpc43_eventwait.  This is done in this way
 *   to help the driver to eliminate race conditions between the command
 *   setup and the subsequent events.
 *
 *   The enabled events persist until either (1) SDCARD_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) SDCARD_EVENTWAIT
 *   returns.
 *
 * Input Parameters:
 *   dev      - An instance of the SD card device interface
 *   eventset - A bitset of events to enable or disable (see SDIOWAIT_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_waitenable(FAR struct sdio_dev_s *dev,
                             sdio_eventset_t eventset)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  uint32_t waitmask;

  mcinfo("Entry!\n");

  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  lpc43_configwaitints(priv, 0, 0, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  waitmask = 0;
  if ((eventset & SDIOWAIT_CMDDONE) != 0)
    {
      waitmask |= SDCARD_CMDDONE_MASK;
    }

  if ((eventset & SDIOWAIT_RESPONSEDONE) != 0)
    {
      waitmask |= SDCARD_RESPDONE_MASK;
    }

  if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
    {
      waitmask |= SDCARD_XFRDONE_MASK;
    }

  /* Enable event-related interrupts */

  lpc43_configwaitints(priv, waitmask, eventset, 0);
}

/****************************************************************************
 * Name: lpc43_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDCARD_WAITEVENTS are disabled when lpc43_eventwait
 *   returns.  SDCARD_WAITEVENTS must be called again before lpc43_eventwait
 *   can be used again.
 *
 * Input Parameters:
 *   dev     - An instance of the SD card device interface
 *   timeout - Maximum time in milliseconds to wait.  Zero means immediate
 *             timeout with no wait.  The timeout value is ignored if
 *             SDIOWAIT_TIMEOUT is not included in the waited-for eventset.
 *
 * Returned Value:
 *   Event set containing the event(s) that ended the wait.  Should always
 *   be non-zero.  All events are disabled after the wait concludes.
 *
 ****************************************************************************/

static sdio_eventset_t lpc43_eventwait(FAR struct sdio_dev_s *dev,
                                       uint32_t timeout)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  sdio_eventset_t wkupevent = 0;
  irqstate_t flags;
  int ret;

  mcinfo("Entry!\n");

  /* There is a race condition here... the event may have completed before
   * we get here.  In this case waitevents will be zero, but wkupevents will
   * be non-zero (and, hopefully, the semaphore count will also be non-zero.
   */

  flags = enter_critical_section();
  DEBUGASSERT(priv->waitevents != 0 || priv->wkupevent != 0);

  /* Check if the timeout event is specified in the event set */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      int delay;

      /* Yes.. Handle a cornercase: The user request a timeout event but
       * with timeout == 0?
       */

      if (!timeout)
        {
          /* Then just tell the caller that we already timed out */

          wkupevent = SDIOWAIT_TIMEOUT;
          goto errout;
        }

      /* Start the watchdog timer */

      delay = MSEC2TICK(timeout);
      ret   = wd_start(priv->waitwdog, delay, (wdentry_t)lpc43_eventtimeout,
                       1, (uint32_t)priv);
      if (ret != OK)
        {
          mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }

  /* Loop until the event (or the timeout occurs). Race conditions are avoided
   * by calling lpc43_waitenable prior to triggering the logic that will cause
   * the wait to terminate.  Under certain race conditions, the waited-for
   * may have already occurred before this function was called!
   */

  for (; ; )
    {
      /* Wait for an event in event set to occur.  If this the event has already
       * occurred, then the semaphore will already have been incremented and
       * there will be no wait.
       */

      lpc43_takesem(priv);
      wkupevent = priv->wkupevent;

      /* Check if the event has occurred.  When the event has occurred, then
       * evenset will be set to 0 and wkupevent will be set to a nonzero value.
       */

      if (wkupevent != 0)
        {
          /* Yes... break out of the loop with wkupevent non-zero */

          break;
        }
    }

  /* Disable event-related interrupts */

  lpc43_configwaitints(priv, 0, 0, 0);

errout:
  leave_critical_section(flags);
  return wkupevent;
}

/****************************************************************************
 * Name: lpc43_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SD card callback events.  This is part of the
 *   the SD card callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in lpc43_registercallback.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this methos.
 *
 * Input Parameters:
 *   dev      - An instance of the SD card device interface
 *   eventset - A bitset of events to enable or disable (see SDIOMEDIA_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_callbackenable(FAR struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;

  mcinfo("Entry!\n");

  mcinfo("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  lpc43_callback(priv);
}

/****************************************************************************
 * Name: lpc43_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to SDCARD_CALLBACKENABLE
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The funtion to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

static int lpc43_registercallback(FAR struct sdio_dev_s *dev,
                                  worker_t callback, void *arg)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;

  mcinfo("Entry!\n");

  /* Disable callbacks and register this callback and is argument */

  mcinfo("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: lpc43_dmarecvsetup
 *
 * Description:
 *   Setup to perform a read DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For read transfers this may mean
 *   invalidating the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   buffer - The memory to DMA from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int lpc43_dmarecvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
                              size_t buflen)
{
  mcinfo("Entry!\n");

  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  //uint32_t blocksize;
  //uint32_t bytecnt;
  uint32_t regval;
  int ret = OK;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Wide bus operation is required for DMA */

  if (priv->widebus)
    {
      /* Save the destination buffer information for use by the interrupt handler */

      priv->buffer    = (uint32_t *)buffer;
      priv->remaining = buflen;
      priv->dmamode   = true;

      /* Reset DMA */

      regval  = lpc43_getreg(LPC43_SDMMC_CTRL);
      regval |= SDMMC_CTRL_FIFORESET | SDMMC_CTRL_DMARESET;
      lpc43_putreg(regval, LPC43_SDMMC_CTRL);
      while (lpc43_getreg(LPC43_SDMMC_CTRL) & SDMMC_CTRL_DMARESET);

      /* Setup DMA list */

      mci_dma_dd[0].des0 = MCI_DMADES0_OWN | MCI_DMADES0_CH | MCI_DMADES0_FS | MCI_DMADES0_LD;
      mci_dma_dd[0].des1 = 512;
      mci_dma_dd[0].des2 = priv->buffer;
      mci_dma_dd[0].des3 = (uint32_t) &mci_dma_dd[1];
    
      lpc43_putreg((uint32_t) &mci_dma_dd[0], LPC43_SDMMC_DBADDR);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: lpc43_dmasendsetup
 *
 * Description:
 *   Setup to perform a write DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For write transfers, this may mean
 *   flushing the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   buffer - The memory to DMA into
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static int lpc43_dmasendsetup(FAR struct sdio_dev_s *dev,
                              FAR const uint8_t *buffer, size_t buflen)
{
  mcinfo("Entry!\n");

  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  uint32_t blocksize;
  uint32_t bytecnt;
  uint32_t regval;
  int ret = OK;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Wide bus operation is required for DMA */

  if (priv->widebus)
    {
      /* Save the destination buffer information for use by the interrupt handler */

      priv->buffer    = (uint32_t *)buffer;
      priv->remaining = buflen;
      priv->dmamode   = true;

      /* Reset DMA */

      regval  = lpc43_getreg(LPC43_SDMMC_CTRL);
      regval |= SDMMC_CTRL_FIFORESET | SDMMC_CTRL_DMARESET;
      lpc43_putreg(regval, LPC43_SDMMC_CTRL);
      while (lpc43_getreg(LPC43_SDMMC_CTRL) & SDMMC_CTRL_DMARESET);

      /* Setup DMA list */

      mci_dma_dd[0].des0 = MCI_DMADES0_OWN | MCI_DMADES0_CH | MCI_DMADES0_LD;
      mci_dma_dd[0].des1 = 512;
      mci_dma_dd[0].des2 = priv->buffer;
      mci_dma_dd[0].des3 = (uint32_t) &mci_dma_dd[1];
    
      lpc43_putreg((uint32_t) &mci_dma_dd[0], LPC43_SDMMC_DBADDR);
    }

  return ret;
}
#endif

/****************************************************************************
 * Initialization/uninitialization/reset
 ****************************************************************************/
/****************************************************************************
 * Name: lpc43_callback
 *
 * Description:
 *   Perform callback.
 *
 * Assumptions:
 *   This function does not execute in the context of an interrupt handler.
 *   It may be invoked on any user thread or scheduled on the work thread
 *   from an interrupt handler.
 *
 ****************************************************************************/

static void lpc43_callback(void *arg)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)arg;

  mcinfo("Entry!\n");

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);
  mcinfo("Callback %p(%p) cbevents: %02x cdstatus: %02x\n",
         priv->callback, priv->cbarg, priv->cbevents, priv->cdstatus);

  if (priv->callback)
    {
      /* Yes.. Check for enabled callback events */

      if ((priv->cdstatus & SDIO_STATUS_PRESENT) != 0)
        {
          /* Media is present.  Is the media inserted event enabled? */

          if ((priv->cbevents & SDIOMEDIA_INSERTED) == 0)
            {
              /* No... return without performing the callback */

              mcinfo("Media is not Inserted!\n");
              return;
            }
        }
      else
        {
          /* Media is not present.  Is the media eject event enabled? */

          if ((priv->cbevents & SDIOMEDIA_EJECTED) == 0)
            {
              /* No... return without performing the callback */

              mcinfo("Media is not Ejected!\n");
              return;
            }
        }

      /* Perform the callback, disabling further callbacks.  Of course, the
       * the callback can (and probably should) re-enable callbacks.
       */

      priv->cbevents = 0;

      /* Callbacks cannot be performed in the context of an interrupt handler.
       * If we are in an interrupt handler, then queue the callback to be
       * performed later on the work thread.
       */

      if (up_interrupt_context())
        {
          /* Yes.. queue it */

           mcinfo("Queuing callback to %p(%p)\n", priv->callback, priv->cbarg);
          (void)work_queue(HPWORK, &priv->cbwork, (worker_t)priv->callback, priv->cbarg, 0);
        }
      else
        {
          /* No.. then just call the callback here */

          mcinfo("Callback to %p(%p)\n", priv->callback, priv->cbarg);
          priv->callback(priv->cbarg);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SD card for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Values:
 *   A reference to an SD card interface structure.  NULL is returned on failures.
 *
 ****************************************************************************/

FAR struct sdio_dev_s *sdio_initialize(int slotno)
{
  uint32_t   regval;

  mcinfo("Entry!\n");

  /* There is only one slot */

  struct lpc43_dev_s *priv = &g_scard_dev;

  /* Enable clocking to the SDIO block */

  regval  = lpc43_getreg(LPC43_CCU1_M4_SDIO_CFG);
  regval |= CCU_CLK_CFG_RUN;
  regval |= CCU_CLK_CFG_AUTO;
  regval |= CCU_CLK_CFG_WAKEUP;
  lpc43_putreg(regval, LPC43_CCU1_M4_SDIO_CFG);

  /* Initialize semaphores */

  sem_init(&priv->waitsem, 0, 0);

  /* The waitsem semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  sem_setprotocol(&priv->waitsem, SEM_PRIO_NONE);

  /* Create a watchdog timer */

  priv->waitwdog = wd_create();
  DEBUGASSERT(priv->waitwdog);

  /* Configure GPIOs for 4-bit, wide-bus operation */

  lpc43_pin_config(PINCONF_SD_DAT0_1);
#ifndef CONFIG_SDIO_WIDTH_D1_ONLY
  lpc43_pin_config(PINCONF_SD_DAT1_1);
  lpc43_pin_config(PINCONF_SD_DAT2_1);
  lpc43_pin_config(PINCONF_SD_DAT3_1);
#endif
  lpc43_pin_config(PINCONF_SD_CD_1);
  lpc43_pin_config(PINCONF_SD_CMD_1);

  //lpc43_pin_config(CLKCONF_SD_CLK_2);
  regval  = getreg32(LPC43_SCU_SFSCLK2);
  regval |= (2 << 3);   /* Disable pull-down and pull-up resistor */
  regval |= (1 << 6);   /* Enable Input buffer */
  regval |= (4);        /* Selects pin function 4 */
  putreg32(regval, LPC43_SCU_SFSCLK2);

  /* Reset the card and assure that it is in the initial, unconfigured
   * state.
   */

  lpc43_reset(&priv->dev);

  mcinfo("Leaving!\n");

  return &g_scard_dev.dev;
}

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic -- posssible from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot
 *
 * Input Parameters:
 *   dev        - An instance of the SD card driver device state structure.
 *   cardinslot - true is a card has been detected in the slot; false if a
 *                card has been removed from the slot.  Only transitions
 *                (inserted->removed or removed->inserted should be reported)
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void sdio_mediachange(FAR struct sdio_dev_s *dev, bool cardinslot)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  sdio_statset_t cdstatus;
  irqstate_t flags;

  mcinfo("Entry!\n");

  /* Update card status */

  flags = enter_critical_section();
  cdstatus = priv->cdstatus;
  if (cardinslot)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }
  mcinfo("cdstatus OLD: %02x NEW: %02x\n", cdstatus, priv->cdstatus);

  /* Perform any requested callback if the status has changed */

  if (cdstatus != priv->cdstatus)
    {
      lpc43_callback(priv);
    }
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report if the card in the slot is
 *   mechanically write protected.
 *
 * Input Parameters:
 *   dev       - An instance of the SD card driver device state structure.
 *   wrprotect - true is a card is writeprotected.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void sdio_wrprotect(FAR struct sdio_dev_s *dev, bool wrprotect)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  irqstate_t flags;

  mcinfo("Entry!\n");

  /* Update card status */

  flags = enter_critical_section();
  if (wrprotect)
    {
      priv->cdstatus |= SDIO_STATUS_WRPROTECTED;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_WRPROTECTED;
    }

  mcinfo("cdstatus: %02x\n", priv->cdstatus);
  leave_critical_section(flags);
}
#endif /* CONFIG_LPC43_SDMMC */
