/****************************************************************************
 * arch/arm/src/lpc43/lpc43_timer.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Alan Carvalho de Assis <acassis@gmail.com>
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
#include <nuttx/arch.h>

#include <sys/types.h>

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "lpc43_timer.h"

#if defined(CONFIG_TIMER) && (defined(CONFIG_LPC43_TMR0) || \
    defined(CONFIG_LPC43_TMR1) || defined(CONFIG_LPC43_TMR2) || \
    defined(CONFIG_LPC43_TMR3) )

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_DEBUG_TIMER_INFO
#  undef CONFIG_LPC43_TMR_REGDEBUG
#endif

/* Clocking *****************************************************************/

/* TODO: Allow selection of any of the input clocks */

#define TMR_FCLK        (BOARD_FCLKOUT_FREQUENCY)
#define TMR_MAXTIMEOUT  ((1000000ULL * (1ULL + TMR_RVALUE_MASK)) / TMR_FCLK)

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct lpc43_lowerhalf_s
{
  FAR const struct timer_ops_s  *ops;  /* Lower half operations */

  /* Private data */

  uint32_t base;            /* Base address of the timer */
  tccb_t   callback;        /* Current user interrupt callback */
  FAR void *arg;            /* Argument passed to the callback function */
  uint32_t timeout;         /* The current timeout value (us) */
  uint32_t adjustment;      /* time lost due to clock resolution truncation (us) */
  uint32_t clkticks;        /* actual clock ticks for current interval */
  bool     started;         /* The timer has been started */
  uint16_t tmrid;           /* Timer id */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#ifdef CONFIG_LPC43_TMR_REGDEBUG
static uint32_t lpc43_getreg(uint32_t addr);
static void     lpc43_putreg(uint32_t val, uint32_t addr);
#else
# define        lpc43_getreg(addr)     getreg32(addr)
# define        lpc43_putreg(val,addr) putreg32(val,addr)
#endif

/* Interrupt handling *******************************************************/

static int      lpc43_interrupt(int irq, FAR void *context, FAR void *arg);

/* "Lower half" driver methods **********************************************/

static int      lpc43_start(FAR struct timer_lowerhalf_s *lower);
static int      lpc43_stop(FAR struct timer_lowerhalf_s *lower);
static int      lpc43_getstatus(FAR struct timer_lowerhalf_s *lower,
                  FAR struct timer_status_s *status);
static int      lpc43_settimeout(FAR struct timer_lowerhalf_s *lower,
                  uint32_t timeout);
static void     lpc43_setcallback(FAR struct timer_lowerhalf_s *lower,
                  tccb_t callback, FAR void *arg);
static int      lpc43_ioctl(FAR struct timer_lowerhalf_s *lower, int cmd,
                  unsigned long arg);

static int      lpc43_setisr(FAR struct timer_lowerhalf_s *lower,
                            int (*handler)(int irq, void *context),
                            int source);
static int      lpc43_setclock(FAR struct timer_lowerhalf_s *lower, uint32_t freq);
static void     lpc43_enableint(FAR struct timer_lowerhalf_s *lower, int source);
static void     lpc43_disableint(FAR struct timer_lowerhalf_s *lower, int source);
static void     lpc43_ackint(FAR struct timer_lowerhalf_s *lower, int source);
static int      lpc43_checkint(FAR struct timer_lowerhalf_s *lower, int source);


/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_tmrops =
{
  .start      = lpc43_start,
  .stop       = lpc43_stop,
  .getstatus  = lpc43_getstatus,
  .settimeout = lpc43_settimeout,
  .setcallback = lpc43_setcallback,
  .ioctl      = lpc43_ioctl,
  .setclock   = lpc43_setclock,
  .setisr     = lpc43_setisr,
  .enableint  = lpc43_enableint,
  .disableint = lpc43_disableint,
  .ackint     = lpc43_ackint,
  .checkint   = lpc43_checkint,
};

/* "Lower half" driver state */

/* TODO - allocating all 6 now, even though we might not need them.
 *        May want to allocate the right number to not be wasteful.
 */

static struct lpc43_lowerhalf_s g_tmrdevs[4];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_getreg
 *
 * Description:
 *   Get the contents of a register
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_TMR_REGDEBUG
static uint32_t lpc43_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t count = 0;
  static uint32_t preval = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same registe last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              tmrinfo("...\n");
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

          tmrinfo("[repeats %d more times]\n", count-3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  tmrinfo("%08lx->%08lx\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: lpc43_putreg
 *
 * Description:
 *   Set the contents of an LPC43 register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_TMR_REGDEBUG
static void lpc43_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  tmrinfo("%08lx<-%08lx\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

void tmr_clk_enable(uint16_t tmrid)
{
  uint32_t regval;

  /* Enable Timer 0 */

  if (tmrid == 0)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER0_CFG);
      regval |= CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER0_CFG);
    }

  /* Enable Timer 1 */

  if (tmrid == 1)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER1_CFG);
      regval |= CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER1_CFG);
    }

  /* Enable Timer 2 */

  if (tmrid == 2)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER2_CFG);
      regval |= CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER2_CFG);
    }

  /* Enable Timer 3 */

  if (tmrid == 3)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER3_CFG);
      regval |= CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER3_CFG);
    }
}

void tmr_clk_disable(uint16_t tmrid)
{
  uint32_t regval;

  /* Enable Timer 0 */

  if (tmrid == 0)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER0_CFG);
      regval &= ~CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER0_CFG);
    }

  /* Enable Timer 1 */

  if (tmrid == 1)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER1_CFG);
      regval &= ~CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER1_CFG);
    }

  /* Enable Timer 2 */

  if (tmrid == 2)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER2_CFG);
      regval &= ~CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER2_CFG);
    }

  /* Enable Timer 3 */

  if (tmrid == 3)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER3_CFG);
      regval &= ~CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER3_CFG);
    }
}

/****************************************************************************
 * Name: lpc43_interrupt
 *
 * Description:
 *   TC interrupt
 *
 * Input Parameters:
 *   Usual interrupt callback arguments.
 *
 * Returned Values:
 *   Always returns OK.
 *
 ****************************************************************************/

static int lpc43_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint8_t chan_int = 0x0f;
  FAR struct lpc43_lowerhalf_s *priv = &g_tmrdevs[irq-LPC43M4_IRQ_TIMER0];

  tmrinfo("Entry\n");
  DEBUGASSERT((irq >= LPC43M4_IRQ_TIMER0) && (irq <= LPC43M4_IRQ_TIMER3));

  /* Check if the interrupt is really pending */

  if ((lpc43_getreg(priv->base + LPC43_TMR_IR_OFFSET) & chan_int) != 0)
    {
      uint32_t timeout;

      /* Is there a registered callback?  If the callback has been
       * nullified, the timer will be stopped.
       */

      if (priv->callback && priv->callback(&priv->timeout, priv->arg))
        {
          /* Calculate new ticks / dither adjustment */

          //priv->clkticks = ((uint64_t)(priv->adjustment + priv->timeout))*TMR_FCLK / 1000000;

          /* Set next interval interval. TODO: make sure the interval is not so soon it will be missed! */

          //lpc43_putreg(priv->clkticks, priv->base + LPC43_TMR_PR_OFFSET);

          //timeout = (1000000ULL * priv->clkticks) / TMR_FCLK;    /* trucated timeout */
          //priv->adjustment = (priv->adjustment + priv->timeout) - timeout;  /* truncated time to be added to next interval (dither) */
        }
      else
        {
          /* No handler or the handler returned false.. stop the timer */

          lpc43_stop((FAR struct timer_lowerhalf_s *)priv);
          tmrinfo("Stopped\n");
        }

      /* Clear the interrupts */

      lpc43_putreg(chan_int, priv->base + LPC43_TMR_IR_OFFSET);
    }

  return OK;
}

/****************************************************************************
 * Name: lpc43_tmr_enable
 *
 * Description:
 *   Reset and enable the timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

void lpc43_tmr_enable(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct lpc43_lowerhalf_s *priv = (FAR struct lpc43_lowerhalf_s *)lower;

  DEBUGASSERT(priv);

  /* Enable the timer */

  lpc43_putreg(TMR_TCR_RESET, priv->base + LPC43_TMR_TCR_OFFSET);
  lpc43_putreg(TMR_TCR_EN, priv->base + LPC43_TMR_TCR_OFFSET);
}

/****************************************************************************
 * Name: lpc43_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc43_start(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct lpc43_lowerhalf_s *priv = (FAR struct lpc43_lowerhalf_s *)lower;
  uint32_t presc_val;
  uint32_t regval;

  tmrinfo("Entry\n");
  DEBUGASSERT(priv);

  if (priv->started)
    {
      return -EINVAL;
    }

  /* Enable timer clock */

  tmr_clk_enable(priv->tmrid);

  /* Set it to Timer Mode */

  lpc43_putreg(0, priv->base + LPC43_TMR_CTCR_OFFSET);

  /* Disable the timer */

  lpc43_putreg(0, priv->base + LPC43_TMR_TCR_OFFSET);

  /* Set prescaler to increase TC each 1 us */

  presc_val = TMR_FCLK / 1000000;
  lpc43_putreg(presc_val - 1, priv->base + LPC43_TMR_PR_OFFSET);

  /* Set MR0 with a timeout value */

  lpc43_putreg(priv->timeout, priv->base + LPC43_TMR_MR0_OFFSET);

  lpc43_putreg(0, priv->base + LPC43_TMR_CCR_OFFSET); /* do not use capture */

  if (priv->callback)
    {
      /* Enable Match on MR0 generate interrupt and auto-restart */

      lpc43_putreg(3, priv->base + LPC43_TMR_MCR_OFFSET);
    }

  /* Enable the timer */

  lpc43_putreg(TMR_TCR_EN, priv->base + LPC43_TMR_TCR_OFFSET);

  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: lpc43_stop
 *
 * Description:
 *   Stop the timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc43_stop(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct lpc43_lowerhalf_s *priv = (FAR struct lpc43_lowerhalf_s *)lower;
  tmrinfo("Entry\n");
  DEBUGASSERT(priv);

  if (!priv->started)
    {
      return -EINVAL;
    }

  /* Disable timer */

  lpc43_putreg(0, priv->base + LPC43_TMR_TCR_OFFSET);

  /* Disable interrupt */

  lpc43_putreg(0, priv->base + LPC43_TMR_MCR_OFFSET);

  /* Disable timer clock */

  tmr_clk_disable(priv->tmrid);

  priv->started = false;

  return OK;
}

/****************************************************************************
 * Name: lpc43_getstatus
 *
 * Description:
 *   Get the current timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the "lower-
 *            half" driver state structure.
 *   status - The location to return the status information.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc43_getstatus(FAR struct timer_lowerhalf_s *lower,
                           FAR struct timer_status_s *status)
{
  FAR struct lpc43_lowerhalf_s *priv = (FAR struct lpc43_lowerhalf_s *)lower;
  uint32_t elapsed;

  tmrinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = 0;
  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  /* Return the actual timeout is milliseconds */

  status->timeout = priv->timeout;

  /* Get the time remaining until the timer expires (in microseconds) */
  /* TODO - check on the +1 in the time left calculation */

  elapsed = lpc43_getreg(priv->base + LPC43_TMR_TC_OFFSET);
  status->timeleft = ((uint64_t)priv->timeout * elapsed) /
    (priv->clkticks + 1);

  tmrinfo("  flags    : %08x\n", status->flags);
  tmrinfo("  timeout  : %d\n", status->timeout);
  tmrinfo("  timeleft : %d\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: lpc43_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower
 *             half" driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc43_settimeout(FAR struct timer_lowerhalf_s *lower,
                            uint32_t timeout)
{
  FAR struct lpc43_lowerhalf_s *priv = (FAR struct lpc43_lowerhalf_s *)lower;

  DEBUGASSERT(priv);

  if (priv->started)
    {
      return -EPERM;
    }

  tmrinfo("Entry: timeout=%d\n", timeout);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > TMR_MAXTIMEOUT)
    {
      tmrerr("ERROR: Cannot represent timeout=%lu > %lu\n",
             timeout, TMR_MAXTIMEOUT);
      return -ERANGE;
    }

  /* Intended timeout */

  priv->timeout = timeout;

  /* Actual clock ticks */

  priv->clkticks = (((uint64_t)timeout * TMR_FCLK) / 1000000);

  /* Truncated timeout */

  timeout = (1000000ULL * priv->clkticks) / TMR_FCLK;

  /* Truncated time to be added to next interval (dither) */

  priv->adjustment = priv->timeout - timeout;

  tmrinfo("fclk=%d clkticks=%d timout=%d, adjustment=%d\n",
          TMR_FCLK, priv->clkticks, priv->timeout, priv->adjustment);

  return OK;
}

/****************************************************************************
 * Name: lpc43_setcallback
 *
 * Description:
 *   Call this user provided timeout callback.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the "lower-half"
 *                driver state structure.
 *   newcallback - The new timer expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Values:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static void lpc43_setcallback(FAR struct timer_lowerhalf_s *lower,
                               tccb_t callback, FAR void *arg)
{
  FAR struct lpc43_lowerhalf_s *priv = (FAR struct lpc43_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();

  DEBUGASSERT(priv);
  tmrinfo("Entry: callback=%p\n", callback);

  /* Save the new callback and its argument */

   priv->callback = callback;
   priv->arg      = arg;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc43_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *   cmd   - The ioctl command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc43_ioctl(FAR struct timer_lowerhalf_s *lower, int cmd,
                    unsigned long arg)
{
  FAR struct lpc43_lowerhalf_s *priv = (FAR struct lpc43_lowerhalf_s *)lower;
  int ret = -ENOTTY;

  DEBUGASSERT(priv);
  tmrinfo("Entry: cmd=%d arg=%ld\n", cmd, arg);
  UNUSED(priv);

  return ret;
}

/****************************************************************************
 * Name: lpc43_setclock
 *
 * Description:
 *   Setup new clock frequency to timer input
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/


static int lpc43_setclock(FAR struct timer_lowerhalf_s *lower, uint32_t freq)
{
  uint64_t freqin;
  uint32_t prescaler;

  tmrinfo("Entry");

  FAR struct lpc43_lowerhalf_s *priv = (FAR struct lpc43_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL);

  /* Disable Timer? */

  if (freq == 0)
    {
      //stm32_tim_disable(dev);
      return 0;
    }

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  switch (priv->base)
    {
#ifdef CONFIG_LPC43_TMR0
      case LPC43_TIMER0_BASE:
        freqin = TMR_FCLK;
        break;
#endif
#ifdef CONFIG_LPC43_TMR1
      case LPC43_TIMER1_BASE:
        freqin = TMR_FCLK;
        break;
#endif
#ifdef CONFIG_LPC43_TMR2
      case LPC43_TIMER2_BASE:
        freqin = TMR_FCLK;
        break;
#endif
#ifdef CONFIG_LPC43_TMR3
      case LPC43_TIMER3_BASE:
        freqin = TMR_FCLK;
        break;
#endif

      default:
        return -EINVAL;
    }

  /* Select a pre-scaler value for this timer using the input clock
   * frequency.
   */

  prescaler = freqin / freq;

  /* We need to decrement value for '1', but only, if that will not to
   * cause underflow.
   */

  if (prescaler > 0)
    {
      prescaler--;
    }

  /* Check for overflow as well. */

  if (prescaler > 0xffffffff)
    {
      prescaler = 0xffffffff;
    }

  /* Enable timer clock */

  tmr_clk_enable(priv->tmrid);

  /* Set it to Timer Mode */

  lpc43_putreg(0, priv->base + LPC43_TMR_CTCR_OFFSET);

  /* Disable the timer */

  lpc43_putreg(0, priv->base + LPC43_TMR_TCR_OFFSET);

  /* Setup prescaler */

  lpc43_putreg(prescaler, priv->base + LPC43_TMR_PR_OFFSET);

  return prescaler;

}

/****************************************************************************
 * Name: lpc43_enableint
 *
 * Description:
 *   Enable timer interrupts
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static void lpc43_enableint(FAR struct timer_lowerhalf_s *lower, int source)
{
  uint32_t regval;

  FAR struct lpc43_lowerhalf_s *priv = (FAR struct lpc43_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL);

  tmrinfo("Entry");

  /* Set MR0 with the timeout value */

  lpc43_putreg(priv->timeout, priv->base + LPC43_TMR_MR0_OFFSET);

  lpc43_putreg(0, priv->base + LPC43_TMR_CCR_OFFSET); /* do not use capture */

  //if (priv->handler)
    {
      /* Enable Match on MR0 generate interrupt and auto-restart */

      regval = lpc43_getreg(priv->base + LPC43_TMR_MCR_OFFSET);
      regval |= 3;
      lpc43_putreg(regval, priv->base + LPC43_TMR_MCR_OFFSET);
    }

  lpc43_tmr_enable(priv);
}

/****************************************************************************
 * Name: lpc43_disableint
 *
 * Description:
 *   Disable timer interrupts
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static void lpc43_disableint(FAR struct timer_lowerhalf_s *lower, int source)
{
  FAR struct lpc43_lowerhalf_s *priv = (FAR struct lpc43_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL);

  tmrinfo("Entry");

  /* Disable interrupt */

  lpc43_putreg(0, priv->base + LPC43_TMR_MCR_OFFSET);
}

/****************************************************************************
 * Name: lpc43_ackint
 *
 * Description:
 *   Acknoledge interrupt
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static void lpc43_ackint(FAR struct timer_lowerhalf_s *lower, int source)
{
  FAR struct lpc43_lowerhalf_s *priv = (FAR struct lpc43_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL);

  tmrinfo("Entry");

  /* Cleared the interrupts */

  lpc43_putreg(0x0f, priv->base + LPC43_TMR_IR_OFFSET);
}

/****************************************************************************
 * Name: lpc43_checkint
 *
 * Description:
 *   Acknoledge interrupt
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc43_checkint(FAR struct timer_lowerhalf_s *lower, int source)
{
  tmrinfo("Entry");

  return 0;
}

/****************************************************************************
 * Name: lpc43_setisr
 *
 * Description:
 *   Acknoledge interrupt
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int      lpc43_setisr(FAR struct timer_lowerhalf_s *lower,
                            int (*handler)(int irq, void *context),
                            int source)
{
  tmrinfo("Entry");

  return 0;
}

/****************************************************************************
 * Name: lpc43_tmr_init
 *
 * Description:
 *   Initialize and return a timer reference
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

FAR struct lpc43_lowerhalf_s *lpc43_tmr_init(int timer)
{
  struct timer_lowerhalf_s *dev = NULL;

  tmrinfo("Entry");

  /* Get structure and enable power */

  switch (timer)
    {
#ifdef CONFIG_LPC43_TMR0
      case 0:
        dev = (struct timer_lowerhalf_s *)&g_tmrdevs[0];
        break;
#endif
#ifdef CONFIG_LPC43_TMR1
      case 1:
        dev = (struct timer_lowerhalf_s *)&g_tmrdevs[1];
        break;
#endif
#ifdef CONFIG_LPC43_TMR2
      case 2:
        dev = (struct timer_lowerhalf_s *)&g_tmrdevs[2];
        break;
#endif
#ifdef CONFIG_LPC43_TMR3
      case 3:
        dev = (struct timer_lowerhalf_s *)&g_tmrdevs[3];
        break;
#endif
      default:
        return NULL;
    }

  /* Is device already allocated */

  /*if (((struct timer_lowerhalf_s *)dev)->mode != LPC43_TMR_MODE_UNUSED)
    {
      return NULL;
    }*/

  return dev;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_tmrinitialize
 *
 * Description:
 *   Initialize the timer.  The timer is initialized and
 *   registers as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the timer.  This should be of the form
 *     /dev/tmr0
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void lpc43_tmrinitialize(FAR const char *devpath, int irq)
{
  int ret;
  FAR struct lpc43_lowerhalf_s *priv = &g_tmrdevs[irq-LPC43M4_IRQ_TIMER0];

  tmrinfo("Entry: devpath=%s\n", devpath);
  DEBUGASSERT((irq >= LPC43M4_IRQ_TIMER0) && (irq <= LPC43M4_IRQ_TIMER3));

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  switch (irq)
    {
#if defined(CONFIG_LPC43_TMR0)
    case LPC43M4_IRQ_TIMER0:
      priv->base = LPC43_TIMER0_BASE;
      priv->tmrid = 0;
      tmrinfo("Using: Timer 0");
      break;
#endif

#if defined(CONFIG_LPC43_TMR1)
    case LPC43M4_IRQ_TIMER1:
      priv->base = LPC43_TIMER1_BASE;
      priv->tmrid = 1;
      tmrinfo("Using: Timer 1");
      break;
#endif

#if defined(CONFIG_LPC43_TMR2)
    case LPC43M4_IRQ_TIMER2:
      priv->base = LPC43_TIMER2_BASE;
      priv->tmrid = 2;
      tmrinfo("Using: Timer 2");
      break;
#endif

#if defined(CONFIG_LPC43_TMR3)
    case LPC43M4_IRQ_TIMER3:
      priv->base = LPC43_TIMER3_BASE;
      priv->tmrid = 3;
      tmrinfo("Using: Timer 3");
      break;
#endif

    default:
      ASSERT(0);
    }

  priv->ops = &g_tmrops;

  (void)irq_attach(irq, lpc43_interrupt, NULL);

  /* Enable NVIC interrupt. */

  up_enable_irq(irq);

  /* Register the timer driver as /dev/timerX */

  ret = timer_register(devpath, (FAR struct timer_lowerhalf_s *)priv);

  tmrinfo("timer_register returned %d\n", ret);

}

#endif /* CONFIG_TIMER && CONFIG_LPC43_TMRx */
