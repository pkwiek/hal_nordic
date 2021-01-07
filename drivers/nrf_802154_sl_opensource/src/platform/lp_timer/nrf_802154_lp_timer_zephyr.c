/*
 * Copyright (c) 2017 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file contains standalone implementation of the nRF 802.15.4 timer abstraction.
 *
 * This implementation is built on top of the RTC peripheral.
 *
 */

#include "platform/lp_timer/nrf_802154_lp_timer.h"

#include <kernel.h>
#include <assert.h>

#include "platform/clock/nrf_802154_clock.h"
#include "nrf_802154_sl_config.h"
#include "nrf_802154_sl_utils.h"


// Enum holding all used compare channels.
typedef enum {LP_TIMER_CHANNEL, SYNC_CHANNEL, CHANNEL_CNT} compare_channel_t;

static volatile uint32_t m_lp_timer_irq_enabled;   ///< Information that RTC interrupt was enabled while entering critical section.
static bool m_timers_initialized;
static struct k_timer m_lp_timers[CHANNEL_CNT];
static volatile uint32_t m_shall_fire_immediately; ///< Information if timer should fire immediately.


/**
 * @brief Start one-shot timer that expires at specified time on desired channel.
 *
 * Start one-shot timer that will expire @p dt microseconds after @p t0 time on channel @p channel.
 *
 * @param[in]  channel     Compare channel on which timer will be started.
 * @param[in]  t0          Number of microseconds representing timer start time.
 * @param[in]  dt          Time of timer expiration as time elapsed from @p t0 [us].
 */
static void timer_start_at(compare_channel_t channel,
                           uint32_t          t0,
                           uint32_t          dt)
{
    uint32_t now = nrf_802154_lp_timer_time_get();
    uint32_t target_time = t0 + dt;

    if ((now + nrf_802154_lp_timer_granularity_get()) >= target_time)
    {
        k_timer_start(&m_lp_timers[channel], K_NO_WAIT, K_NO_WAIT);
    }
    else
    {
        k_timer_start(&m_lp_timers[channel],
                      K_USEC(k_ticks_to_us_floor32(k_uptime_ticks()) - target_time),
                      K_NO_WAIT);
    }
}

static void lp_timer_handler(struct k_timer *timer)
{
    if (m_lp_timer_irq_enabled)
    {
        m_shall_fire_immediately = 1;
    }
    else
    {
        nrf_802154_lp_timer_fired();
    }
}

static void sync_timer_handler(struct k_timer *timer)
{
    nrf_802154_lp_timer_synchronized();
}

void nrf_802154_lp_timer_init(void)
{
    m_lp_timer_irq_enabled = 0;
    m_shall_fire_immediately = 0;

    if (!m_timers_initialized)
    {
        m_timers_initialized = true;
        k_timer_init(&m_lp_timers[LP_TIMER_CHANNEL], lp_timer_handler, NULL);
        k_timer_init(&m_lp_timers[SYNC_CHANNEL], sync_timer_handler, NULL);
    }
}

void nrf_802154_lp_timer_deinit(void)
{
    // Intentionally empty - k_timers cannot be deinitialized
}

void nrf_802154_lp_timer_critical_section_enter(void)
{
    m_lp_timer_irq_enabled = 1;
}

void nrf_802154_lp_timer_critical_section_exit(void)
{
    m_lp_timer_irq_enabled = 0;

    if (m_shall_fire_immediately)
    {
        m_shall_fire_immediately = 0;
        nrf_802154_lp_timer_fired();
    }
}

uint32_t nrf_802154_lp_timer_time_get(void)
{
    return k_ticks_to_us_floor32(k_uptime_ticks());
}

uint32_t nrf_802154_lp_timer_granularity_get(void)
{
    return k_ticks_to_us_floor32(1);
}

void nrf_802154_lp_timer_start(uint32_t t0, uint32_t dt)
{
    timer_start_at(LP_TIMER_CHANNEL, t0, dt);
}

bool nrf_802154_lp_timer_is_running(void)
{
    if ((k_timer_status_get(&m_lp_timers[LP_TIMER_CHANNEL]) == 0) &&
        (k_timer_remaining_get(&m_lp_timers[LP_TIMER_CHANNEL]) != 0))
    {
        return true;
    }

    return false;
}

void nrf_802154_lp_timer_stop(void)
{
    k_timer_stop(&m_lp_timers[LP_TIMER_CHANNEL]);
}

void nrf_802154_lp_timer_sync_start_now(void)
{
    k_timer_start(&m_lp_timers[SYNC_CHANNEL], K_NO_WAIT, K_NO_WAIT);
}

void nrf_802154_lp_timer_sync_start_at(uint32_t t0, uint32_t dt)
{
    timer_start_at(SYNC_CHANNEL, t0, dt);
}

void nrf_802154_lp_timer_sync_stop(void)
{
    k_timer_stop(&m_lp_timers[SYNC_CHANNEL]);
}

uint32_t nrf_802154_lp_timer_sync_event_get(void)
{
    __ASSERT(false, "k_timers do not have sync events");
    return 0;
}

uint32_t nrf_802154_lp_timer_sync_time_get(void)
{
    return nrf_802154_lp_timer_time_get();
}

#ifndef UNITY_ON_TARGET
__WEAK void nrf_802154_lp_timer_synchronized(void)
{
    // Intentionally empty
}

#endif // UNITY_ON_TARGET
