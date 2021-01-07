/*
 * Copyright (c) 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "nrf_802154_sl_coex.h"
#include "rsch/nrf_802154_rsch.h"
#include "rsch/coex/nrf_802154_wifi_coex.h"

#ifdef __cplusplus
extern "C" {
#endif

nrf_802154_wifi_coex_interface_type_id_t nrf_802154_wifi_coex_interface_type_id_get(void)
{
    return NRF_802154_WIFI_COEX_IF_NONE;
}

void nrf_802154_wifi_coex_cfg_3wire_get(nrf_802154_wifi_coex_3wire_if_config_t * p_cfg)
{
    (void)p_cfg;

    return;
}

void nrf_802154_wifi_coex_cfg_3wire_set(const nrf_802154_wifi_coex_3wire_if_config_t * p_cfg)
{
    (void)p_cfg;

    return;
}

nrf_802154_wifi_coex_ret_t nrf_802154_wifi_coex_init(void)
{
    nrf_802154_wifi_coex_granted(WIFI_COEX_REQUEST_STATE_NO_REQUEST);

    return NRF_802154_WIFI_COEX_RET_DEFAULT_ERR;
}

void nrf_802154_wifi_coex_uninit(void)
{
    // Intentionally empty
}

bool nrf_802154_wifi_coex_is_enabled(void)
{
    return false;
}

void nrf_802154_wifi_coex_prio_request(rsch_prio_t priority)
{
    (void)priority;
    // Intentionally empty
}

#ifdef __cplusplus
}
#endif
