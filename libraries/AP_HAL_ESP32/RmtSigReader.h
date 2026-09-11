/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "AP_HAL_ESP32.h"
#include "driver/rmt_rx.h"
#include "atomic"

class ESP32::RmtSigReader
{
public:
    static constexpr uint32_t frequency = 1000000;
    static constexpr size_t max_pulses = 128;
    static constexpr uint32_t idle_threshold = 3000;

    void init();
    bool read(uint32_t &width_high, uint32_t &width_low);

private:
    struct RxBuffer {
        rmt_symbol_word_t symbols[max_pulses];
    };

    static bool on_receive_done(
        rmt_channel_handle_t channel,
        const rmt_rx_done_event_data_t *edata,
        void *user_data);

    bool add_item(uint32_t duration, bool level);
    void process_symbol(const rmt_symbol_word_t &symbol);

    rmt_channel_handle_t channel = nullptr;
    rmt_receive_config_t receive_config{};

    RxBuffer rx_buffers[2];

    std::atomic<uint8_t> active_buffer{0};
    std::atomic<int8_t> ready_buffer{-1};
    std::atomic<int8_t> processing_buffer{-1};

    size_t current_symbol = 0;
    bool processing = false;

    uint32_t last_high = 0;
    uint32_t ready_high = 0;
    uint32_t ready_low = 0;
    bool pulse_ready = false;
};
