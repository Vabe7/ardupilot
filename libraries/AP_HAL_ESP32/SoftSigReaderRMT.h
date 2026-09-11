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
 * Code by David "Buzz" Bussenschutt and others
 *
  */
#pragma once

#include <atomic>

#include "AP_HAL_ESP32.h"
#include "driver/rmt_rx.h"

namespace ESP32
{

class SoftSigReaderRMT
{
public:
    static SoftSigReaderRMT *get_instance()
    {
        return _instance;
    }

    void init();
    bool read(uint32_t &widths0, uint32_t &widths1);
private:
    static constexpr uint32_t rmt_frequency = 8000000;
    static constexpr uint32_t rmt_tick_ns = 1000 / 8; // 125 ns

    // Old driver:
    //
    // RMT_CLK_DIV = 10
    // APB = 80 MHz
    // => 8 MHz RMT clock
    // => 0.125 us/tick
    //
    // Old filter_ticks_thresh = 100
    // => 12.5 us
    static constexpr uint32_t filter_ticks = 100;

    // PPM frame timeout.
    static constexpr uint32_t ppm_timeout_us = 3500;

    // Maximum number of RMT symbols we retain for one PPM frame.
    //
    // The old implementation used arrays of 16 uint32_t values and
    // processed rx_size / 4 - 1 symbols, so 16 symbols is sufficient
    // for the original behaviour.
    static constexpr size_t max_symbols = 16;

    struct RxBuffer {
        rmt_symbol_word_t symbols[max_symbols];
    };

    static bool on_receive_done(
        rmt_channel_handle_t channel,
        const rmt_rx_done_event_data_t *edata,
        void *user_data);

    bool start_receive(uint8_t buffer);

    rmt_channel_handle_t channel = nullptr;

    rmt_receive_config_t receive_config{};

    RxBuffer rx_buffers[2];

    std::atomic<uint8_t> active_buffer{0};
    std::atomic<int8_t> ready_buffer{-1};
    std::atomic<int8_t> processing_buffer{-1};

    static SoftSigReaderRMT *_instance;

    uint16_t channeldata0[max_symbols]{};
    uint16_t channeldata1[max_symbols]{};

    int channel_count = 0;
    int channel_pointer = -1;

    bool started = false;
};

} // namespace ESP32