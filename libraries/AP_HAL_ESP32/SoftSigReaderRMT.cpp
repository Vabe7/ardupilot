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
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include "SoftSigReaderRMT.h"

using namespace ESP32;

SoftSigReaderRMT *SoftSigReaderRMT::_instance = nullptr;

extern const AP_HAL::HAL& hal;


void SoftSigReaderRMT::init()
{
#ifndef HAL_ESP32_RMT_RX_PIN_NUMBER
#error HAL_ESP32_RMT_RX_PIN_NUMBER undefined in libraries/AP_HAL_ESP32/boards/esp32... .h
#endif

    _instance = this;

    /*
     * The old driver used:
     *
     *     RMT_CLK_DIV = 10
     *
     * with the 80 MHz APB clock.
     *
     * Therefore:
     *
     *     80 MHz / 10 = 8 MHz
     *
     * and one RMT tick is 125 ns.
     */
    rmt_rx_channel_config_t channel_config = {};

    channel_config.gpio_num =
        static_cast<gpio_num_t>(HAL_ESP32_RMT_RX_PIN_NUMBER);

    channel_config.clk_src = RMT_CLK_SRC_DEFAULT;
    channel_config.resolution_hz = rmt_frequency;

    /*
     * One hardware RMT memory block was used by the old driver.
     *
     * We only need 16 symbols for the data actually consumed by
     * this implementation.
     */
    channel_config.mem_block_symbols = max_symbols;

    esp_err_t err =
        rmt_new_rx_channel(&channel_config, &channel);

    if (err != ESP_OK) {
        channel = nullptr;
        return;
    }

    rmt_rx_event_callbacks_t callbacks = {};

    callbacks.on_recv_done =
        &SoftSigReaderRMT::on_receive_done;

    err = rmt_rx_register_event_callbacks(
        channel,
        &callbacks,
        this);

    if (err != ESP_OK) {
        rmt_del_channel(channel);
        channel = nullptr;
        return;
    }

    err = rmt_enable(channel);

    if (err != ESP_OK) {
        rmt_del_channel(channel);
        channel = nullptr;
        return;
    }

    /*
     * Equivalent to the old:
     *
     *     filter_ticks_thresh = 100
     *
     * At 8 MHz:
     *
     *     100 ticks = 12.5 us
     *
     * The new API expresses this in nanoseconds.
     */
    receive_config.signal_range_min_ns = 12500;

    /*
     * Equivalent to:
     *
     *     idle_threshold =
     *         PPM_IMEOUT_US * RMT_TICK_US
     *
     * with 3500 us and 8 ticks/us:
     *
     *     3500 us = 3,500,000 ns
     */
    receive_config.signal_range_max_ns =
        ppm_timeout_us * 1000UL;

    active_buffer.store(0, std::memory_order_relaxed);
    ready_buffer.store(-1, std::memory_order_relaxed);
    processing_buffer.store(-1, std::memory_order_relaxed);

    channel_pointer = -1;
    channel_count = 0;
    started = false;
}


/*
 * Start an RMT receive transaction.
 *
 * This is intentionally kept separate from init(), because the old
 * implementation deliberately delayed starting RMT until read() was
 * first called.
 */
bool SoftSigReaderRMT::start_receive(uint8_t buffer)
{
    if (channel == nullptr) {
        return false;
    }

    const esp_err_t err = rmt_receive(
        channel,
        rx_buffers[buffer].symbols,
        sizeof(rx_buffers[buffer].symbols),
        &receive_config);

    return err == ESP_OK;
}


/*
 * RMT receive callback.
 *
 * This function executes in ISR context.
 *
 * IMPORTANT:
 * Do not decode PPM data here.
 *
 * Its only job is to:
 *   1. publish the completed buffer
 *   2. select another buffer
 *   3. restart RMT reception
 *
 * If the reader has not consumed the previous buffer and no buffer
 * is available, the newly completed frame is dropped rather than
 * overwriting memory currently being processed.
 */
bool SoftSigReaderRMT::on_receive_done(
    rmt_channel_handle_t channel,
    const rmt_rx_done_event_data_t *edata,
    void *user_data)
{
    SoftSigReaderRMT *self =
        static_cast<SoftSigReaderRMT *>(user_data);

    const uint8_t completed =
        self->active_buffer.load(std::memory_order_relaxed);

    const uint8_t next = completed ^ 1U;

    /*
     * Try to publish the completed buffer.
     *
     * If ready_buffer is already occupied, the previous frame
     * has not yet been consumed.
     */
    int8_t expected = -1;

    if (!self->ready_buffer.compare_exchange_strong(
            expected,
            static_cast<int8_t>(completed),
            std::memory_order_release,
            std::memory_order_relaxed)) {

        /*
         * The other buffer may be being processed, but the buffer
         * that just completed is safe to reuse.
         *
         * Drop this frame and immediately start a new reception
         * using the same buffer.
         */
        self->active_buffer.store(
            completed,
            std::memory_order_relaxed);

        rmt_receive(
            channel,
            self->rx_buffers[completed].symbols,
            sizeof(self->rx_buffers[completed].symbols),
            &self->receive_config);

        return false;
    }

    /*
     * The completed buffer is now owned by read().
     *
     * Normally the other buffer is free and can become the active
     * receive buffer.
     */
    const int8_t processing =
        self->processing_buffer.load(
            std::memory_order_acquire);

    if (processing == static_cast<int8_t>(next)) {

        /*
         * read() is currently using the other buffer.
         *
         * There is no free buffer, so we cannot safely receive into
         * it. Drop the just-completed frame and reuse that same
         * buffer.
         */
        self->ready_buffer.store(
            -1,
            std::memory_order_release);

        self->active_buffer.store(
            completed,
            std::memory_order_relaxed);

        rmt_receive(
            channel,
            self->rx_buffers[completed].symbols,
            sizeof(self->rx_buffers[completed].symbols),
            &self->receive_config);

        return false;
    }

    self->active_buffer.store(
        next,
        std::memory_order_relaxed);

    rmt_receive(
        channel,
        self->rx_buffers[next].symbols,
        sizeof(self->rx_buffers[next].symbols),
        &self->receive_config);

    return false;
}


bool SoftSigReaderRMT::read(
    uint32_t &widths0,
    uint32_t &widths1)
{
    if (channel == nullptr) {
        return false;
    }

    /*
     * Preserve the delayed-start behaviour of the old
     * implementation.
     */
    if (!started) {

        const uint8_t buffer =
            active_buffer.load(std::memory_order_relaxed);

        if (!start_receive(buffer)) {
            return false;
        }

        started = true;
    }


    /*
     * If we are not currently returning channels from a frame,
     * acquire the next completed RMT buffer.
     */
    if (channel_pointer < 0) {

        const int8_t buffer =
            ready_buffer.exchange(
                -1,
                std::memory_order_acq_rel);

        if (buffer < 0) {
            return false;
        }

        processing_buffer.store(
            buffer,
            std::memory_order_release);

        /*
         * We don't get the exact byte count from the new RMT
         * callback in the same way as the old ringbuffer API.
         *
         * Instead, process the symbol array until the first
         * zero-duration symbol.
         */
        channel_count = 0;

        for (size_t i = 0; i < max_symbols; i++) {

            const rmt_symbol_word_t &symbol =
                rx_buffers[buffer].symbols[i];

            /*
             * The RMT receiver terminates the frame at the idle
             * threshold. A zero-duration symbol therefore marks
             * the unused part of our software buffer.
             */
            if (symbol.duration0 == 0 &&
                symbol.duration1 == 0) {
                break;
            }

            /*
             * The old code deliberately discarded the last
             * RMT symbol:
             *
             *     channels = (rx_size / 4) - 1;
             *
             * That symbol represents the final idle portion.
             *
             * We preserve that behaviour by requiring at least
             * one symbol after the current one.
             */
            if (i + 1 >= max_symbols) {
                break;
            }

            channeldata0[channel_count] =
                static_cast<uint16_t>(
                    symbol.duration0 / 8);

            channeldata1[channel_count] =
                static_cast<uint16_t>(
                    symbol.duration1 / 8);

            channel_count++;

            if (channel_count >=
                static_cast<int>(max_symbols - 1)) {
                break;
            }
        }

        processing_buffer.store(
            -1,
            std::memory_order_release);

        /*
         * The first channel is now ready to be returned.
         */
        if (channel_count > 0) {
            channel_pointer = 0;
        } else {
            channel_pointer = -1;
            return false;
        }
    }


    /*
     * Return one PPM channel per read() call.
     */
    if (channel_pointer >= 0 &&
        channel_pointer < channel_count) {

        widths0 =
            channeldata0[channel_pointer];

        widths1 =
            channeldata1[channel_pointer];

        channel_pointer++;

        /*
         * Preserve the original behaviour:
         *
         * after the 8th channel, insert the wide PPM idle pulse.
         */
        if (channel_pointer == 9) {
            widths0 = 3000;
            widths1 = 1000;
        }

        /*
         * After returning the synthetic channel, finish this
         * frame on the next call.
         */
        if (channel_pointer > 9) {
            channel_pointer = -1;
            channel_count = 0;
            return false;
        }

        return true;
    }


    channel_pointer = -1;
    channel_count = 0;

    return false;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32