#include <AP_HAL/HAL.h>
#include "RmtSigReader.h"

#ifdef HAL_ESP32_RCIN

using namespace ESP32;

void RmtSigReader::init()
{
    // RMT RX channel configuration
    rmt_rx_channel_config_t channel_config = {};
    channel_config.gpio_num = HAL_ESP32_RCIN;
    channel_config.clk_src = RMT_CLK_SRC_DEFAULT;
    channel_config.resolution_hz = frequency;
    channel_config.mem_block_symbols = max_pulses;

    esp_err_t err = rmt_new_rx_channel(&channel_config, &channel);
    if (err != ESP_OK) {
        channel = nullptr;
        return;
    }

    // RX event callback.
    rmt_rx_event_callbacks_t callbacks = {};
    callbacks.on_recv_done = &RmtSigReader::on_receive_done;

    err = rmt_rx_register_event_callbacks(channel, &callbacks, this);
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

    // At 1 MHz:
    //   1 tick = 1 us
    //
    // Old filter_ticks_thresh = 8  -> 8 us
    // Old idle_threshold       = 3000 -> 3 ms
    receive_config.signal_range_min_ns = 8000;
    receive_config.signal_range_max_ns =
        idle_threshold * 1000UL;

    active_buffer.store(0, std::memory_order_relaxed);
    ready_buffer.store(-1, std::memory_order_relaxed);
    processing_buffer.store(-1, std::memory_order_relaxed);

    current_symbol = 0;
    processing = false;

    last_high = 0;
    ready_high = 0;
    ready_low = 0;
    pulse_ready = false;

    // Start the first reception.
    err = rmt_receive(
        channel,
        rx_buffers[0].symbols,
        sizeof(rx_buffers[0].symbols),
        &receive_config);

    if (err != ESP_OK) {
        rmt_disable(channel);
        rmt_del_channel(channel);
        channel = nullptr;
    }
}


/*
 * This callback executes in ISR context.
 *
 * Keep this function deliberately small:
 *   - identify the completed buffer
 *   - publish it to read()
 *   - select another buffer
 *   - restart RMT reception
 *
 * No pulse decoding is performed here.
 */
bool RmtSigReader::on_receive_done(
    rmt_channel_handle_t channel,
    const rmt_rx_done_event_data_t *edata,
    void *user_data)
{
    RmtSigReader *self = static_cast<RmtSigReader *>(user_data);

    const uint8_t completed =
        self->active_buffer.load(std::memory_order_relaxed);

    const uint8_t next = completed ^ 1U;

    /*
     * If there is currently no buffer waiting to be processed,
     * publish the completed buffer.
     *
     * Otherwise read() has not consumed the previous frame yet.
     * In that case we drop this newly completed frame and reuse
     * the same buffer for the next reception.
     *
     * This is intentional: dropping a frame is preferable to
     * overwriting a buffer currently being processed.
     */
    int8_t expected = -1;

    if (self->ready_buffer.compare_exchange_strong(
            expected,
            static_cast<int8_t>(completed),
            std::memory_order_release,
            std::memory_order_relaxed)) {

        /*
         * The other buffer can only be used if read() isn't
         * currently processing it.
         */
        const int8_t processing =
            self->processing_buffer.load(std::memory_order_acquire);

        if (processing == static_cast<int8_t>(next)) {
            /*
             * No free buffer.
             *
             * We cannot safely start RX into 'next', so instead
             * reuse the just-completed buffer and drop this frame.
             */
            self->ready_buffer.store(-1, std::memory_order_release);
            self->active_buffer.store(completed, std::memory_order_relaxed);

            rmt_receive(
                channel,
                self->rx_buffers[completed].symbols,
                sizeof(self->rx_buffers[completed].symbols),
                &self->receive_config);

            return false;
        }

        /*
         * The other buffer is free.
         */
        self->active_buffer.store(next, std::memory_order_relaxed);

        rmt_receive(
            channel,
            self->rx_buffers[next].symbols,
            sizeof(self->rx_buffers[next].symbols),
            &self->receive_config);

    } else {
        /*
         * A buffer is already waiting to be processed.
         *
         * Reuse the just-completed buffer. This drops the new
         * frame but keeps the system safe.
         */
        self->active_buffer.store(completed, std::memory_order_relaxed);

        rmt_receive(
            channel,
            self->rx_buffers[completed].symbols,
            sizeof(self->rx_buffers[completed].symbols),
            &self->receive_config);
    }

    return false;
}


bool RmtSigReader::add_item(uint32_t duration, bool level)
{
    bool has_more = true;
    if (duration == 0) {
        has_more = false;
        duration = idle_threshold;
    }
    if (level) {
        if (last_high == 0) {
            last_high = duration;
        }
    } else {
        if (last_high != 0) {
            ready_high = last_high;
            ready_low = duration;
            pulse_ready = true;
            last_high = 0;
        }
    }
    return has_more;
}

bool RmtSigReader::read(uint32_t &width_high, uint32_t &width_low)
{
    if (channel == nullptr) {
        return false;
    }

    /*
     * Acquire a completed RMT buffer if we aren't currently
     * processing one.
     */
    if (!processing) {
        int8_t buffer = ready_buffer.exchange(
            -1,
            std::memory_order_acq_rel);

        if (buffer < 0) {
            return false;
        }

        processing_buffer.store(
            buffer,
            std::memory_order_release);

        current_symbol = 0;
        processing = true;

        last_high = 0;
        pulse_ready = false;
    }

    const int8_t buffer =
        processing_buffer.load(std::memory_order_acquire);

    if (buffer < 0) {
        processing = false;
        return false;
    }

    /*
     * Process the RMT symbols in the completed buffer.
     *
     * Each rmt_symbol_word_t contains two level/duration pairs,
     * equivalent to the old rmt_item32_t.
     */
    while (current_symbol < max_pulses) {

        const rmt_symbol_word_t &symbol =
            rx_buffers[buffer].symbols[current_symbol++];

        if (!add_item(symbol.duration0, symbol.level0)) {
            processing = false;
            processing_buffer.store(
                -1,
                std::memory_order_release);
            break;
        }

        if (!add_item(symbol.duration1, symbol.level1)) {
            processing = false;
            processing_buffer.store(
                -1,
                std::memory_order_release);
            break;
        }

        if (pulse_ready) {
            width_high = ready_high;
            width_low = ready_low;
            pulse_ready = false;

            return true;
        }
    }

    /*
     * End of the RMT buffer.
     */
    if (current_symbol >= max_pulses) {
        processing = false;
        processing_buffer.store(
            -1,
            std::memory_order_release);
    }

    return false;
}

#endif