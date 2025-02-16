#ifndef ENCODER_H
    #define ENCODER_H

    #include "hardware/pio.h"
    #include "optical_encoder.pio.h"

    // Constants
    const float TWO_PI = 3.14 * 2;
    #define MAX_COUNT_RATE 50

    // Struct for handling encoder
    typedef struct EncoderCounter {
        uint pin;
        PIO pio;
        uint sm;
        uint offset;
        int32_t counts;
        int32_t delta;
        bool direction;
    } EncoderCounter;

    // Function to configure both struct EncoderCounter and its corresponding PIO State machine
    void configure_encoder_counter(EncoderCounter* encoder_counter, uint pin) {
        encoder_counter->pin = pin;

        bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&optical_encoder_program, &encoder_counter->pio,
        &encoder_counter->sm, &encoder_counter->offset, encoder_counter->pin, 1, true);
        hard_assert(success);

        // Initialize the PIO state machine
        encoder_init(encoder_counter->pio, encoder_counter->sm, encoder_counter->offset, encoder_counter->pin, MAX_COUNT_RATE);
        encoder_counter->counts = 0;
        encoder_counter->direction = false;
    }

    // Function to get counts using struct EncoderCounter as input
    void update_encoder_values(EncoderCounter* encoder_counter) {
        int32_t last_count = encoder_counter->counts;
        encoder_get_count(encoder_counter->pio, encoder_counter->sm, &encoder_counter->counts);
        encoder_counter->delta = encoder_counter->counts - last_count;
    }

    int32_t get_encoder_counts(EncoderCounter encoder_counter) {
        return encoder_counter.counts;
    }

    int32_t get_encoder_delta(EncoderCounter encoder_counter) {
        return encoder_counter.delta;
    }

    // Function to provide motor direction to encoder SM
    void set_encoder_motor_direction(EncoderCounter* encoder_counter, bool direction) {
        if (encoder_counter->direction == direction) return;
        encoder_counter->direction = direction;
        pio_sm_put_blocking(encoder_counter->pio, encoder_counter->sm, encoder_counter->direction);
    }

#endif