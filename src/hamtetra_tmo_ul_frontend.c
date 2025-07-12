/*
 * This module acts as a smart frontend for the TMO uplink receiver.
 * Its purpose is to solve two problems when using a sensitive SDR in FDD mode
 * with low antenna isolation:
 * 1. It prevents the underlying modem from being spammed by low-level noise
 *    and TX leakage by using an adaptive power squelch.
 * 2. It prevents the underlying modem from getting stuck in a desensitized state
 *    after receiving a strong burst, by destroying and re-creating the modem
 *    instance during quiet periods.
 */

#include "hamtetra_tmo_ul_frontend.h"
#include "modem/burst_dpsk_receiver.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --- Squelch Configuration ---
// A signal must be this many times stronger than the noise floor to open the squelch.
#define SQUELCH_RATIO 5.0f
// The adaptation speed for the noise floor estimate (IIR filter alpha).
#define NOISE_FILTER_ALPHA 0.01f

// State for the TMO Uplink Frontend
struct hamtetra_tmo_ul_frontend
{
    // The underlying "black box" modem being managed
    const struct receiver_code *real_receiver;
    void *real_receiver_arg;

    // Saved state required to re-initialize the modem
    struct burst_dpsk_receiver_conf conf_copy;
    const struct rx_output_code *output_cb_copy;
    void *output_cb_arg_copy;

    // Squelch and state-management variables
    float noise_level; // Running average of the noise floor power
    bool needs_reset;  // Flag indicating the modem has processed a strong signal
};

// Resets the underlying modem to its initial, sensitive state.
static void reset_modem(struct hamtetra_tmo_ul_frontend *self)
{
    fprintf(stderr, ">>> [UL Frontend] Resetting underlying modem state... <<<\n");

    // Destroy the old modem instance, discarding its potentially "deaf" state.
    self->real_receiver->destroy(self->real_receiver_arg);

    // Create a new instance using the saved initial configuration.
    self->real_receiver_arg = self->real_receiver->init(&self->conf_copy);
    if (!self->real_receiver_arg)
    {
        fprintf(stderr, "[UL Frontend ERROR] Failed to re-initialize modem!\n");
        return;
    }

    // Re-apply the output callbacks to the new instance.
    if (self->output_cb_copy)
    {
        self->real_receiver->set_callbacks(self->real_receiver_arg, self->output_cb_copy, self->output_cb_arg_copy);
    }
    self->needs_reset = false;
}

// --- SUO Receiver Interface Implementation ---

static int frontend_execute(void *arg, const sample_t *samples, size_t nsamp, timestamp_t timestamp)
{
    struct hamtetra_tmo_ul_frontend *self = arg;
    float current_power = 0.0f;

    // Calculate the average power (I^2 + Q^2) of the incoming sample block.
    if (nsamp > 0)
    {
        for (size_t i = 0; i < nsamp; i++)
        {
            current_power += crealf(samples[i]) * crealf(samples[i]) + cimagf(samples[i]) * cimagf(samples[i]);
        }
        current_power /= nsamp;
    }

    // Open the squelch only if the block's power exceeds the learned noise floor by a set ratio.
    if (current_power > self->noise_level * SQUELCH_RATIO)
    {
        // Squelch Open: Pass samples to the modem and flag its state as dirty.
        self->real_receiver->execute(self->real_receiver_arg, samples, nsamp, timestamp);
        self->needs_reset = true;
    }
    else
    {
        // Squelch Closed: Update the noise floor and, if necessary, reset the modem.
        self->noise_level += (current_power - self->noise_level) * NOISE_FILTER_ALPHA;
        if (self->needs_reset)
        {
            reset_modem(self);
        }
    }
    return 0;
}

static int frontend_set_callbacks(void *arg, const struct rx_output_code *output, void *output_arg)
{
    struct hamtetra_tmo_ul_frontend *self = arg;
    // Store callbacks to re-apply them after a modem reset.
    self->output_cb_copy = output;
    self->output_cb_arg_copy = output_arg;
    return self->real_receiver->set_callbacks(self->real_receiver_arg, output, output_arg);
}

static void *frontend_init(const void *conf_v)
{
    struct hamtetra_tmo_ul_frontend *self = calloc(1, sizeof(*self));
    if (!self)
    {
        perror("[UL Frontend ERROR] Failed to allocate memory");
        return NULL;
    }

    fprintf(stderr, ">>> [UL Frontend] Initializing Uplink Frontend with Re-Init Squelch... <<<\n");

    // Initialize all state variables.
    self->noise_level = 0.00001f; // Start with a small, non-zero noise floor.
    self->needs_reset = false;
    self->output_cb_copy = NULL;
    self->output_cb_arg_copy = NULL;

    // Store a copy of the modem's configuration for later re-initialization.
    memcpy(&self->conf_copy, conf_v, sizeof(self->conf_copy));

    self->real_receiver = &burst_dpsk_receiver_code;
    self->real_receiver_arg = self->real_receiver->init(&self->conf_copy);
    if (!self->real_receiver_arg)
    {
        free(self);
        return NULL;
    }
    return self;
}

// Pass-through functions to the underlying modem's interface
static void *frontend_init_conf()
{
    return burst_dpsk_receiver_code.init_conf();
}
static int frontend_set_conf(void *conf, const char *key, const char *val)
{
    return burst_dpsk_receiver_code.set_conf(conf, key, val);
}
static int frontend_destroy(void *arg)
{
    struct hamtetra_tmo_ul_frontend *self = arg;
    if (self)
    {
        self->real_receiver->destroy(self->real_receiver_arg);
        free(self);
    }
    return 0;
}

// The public interface for this entire module.
const struct receiver_code hamtetra_tmo_ul_frontend_code = {
    .name = "hamtetra_tmo_ul_frontend",
    .init = frontend_init,
    .destroy = frontend_destroy,
    .init_conf = frontend_init_conf,
    .set_conf = frontend_set_conf,
    .set_callbacks = frontend_set_callbacks,
    .execute = frontend_execute};
