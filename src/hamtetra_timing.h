#ifndef HAMTETRA_TIMING_H
#define HAMTETRA_TIMING_H

#include <stdint.h>

// Total slots in one hyperframe
#define TIMING_SLOTS 4320
// Number of previous transmit timestamps to store for "echo" rejection
#define TIMING_TX_TIMES 4

struct slotter_state;

struct timing_state
{
	struct slotter_state *slotter;

	// Parameter: length of a slot
	uint64_t slot_time;
	// Length of a symbol
	int64_t sym_time;
	// Calibration added to received timestamps
	int64_t cal_time;

	// Next transmission slot
	uint64_t tx_time;
	unsigned tx_slot; // Combined slot number, counting from 0 to 4319

	unsigned char prev_dmo; // Flag: a DMO burst was transmitted in the previous slot

	unsigned char use_interslot_bits; // Configuration flag
	unsigned char use_calibration;	  // Configuration flag

	/* A few previous transmit timestamps, used to reject
	 * own transmissions being received */
	unsigned char tx_n;
	uint64_t tx_times[TIMING_TX_TIMES];
};

struct timing_slot
{
	uint64_t time; // Timestamp

	/* For RX: Time difference from expected timestamp
	 * For TX: Not used for now */
	int64_t diff;

	unsigned char tn; // Timeslot Number (1 to 4)
	unsigned char fn; // TDMA Frame Number (1 to 18)
	unsigned char mn; // TDMA Multiframe Number (1 to 60)
	unsigned char hn; // TDMA Hyperframe Number?
	int changed;	  // timing slot modified during the process?
};

/* Allocate and initialize a timing state.
 * Parameters TODO. */
struct timing_state *timing_init();

/* Process a received burst.
 * bits is an array of values 0 and 1.
 * len is the number of array members.
 * ts is a timestamp of the received burst in nanoseconds.
 * Return value is 0 on success. */
int timing_rx_burst(struct timing_state *s, const uint8_t *bits, int len, uint64_t ts);

/* Produce a burst to be transmitted in near future.
 *
 * The function is regularly called to check whether
 * it's the time to produce a transmit burst.
 *
 * Return value is the number of bits in the burst,
 * -1 if there is no burst to transmit at the moment.
 *
 * Timestamp of the burst is returned in *ts.
 * A "deadline" is given in *ts:
 * if a burst should be transmitted before the time given in *ts,
 * it should be returned in this call.
 */
int timing_tx_burst(struct timing_state *s, uint8_t *bits, int maxlen, uint64_t *ts);

/* Resynchronize time counters. */
int timing_resync(struct timing_state *s, struct timing_slot *slot);

#endif
