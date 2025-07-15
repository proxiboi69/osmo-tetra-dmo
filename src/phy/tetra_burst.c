/* Implementation of TETRA Physical Layer, i.e. what is _below_
 * CRC, FEC, Interleaving and Scrambling */

/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hamtetra_mac.h"
#include <phy/tetra_burst.h>
#include <phy/tetra_burst_bits.h>
#include <tetra_common.h>

#define DQPSK4_BITS_PER_SYM 2

#define SB_BLK1_OFFSET ((6 + 1 + 40) * DQPSK4_BITS_PER_SYM)
#define SB_BBK_OFFSET ((6 + 1 + 40 + 60 + 19) * DQPSK4_BITS_PER_SYM)
#define SB_BLK2_OFFSET ((6 + 1 + 40 + 60 + 19 + 15) * DQPSK4_BITS_PER_SYM)

#define SB_BLK1_BITS (60 * DQPSK4_BITS_PER_SYM)
#define SB_BBK_BITS (15 * DQPSK4_BITS_PER_SYM)
#define SB_BLK2_BITS (108 * DQPSK4_BITS_PER_SYM)

#define NDB_BLK1_OFFSET ((5 + 1 + 1) * DQPSK4_BITS_PER_SYM)
#define NDB_BBK1_OFFSET ((5 + 1 + 1 + 108) * DQPSK4_BITS_PER_SYM)
#define NDB_BBK2_OFFSET ((5 + 1 + 1 + 108 + 7 + 11) * DQPSK4_BITS_PER_SYM)
#define NDB_BLK2_OFFSET ((5 + 1 + 1 + 108 + 7 + 11 + 8) * DQPSK4_BITS_PER_SYM)

#define NDB_BBK1_BITS (7 * DQPSK4_BITS_PER_SYM)
#define NDB_BBK2_BITS (8 * DQPSK4_BITS_PER_SYM)
#define NDB_BLK_BITS (108 * DQPSK4_BITS_PER_SYM)
#define NDB_BBK_BITS SB_BBK_BITS

#define DMO_SB_BLK1_OFFSET ((6 + 1 + 40) * DQPSK4_BITS_PER_SYM)
#define DMO_SB_BLK2_OFFSET ((6 + 1 + 40 + 60 + 19) * DQPSK4_BITS_PER_SYM)

#define DMO_SB_BLK1_BITS (60 * DQPSK4_BITS_PER_SYM)
#define DMO_SB_BLK2_BITS (108 * DQPSK4_BITS_PER_SYM)

#define DMO_DNB_BLK1_OFFSET ((6 + 1) * DQPSK4_BITS_PER_SYM)
#define DMO_DNB_BLK2_OFFSET ((6 + 1 + 108 + 11) * DQPSK4_BITS_PER_SYM)
#define DMO_DNB_BLK_BITS (108 * DQPSK4_BITS_PER_SYM)

/* 9.4.4.3.1 Frequency Correction Field */
static const uint8_t f_bits[80] = {
	/* f1 .. f8 = 1 */
	1, 1, 1, 1, 1, 1, 1, 1,
	/* f73..f80 = 1*/
	[72] = 1, [73] = 1, [74] = 1, [75] = 1,
	[76] = 1, [77] = 1, [78] = 1, [79] = 1};

/* 9.4.4.3.2 Normal Training Sequence */
static const uint8_t n_bits[22] = {1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0};
static const uint8_t p_bits[22] = {0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0};
static const uint8_t q_bits[22] = {1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1};
static const uint8_t N_bits[33] = {1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t P_bits[33] = {1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0};

/* 9.4.4.3.3 Extended training sequence */
static const uint8_t x_bits[30] = {1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1};
static const uint8_t X_bits[45] = {0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0};

/* 9.4.4.3.4 Synchronization training sequence */
static const uint8_t y_bits[38] = {1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1};

/* 9.4.4.3.5 Tail bits */
static const uint8_t t_bits[4] = {1, 1, 0, 0};
static const uint8_t T_bits[6] = {1, 1, 1, 0, 0, 0};

/* 9.4.4.3.6 Phase adjustment bits */
enum phase_adj_bits
{
	HA,
	HB,
	HC,
	HD,
	HE,
	HF,
	HG,
	HH,
	HI,
	HJ,
	HK,
	HL
};
struct phase_adj_n
{
	uint16_t n1;
	uint16_t n2;
};

/* DMO EN 300 396-2 - 9.4.3.3.3 Normal Training sequence and preamble */
static const uint8_t j_bits[12] = {0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1};
static const uint8_t k_bits[12] = {1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1};
static const uint8_t l_bits[12] = {0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1};

/* DMO EN 300 396-2 - 9.4.3.3.2 Inter-slot frequency correction bits */
static const uint8_t g_bits[40] = {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};

/* Table 8.14 */
static const struct phase_adj_n phase_adj_n[] = {
	[HA] = {.n1 = 8, .n2 = 122},
	[HB] = {.n1 = 123, .n2 = 249},
	[HC] = {.n1 = 8, .n2 = 108},
	[HD] = {.n1 = 109, .n2 = 249},
	[HE] = {.n1 = 112, .n2 = 230},
	[HF] = {.n1 = 1, .n2 = 111},
	[HG] = {.n1 = 3, .n2 = 117},
	[HH] = {.n1 = 118, .n2 = 224},
	[HI] = {.n1 = 3, .n2 = 103},
	[HJ] = {.n1 = 104, .n2 = 224},
	[HK] = {.n1 = 8, .n2 = 126},
	[HL] = {.n1 = 8, .n2 = 126}};

static const int8_t bits2phase[] = {
	[0] = 1,  /* +pi/4 needs to become -pi/4 */
	[1] = -1, /* -pi/4 needs to become +pi/4 */
	[2] = +3, /* +3pi/4 needs to become -3pi/4 */
	[3] = -3, /* -3pi/4 needs to become +3pi/4 */
};

/* offset everything by 3 in order to get positive array index */
#define PHASE(x) ((x) + 3)
struct phase2bits
{
	int8_t phase;
	uint8_t bits[2];
};
static const struct phase2bits phase2bits[] = {
	[PHASE(-3)] = {-3, {1, 1}},
	[PHASE(-1)] = {-1, {0, 1}},
	[PHASE(1)] = {1, {0, 0}},
	[PHASE(3)] = {3, {1, 0}},
};

static int32_t calc_phase_adj(int32_t phase)
{
	int32_t adj_phase = -(phase % 8);

	/* 'wrap around' to get a value in the range between +3 / -3 */
	if (adj_phase > 3)
		adj_phase -= 8;
	else if (adj_phase < -3)
		adj_phase += 8;

	return adj_phase;
}

/* return the cumulative phase shift of all bits (in units of pi/4) */
int32_t sum_up_phase(const uint8_t *bits, unsigned int sym_count)
{
	uint8_t sym_in;
	int32_t sum_phase = 0;
	unsigned int n;

	for (n = 0; n < sym_count; n++)
	{
		/* offset '-1' due to array-index starting at 0 */
		uint32_t bn = 2 * n;
		sym_in = bits[bn];
		sym_in |= bits[bn + 1] << 1;

		sum_phase += bits2phase[sym_in];
	}

	// printf("phase sum over %u symbols: %dpi/4, mod 8 = %dpi/4, wrap = %dpi/4\n",
	// 	   sym_count, sum_phase, sum_phase % 8, calc_phase_adj(sum_phase));
	return sum_phase;
}

/* compute phase adjustment bits according to 'pa' and write them to {out, out+2} */
void put_phase_adj_bits(const uint8_t *bits, enum phase_adj_bits pa, uint8_t *out)
{
	int32_t sum_phase, adj_phase;
	const struct phase_adj_n *pan = &phase_adj_n[pa];
	const struct phase2bits *p2b;

	/* offset '-1' due to array-index starting at 0 */
	sum_phase = sum_up_phase(bits + 2 * (pan->n1 - 1), 1 + pan->n2 - pan->n1);
	adj_phase = calc_phase_adj(sum_phase);

	p2b = &phase2bits[adj_phase];

	*out++ = p2b->bits[0];
	*out++ = p2b->bits[1];
}

/* 9.4.4.2.6 Synchronization continuous downlink burst */
int build_sync_c_d_burst(uint8_t *buf, const uint8_t *sb, const uint8_t *bb, const uint8_t *bkn)
{
	uint8_t *cur = buf;
	uint8_t *hc, *hd;

	/* Normal Training Sequence: q11 to q22 */
	memcpy(cur, q_bits + 10, 12);
	cur += 12;

	/* Phase adjustment bits: hc1 to hc2 */
	hc = cur;
	cur += 2;

	/* Frequency correction: f1 to f80 */
	memcpy(cur, f_bits, 80);
	cur += 80;

	/* Scrambled synchronization block 1 bits: sb(1) to sb(120) */
	memcpy(cur, sb, 120);
	cur += 120;

	/* Synchronization training sequence: y1 to y38 */
	memcpy(cur, y_bits, 38);
	cur += 38;

	/* Scrambled broadcast bits: bb(1) to bb(30) */
	memcpy(cur, bb, 30);
	cur += 30;

	/* Scrambled block2 bits: bkn2(1) to bkn2(216) */
	memcpy(cur, bkn, 216);
	cur += 216;

	/* Phase adjustment bits: hd1 to hd2 */
	hd = cur;
	cur += 2;

	/* Normal training sequence 3: q1 to q10 */
	memcpy(cur, q_bits, 10);
	cur += 10;

	/* put in the phase adjustment bits */
	put_phase_adj_bits(buf, HC, hc);
	put_phase_adj_bits(buf, HD, hd);

	return cur - buf;
}

/* 9.4.4.2.5 Normal continuous downlink burst */
int build_norm_c_d_burst(uint8_t *buf, const uint8_t *bkn1, const uint8_t *bb, const uint8_t *bkn2, int two_log_chan)
{
	uint8_t *cur = buf;
	uint8_t *ha, *hb;

	/* Normal Training Sequence: q11 to q22 */
	memcpy(cur, q_bits + 10, 12);
	cur += 12;

	/* Phase adjustment bits: hc1 to hc2 */
	ha = cur;
	cur += 2;

	/* Scrambled block 1 bits: bkn1(1) to bkn1(216) */
	memcpy(cur, bkn1, 216);
	cur += 216;

	/* Scrambled broadcast bits: bb(1) to bb(14) */
	memcpy(cur, bb, 14);
	cur += 14;

	/* Normal training sequence: n1 to n22 or p1 to p22 */
	if (two_log_chan)
		memcpy(cur, p_bits, 22);
	else
		memcpy(cur, n_bits, 22);
	cur += 22;

	/* Scrambled broadcast bits: bb(15) to bb(30) */
	memcpy(cur, bb + 14, 16);
	cur += 16;

	/* Scrambled block2 bits: bkn2(1) to bkn2(216) */
	memcpy(cur, bkn2, 216);
	cur += 216;

	/* Phase adjustment bits: hd1 to hd2 */
	hb = cur;
	cur += 2;

	/* Normal training sequence 3: q1 to q10 */
	memcpy(cur, q_bits, 10);
	cur += 10;

	/* put in the phase adjustment bits */
	put_phase_adj_bits(buf, HA, ha);
	put_phase_adj_bits(buf, HB, hb);

	return cur - buf;
}

/* EN 300 396-2 - 9.4.3.2.1 DM Synchronization Burst */
int build_dm_sync_burst(uint8_t *buf, const uint8_t *bkn1, const uint8_t *bkn2)
{
	uint8_t *cur = buf;
	uint8_t *hl;

	/* Preamble bits: l1 to l12 */
	memcpy(cur, l_bits, 12);
	cur += 12;

	/* Phase adjustment bits: hl1 to hl2 */
	hl = cur;
	cur += 2;

	/* Frequency correction bits: f1 to f80 */
	memcpy(cur, f_bits, 80);
	cur += 80;

	/* Scrambled block 1 bits: bkn1(1) to bkn1(120) */
	memcpy(cur, bkn1, 120);
	cur += 120;

	/* Synchronization training sequence: y1 to y38 */
	memcpy(cur, y_bits, 38);
	cur += 38;

	/* Scrambled block2 bits: bkn2(1) to bkn2(216) */
	memcpy(cur, bkn2, 216);
	cur += 216;

	/* Tail bits: t1 to t2 */
	memcpy(cur, t_bits + 2, 2);
	cur += 2;

	/* put in the phase adjustment bits */
	put_phase_adj_bits(buf, HL, hl);

	return cur - buf;
}

/* EN 300 396-2 - 9.4.3.2.1 DM Normal Burst */
/* Type: 1 if preamble 1 and training sequence 1 (TCH, SCH/F), any other if preamble 2 and training sequence 2 (STCH+TCH, STCH+STCH)*/
int build_dm_norm_burst(uint8_t *buf, const uint8_t *bkn1, const uint8_t *bkn2, const uint8_t type)
{
	uint8_t *cur = buf;
	uint8_t *hk;

	/* Preamble bits: j1 to j12 or k1 to k12*/
	if (type == 1)
	{
		memcpy(cur, j_bits, 12);
	}
	else
	{
		memcpy(cur, k_bits, 12);
	}
	cur += 12;

	/* Phase adjustment bits: hk1 to hk2 */
	hk = cur;
	cur += 2;

	/* Scrambled block 1 bits: bkn1(1) to bkn1(120) */
	memcpy(cur, bkn1, 216);
	cur += 216;

	/* Normal training sequence: n1 to n22 or p2 to p22 */
	if (type == 1)
	{
		memcpy(cur, n_bits, 22);
	}
	else
	{
		memcpy(cur, p_bits, 22);
	}
	cur += 22;

	/* Scrambled block2 bits: bkn2(1) to bkn2(216) */
	memcpy(cur, bkn2, 216);
	cur += 216;

	/* Tail bits: t1 to t2 */
	memcpy(cur, t_bits + 2, 2);
	cur += 2;

	/* put in the phase adjustment bits */
	put_phase_adj_bits(buf, HK, hk);

	return cur - buf;
}

int tetra_find_train_seq(const uint8_t *in, unsigned int end_of_in,
						 uint32_t mask_of_train_seq, unsigned int *offset)
{
	static uint32_t tsq_bytes[5];

	if (tsq_bytes[0] == 0)
	{
#define FILTER_LOOKAHEAD_LEN 22
#define FILTER_LOOKAHEAD_MASK ((1 << FILTER_LOOKAHEAD_LEN) - 1)
		for (int i = 0; i < FILTER_LOOKAHEAD_LEN; i++)
		{
			tsq_bytes[0] = (tsq_bytes[0] << 1) | y_bits[i];
			tsq_bytes[1] = (tsq_bytes[1] << 1) | n_bits[i];
			tsq_bytes[2] = (tsq_bytes[2] << 1) | p_bits[i];
			tsq_bytes[3] = (tsq_bytes[3] << 1) | q_bits[i];
			tsq_bytes[4] = (tsq_bytes[4] << 1) | x_bits[i];
		}
	}

	uint32_t filter = 0;

	for (int i = 0; i < FILTER_LOOKAHEAD_LEN - 2; i++)
		filter = (filter << 1) | in[i];

	const uint8_t *cur;

	for (cur = in; cur < in + end_of_in; cur++)
	{
		filter = ((filter << 1) | cur[FILTER_LOOKAHEAD_LEN - 1]) & FILTER_LOOKAHEAD_MASK;

		int match = 0;

		for (int i = 0; i < 5; i++)
			if (filter == tsq_bytes[i])
				match = 1;

		if (!match)
			continue;

		int remain_len = (in + end_of_in) - cur;

		if (mask_of_train_seq & (1 << TETRA_TRAIN_SYNC) &&
			remain_len >= sizeof(y_bits) &&
			!memcmp(cur, y_bits, sizeof(y_bits)))
		{
			*offset = (cur - in);
			return TETRA_TRAIN_SYNC;
		}
		if (mask_of_train_seq & (1 << TETRA_TRAIN_NORM_1) &&
			remain_len >= sizeof(n_bits) &&
			!memcmp(cur, n_bits, sizeof(n_bits)))
		{
			*offset = (cur - in);
			return TETRA_TRAIN_NORM_1;
		}
		if (mask_of_train_seq & (1 << TETRA_TRAIN_NORM_2) &&
			remain_len >= sizeof(p_bits) &&
			!memcmp(cur, p_bits, sizeof(p_bits)))
		{
			*offset = (cur - in);
			return TETRA_TRAIN_NORM_2;
		}
		if (mask_of_train_seq & (1 << TETRA_TRAIN_NORM_3) &&
			remain_len >= sizeof(q_bits) &&
			!memcmp(cur, q_bits, sizeof(q_bits)))
		{
			*offset = (cur - in);
			return TETRA_TRAIN_NORM_3;
		}
		if (mask_of_train_seq & (1 << TETRA_TRAIN_EXT) &&
			remain_len >= sizeof(x_bits) &&
			!memcmp(cur, x_bits, sizeof(x_bits)))
		{
			*offset = (cur - in);
			return TETRA_TRAIN_EXT;
		}
	}
	return -1;
}

void tetra_burst_rx_cb(const uint8_t *burst, unsigned int len, enum tetra_train_seq type, void *priv)
{
	uint8_t bbk_buf[NDB_BBK_BITS];
	uint8_t ndbf_buf[2 * NDB_BLK_BITS];

	switch (type)
	{
	case TETRA_TRAIN_SYNC:
		/* Split SB1, SB2 and Broadcast Block */
		/* send three parts of the burst via TP-SAP into lower MAC */
		tp_sap_udata_ind(TPSAP_T_SB1, burst + SB_BLK1_OFFSET, SB_BLK1_BITS, priv);
		tp_sap_udata_ind(TPSAP_T_BBK, burst + SB_BBK_OFFSET, SB_BBK_BITS, priv);
		tp_sap_udata_ind(TPSAP_T_SB2, burst + SB_BLK2_OFFSET, SB_BLK2_BITS, priv);
		break;
	case TETRA_TRAIN_NORM_2:
		/* re-combine the broadcast block */
		memcpy(bbk_buf, burst + NDB_BBK1_OFFSET, NDB_BBK1_BITS);
		memcpy(bbk_buf + NDB_BBK1_BITS, burst + NDB_BBK2_OFFSET, NDB_BBK2_BITS);
		/* send three parts of the burst via TP-SAP into lower MAC */
		tp_sap_udata_ind(TPSAP_T_BBK, bbk_buf, NDB_BBK_BITS, priv);
		tp_sap_udata_ind(TPSAP_T_NDB, burst + NDB_BLK1_OFFSET, NDB_BLK_BITS, priv);
		tp_sap_udata_ind(TPSAP_T_NDB, burst + NDB_BLK2_OFFSET, NDB_BLK_BITS, priv);
		break;
	case TETRA_TRAIN_NORM_1:
		/* re-combine the broadcast block */
		memcpy(bbk_buf, burst + NDB_BBK1_OFFSET, NDB_BBK1_BITS);
		memcpy(bbk_buf + NDB_BBK1_BITS, burst + NDB_BBK2_OFFSET, NDB_BBK2_BITS);
		/* re-combine the two parts */
		memcpy(ndbf_buf, burst + NDB_BLK1_OFFSET, NDB_BLK_BITS);
		memcpy(ndbf_buf + NDB_BLK_BITS, burst + NDB_BLK2_OFFSET, NDB_BLK_BITS);
		/* send two parts of the burst via TP-SAP into lower MAC */
		tp_sap_udata_ind(TPSAP_T_BBK, bbk_buf, NDB_BBK_BITS, priv);
		tp_sap_udata_ind(TPSAP_T_SCH_F, ndbf_buf, 2 * NDB_BLK_BITS, priv);
		break;
	default:
		// did we forgot something?
		break;
	}
}

void tetra_burst_dmo_rx_cb(const uint8_t *burst, unsigned int len, enum tetra_train_seq type, void *priv)
{
	uint8_t dnbf_buf[2 * DMO_DNB_BLK_BITS];

	switch (type)
	{
	case TETRA_TRAIN_SYNC:
		/* Split SB1, SB2 Block */
		/* send parts of the burst via DP-SAP into lower MAC */
		dp_sap_udata_ind(DPSAP_T_SCH_S, burst + DMO_SB_BLK1_OFFSET, DMO_SB_BLK1_BITS, priv);
		dp_sap_udata_ind(DPSAP_T_SCH_H, burst + DMO_SB_BLK2_OFFSET, DMO_SB_BLK2_BITS, priv);
		break;
	case TETRA_TRAIN_NORM_2:
		/* send two logical channels of the DNB burst via DP-SAP into lower MAC (396-2 9.4.3.3.3) */
		dp_sap_udata_ind(DPSAP_T_DNB1, burst + DMO_DNB_BLK1_OFFSET, DMO_DNB_BLK_BITS, priv);
		dp_sap_udata_ind(DPSAP_T_DNB1, burst + DMO_DNB_BLK2_OFFSET, DMO_DNB_BLK_BITS, priv);
		break;
	case TETRA_TRAIN_NORM_1:
		/* re-combine the two block parts */
		memcpy(dnbf_buf, burst + DMO_DNB_BLK1_OFFSET, DMO_DNB_BLK_BITS);
		memcpy(dnbf_buf + DMO_DNB_BLK_BITS, burst + DMO_DNB_BLK2_OFFSET, DMO_DNB_BLK_BITS);
		/* send one logical channel of the DNB burst via DP-SAP into lower MAC */
		dp_sap_udata_ind(DPSAP_T_SCH_F, dnbf_buf, 2 * DMO_DNB_BLK_BITS, priv);
		break;
	default:
		printf("#### hello from burst with to-do training sequence\n");
		// did we forgot something?
		break;
	}
}

static int count_errs(const uint8_t *a, const uint8_t *b, int len)
{
	int i, errs = 0;
	for (i = 0; i < len; i++)
		if (a[i] != b[i])
			errs++;
	return errs;
}

/* Note: this may use different indexing for the bits
 * than tetra_burst_dmo_rx_cb. */
void tetra_burst_dmo_rx_cb2(const uint8_t *burst, unsigned int len, enum tetra_train_seq type, void *priv)
{
	uint8_t dnbf_buf[2 * DMO_DNB_BLK_BITS];
	struct tetra_mac_state *tms = priv;

	switch (type)
	{
	case TETRA_TRAIN_SYNC:
	{
		const struct dmo_sync_bits *b = (void *)burst;
		/* Split SB1, SB2 Block */
		/* send parts of the burst via DP-SAP into lower MAC */
		mac_dp_sap_udata_ind_filter(DPSAP_T_SCH_S, b->block1, sizeof(b->block1), priv);
		mac_dp_sap_udata_ind_filter(DPSAP_T_SCH_H, b->block2, sizeof(b->block2), priv);
		break;
	}
	case TETRA_TRAIN_NORM_2:
	{
		const struct dmo_normal_bits *b = (void *)burst;
		/* send two logical channels of the DNB burst via DP-SAP into lower MAC (396-2 9.4.3.3.3) */
		mac_dp_sap_udata_ind_filter(DPSAP_T_STCH, b->block1, sizeof(b->block1), priv);
		mac_dp_sap_udata_ind_filter(DPSAP_T_DNB1, b->block2, sizeof(b->block2), priv);
		break;
	}
	case TETRA_TRAIN_NORM_1:
	{
		const struct dmo_normal_bits *b = (void *)burst;
		/* re-combine the two block parts */
		memcpy(dnbf_buf, b->block1, DMO_DNB_BLK_BITS);
		memcpy(dnbf_buf + DMO_DNB_BLK_BITS, b->block2, DMO_DNB_BLK_BITS);
		/* send one logical channel of the DNB burst via DP-SAP into lower MAC */
		if (tms->mode_of_operation == DM_MAC_MODE_TRAFFIC_SPEECH)
		{
			mac_dp_sap_udata_ind_filter(DPSAP_T_TCH, dnbf_buf, 2 * DMO_DNB_BLK_BITS, priv);
		}
		else
		{
			mac_dp_sap_udata_ind_filter(DPSAP_T_SCH_F, dnbf_buf, 2 * DMO_DNB_BLK_BITS, priv);
		}
		break;
	}
	default:
		printf("#### hello from burst with to-do training sequence of type :%d\n", type);
		// did we forgot something?
		break;
	}
}

enum tetra_train_seq tetra_check_train(const uint8_t *burst, unsigned int len)
{
	{
		const struct dmo_sync_bits *b = (void *)burst;
		if (count_errs(b->train, y_bits, sizeof(y_bits)) <= 2)
			return TETRA_TRAIN_SYNC;
	}
	{
		const struct dmo_normal_bits *b = (void *)burst;
		// TODO: check preamble too
		if (count_errs(b->train, n_bits, sizeof(n_bits)) <= 0)
			return TETRA_TRAIN_NORM_1;
		if (count_errs(b->train, p_bits, sizeof(p_bits)) <= 0)
			return TETRA_TRAIN_NORM_2;
	}
	return TETRA_TRAIN_INVALID;
}
