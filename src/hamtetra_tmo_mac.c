/* Provides TMO BTS MAC layer operations for handling control channels and signaling. */

#include <arpa/inet.h>
#include <assert.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/bitvec.h>
#include <osmocom/core/talloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hamtetra_config.h"
#include "hamtetra_mac.h"
#include "hamtetra_tmo_mac.h"
#include "lower_mac/crc_simple.h"
#include "lower_mac/tetra_conv_enc.h"
#include "lower_mac/tetra_interleave.h"
#include "lower_mac/tetra_rm3014.h"
#include "lower_mac/tetra_scramb.h"
#include "phy/tetra_burst.h"
#include "phy/tetra_burst_sync.h"

// TMO BTS operation counters
static uint32_t bsch_counter = 0;
static uint32_t bcch_counter = 0;

// Initializes TMO BTS MAC layer components and counters
void tmo_mac_init(void)
{
    printf("Initializing TMO BTS MAC Layer\n");
    bsch_counter = 0;
    bcch_counter = 0;
}

// Constructs a fully ETSI-compliant SYNC-PDU.
// Reference: ETSI TS 100 392-2 v3.9.1, Table 21.76 (SYNC PDU contents)
// Reference: ETSI TS 100 392-2 v3.9.1, Table 18.17 (D-MLE-SYNC contents)
int build_tmo_sync_pdu(uint8_t *sync_pdu_bits, struct timing_slot *slot)
{
    uint8_t pdu_packed[8];
    struct bitvec bv;
    memset(&bv, 0, sizeof(struct bitvec));
    bv.data = pdu_packed;
    bv.data_len = sizeof(pdu_packed);
    memset(pdu_packed, 0, sizeof(pdu_packed));

    // --- MAC-level fields (Table 21.76) ---
    bitvec_set_uint(&bv, 3, 4);            // System code (0011b for modern systems)
    bitvec_set_uint(&bv, TMO_CC, 6);       // Colour code
    bitvec_set_uint(&bv, slot->tn - 1, 2); // Timeslot number
    bitvec_set_uint(&bv, slot->fn, 5);     // Frame number
    bitvec_set_uint(&bv, slot->mn, 6);     // Multiframe number
    bitvec_set_uint(&bv, 0, 2);            // Sharing mode (00b = Continuous transmission)
    bitvec_set_uint(&bv, 0, 3);            // TS reserved frames
    bitvec_set_uint(&bv, 0, 1);            // U-plane DTX
    bitvec_set_uint(&bv, 0, 1);            // Frame 18 extension
    bitvec_set_uint(&bv, 0, 1);            // Reserved

    // --- MLE-level fields / TM-SDU (Table 18.17) ---
    bitvec_set_uint(&bv, TMO_MCC, 10); // MCC
    bitvec_set_uint(&bv, TMO_MNC, 14); // MNC
    bitvec_set_uint(&bv, 0, 2);        // Neighbour cell broadcast
    bitvec_set_uint(&bv, 0, 2);        // Cell load CA
    bitvec_set_uint(&bv, 1, 1);        // Late entry supported

    osmo_pbit2ubit(sync_pdu_bits, pdu_packed, 60);
    return 60;
}

// Constructs a fully ETSI-compliant SYSINFO-PDU.
// Reference: ETSI TS 100 392-2 v3.9.1, Table 21.65 (SYSINFO PDU contents)
// Reference: ETSI TS 100 392-2 v3.9.1, Table 21.66 (Default definition for access code A)
// Reference: ETSI TS 100 392-2 v3.9.1, Table 18.19 (D-MLE-SYSINFO contents)
int build_tmo_sysinfo_pdu(uint8_t *sysinfo_bits, struct timing_slot *slot, uint8_t sysinfo_type)
{
    uint8_t pdu_packed[16];
    struct bitvec bv_mac;
    memset(&bv_mac, 0, sizeof(struct bitvec));
    bv_mac.data = pdu_packed;
    bv_mac.data_len = sizeof(pdu_packed);
    memset(pdu_packed, 0, sizeof(pdu_packed));

    // --- MAC PDU Header (Table 21.65) ---
    bitvec_set_uint(&bv_mac, 2, 2); // MAC PDU type (10b = Broadcast PDU)
    bitvec_set_uint(&bv_mac, 0, 2); // Broadcast type (00b = SYSINFO PDU)

    // --- Radio Parameters: Frequencies & Channels (Table 21.65) ---
    bitvec_set_uint(&bv_mac, 1200, 12); // Main carrier
    bitvec_set_uint(&bv_mac, 4, 4);     // Frequency band
    bitvec_set_uint(&bv_mac, 0, 2);     // Offset
    bitvec_set_uint(&bv_mac, 0, 3);     // Duplex spacing
    bitvec_set_uint(&bv_mac, 0, 1);     // Reverse operation

    // --- Radio Parameters: Access & Power Control (Table 21.65) ---
    bitvec_set_uint(&bv_mac, 0, 2);  // Number of common SCCHs (00b = None)
    bitvec_set_uint(&bv_mac, 4, 3);  // MS_TXPWR_MAX_CELL (100b = 30 dBm)
    bitvec_set_uint(&bv_mac, 3, 4);  // RXLEV_ACCESS_MIN (0011b = -110 dBm)
    bitvec_set_uint(&bv_mac, 4, 4);  // ACCESS_PARAMETER (0100b = -45 dBm)
    bitvec_set_uint(&bv_mac, 15, 4); // RADIO_DOWNLINK_TIMEOUT (1111b = 2160 timeslots)

    // --- System Timing Information (Table 21.65) ---
    bitvec_set_uint(&bv_mac, 0, 1);  // Hyperframe / cipher key flag (0 = Hyperframe number)
    bitvec_set_uint(&bv_mac, 0, 16); // Hyperframe number

    // --- Optional Field Block with Access Parameters (Table 21.65 & 21.66) ---
    bitvec_set_uint(&bv_mac, 2, 2); // Optional field flag (10b = "Default definition for access code A")
    bitvec_set_uint(&bv_mac, 0, 4); // IMM (Immediate) (0000b = Always randomize)
    bitvec_set_uint(&bv_mac, 7, 4); // WT (Waiting Time) (0111b = 7 opportunities)
    bitvec_set_uint(&bv_mac, 6, 4); // Nu (Number of transmissions) (0110b = 6 retries)
    bitvec_set_uint(&bv_mac, 0, 1); // Frame-length factor (0b = Multiply by 1)
    bitvec_set_uint(&bv_mac, 0, 4); // Timeslot pointer (0000b = Same as downlink)
    bitvec_set_uint(&bv_mac, 0, 3); // Minimum PDU priority (000b = Priority 0)

    // --- D-MLE-SYSINFO Payload / TM-SDU (Table 18.19) ---
    bitvec_set_uint(&bv_mac, TMO_LA, 14); // Location area (LA)
    bitvec_set_uint(&bv_mac, 65535, 16);  // Subscriber class

    // BS Service Details
    uint16_t service_details = 0;
    service_details |= (1 << 11); // Registration required
    service_details |= (1 << 10); // De-registration required
    service_details |= (0 << 9);  // Priority cell
    service_details |= (1 << 8);  // Cell never uses minimum mode
    service_details |= (0 << 7);  // Migration supported
    service_details |= (1 << 6);  // System wide services supported
    service_details |= (1 << 5);  // TETRA voice service supported
    service_details |= (1 << 4);  // Circuit mode data service supported
    service_details |= (0 << 3);  // Reserved
    service_details |= (1 << 2);  // SNDCP service supported
    service_details |= (0 << 1);  // Air interface encryption
    service_details |= (1 << 0);  // Advanced link supported
    bitvec_set_uint(&bv_mac, service_details, 12);

    osmo_pbit2ubit(sysinfo_bits, pdu_packed, 124);
    return 124;
}

// Assembles a compliant composite burst carrying SYNC and SYSINFO PDUs.
int build_tmo_bcch_burst(uint8_t *bits, struct timing_slot *slot, uint8_t sysinfo_type)
{
    uint8_t sb_pdu_bits[60], sb_type2[80], sb_master[80 * 4], sb_type3[120], sb_type4[120], sb_type5[120];
    uint8_t si_type2[144], si_master[144 * 4], si_type3[216], si_type4[216], si_type5[216];
    uint8_t bb_type5[30];
    uint8_t burst[510];
    uint16_t crc;
    uint32_t bb_rm3014, bb_rm3014_be;
    struct conv_enc_state *ces;

    // Block 1: SYNC-PDU
    build_tmo_sync_pdu(sb_pdu_bits, slot);
    memcpy(sb_type2, sb_pdu_bits, 60);
    crc = ~crc16_ccitt_bits(sb_type2, 60);
    for (int i = 0; i < 16; i++)
    {
        sb_type2[60 + i] = (crc >> (15 - i)) & 1;
    }
    memset(&sb_type2[76], 0, 4);

    ces = calloc(1, sizeof(*ces));
    assert(ces != NULL);
    conv_enc_init(ces);
    conv_enc_input(ces, sb_type2, 80, sb_master);
    get_punctured_rate(TETRA_RCPC_PUNCT_2_3, sb_master, 120, sb_type3);
    free(ces);
    block_interleave(120, 11, sb_type3, sb_type4);
    memcpy(sb_type5, sb_type4, 120);
    tetra_scramb_bits(SCRAMB_INIT, sb_type5, 120);

    // Block 2: SYSINFO-PDU
    build_tmo_sysinfo_pdu(si_type2, slot, sysinfo_type);
    crc = ~crc16_ccitt_bits(si_type2, 124);
    for (int i = 0; i < 16; i++)
    {
        si_type2[124 + i] = (crc >> (15 - i)) & 1;
    }
    memset(&si_type2[140], 0, 4);

    ces = calloc(1, sizeof(*ces));
    assert(ces != NULL);
    conv_enc_init(ces);
    conv_enc_input(ces, si_type2, 144, si_master);
    get_punctured_rate(TETRA_RCPC_PUNCT_2_3, si_master, 216, si_type3);
    free(ces);
    block_interleave(216, 101, si_type3, si_type4);
    memcpy(si_type5, si_type4, 216);
    uint32_t scrambler_key = tetra_scramb_get_init(TMO_MCC, TMO_MNC, TMO_CC);
    tetra_scramb_bits(scrambler_key, si_type5, 216);

    // Block 3: AACH
    uint16_t aach_value = 0;
    if (slot->tn == 1 && slot->fn == 18)
    {
        aach_value = (0 << 12);
    }
    else
    {
        aach_value = (1 << 12) | (0 << 6);
    }
    bb_rm3014 = tetra_rm3014_compute(aach_value);
    bb_rm3014_be = htonl(bb_rm3014);
    bb_rm3014_be <<= 2;
    osmo_pbit2ubit(bb_type5, (uint8_t *)&bb_rm3014_be, 30);

    // Block 4: Burst Assembly
    build_sync_c_d_burst(burst, sb_type5, bb_type5, si_type5);
    memcpy(bits, burst, 510);
    return 510;
}

// Builds a proper Normal Downlink Burst for idle slots.
int build_tmo_dsch_burst(uint8_t *bits, struct timing_slot *slot)
{
    uint8_t bkn1_master[216 * 4], bkn2_master[216 * 4];
    uint8_t bkn1_type3[216], bkn2_type3[216], bkn1_type4[216], bkn2_type4[216];
    uint8_t bkn1_type5[216], bkn2_type5[216];
    uint8_t burst[510];
    uint8_t bkn1_bits[216] = {0}, bkn2_bits[216] = {0};
    uint8_t bb_type5[30];
    uint32_t bb_rm3014, bb_rm3014_be;

    struct conv_enc_state *ces1 = calloc(1, sizeof(*ces1));
    assert(ces1 != NULL);
    conv_enc_init(ces1);
    conv_enc_input(ces1, bkn1_bits, 216, bkn1_master);
    get_punctured_rate(TETRA_RCPC_PUNCT_2_3, bkn1_master, 216, bkn1_type3);
    free(ces1);

    struct conv_enc_state *ces2 = calloc(1, sizeof(*ces2));
    assert(ces2 != NULL);
    conv_enc_init(ces2);
    conv_enc_input(ces2, bkn2_bits, 216, bkn2_master);
    get_punctured_rate(TETRA_RCPC_PUNCT_2_3, bkn2_master, 216, bkn2_type3);
    free(ces2);

    block_interleave(216, 11, bkn1_type3, bkn1_type4);
    block_interleave(216, 11, bkn2_type3, bkn2_type4);
    memcpy(bkn1_type5, bkn1_type4, 216);
    memcpy(bkn2_type5, bkn2_type4, 216);

    uint32_t scrambler_key = tetra_scramb_get_init(TMO_MCC, TMO_MNC, TMO_CC);
    tetra_scramb_bits(scrambler_key, bkn1_type5, 216);
    tetra_scramb_bits(scrambler_key, bkn2_type5, 216);

    bb_rm3014 = tetra_rm3014_compute(0);
    bb_rm3014_be = htonl(bb_rm3014);
    bb_rm3014_be <<= 2;
    osmo_pbit2ubit(bb_type5, (uint8_t *)&bb_rm3014_be, 30);

    build_norm_c_d_burst(burst, bkn1_type5, bb_type5, bkn2_type5, 0);
    memcpy(bits, burst, 510);
    return 510;
}

// Selects and builds the appropriate downlink burst.
int mac_request_tx_buffer_content_tmo(uint8_t *bits, struct timing_slot *slot)
{
    int len = -1;

    // printf("[MAC] TX slot: TN=%u FN=%u MN=%u -> ", slot->tn, slot->fn, slot->mn);

    // Main Control Channel (MCCH) is on Timeslot 1
    if (slot->tn == 1)
    {
        // The anchor broadcast containing SYNC and SYSINFO is on Frame 18.
        if (slot->fn == 18)
        {
            // printf("BSCH/BNCH (Anchor)\n");

            len = build_tmo_bcch_burst(bits, slot, 1);
            bsch_counter++;
            bcch_counter++;
        }
        else
        {
            // All other frames on the MCCH (1-17) are idle.
            // printf("MCCH/IDLE\n");
            len = build_tmo_dsch_burst(bits, slot);
        }
    }
    // Traffic Channels (TCH) on Timeslots 2, 3, 4
    else
    {
        // Fill unallocated TCHs with BCCH "filler" bursts to aid cell discovery.
        // printf("TCH/BCCH (Filler)\n");
        len = build_tmo_bcch_burst(bits, slot, 1);
        bcch_counter++;
    }

    return len;
}
