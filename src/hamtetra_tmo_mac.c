/* Provides TMO BTS MAC layer operations for handling control channels and signaling. */

#include <arpa/inet.h>
#include <assert.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/bitvec.h>
#include <osmocom/core/talloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hamtetra_mac.h"
#include "hamtetra_tmo_config.h"
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

// Constructs SYNC-PDU with 8-bit padding for correct MCC/MNC alignment.
int build_tmo_sync_pdu(uint8_t *sync_pdu_bits, struct timing_slot *slot)
{
    uint8_t pdu_sync_packed[8];
    struct bitvec bv;
    memset(&bv, 0, sizeof(struct bitvec));
    bv.data = pdu_sync_packed;
    bv.data_len = sizeof(pdu_sync_packed);
    memset(pdu_sync_packed, 0, sizeof(pdu_sync_packed));

    bitvec_set_uint(&bv, 0, 4);
    bitvec_set_uint(&bv, TMO_CC, 6);
    bitvec_set_uint(&bv, slot->tn, 2);
    bitvec_set_uint(&bv, slot->fn, 5);
    bitvec_set_uint(&bv, slot->mn, 6);
    bitvec_set_uint(&bv, 0, 8); // Critical 8-bit padding
    bitvec_set_uint(&bv, TMO_MCC, 10);
    bitvec_set_uint(&bv, TMO_MNC, 14);
    bitvec_set_uint(&bv, 0, 5);

    osmo_pbit2ubit(sync_pdu_bits, pdu_sync_packed, 60);
    return 60;
}

// Generates a pure SYNC burst for the BSCH, using the fixed default scrambler key.
int build_tmo_sync_burst(uint8_t *bits, struct timing_slot *slot)
{
    static uint8_t sb_pdu_bits[60], sb_type2[80], sb_master[80 * 4], sb_type3[120], sb_type4[120], sb_type5[120];
    static uint8_t dummy_block[216];
    uint8_t bb_type5[30];
    static uint8_t burst[255 * 2];
    uint16_t crc;
    uint32_t bb_rm3014, bb_rm3014_be;
    struct conv_enc_state *ces;

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

    memset(dummy_block, 0, sizeof(dummy_block));
    uint32_t scrambler_key = tetra_scramb_get_init(TMO_MCC, TMO_MNC, TMO_CC);
    tetra_scramb_bits(scrambler_key, dummy_block, 216);

    // AACH for a pure SYNC burst is simple, but must indicate a common channel
    uint8_t aach_data[] = {0x81, 0x20};
    bb_rm3014 = tetra_rm3014_compute(*(uint16_t *)aach_data);
    bb_rm3014_be = htonl(bb_rm3014);
    bb_rm3014_be <<= 2;
    osmo_pbit2ubit(bb_type5, (uint8_t *)&bb_rm3014_be, 30);

    // Second data block is dummy data, can reuse sb_type5 for simplicity
    build_sync_c_d_burst(burst, sb_type5, bb_type5, dummy_block);
    memcpy(bits, burst, 510);
    printf("[TMO BTS] Generated PURE SYNC burst on main control channel\n");
    return 510;
}

// Creates the specific, flat SYSINFO PDU payload.
int build_tmo_sysinfo_pdu(uint8_t *sysinfo_bits, struct timing_slot *slot, uint8_t sysinfo_type)
{
    uint8_t pdu_packed[27];
    struct bitvec bv;
    memset(&bv, 0, sizeof(struct bitvec));
    bv.data = pdu_packed;
    bv.data_len = sizeof(pdu_packed);
    memset(pdu_packed, 0, sizeof(pdu_packed));

    // // The sysinfo_type parameter is ignored, as we only have one type of SYSINFO burst now.
    // (void)sysinfo_type;

    // Build the PDU
    bitvec_set_uint(&bv, 0b1000, 4);  // MAC Header: Broadcast type 0
    bitvec_set_uint(&bv, 1200, 12);   // Main Carrier
    bitvec_set_uint(&bv, 4, 4);       // Frequency Band
    bitvec_set_uint(&bv, 0, 2);       // Offset
    bitvec_set_uint(&bv, 0, 3);       // Duplex Spacing
    bitvec_set_uint(&bv, 0, 1);       // Reverse Operation
    bitvec_set_uint(&bv, 0, 2);       // NumberOfCommon_SC
    bitvec_set_uint(&bv, 4, 3);       // MS_TXPwr_Max_Cell
    bitvec_set_uint(&bv, 0, 4);       // RXLevel_Access_Min
    bitvec_set_uint(&bv, 0, 4);       // Access_Parameter
    bitvec_set_uint(&bv, 0, 4);       // Radio_Downlink_Timeout
    bitvec_set_uint(&bv, 0, 1);       // Hyperframe_or_Cipher_key_flag
    bitvec_set_uint(&bv, 0, 16);      // Hyperframe
    bitvec_set_uint(&bv, 0, 2);       // Optional_field_flag
    bitvec_set_uint(&bv, 1, 14);      // Location Area
    bitvec_set_uint(&bv, 0xFFFF, 16); // Subscriber_Class (all enabled)
    bitvec_set_uint(&bv, 1, 1);       // Registration_required
    bitvec_set_uint(&bv, 1, 1);       // De_registration_required
    bitvec_set_uint(&bv, 0, 1);       // Priority_cell
    bitvec_set_uint(&bv, 0, 1);       // Cell_never_uses_minimum_mode
    bitvec_set_uint(&bv, 0, 1);       // Migration_supported
    bitvec_set_uint(&bv, 1, 1);       // System_wide_services
    bitvec_set_uint(&bv, 1, 1);       // TETRA_voice_service
    bitvec_set_uint(&bv, 1, 1);       // Circuit_mode_data_service
    bitvec_set_uint(&bv, 0, 1);       // Reserved
    bitvec_set_uint(&bv, 1, 1);       // SNDCP_Service
    bitvec_set_uint(&bv, 0, 1);       // Air_interface_encryption
    bitvec_set_uint(&bv, 1, 1);       // Advanced_link_supported

    osmo_pbit2ubit(sysinfo_bits, pdu_packed, bv.cur_bit);
    return bv.cur_bit;
}

// Assembles BCCH burst with all validated physical layer fixes.
int build_tmo_bcch_burst(uint8_t *bits, struct timing_slot *slot, uint8_t sysinfo_type)
{
    static uint8_t sb_pdu_bits[60], sb_type2[80], sb_master[80 * 4], sb_type3[120], sb_type4[120], sb_type5[120];
    static uint8_t si_pdu_bits[124], si_type2[144], si_master[144 * 4], si_type3[216], si_type4[216], si_type5[216];
    uint8_t bb_type5[30];
    static uint8_t burst[255 * 2];
    uint16_t crc;
    uint32_t bb_rm3014, bb_rm3014_be;
    struct conv_enc_state *ces;

    // --- 1. Prepare Block 1 (SYNC-PDU) ---
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

    // --- 2. Prepare Block 2 (SYSINFO-PDU) ---
    memset(si_type2, 0, sizeof(si_type2));
    int si_len = build_tmo_sysinfo_pdu(si_type2, slot, sysinfo_type);
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

    // --- 3. Prepare Broadcast Block (AACH) ---
    uint8_t aach_data[] = {0x00, 0x00};
    bb_rm3014 = tetra_rm3014_compute(*(uint16_t *)aach_data);
    bb_rm3014_be = htonl(bb_rm3014);
    bb_rm3014_be <<= 2;
    osmo_pbit2ubit(bb_type5, (uint8_t *)&bb_rm3014_be, 30);

    // --- 4. Build the final physical burst ---
    build_sync_c_d_burst(burst, sb_type5, bb_type5, si_type5);

    memcpy(bits, burst, 510);
    printf("[PHY] Transmitted composite SYSINFO-in-SYNC burst on TN %d\n", slot->tn);
    return 510;
}

// Builds a proper Normal Downlink Burst for idle slots.
int build_tmo_dsch_burst(uint8_t *bits, struct timing_slot *slot)
{
    static uint8_t bkn1_master[216 * 4], bkn2_master[216 * 4];
    static uint8_t bkn1_type3[216], bkn2_type3[216], bkn1_type4[216], bkn2_type4[216];
    static uint8_t bkn1_type5[216], bkn2_type5[216];
    static uint8_t burst[255 * 2];
    static uint8_t bkn1_bits[216], bkn2_bits[216];
    uint8_t bb_type5[30];
    uint32_t bb_rm3014, bb_rm3014_be;

    memset(bkn1_bits, 0, sizeof(bkn1_bits));
    memset(bkn2_bits, 0, sizeof(bkn2_bits));

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

    uint8_t aach_data[] = {0x00, 0x00};
    bb_rm3014 = tetra_rm3014_compute(*(uint16_t *)aach_data);
    bb_rm3014_be = htonl(bb_rm3014);
    bb_rm3014_be <<= 2;
    osmo_pbit2ubit(bb_type5, (uint8_t *)&bb_rm3014_be, 30);

    build_norm_c_d_burst(burst, bkn1_type5, bb_type5, bkn2_type5, 0);
    memcpy(bits, burst, 510);
    return 510;
}

// Main entry point called by the scheduler.
int mac_request_tx_buffer_content_tmo(uint8_t *bits, struct timing_slot *slot)
{
    int len = -1;
    printf("[TMO BTS] TX slot: %2u %2u %2u - ", slot->tn, slot->fn, slot->mn);

    // Main Control Channel (Timeslot 1)
    if (slot->tn == 1)
    {
        // On frames 1 and 11, send a "pure" SYNC burst for network acquisition.
        if (slot->fn == 1 || slot->fn == 11)
        {
            printf("PURE SYNC - ");
            len = build_tmo_sync_burst(bits, slot); // This is now the "pure" sync burst
            bsch_counter++;
        }
        else
        {
            // On other frames, TN1 is a regular control channel, send dummy data.
            printf("Control/DSCH - ");
            len = build_tmo_dsch_burst(bits, slot);
        }
    }
    // Traffic Channels (Timeslots 2, 3, 4)
    else
    {
        // Broadcast SYSINFO on all idle traffic slots.
        printf("IDLE/SYSINFO - ");
        // The old `bcch_burst` function is now repurposed to build the composite SYSINFO-in-SYNC burst.
        len = build_tmo_bcch_burst(bits, slot, 1);
        bcch_counter++;
    }

    printf("burst len: %d\n", len);
    return len;
}
