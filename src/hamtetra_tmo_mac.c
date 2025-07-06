/* Provides TMO BTS MAC layer operations for handling control channels and signaling */

#include <arpa/inet.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/bitvec.h>
#include <osmocom/core/talloc.h>

#include "hamtetra_tmo_config.h"
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
static uint32_t sync_burst_counter = 0;
static uint32_t bcch_counter = 0;

#define swap16(x) ((x) << 8) | ((x) >> 8)

// Initializes TMO BTS MAC layer components and counters
void tmo_mac_init(void)
{
    printf("Initializing TMO BTS MAC Layer\n");

    // Initialize TMO-specific counters
    bsch_counter = 0;
    sync_burst_counter = 0;
    bcch_counter = 0;
}

// Constructs SYNC-PDU data for TMO BTS with proper MCC/MNC encoding
int build_tmo_sync_pdu(uint8_t *sync_pdu_bits, struct timing_slot *slot)
{
    // TMO SYNC-PDU structure compatible with osmo-tetra decoder expectations
    // The decoder expects MCC at bits 31-40 and MNC at bits 41-54
    uint8_t pdu_sync[8]; // 60 bits = 8 bytes
    struct bitvec bv;

    memset(&pdu_sync, 0, sizeof(pdu_sync));
    memset(&bv, 0, sizeof(bv));
    bv.data = pdu_sync;
    bv.data_len = sizeof(pdu_sync);

    // Build SYNC PDU with MCC/MNC at expected positions
    // Bits 0-3: System Code (4 bits) - indicates TMO system
    bitvec_set_uint(&bv, 0, 4); // 0 = TMO (different from DMO which uses 13)

    // Bits 4-9: Colour Code (6 bits)
    bitvec_set_uint(&bv, TMO_CC, 6);

    // Bits 10-11: Timeslot Number (2 bits)
    bitvec_set_uint(&bv, slot->tn, 2);

    // Bits 12-16: Frame Number (5 bits)
    bitvec_set_uint(&bv, slot->fn, 5);

    // Bits 17-22: Multiframe Number (6 bits)
    bitvec_set_uint(&bv, slot->mn, 6);

    // Bits 23-30: Reserved/padding (8 bits) to align MCC at bit 31
    bitvec_set_uint(&bv, 0, 8);

    // Bits 31-40: MCC (10 bits) - decoder expects this here!
    bitvec_set_uint(&bv, TMO_MCC, 10);

    // Bits 41-54: MNC (14 bits) - decoder expects this here!
    bitvec_set_uint(&bv, TMO_MNC, 14);

    // Remaining bits (55-59): Reserved (5 bits)
    bitvec_set_uint(&bv, 0, 5);

    printf("[TMO SYNC] Built SYNC PDU with CC=%u at bits 4-9, MCC=%u at bits 31-40, MNC=%u at bits 41-54\n", TMO_CC, TMO_MCC, TMO_MNC);

    // Convert packed bits to unpacked bit array
    osmo_pbit2ubit(sync_pdu_bits, pdu_sync, 60);

    return 60;
}

// Generates complete TMO SYNC burst with synchronization and timing information
int build_tmo_sync_burst(uint8_t *bits, struct timing_slot *slot)
{
    // Enhanced TMO BTS implementation with proper SYNC-PDU
    uint8_t sb_type2[80];
    uint8_t sb_master[80 * 4];
    uint8_t sb_type3[120];
    uint8_t sb_type4[120];
    uint8_t sb_type5[120];
    uint8_t bb_type5[30];
    uint8_t burst[255 * 2]; // Full TETRA burst (510 symbols)
    uint16_t crc;
    uint8_t *cur;
    uint32_t bb_rm3014, bb_rm3014_be;

    // Create proper SYNC-PDU for TMO BTS
    memset(sb_type2, 0, sizeof(sb_type2));
    cur = sb_type2;

    // Build proper SYNC-PDU content
    uint8_t sync_pdu_bits[60];
    build_tmo_sync_pdu(sync_pdu_bits, slot);

    // Convert bits to packed bytes and add to type2
    uint8_t sync_pdu_packed[8];
    for (int i = 0; i < 8; i++)
    {
        sync_pdu_packed[i] = 0;
        for (int j = 0; j < 8 && (i * 8 + j) < 60; j++)
        {
            sync_pdu_packed[i] |= (sync_pdu_bits[i * 8 + j] << (7 - j));
        }
    }

    cur += osmo_pbit2ubit(sb_type2, sync_pdu_packed, 60);

    crc = ~crc16_ccitt_bits(sb_type2, 60);
    crc = swap16(crc);
    cur += osmo_pbit2ubit(cur, (uint8_t *)&crc, 16);

    // Append 4 tail bits: type-2 bits
    cur += 4;

    // Run rate 2/3 RCPC code: type-3 bits
    struct conv_enc_state *ces = calloc(1, sizeof(*ces));
    conv_enc_init(ces);
    conv_enc_input(ces, sb_type2, 80, sb_master);
    get_punctured_rate(TETRA_RCPC_PUNCT_2_3, sb_master, 120, sb_type3);
    free(ces);

    // Run (120,11) block interleaving: type-4 bits
    block_interleave(120, 11, sb_type3, sb_type4);

    // Run scrambling (all-zero): type-5 bits
    memcpy(sb_type5, sb_type4, 120);
    tetra_scramb_bits(SCRAMB_INIT, sb_type5, 120);

    // Create basic AACH content
    uint8_t aach_data[] = {0x00, 0x01}; // Basic AACH data
    bb_rm3014 = tetra_rm3014_compute(*(uint16_t *)aach_data);
    bb_rm3014_be = htonl(bb_rm3014);
    bb_rm3014_be <<= 2;
    osmo_pbit2ubit(bb_type5, (uint8_t *)&bb_rm3014_be, 30);

    // Build the actual burst
    build_sync_c_d_burst(burst, sb_type5, bb_type5, sb_type5); // Reuse sb_type5 for SI

    // Copy full burst to output buffer
    memcpy(bits, burst, 510);

    printf("[TMO BTS] Generated SYNC burst for slot %u/%u (510 symbols)\n", slot->fn, slot->tn);
    return 510;
}

// Creates SYSINFO-PDU content based on system parameters and type specification
int build_tmo_sysinfo_pdu(uint8_t *sysinfo_bits, struct timing_slot *slot, uint8_t sysinfo_type)
{
    // SYSINFO-PDU structure based on ETSI EN 300 392-2
    uint8_t sysinfo_packed[27]; // 216 bits = 27 bytes
    struct bitvec bv;

    memset(sysinfo_packed, 0, sizeof(sysinfo_packed));
    memset(&bv, 0, sizeof(bv));
    bv.data = sysinfo_packed;
    bv.data_len = sizeof(sysinfo_packed);

    // PDU Type (4 bits) - SYSINFO-PDU
    bitvec_set_uint(&bv, 0xC, 4); // 1100 = SYSINFO-PDU type

    // SYSINFO Type (4 bits)
    bitvec_set_uint(&bv, sysinfo_type, 4);

    switch (sysinfo_type)
    {
    case 1: // Main system information
        // MCC field (10 bits)
        bitvec_set_uint(&bv, TMO_MCC, 10);

        // MNC field (14 bits)
        bitvec_set_uint(&bv, TMO_MNC, 14);

        // Colour Code (6 bits)
        bitvec_set_uint(&bv, TMO_CC, 6);

        // Timeslot Number (2 bits)
        bitvec_set_uint(&bv, slot->tn, 2);

        // Frame Number (5 bits)
        bitvec_set_uint(&bv, slot->fn, 5);

        // Cell Reselection Parameters (16 bits)
        bitvec_set_uint(&bv, 0x1234, 16); // Example values

        // Access Parameter (8 bits)
        bitvec_set_uint(&bv, 0x0F, 8); // Allow all access classes

        // Radio Parameters (16 bits)
        bitvec_set_uint(&bv, 0x5678, 16); // Example radio parameters
        break;

    case 2: // Neighbor cell information
        // Number of neighbor cells (4 bits)
        bitvec_set_uint(&bv, 2, 4); // Example: 2 neighbor cells

        // Neighbor cell 1 (32 bits total)
        bitvec_set_uint(&bv, 4301, 16); // 430.1 MHz (reduced to fit 16 bits)
        bitvec_set_uint(&bv, 0x0001, 16);

        // Neighbor cell 2 (32 bits total)
        bitvec_set_uint(&bv, 4303, 16); // 430.3 MHz (reduced to fit 16 bits)
        bitvec_set_uint(&bv, 0x0002, 16);
        break;

    case 3: // Access and timing parameters
        // Registration Timer (8 bits) - minutes
        bitvec_set_uint(&bv, 30, 8); // 30 minutes

        // Common or Individual Channel (1 bit)
        bitvec_set_uint(&bv, 1, 1); // Common channel

        // Access Rights (8 bits)
        bitvec_set_uint(&bv, 0xFF, 8); // All rights enabled

        // Subscriber Class (4 bits)
        bitvec_set_uint(&bv, 0x0F, 4); // All subscriber classes

        // BS Service Details (16 bits)
        bitvec_set_uint(&bv, 0x9ABC, 16); // Service capabilities
        break;

    default:
        // Fill with pattern for unknown types
        bitvec_set_uint(&bv, 0xAAAA, 16); // Alternating pattern
        break;
    }

    // Convert packed bitvec result to unpacked bit array
    memset(sysinfo_bits, 0, 216);
    osmo_pbit2ubit(sysinfo_bits, sysinfo_packed, bv.cur_bit);

    printf("[TMO SYSINFO] Generated %d bits for SYSINFO type %d\n", bv.cur_bit, sysinfo_type);
    printf("[TMO SYSINFO] First 32 unpacked bits: ");
    for (int i = 0; i < 32 && i < bv.cur_bit; i++)
    {
        printf("%u", sysinfo_bits[i]);
    }
    printf("\n");

    return bv.cur_bit;
}

// Assembles BCCH burst containing system information broadcast messages
int build_tmo_bcch_burst(uint8_t *bits, struct timing_slot *slot, uint8_t sysinfo_type)
{
    // BCCH uses normal continuous downlink burst format
    uint8_t sysinfo_bits[216];
    uint8_t sysinfo_type2[27]; // Packed bytes
    uint8_t bkn2_type2[27];    // Block 2 (can be dummy or additional info)
    uint8_t sysinfo_master[216 * 4];
    uint8_t bkn2_master[216 * 4];
    uint8_t sysinfo_type3[216];
    uint8_t bkn2_type3[216];
    uint8_t sysinfo_type4[216];
    uint8_t bkn2_type4[216];
    uint8_t sysinfo_type5[216];
    uint8_t bkn2_type5[216];
    uint8_t bb_type5[30];
    uint8_t burst[255 * 2]; // Full TETRA burst (510 symbols)
    uint16_t crc;
    uint32_t bb_rm3014, bb_rm3014_be;

    // Build SYSINFO-PDU content
    printf("[TMO BCCH] Building SYSINFO type %u for slot %u/%u\n", sysinfo_type, slot->fn, slot->tn);
    int sysinfo_len = build_tmo_sysinfo_pdu(sysinfo_bits, slot, sysinfo_type);
    printf("[TMO BCCH] Built SYSINFO PDU with %d bits\n", sysinfo_len);

    // Add CRC to SYSINFO data
    crc = ~crc16_ccitt_bits(sysinfo_bits, sysinfo_len);
    crc = swap16(crc);
    printf("[TMO BCCH] Added CRC: 0x%04x\n", crc);

    // Pack the SYSINFO bits into bytes for processing (LSB-first to match TETRA air interface)
    memset(sysinfo_type2, 0, sizeof(sysinfo_type2));
    printf("[TMO BCCH] Packing SYSINFO bits LSB-first to preserve MCC/MNC encoding\n");
    for (int i = 0; i < (sysinfo_len + 16); i += 8)
    { // +16 for CRC
        uint8_t byte_val = 0;
        for (int j = 0; j < 8 && (i + j) < (sysinfo_len + 16); j++)
        {
            if ((i + j) < sysinfo_len)
            {
                byte_val |= (sysinfo_bits[i + j] << j); // LSB-first packing
            }
            else if ((i + j) < (sysinfo_len + 16))
            {
                // Add CRC bits LSB-first too
                int crc_bit_idx = (i + j) - sysinfo_len;
                byte_val |= (((crc >> crc_bit_idx) & 1) << j); // LSB-first CRC
            }
        }
        if ((i / 8) < sizeof(sysinfo_type2))
        {
            sysinfo_type2[i / 8] = byte_val;
        }
    }

    // Create dummy data for block 2
    memset(bkn2_type2, 0, sizeof(bkn2_type2));

    // Convert packed bytes to bit arrays
    uint8_t sysinfo_unpacked[216], bkn2_bits[216];
    osmo_pbit2ubit(sysinfo_unpacked, sysinfo_type2, 216);
    osmo_pbit2ubit(bkn2_bits, bkn2_type2, 216);

    // Process SYSINFO block: Run rate 2/3 RCPC code
    struct conv_enc_state *ces1 = calloc(1, sizeof(*ces1));
    conv_enc_init(ces1);
    conv_enc_input(ces1, sysinfo_unpacked, 216, sysinfo_master);
    get_punctured_rate(TETRA_RCPC_PUNCT_2_3, sysinfo_master, 216, sysinfo_type3);
    free(ces1);

    // Process block 2: Run rate 2/3 RCPC code
    struct conv_enc_state *ces2 = calloc(1, sizeof(*ces2));
    conv_enc_init(ces2);
    conv_enc_input(ces2, bkn2_bits, 216, bkn2_master);
    get_punctured_rate(TETRA_RCPC_PUNCT_2_3, bkn2_master, 216, bkn2_type3);
    free(ces2);

    // Run block interleaving for both blocks
    block_interleave(216, 11, sysinfo_type3, sysinfo_type4);
    block_interleave(216, 11, bkn2_type3, bkn2_type4);

    // Run scrambling for both blocks
    memcpy(sysinfo_type5, sysinfo_type4, 216);
    memcpy(bkn2_type5, bkn2_type4, 216);
    tetra_scramb_bits(SCRAMB_INIT, sysinfo_type5, 216);
    tetra_scramb_bits(SCRAMB_INIT, bkn2_type5, 216);

    // Create AACH content for BCCH with proper channel allocation info
    uint8_t aach_data[] = {0x02, 0x10}; // BCCH indicator + system info present
    bb_rm3014 = tetra_rm3014_compute(*(uint16_t *)aach_data);
    bb_rm3014_be = htonl(bb_rm3014);
    bb_rm3014_be <<= 2;
    osmo_pbit2ubit(bb_type5, (uint8_t *)&bb_rm3014_be, 30);

    // Debug: dump first few bytes of packed SYSINFO data
    printf("[TMO BCCH] Packed SYSINFO bytes (first 8, LSB-first): ");
    for (int i = 0; i < 8 && i < sizeof(sysinfo_type2); i++)
    {
        printf("%02x ", sysinfo_type2[i]);
    }
    printf("\n");

    // Build normal continuous downlink burst for TMO BCCH SYSINFO
    // BCCH uses Normal Burst (Type 1) per ETSI TS 100 392-2, Clause 7.1.1.1
    // Use two_log_chan=1 to use different training sequence (p_bits) for BCCH
    printf("[TMO BCCH] Using training sequence p_bits (two_log_chan=1) for BCCH\n");
    build_norm_c_d_burst(burst, sysinfo_type5, bb_type5, bkn2_type5, 1);

    // Copy full burst to output buffer
    memcpy(bits, burst, 510);

    printf("[TMO BCCH] Transmitted SYSINFO type %u burst (%d symbols)\n", sysinfo_type, 510);
    return 510;
}

// Generates DSCH dummy burst for maintaining continuous carrier transmission
int build_tmo_dsch_burst(uint8_t *bits, struct timing_slot *slot)
{
    // DSCH uses normal continuous downlink burst format
    // Carries dummy data to maintain continuous carrier
    uint8_t bkn1_type2[27]; // Block 1 data (216 bits = 27 bytes for dummy)
    uint8_t bkn2_type2[27]; // Block 2 data (216 bits = 27 bytes for dummy)
    uint8_t bkn1_master[216 * 4];
    uint8_t bkn2_master[216 * 4];
    uint8_t bkn1_type3[216];
    uint8_t bkn2_type3[216];
    uint8_t bkn1_type4[216];
    uint8_t bkn2_type4[216];
    uint8_t bkn1_type5[216];
    uint8_t bkn2_type5[216];
    uint8_t bb_type5[30];
    uint8_t burst[255 * 2]; // Full TETRA burst (510 symbols)
    uint32_t bb_rm3014, bb_rm3014_be;

    // Create dummy data blocks (all zeros)
    memset(bkn1_type2, 0, sizeof(bkn1_type2));
    memset(bkn2_type2, 0, sizeof(bkn2_type2));

    // Convert packed bytes to bit arrays
    uint8_t bkn1_bits[216], bkn2_bits[216];
    osmo_pbit2ubit(bkn1_bits, bkn1_type2, 216);
    osmo_pbit2ubit(bkn2_bits, bkn2_type2, 216);

    // Process block 1: Run rate 2/3 RCPC code
    struct conv_enc_state *ces1 = calloc(1, sizeof(*ces1));
    conv_enc_init(ces1);
    conv_enc_input(ces1, bkn1_bits, 216, bkn1_master);
    get_punctured_rate(TETRA_RCPC_PUNCT_2_3, bkn1_master, 216, bkn1_type3);
    free(ces1);

    // Process block 2: Run rate 2/3 RCPC code
    struct conv_enc_state *ces2 = calloc(1, sizeof(*ces2));
    conv_enc_init(ces2);
    conv_enc_input(ces2, bkn2_bits, 216, bkn2_master);
    get_punctured_rate(TETRA_RCPC_PUNCT_2_3, bkn2_master, 216, bkn2_type3);
    free(ces2);

    // Run block interleaving for both blocks
    block_interleave(216, 11, bkn1_type3, bkn1_type4);
    block_interleave(216, 11, bkn2_type3, bkn2_type4);

    // Run scrambling for both blocks
    memcpy(bkn1_type5, bkn1_type4, 216);
    memcpy(bkn2_type5, bkn2_type4, 216);
    tetra_scramb_bits(SCRAMB_INIT, bkn1_type5, 216);
    tetra_scramb_bits(SCRAMB_INIT, bkn2_type5, 216);

    // Create dummy AACH content
    uint8_t aach_data[] = {0x00, 0x00}; // Dummy AACH data
    bb_rm3014 = tetra_rm3014_compute(*(uint16_t *)aach_data);
    bb_rm3014_be = htonl(bb_rm3014);
    bb_rm3014_be <<= 2;
    osmo_pbit2ubit(bb_type5, (uint8_t *)&bb_rm3014_be, 30);

    // Build normal continuous downlink burst (not sync burst)
    build_norm_c_d_burst(burst, bkn1_type5, bb_type5, bkn2_type5, 0);

    // Copy full burst to output buffer
    memcpy(bits, burst, 510);

    return 510;
}

// Schedules TMO BTS burst transmission based on slot timing information
int mac_request_tx_buffer_content_tmo(uint8_t *bits, struct timing_slot *slot)
{
    int len = -1;

    printf("[TMO BTS] TX slot: %2u %2u %2u - ", slot->tn, slot->fn, slot->mn);

    // TMO BTS transmission logic with BCCH and continuous transmission
    // Implements BCCH system information and DSCH for continuous carrier

    // Transmit SYNC burst on slot 1 of frame 1 and 11 (BSCH slots)
    if (slot->tn == 1 && (slot->fn == 1 || slot->fn == 11))
    {
        printf("BSCH/SYNC - ");
        len = build_tmo_sync_burst(bits, slot);
        bsch_counter++;
    }
    // BCCH transmissions on specific control slots (slot 1)
    else if (slot->tn == 1)
    {
        // BCCH scheduling: transmit different SYSINFO types on different frames
        // Frame 3: SYSINFO type 1 (main system info)
        // Frame 6: SYSINFO type 2 (neighbor cells)
        // Frame 9: SYSINFO type 3 (access parameters)
        // Other frames: DSCH

        if (slot->fn == 3)
        {
            printf("BCCH/SYSINFO-1 - ");
            len = build_tmo_bcch_burst(bits, slot, 1); // Main system info
            bcch_counter++;
        }
        else if (slot->fn == 6)
        {
            printf("BCCH/SYSINFO-2 - ");
            len = build_tmo_bcch_burst(bits, slot, 2); // Neighbor cells
            bcch_counter++;
        }
        else if (slot->fn == 9)
        {
            printf("BCCH/SYSINFO-3 - ");
            len = build_tmo_bcch_burst(bits, slot, 3); // Access parameters
            bcch_counter++;
        }
        else
        {
            printf("Control/DSCH - ");
            len = build_tmo_dsch_burst(bits, slot);
        }
    }
    // Traffic slots (slots 2, 3, 4) - use DSCH for continuous transmission
    else
    {
        printf("Traffic/DSCH - ");
        len = build_tmo_dsch_burst(bits, slot);
    }

    printf("burst len: %d\n", len);
    return len;
}
