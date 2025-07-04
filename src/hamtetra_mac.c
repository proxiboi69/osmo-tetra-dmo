/*  Contains functionality to implement the business logic of DM MAC layer 
 *
 */

#include <osmocom/core/talloc.h>
#include <osmocom/core/bits.h>
#include <arpa/inet.h>

#include "hamtetra_config.h"
#include "hamtetra_mac.h"
#include "hamtetra_pdu_generator.h"
#include "lower_mac/crc_simple.h"
#include "lower_mac/tetra_conv_enc.h"
#include "lower_mac/tetra_interleave.h"
#include "lower_mac/tetra_scramb.h"
#include "lower_mac/tetra_rm3014.h"
#include "phy/tetra_burst.h"
#include "phy/tetra_burst_sync.h"

void *tetra_tall_ctx;

struct tetra_mac_state *tms;
struct tetra_tdma_time time;


// timeslot struct
struct timeslot {
    uint8_t fn;             // frame 1-18
    uint8_t tn;             // timeslot 1-4
    uint8_t slave_fn;       // slave link frame nr
    uint8_t slave_tn;       // slave link timeslot nr
    int len;                // length of burst
    uint8_t delayed;        // send n. multiframes later
    uint8_t burst[512];     // burst bit-per-byte encoded 
};

// multiframe buffer, 18 frames with 4 timeslots = 72 timeslots
struct multiframe {
    struct timeslot tn[72]; 
};

// global frame buffer to both threads
struct multiframe frame_buf_master;
struct multiframe frame_buf_sent;

// initialize the global framebuffer with timeslot numbers and timestamps
void initialize_framebuffer(uint64_t base_time_ts)
{
    for (int i = 0; i<72; i++) {
        struct timeslot *tn = &frame_buf_master.tn[i];
        tn->tn = (i%4) + 1;
        tn->fn = floor( (float)i/4 ) + 1;
        tn->slave_tn = ((i-3)%4) + 1;
        tn->slave_fn = floor( (float)(i-3)/4 ) + 1;
        tn->len = 0;
        tn->delayed = 0;
    }


    for (int i = 0; i<72; i++) {
        struct timeslot *tn = &frame_buf_sent.tn[i];
        tn->tn = (i%4) + 1;
        tn->fn = floor( (float)i/4 ) + 1;
        tn->slave_tn = ((i-3)%4) + 1;
        tn->slave_fn = floor( (float)(i-3)/4 ) + 1;
        tn->len = -1;
        tn->delayed = 0;
    }

};

// set next start timestamp of a timeslot for next round
void reprime_timeslot(uint8_t slotnum) {
    struct timeslot *tn = &frame_buf_master.tn[slotnum];
    if (tn->delayed>0) {
        tn->delayed--;
    } else {
        tn->len = 0;
    }

};

void sent_buffer_set(struct timing_slot *slot, int len) {
    uint8_t slotnum = (4*(slot->fn-1))+(slot->tn-1);
    frame_buf_sent.tn[slotnum].len = len;
}

int sent_buffer_get(struct timing_slot *slot) {
    uint8_t slotnum = (4*(slot->fn-1))+(slot->tn-1);
    return frame_buf_sent.tn[slotnum].len;
}


// lower mac wants to send a burst
void dp_sap_udata_req(enum dp_sap_data_type type, const uint8_t *bits, unsigned int len, struct tetra_tdma_time tdma_time, struct tetra_mac_state *tms_req)
{
    uint8_t slotnum = (4*(tdma_time.fn-1))+(tdma_time.tn-1);
    if (tdma_time.link == DM_LINK_SLAVE) {
        slotnum = (slotnum+3) % 72;
    }
    printf("BURST OUT - scheduled to %d link buffer slot %d/%d (%d)\n", tdma_time.link, tdma_time.fn, tdma_time.tn, slotnum);
    struct timeslot *burst_slot;
    burst_slot = &frame_buf_master.tn[slotnum];
    memcpy(burst_slot->burst, bits, len);
    burst_slot->len = len;

    if (tms_req->channel_state != tms->channel_state) {
        tms->channel_state = tms_req->channel_state;
    }
    if (tms_req->channel_state_last_chg != tms->channel_state_last_chg) {
        tms->channel_state_last_chg = tms_req->channel_state_last_chg;
    }

}

void dpc_sap_udata_req(struct tetra_mac_state *tms_req)
{
    if (tms_req->channel_state != tms->channel_state) {
        tms->channel_state = tms_req->channel_state;
    }
    if (tms_req->channel_state_last_chg != tms->channel_state_last_chg) {
        tms->channel_state_last_chg = tms_req->channel_state_last_chg;
    }

}


unsigned int multiframe_counter = 0;
uint8_t multiframes_since_last_presence = 0;
int presence_signal_counter = 0;

// TMO BTS specific variables
static enum tetra_infrastructure_mode operating_mode = TETRA_INFRA_DMO; // Default to DMO
static uint32_t bsch_counter = 0;
static uint32_t sync_burst_counter = 0;
static uint32_t bcch_counter = 0;

#define swap16(x) ((x)<<8)|((x)>>8)

void mac_hamtetra_init()
{
    // initialize MAC state structure
	tms = talloc_zero(tetra_tall_ctx, struct tetra_mac_state);
	tetra_mac_state_init(tms);
    tms->channel_state = DM_CHANNEL_S_DMREP_IDLE_UNKNOWN;

    // initialiaze send buffer
    initialize_framebuffer(0);

    presence_signal_counter = presence_signal_multiframe_count[DT254]*18*4;
    
    // Initialize TMO-specific counters
    bsch_counter = 0;
    sync_burst_counter = 0;
    bcch_counter = 0;
}

void mac_hamtetra_set_mode(enum tetra_infrastructure_mode mode)
{
    operating_mode = mode;
    if (mode == TETRA_INFRA_TMO) {
        printf("MAC: Setting TMO BTS mode\n");
        tms->infra_mode = TETRA_INFRA_TMO;
    } else {
        printf("MAC: Setting DMO mode\n");
        tms->infra_mode = TETRA_INFRA_DMO;
    }
}

// Build proper SYNC-PDU content for TMO BTS
int build_tmo_sync_pdu(uint8_t *sync_pdu_bits, struct timing_slot *slot)
{
    // SYNC-PDU structure (60 bits total)
    // Based on ETSI EN 300 392-2 clause 18.4.2
    memset(sync_pdu_bits, 0, 60);
    
    int bit_pos = 0;
    
    // System Code (4 bits) - indicates TMO system
    sync_pdu_bits[bit_pos++] = 0; // TMO system
    sync_pdu_bits[bit_pos++] = 0;
    sync_pdu_bits[bit_pos++] = 0;
    sync_pdu_bits[bit_pos++] = 1;
    
    // Colour Code (6 bits) - network color code
    for (int i = 0; i < 6; i++) {
        sync_pdu_bits[bit_pos++] = (1 >> (5-i)) & 1; // Color code = 1
    }
    
    // Timeslot Number (2 bits)
    sync_pdu_bits[bit_pos++] = (slot->tn >> 1) & 1;
    sync_pdu_bits[bit_pos++] = slot->tn & 1;
    
    // Frame Number (5 bits)
    for (int i = 0; i < 5; i++) {
        sync_pdu_bits[bit_pos++] = (slot->fn >> (4-i)) & 1;
    }
    
    // Multiframe Number (6 bits)
    for (int i = 0; i < 6; i++) {
        sync_pdu_bits[bit_pos++] = (slot->mn >> (5-i)) & 1;
    }
    
    // MCC (10 bits) - Mobile Country Code (example: 262 for Germany)
    uint16_t mcc = 262;
    for (int i = 0; i < 10; i++) {
        sync_pdu_bits[bit_pos++] = (mcc >> (9-i)) & 1;
    }
    
    // MNC (14 bits) - Mobile Network Code (example: 42)
    uint16_t mnc = 42;
    for (int i = 0; i < 14; i++) {
        sync_pdu_bits[bit_pos++] = (mnc >> (13-i)) & 1;
    }
    
    // LA (14 bits) - Location Area (example: 1)
    uint16_t la = 1;
    for (int i = 0; i < 14; i++) {
        sync_pdu_bits[bit_pos++] = (la >> (13-i)) & 1;
    }
    
    // Reserved bits (fill remaining)
    while (bit_pos < 60) {
        sync_pdu_bits[bit_pos++] = 0;
    }
    
    return 60;
}

// Build SYSINFO-PDU content for BCCH
int build_tmo_sysinfo_pdu(uint8_t *sysinfo_bits, struct timing_slot *slot, uint8_t sysinfo_type)
{
    // SYSINFO-PDU structure based on ETSI EN 300 392-2
    memset(sysinfo_bits, 0, 216);  // 216 bits for block data
    
    int bit_pos = 0;
    
    // PDU Type (4 bits) - SYSINFO-PDU
    sysinfo_bits[bit_pos++] = 1;  // SYSINFO-PDU type
    sysinfo_bits[bit_pos++] = 1;
    sysinfo_bits[bit_pos++] = 0;
    sysinfo_bits[bit_pos++] = 0;
    
    // SYSINFO Type (4 bits)
    for (int i = 0; i < 4; i++) {
        sysinfo_bits[bit_pos++] = (sysinfo_type >> (3-i)) & 1;
    }
    
    switch (sysinfo_type) {
        case 1: // Main system information
            // MCC (10 bits) - Mobile Country Code
            uint16_t mcc = 262;  // Germany
            for (int i = 0; i < 10; i++) {
                sysinfo_bits[bit_pos++] = (mcc >> (9-i)) & 1;
            }
            
            // MNC (14 bits) - Mobile Network Code
            uint16_t mnc = 42;
            for (int i = 0; i < 14; i++) {
                sysinfo_bits[bit_pos++] = (mnc >> (13-i)) & 1;
            }
            
            // Colour Code (6 bits)
            for (int i = 0; i < 6; i++) {
                sysinfo_bits[bit_pos++] = (1 >> (5-i)) & 1;  // Color code = 1
            }
            
            // Timeslot Number (2 bits)
            sysinfo_bits[bit_pos++] = (slot->tn >> 1) & 1;
            sysinfo_bits[bit_pos++] = slot->tn & 1;
            
            // Frame Number (5 bits)
            for (int i = 0; i < 5; i++) {
                sysinfo_bits[bit_pos++] = (slot->fn >> (4-i)) & 1;
            }
            
            // Cell Reselection Parameters (16 bits)
            uint16_t cell_resel = 0x1234;  // Example values
            for (int i = 0; i < 16; i++) {
                sysinfo_bits[bit_pos++] = (cell_resel >> (15-i)) & 1;
            }
            
            // Access Parameter (8 bits)
            uint8_t access_param = 0x0F;  // Allow all access classes
            for (int i = 0; i < 8; i++) {
                sysinfo_bits[bit_pos++] = (access_param >> (7-i)) & 1;
            }
            
            // Radio Parameters (16 bits)
            uint16_t radio_param = 0x5678;  // Example radio parameters
            for (int i = 0; i < 16; i++) {
                sysinfo_bits[bit_pos++] = (radio_param >> (15-i)) & 1;
            }
            break;
            
        case 2: // Neighbor cell information
            // Number of neighbor cells (4 bits)
            uint8_t num_neighbors = 2;  // Example: 2 neighbor cells
            for (int i = 0; i < 4; i++) {
                sysinfo_bits[bit_pos++] = (num_neighbors >> (3-i)) & 1;
            }
            
            // Neighbor cell 1 (32 bits total)
            uint16_t neighbor1_freq = 430100;  // 430.1 MHz in kHz
            for (int i = 0; i < 16; i++) {
                sysinfo_bits[bit_pos++] = (neighbor1_freq >> (15-i)) & 1;
            }
            uint16_t neighbor1_id = 0x0001;
            for (int i = 0; i < 16; i++) {
                sysinfo_bits[bit_pos++] = (neighbor1_id >> (15-i)) & 1;
            }
            
            // Neighbor cell 2 (32 bits total)
            uint16_t neighbor2_freq = 430300;  // 430.3 MHz in kHz
            for (int i = 0; i < 16; i++) {
                sysinfo_bits[bit_pos++] = (neighbor2_freq >> (15-i)) & 1;
            }
            uint16_t neighbor2_id = 0x0002;
            for (int i = 0; i < 16; i++) {
                sysinfo_bits[bit_pos++] = (neighbor2_id >> (15-i)) & 1;
            }
            break;
            
        case 3: // Access and timing parameters
            // Registration Timer (8 bits) - minutes
            uint8_t reg_timer = 30;  // 30 minutes
            for (int i = 0; i < 8; i++) {
                sysinfo_bits[bit_pos++] = (reg_timer >> (7-i)) & 1;
            }
            
            // Common or Individual Channel (1 bit)
            sysinfo_bits[bit_pos++] = 1;  // Common channel
            
            // Access Rights (8 bits)
            uint8_t access_rights = 0xFF;  // All rights enabled
            for (int i = 0; i < 8; i++) {
                sysinfo_bits[bit_pos++] = (access_rights >> (7-i)) & 1;
            }
            
            // Subscriber Class (4 bits)
            uint8_t sub_class = 0x0F;  // All subscriber classes
            for (int i = 0; i < 4; i++) {
                sysinfo_bits[bit_pos++] = (sub_class >> (3-i)) & 1;
            }
            
            // BS Service Details (16 bits)
            uint16_t bs_service = 0x9ABC;  // Service capabilities
            for (int i = 0; i < 16; i++) {
                sysinfo_bits[bit_pos++] = (bs_service >> (15-i)) & 1;
            }
            break;
            
        default:
            // Fill with pattern for unknown types
            for (int i = bit_pos; i < 200; i++) {
                sysinfo_bits[i] = (i % 2);  // Alternating pattern
            }
            bit_pos = 200;
            break;
    }
    
    // Fill remaining bits with zeros or padding
    while (bit_pos < 200) {  // Leave room for CRC
        sysinfo_bits[bit_pos++] = 0;
    }
    
    return bit_pos;
}

// TMO BTS specific function to build SYNC and BSCH bursts
int build_tmo_sync_burst(uint8_t *bits, struct timing_slot *slot)
{
    // Enhanced TMO BTS implementation with proper SYNC-PDU
    // Based on conv_enc_test.c build_sb() function
    uint8_t sb_type2[80];
    uint8_t sb_master[80*4];
    uint8_t sb_type3[120];
    uint8_t sb_type4[120];
    uint8_t sb_type5[120];
    uint8_t bb_type5[30];
    uint8_t burst[255*2];  // Full TETRA burst (510 symbols)
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
    for (int i = 0; i < 8; i++) {
        sync_pdu_packed[i] = 0;
        for (int j = 0; j < 8 && (i*8+j) < 60; j++) {
            sync_pdu_packed[i] |= (sync_pdu_bits[i*8+j] << (7-j));
        }
    }
    
    cur += osmo_pbit2ubit(sb_type2, sync_pdu_packed, 60);
    
    crc = ~crc16_ccitt_bits(sb_type2, 60);
    crc = swap16(crc);
    cur += osmo_pbit2ubit(cur, (uint8_t *) &crc, 16);
    
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
    uint8_t aach_data[] = {0x00, 0x01};  // Basic AACH data
    bb_rm3014 = tetra_rm3014_compute(*(uint16_t *)aach_data);
    bb_rm3014_be = htonl(bb_rm3014);
    bb_rm3014_be <<= 2;
    osmo_pbit2ubit(bb_type5, (uint8_t *) &bb_rm3014_be, 30);
    
    // Build the actual burst
    build_sync_c_d_burst(burst, sb_type5, bb_type5, sb_type5); // Reuse sb_type5 for SI
    
    // TETRA burst structure requires 510 symbols total
    // The burst array is already sized for 510 symbols (255*2)
    // Copy full burst to output buffer
    memcpy(bits, burst, 510);
    
    printf("[TMO BTS] Generated SYNC burst for slot %u/%u (510 symbols)\n", slot->fn, slot->tn);
    return 510;
}

// Build DSCH (Dummy Synchronization Channel) burst for continuous transmission
int build_tmo_dsch_burst(uint8_t *bits, struct timing_slot *slot)
{
    // DSCH uses normal continuous downlink burst format
    // Carries dummy data to maintain continuous carrier
    uint8_t bkn1_type2[27];  // Block 1 data (216 bits = 27 bytes for dummy)
    uint8_t bkn2_type2[27];  // Block 2 data (216 bits = 27 bytes for dummy)
    uint8_t bkn1_master[216*4];
    uint8_t bkn2_master[216*4];
    uint8_t bkn1_type3[216];
    uint8_t bkn2_type3[216];
    uint8_t bkn1_type4[216];
    uint8_t bkn2_type4[216];
    uint8_t bkn1_type5[216];
    uint8_t bkn2_type5[216];
    uint8_t bb_type5[30];
    uint8_t burst[255*2];  // Full TETRA burst (510 symbols)
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
    uint8_t aach_data[] = {0x00, 0x00};  // Dummy AACH data
    bb_rm3014 = tetra_rm3014_compute(*(uint16_t *)aach_data);
    bb_rm3014_be = htonl(bb_rm3014);
    bb_rm3014_be <<= 2;
    osmo_pbit2ubit(bb_type5, (uint8_t *) &bb_rm3014_be, 30);
    
    // Build normal continuous downlink burst (not sync burst)
    build_norm_c_d_burst(burst, bkn1_type5, bb_type5, bkn2_type5, 0);
    
    // Copy full burst to output buffer
    memcpy(bits, burst, 510);
    
    return 510;
}

// Build BCCH (Broadcast Control Channel) burst for system information
int build_tmo_bcch_burst(uint8_t *bits, struct timing_slot *slot, uint8_t sysinfo_type)
{
    // BCCH uses normal continuous downlink burst format
    // Carries SYSINFO-PDU content for network information
    uint8_t sysinfo_bits[216];
    uint8_t sysinfo_type2[27];  // Packed bytes
    uint8_t bkn2_type2[27];     // Block 2 (can be dummy or additional info)
    uint8_t sysinfo_master[216*4];
    uint8_t bkn2_master[216*4];
    uint8_t sysinfo_type3[216];
    uint8_t bkn2_type3[216];
    uint8_t sysinfo_type4[216];
    uint8_t bkn2_type4[216];
    uint8_t sysinfo_type5[216];
    uint8_t bkn2_type5[216];
    uint8_t bb_type5[30];
    uint8_t burst[255*2];  // Full TETRA burst (510 symbols)
    uint16_t crc;
    uint32_t bb_rm3014, bb_rm3014_be;
    
    // Build SYSINFO-PDU content
    int sysinfo_len = build_tmo_sysinfo_pdu(sysinfo_bits, slot, sysinfo_type);
    
    // Add CRC to SYSINFO data
    crc = ~crc16_ccitt_bits(sysinfo_bits, sysinfo_len);
    crc = swap16(crc);
    
    // Pack the SYSINFO bits into bytes for processing
    memset(sysinfo_type2, 0, sizeof(sysinfo_type2));
    for (int i = 0; i < (sysinfo_len + 16); i += 8) {  // +16 for CRC
        uint8_t byte_val = 0;
        for (int j = 0; j < 8 && (i + j) < sysinfo_len; j++) {
            byte_val |= (sysinfo_bits[i + j] << (7 - j));
        }
        if (i < 16) { // Add CRC bits
            for (int j = 0; j < 8 && (i + j) < 16; j++) {
                byte_val |= (((crc >> (15 - (i + j))) & 1) << (7 - j));
            }
        }
        if ((i / 8) < sizeof(sysinfo_type2)) {
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
    
    // Create AACH content for BCCH
    uint8_t aach_data[] = {0x01, 0x00};  // BCCH indicator in AACH
    bb_rm3014 = tetra_rm3014_compute(*(uint16_t *)aach_data);
    bb_rm3014_be = htonl(bb_rm3014);
    bb_rm3014_be <<= 2;
    osmo_pbit2ubit(bb_type5, (uint8_t *) &bb_rm3014_be, 30);
    
    // Build normal continuous downlink burst with SYSINFO content
    build_norm_c_d_burst(burst, sysinfo_type5, bb_type5, bkn2_type5, 0);
    
    // Copy full burst to output buffer
    memcpy(bits, burst, 510);
    
    return 510;
}

int mac_request_tx_buffer_content_tmo(uint8_t *bits, struct timing_slot *slot)
{
    int len = -1;
    
    printf("[TMO BTS] TX slot: %2u %2u %2u - ", slot->tn, slot->fn, slot->mn);
    
    // TMO BTS transmission logic with BCCH and continuous transmission
    // Implements BCCH system information and DSCH for continuous carrier
    
    // Transmit SYNC burst on slot 1 of frame 1 and 11 (BSCH slots)
    if (slot->tn == 1 && (slot->fn == 1 || slot->fn == 11)) {
        printf("BSCH/SYNC - ");
        len = build_tmo_sync_burst(bits, slot);
        bsch_counter++;
    }
    // BCCH transmissions on specific control slots (slot 1)
    else if (slot->tn == 1) {
        // BCCH scheduling: transmit different SYSINFO types on different frames
        // Frame 3: SYSINFO type 1 (main system info)
        // Frame 6: SYSINFO type 2 (neighbor cells)
        // Frame 9: SYSINFO type 3 (access parameters)
        // Other frames: DSCH
        
        if (slot->fn == 3) {
            printf("BCCH/SYSINFO-1 - ");
            len = build_tmo_bcch_burst(bits, slot, 1);  // Main system info
            bcch_counter++;
        }
        else if (slot->fn == 6) {
            printf("BCCH/SYSINFO-2 - ");
            len = build_tmo_bcch_burst(bits, slot, 2);  // Neighbor cells
            bcch_counter++;
        }
        else if (slot->fn == 9) {
            printf("BCCH/SYSINFO-3 - ");
            len = build_tmo_bcch_burst(bits, slot, 3);  // Access parameters
            bcch_counter++;
        }
        else {
            printf("Control/DSCH - ");
            len = build_tmo_dsch_burst(bits, slot);
        }
    }
    // Traffic slots (slots 2, 3, 4) - use DSCH for continuous transmission
    else {
        printf("Traffic/DSCH - ");
        len = build_tmo_dsch_burst(bits, slot);
    }
    
    printf("burst len: %d\n", len);
    return len;
}

int mac_request_tx_buffer_content(uint8_t *bits, struct timing_slot *slot)
{
    uint8_t slotnum = (4*(slot->fn-1))+(slot->tn-1);
    int len = -1;

    tms->channel_state_last_chg += 1; // increment last change counter

    // Handle TMO BTS mode
    if (operating_mode == TETRA_INFRA_TMO) {
        return mac_request_tx_buffer_content_tmo(bits, slot);
    }

    // channel house keeping for DMO mode
    switch (tms->channel_state) {
        case DM_CHANNEL_S_DMREP_IDLE_UNKNOWN:
            printf("[DM_CHANNEL_S_DMREP_IDLE_UNKNOWN] last chg: %ld - TX slot: %2u %2u %2u", tms->channel_state_last_chg, slot->tn, slot->fn, slot->mn);

            // change channel state to FREE if it has been idle for two multiframes since become UNKNOWN
            if (tms->channel_state_last_chg > 143) {
                tms->channel_state = DM_CHANNEL_S_DMREP_IDLE_FREE;
                tms->channel_state_last_chg = 0;
                presence_signal_counter = presence_signal_multiframe_count[DT254]*18*4;
            }
            break;

        case DM_CHANNEL_S_DMREP_IDLE_FREE:
            printf("[DM_CHANNEL_S_DMREP_IDLE_FREE] last chg: %ld - TX slot: %2u %2u %2u ", tms->channel_state_last_chg, slot->tn, slot->fn, slot->mn);

            // send DPRES-SYNC presence signal burst periodically on IDLE channel
            if (--presence_signal_counter < (DN253*4)){
                printf(" DPRES IDLE - ");
                uint8_t countdown = (presence_signal_counter-1) / 4;
                len = build_pdu_dpress_sync(slot->fn, slot->tn, DM_LINK_SLAVE, countdown, 0, bits);
                if (presence_signal_counter == 0) {
                    presence_signal_counter = presence_signal_multiframe_count[DT254]*18*4;
                }
                sent_buffer_set(slot, len);
                printf(" - burst len: %d\n", len);
                return len;
            }
            break;

        case DM_CHANNEL_S_DMREP_ACTIVE_OCCUPIED:
            printf("[DM_CHANNEL_S_DMREP_ACTIVE_OCCUPIED] last chg: %ld - TX slot: %2u %2u %2u", tms->channel_state_last_chg, slot->tn, slot->fn, slot->mn);

            // Send DM-REP presence signal in DSB in slot 3 at frames 1, 7 and 13 (master-link)
            if (frame_buf_master.tn[slotnum].len < 1) {
                if ((slot->fn == 1 || slot->fn == 7 || slot->fn==13) && slot->tn == 3) {
                    printf(" DPRES OCC - ");
                    len = build_pdu_dpress_sync(slot->fn, slot->tn, DM_LINK_MASTER, 0, 1, bits);
                    sent_buffer_set(slot, len);
                    printf(" - burst len: %d\n", len);
                    return len;
                }

            }
            break;

        case DM_CHANNEL_S_DMREP_ACTIVE_RESERVED:
            printf("[DM_CHANNEL_S_DMREP_ACTIVE_RESERVED] last chg: %ld - TX slot: %2u %2u %2u", tms->channel_state_last_chg, slot->tn, slot->fn, slot->mn);

            // Channel in reserved state over 5 multiframes without any bursts? Most probably an error so reset state back to IDLE
            if (tms->channel_state_last_chg>360) {
                tms->channel_state = DM_CHANNEL_S_DMREP_IDLE_FREE;
                tms->channel_state_last_chg = 0;
       			tms->cur_burst.is_traffic = 0;
                tms->mode_of_operation = DM_MAC_MODE_SYNC_SIGNALLING;

            }
            break;


        default:
            printf("[STATE %d] last chg: %ld - TX slot: %2u %2u %2u", tms->channel_state, tms->channel_state_last_chg, slot->tn, slot->fn, slot->mn); 
            break;

    }

    // buffer contains something to send
    if (frame_buf_master.tn[slotnum].len > 0){
        if (frame_buf_master.tn[slotnum].delayed > 0) {
            frame_buf_master.tn[slotnum].delayed--;
        } else {
            struct timeslot *burst_slot;
            burst_slot = &frame_buf_master.tn[slotnum];
            memcpy(bits, burst_slot->burst, burst_slot->len);
            len = burst_slot->len;
            burst_slot->len = 0; // clear the sent
        }
    } 
    
    sent_buffer_set(slot,len);

    printf(" - burst len: %d\n", len);
    return len;
}

/* intermediate function between PHY and lower mac DP-SAP unitdata indicate function to maintain state of the channel and filter echos */
void mac_dp_sap_udata_ind_filter(enum dp_sap_data_type type, const uint8_t *bits, unsigned int len, void *priv)
{
    struct tetra_mac_state *tms = priv;
    struct timing_slot *slot = tms->slot;
    int flen = sent_buffer_get(slot);
    // if everything good, pass parameter through
    if (flen < 0 || slot->changed==1) {
        // printf("mac_dp_sap_udata_ind_filter says hello (%d/%d:%d)\n", slot->fn, slot->tn, flen);
        dp_sap_udata_ind(type, bits, len, priv);
    } else {
        printf("[MAC FILTER] burst filtered (%d/%d:%d)\n", slot->fn, slot->tn, flen);

    }

}
