#ifndef HAMTETRA_TMO_MAC_H
#define HAMTETRA_TMO_MAC_H

#include "hamtetra_timing.h"
#include <stdint.h>

void tmo_mac_init(void);

int build_tmo_sync_pdu(uint8_t *sync_pdu_bits, struct timing_slot *slot);
int build_tmo_sysinfo_pdu(uint8_t *sysinfo_bits, struct timing_slot *slot, uint8_t sysinfo_type);
int build_tmo_bcch_burst(uint8_t *bits, struct timing_slot *slot, uint8_t sysinfo_type);
int build_tmo_dsch_burst(uint8_t *bits, struct timing_slot *slot);
int mac_request_tx_buffer_content_tmo(uint8_t *bits, struct timing_slot *slot);

#endif
