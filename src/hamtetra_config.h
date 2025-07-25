#ifndef HAMTETRA_CONFIG_H
#define HAMTETRA_CONFIG_H

#define REP_MCC 244      // DM-REP Mobile Country Code
#define REP_MNC 2        // DM-REP Mobile Network Code
#define REP_ADDRESS 1099 // DM-REP Address - over-the-air 10-bit long, so least bits will be used from value over 1024

#define DN232 3 // number of frames to transmit DM-SETUP (PRES) => 2
#define DN233 1 // number of frames to transmit DSB heading a DM-SDS UDATA/DATA => 2
#define DN253 2 // number of frames in with DM-REP transmits  the free-channel presence signal (2-4)
#define DT254 2 // Presence signal every 2 multiframes

#define TMO_CC 1      // Colour Code
#define TMO_MCC 1023  // Mobile Country Code
#define TMO_MNC 16383 // Mobile Network Code
#define TMO_LA 1      // Location Area

#endif
