CFLAGS=-g -Wall `pkg-config --cflags libosmocore 2> /dev/null` -I. -I../../suo/libsuo
LDLIBS=`pkg-config --libs libosmocore 2> /dev/null` -losmocore -lzmq -lm
# These are only needed for hamtetra_main2:
LDLIBS += -lSoapySDR -lliquid -lasound

all: float_to_bits tunctl hamtetra_main2

debug: CFLAGS := -lasan $(CFLAGS) -fsanitize=address -fno-omit-frame-pointer -g -O0
debug: LDLIBS := -lasan $(LDLIBS)
debug: all

%.o: %.c
	$(CC) $(CFLAGS) -c $^ -o $@

libosmo-tetra-phy.a: phy/tetra_burst_sync.o phy/tetra_burst.o
	$(AR) r $@ $^

libosmo-tetra-mac.a: lower_mac/tetra_conv_enc.o lower_mac/tch_reordering.o tetra_tdma.o lower_mac/tetra_scramb.o lower_mac/tetra_rm3014.o lower_mac/tetra_interleave.o lower_mac/crc_simple.o tetra_common.o lower_mac/viterbi.o lower_mac/viterbi_cch.o lower_mac/viterbi_tch.o lower_mac/tetra_lower_mac.o tetra_upper_mac.o tetra_mac_pdu.o tetra_dll_pdu.o tetra_llc_pdu.o tetra_llc.o tetra_mle_pdu.o tetra_mm_pdu.o tetra_cmce_pdu.o tetra_sndcp_pdu.o tetra_gsmtap.o tuntap.o hamtetra_pdu_generator.o hamtetra_mac.o hamtetra_tmo_mac.o
	$(AR) r $@ $^

float_to_bits: float_to_bits.o

crc_test: crc_test.o tetra_common.o libosmo-tetra-mac.a

hamtetra_main2: hamtetra_main2.o hamtetra_timing.o hamtetra_slotter.o pdus.o libosmo-tetra-phy.a libosmo-tetra-mac.a ../../suo/libsuo/build/libsuo-dsp.a ../../suo/libsuo/build/libsuo-io.a

conv_enc_test: conv_enc_test.o testpdu.o libosmo-tetra-phy.a libosmo-tetra-mac.a

tunctl: tunctl.o

clean:
	@rm -f tunctl float_to_bits crc_test tetra-rx tetra-main hamtetra_main2 conv_enc_test *.o phy/*.o lower_mac/*.o *.a

.PHONY: clean
