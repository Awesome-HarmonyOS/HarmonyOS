
# To compile on SunOS: add "-lsocket -lnsl" to LDFLAGS
# To compile with PKCS11: add "-lpkcs11-helper" to LDFLAGS

CFLAGS	?= -O2
WARNING_CFLAGS ?= -Wall -W -Wdeclaration-after-statement -Wno-unused-function -Wno-unused-value
LDFLAGS ?=

LOCAL_CFLAGS = $(WARNING_CFLAGS) -I../include -D_FILE_OFFSET_BITS=64
LOCAL_LDFLAGS = -L../library 			\
		-lmbedtls$(SHARED_SUFFIX)	\
		-lmbedx509$(SHARED_SUFFIX)	\
		-lmbedcrypto$(SHARED_SUFFIX)

ifndef SHARED
DEP=../library/libmbedcrypto.a ../library/libmbedx509.a ../library/libmbedtls.a
else
DEP=../library/libmbedcrypto.$(DLEXT) ../library/libmbedx509.$(DLEXT) ../library/libmbedtls.$(DLEXT)
endif

ifdef DEBUG
LOCAL_CFLAGS += -g3
endif

# if we're running on Windows, build for Windows
ifdef WINDOWS
WINDOWS_BUILD=1
endif

ifdef WINDOWS_BUILD
DLEXT=dll
EXEXT=.exe
LOCAL_LDFLAGS += -lws2_32
ifdef SHARED
SHARED_SUFFIX=.$(DLEXT)
endif
else
DLEXT=so
EXEXT=
SHARED_SUFFIX=
endif

# Zlib shared library extensions:
ifdef ZLIB
LOCAL_LDFLAGS += -lz
endif

APPS =	test_suite_aes.ecb$(EXEXT)	test_suite_aes.cbc$(EXEXT)	\
	test_suite_aes.cfb$(EXEXT)	test_suite_aes.rest$(EXEXT)	\
	test_suite_arc4$(EXEXT)		test_suite_asn1write$(EXEXT)	\
	test_suite_base64$(EXEXT)	test_suite_blowfish$(EXEXT)	\
	test_suite_camellia$(EXEXT)	test_suite_ccm$(EXEXT)		\
	test_suite_cmac$(EXEXT)						\
	test_suite_cipher.aes$(EXEXT)					\
	test_suite_cipher.arc4$(EXEXT)	test_suite_cipher.ccm$(EXEXT)	\
	test_suite_cipher.gcm$(EXEXT)					\
	test_suite_cipher.blowfish$(EXEXT)				\
	test_suite_cipher.camellia$(EXEXT)				\
	test_suite_cipher.des$(EXEXT)	test_suite_cipher.null$(EXEXT)	\
	test_suite_cipher.padding$(EXEXT)				\
	test_suite_ctr_drbg$(EXEXT)	test_suite_debug$(EXEXT)	\
	test_suite_des$(EXEXT)		test_suite_dhm$(EXEXT)		\
	test_suite_ecdh$(EXEXT)		test_suite_ecdsa$(EXEXT)	\
	test_suite_ecjpake$(EXEXT)	test_suite_ecp$(EXEXT)		\
	test_suite_error$(EXEXT)	test_suite_entropy$(EXEXT)	\
	test_suite_gcm.aes128_de$(EXEXT)				\
	test_suite_gcm.aes192_de$(EXEXT)				\
	test_suite_gcm.aes256_de$(EXEXT)				\
	test_suite_gcm.aes128_en$(EXEXT)				\
	test_suite_gcm.aes192_en$(EXEXT)				\
	test_suite_gcm.aes256_en$(EXEXT)				\
	test_suite_gcm.camellia$(EXEXT)					\
	test_suite_hmac_drbg.misc$(EXEXT)				\
	test_suite_hmac_drbg.no_reseed$(EXEXT)				\
	test_suite_hmac_drbg.nopr$(EXEXT)				\
	test_suite_hmac_drbg.pr$(EXEXT)					\
	test_suite_md$(EXEXT)		test_suite_mdx$(EXEXT)		\
	test_suite_memory_buffer_alloc$(EXEXT)				\
	test_suite_mpi$(EXEXT)						\
	test_suite_pem$(EXEXT)			test_suite_pkcs1_v15$(EXEXT)	\
	test_suite_pkcs1_v21$(EXEXT)	test_suite_pkcs5$(EXEXT)	\
	test_suite_pkparse$(EXEXT)	test_suite_pkwrite$(EXEXT)	\
	test_suite_pk$(EXEXT)						\
	test_suite_rsa$(EXEXT)		test_suite_shax$(EXEXT)		\
	test_suite_ssl$(EXEXT)		test_suite_timing$(EXEXT)			\
	test_suite_x509parse$(EXEXT)	test_suite_x509write$(EXEXT)	\
	test_suite_xtea$(EXEXT)		test_suite_version$(EXEXT)

.SILENT:

.PHONY: all check test clean

all: $(APPS)

$(DEP):
	$(MAKE) -C ../library

# invoke perl explicitly for the sake of mingw32-make

test_suite_aes.ecb.c : suites/test_suite_aes.function suites/test_suite_aes.ecb.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_aes test_suite_aes.ecb

test_suite_aes.cbc.c : suites/test_suite_aes.function suites/test_suite_aes.cbc.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_aes test_suite_aes.cbc

test_suite_aes.cfb.c : suites/test_suite_aes.function suites/test_suite_aes.cfb.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_aes test_suite_aes.cfb

test_suite_aes.rest.c : suites/test_suite_aes.function suites/test_suite_aes.rest.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_aes test_suite_aes.rest

test_suite_cipher.aes.c : suites/test_suite_cipher.function suites/test_suite_cipher.aes.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_cipher test_suite_cipher.aes

test_suite_cipher.arc4.c : suites/test_suite_cipher.function suites/test_suite_cipher.arc4.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_cipher test_suite_cipher.arc4

test_suite_cipher.ccm.c : suites/test_suite_cipher.function suites/test_suite_cipher.ccm.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_cipher test_suite_cipher.ccm

test_suite_cipher.gcm.c : suites/test_suite_cipher.function suites/test_suite_cipher.gcm.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_cipher test_suite_cipher.gcm

test_suite_cipher.blowfish.c : suites/test_suite_cipher.function suites/test_suite_cipher.blowfish.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_cipher test_suite_cipher.blowfish

test_suite_cipher.camellia.c : suites/test_suite_cipher.function suites/test_suite_cipher.camellia.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_cipher test_suite_cipher.camellia

test_suite_cipher.des.c : suites/test_suite_cipher.function suites/test_suite_cipher.des.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_cipher test_suite_cipher.des

test_suite_cipher.null.c : suites/test_suite_cipher.function suites/test_suite_cipher.null.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_cipher test_suite_cipher.null

test_suite_cipher.padding.c : suites/test_suite_cipher.function suites/test_suite_cipher.padding.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_cipher test_suite_cipher.padding

test_suite_gcm.aes128_de.c : suites/test_suite_gcm.function suites/test_suite_gcm.aes128_de.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_gcm test_suite_gcm.aes128_de

test_suite_gcm.aes192_de.c : suites/test_suite_gcm.function suites/test_suite_gcm.aes192_de.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_gcm test_suite_gcm.aes192_de

test_suite_gcm.aes256_de.c : suites/test_suite_gcm.function suites/test_suite_gcm.aes256_de.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_gcm test_suite_gcm.aes256_de

test_suite_gcm.aes128_en.c : suites/test_suite_gcm.function suites/test_suite_gcm.aes128_en.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_gcm test_suite_gcm.aes128_en

test_suite_gcm.aes192_en.c : suites/test_suite_gcm.function suites/test_suite_gcm.aes192_en.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_gcm test_suite_gcm.aes192_en

test_suite_gcm.aes256_en.c : suites/test_suite_gcm.function suites/test_suite_gcm.aes256_en.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_gcm test_suite_gcm.aes256_en

test_suite_gcm.camellia.c : suites/test_suite_gcm.function suites/test_suite_gcm.camellia.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_gcm test_suite_gcm.camellia

test_suite_hmac_drbg.misc.c : suites/test_suite_hmac_drbg.function suites/test_suite_hmac_drbg.misc.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_hmac_drbg test_suite_hmac_drbg.misc

test_suite_hmac_drbg.no_reseed.c : suites/test_suite_hmac_drbg.function suites/test_suite_hmac_drbg.no_reseed.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_hmac_drbg test_suite_hmac_drbg.no_reseed

test_suite_hmac_drbg.nopr.c : suites/test_suite_hmac_drbg.function suites/test_suite_hmac_drbg.nopr.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_hmac_drbg test_suite_hmac_drbg.nopr

test_suite_hmac_drbg.pr.c : suites/test_suite_hmac_drbg.function suites/test_suite_hmac_drbg.pr.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites test_suite_hmac_drbg test_suite_hmac_drbg.pr

%.c : suites/%.function suites/%.data scripts/generate_code.pl suites/helpers.function suites/main_test.function
	echo "  Gen   $@"
	perl scripts/generate_code.pl suites $* $*

test_suite_aes.ecb$(EXEXT): test_suite_aes.ecb.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_aes.cbc$(EXEXT): test_suite_aes.cbc.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_aes.cfb$(EXEXT): test_suite_aes.cfb.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_aes.rest$(EXEXT): test_suite_aes.rest.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_arc4$(EXEXT): test_suite_arc4.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_asn1write$(EXEXT): test_suite_asn1write.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_base64$(EXEXT): test_suite_base64.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_blowfish$(EXEXT): test_suite_blowfish.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_camellia$(EXEXT): test_suite_camellia.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_ccm$(EXEXT): test_suite_ccm.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_cmac$(EXEXT): test_suite_cmac.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_cipher.aes$(EXEXT): test_suite_cipher.aes.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_cipher.arc4$(EXEXT): test_suite_cipher.arc4.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_cipher.ccm$(EXEXT): test_suite_cipher.ccm.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_cipher.gcm$(EXEXT): test_suite_cipher.gcm.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_cipher.blowfish$(EXEXT): test_suite_cipher.blowfish.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_cipher.camellia$(EXEXT): test_suite_cipher.camellia.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_cipher.des$(EXEXT): test_suite_cipher.des.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_cipher.null$(EXEXT): test_suite_cipher.null.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_cipher.padding$(EXEXT): test_suite_cipher.padding.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_ctr_drbg$(EXEXT): test_suite_ctr_drbg.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_des$(EXEXT): test_suite_des.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_dhm$(EXEXT): test_suite_dhm.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_ecdh$(EXEXT): test_suite_ecdh.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_ecdsa$(EXEXT): test_suite_ecdsa.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_ecjpake$(EXEXT): test_suite_ecjpake.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_ecp$(EXEXT): test_suite_ecp.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_entropy$(EXEXT): test_suite_entropy.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_error$(EXEXT): test_suite_error.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_gcm.aes128_de$(EXEXT): test_suite_gcm.aes128_de.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_gcm.aes192_de$(EXEXT): test_suite_gcm.aes192_de.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_gcm.aes256_de$(EXEXT): test_suite_gcm.aes256_de.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_gcm.aes128_en$(EXEXT): test_suite_gcm.aes128_en.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_gcm.aes192_en$(EXEXT): test_suite_gcm.aes192_en.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_gcm.aes256_en$(EXEXT): test_suite_gcm.aes256_en.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_gcm.camellia$(EXEXT): test_suite_gcm.camellia.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_hmac_drbg.misc$(EXEXT): test_suite_hmac_drbg.misc.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_hmac_drbg.no_reseed$(EXEXT): test_suite_hmac_drbg.no_reseed.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_hmac_drbg.nopr$(EXEXT): test_suite_hmac_drbg.nopr.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_hmac_drbg.pr$(EXEXT): test_suite_hmac_drbg.pr.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_md$(EXEXT): test_suite_md.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_mdx$(EXEXT): test_suite_mdx.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_memory_buffer_alloc$(EXEXT): test_suite_memory_buffer_alloc.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_mpi$(EXEXT): test_suite_mpi.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_pem$(EXEXT): test_suite_pem.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_pkcs1_v15$(EXEXT): test_suite_pkcs1_v15.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_pkcs1_v21$(EXEXT): test_suite_pkcs1_v21.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_pkcs5$(EXEXT): test_suite_pkcs5.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_pkparse$(EXEXT): test_suite_pkparse.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_pkwrite$(EXEXT): test_suite_pkwrite.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_pk$(EXEXT): test_suite_pk.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_rsa$(EXEXT): test_suite_rsa.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_shax$(EXEXT): test_suite_shax.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_ssl$(EXEXT): test_suite_ssl.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_timing$(EXEXT): test_suite_timing.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_x509parse$(EXEXT): test_suite_x509parse.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_x509write$(EXEXT): test_suite_x509write.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_xtea$(EXEXT): test_suite_xtea.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_debug$(EXEXT): test_suite_debug.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

test_suite_version$(EXEXT): test_suite_version.c $(DEP)
	echo "  CC    $<"
	$(CC) $(LOCAL_CFLAGS) $(CFLAGS) $<	$(LOCAL_LDFLAGS) $(LDFLAGS) -o $@

clean:
ifndef WINDOWS
	rm -f $(APPS) *.c
else
	del /Q /F *.c *.exe
endif

check: $(APPS)
	perl scripts/run-test-suites.pl

test: check
