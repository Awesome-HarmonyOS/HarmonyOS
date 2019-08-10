## This file contains a record of how some of the test data was
## generated. The final build products are committed to the repository
## as well to make sure that the test data is identical. You do not
## need to use this makefile unless you're extending mbed TLS's tests.

## Many data files were generated prior to the existence of this
## makefile, so the method of their generation was not recorded.

## Note that in addition to depending on the version of the data
## generation tool, many of the build outputs are randomized, so
## running this makefile twice would not produce the same results.

## Tools
OPENSSL ?= openssl

## Build the generated test data. Note that since the final outputs
## are committed to the repository, this target should do nothing on a
## fresh checkout. Furthermore, since the generation is randomized,
## re-running the same targets may result in differing files. The goal
## of this makefile is primarily to serve as a record of how the
## targets were generated in the first place.
default: all_final

all_intermediate := # temporary files
all_final := # files used by tests



################################################################
#### Generate certificates from existing keys
################################################################

test_ca_key_file_rsa = test-ca.key
test_ca_pwd_rsa = PolarSSLTest
test_ca_config_file = test-ca.opensslconf

test-ca.csr: $(test_ca_key_file_rsa) $(test_ca_config_file)
	$(OPENSSL) req -new -config $(test_ca_config_file) -key $(test_ca_key_file_rsa) -passin "pass:$(test_ca_pwd_rsa)" -subj "/C=NL/O=PolarSSL/CN=PolarSSL Test CA" -out $@
all_intermediate += test-ca.csr
test-ca-sha1.crt: $(test_ca_key_file_rsa) $(test_ca_config_file) test-ca.csr
	$(OPENSSL) req -x509 -config $(test_ca_config_file) -key $(test_ca_key_file_rsa) -passin "pass:$(test_ca_pwd_rsa)" -set_serial 0 -days 3653 -sha1 -in test-ca.csr -out $@
all_final += test-ca-sha1.crt
test-ca-sha256.crt: $(test_ca_key_file_rsa) $(test_ca_config_file) test-ca.csr
	$(OPENSSL) req -x509 -config $(test_ca_config_file) -key $(test_ca_key_file_rsa) -passin "pass:$(test_ca_pwd_rsa)" -set_serial 0 -days 3653 -sha256 -in test-ca.csr -out $@
all_final += test-ca-sha256.crt

cli_crt_key_file_rsa = cli-rsa.key
cli_crt_extensions_file = cli.opensslconf

cli-rsa.csr: $(cli_crt_key_file_rsa)
	$(OPENSSL) req -new -key $(cli_crt_key_file_rsa) -passin "pass:$(test_ca_pwd_rsa)" -subj "/C=NL/O=PolarSSL/CN=PolarSSL Client 2" -out $@
all_intermediate += cli-rsa.csr
cli-rsa-sha1.crt: $(cli_crt_key_file_rsa) test-ca-sha1.crt cli-rsa.csr
	$(OPENSSL) x509 -req -extfile $(cli_crt_extensions_file) -extensions cli-rsa -CA test-ca-sha1.crt -CAkey $(test_ca_key_file_rsa) -passin "pass:$(test_ca_pwd_rsa)" -set_serial 4 -days 3653 -sha1 -in cli-rsa.csr -out $@
all_final += cli-rsa-sha1.crt
cli-rsa-sha256.crt: $(cli_crt_key_file_rsa) test-ca-sha256.crt cli-rsa.csr
	$(OPENSSL) x509 -req -extfile $(cli_crt_extensions_file) -extensions cli-rsa -CA test-ca-sha256.crt -CAkey $(test_ca_key_file_rsa) -passin "pass:$(test_ca_pwd_rsa)" -set_serial 4 -days 3653 -sha256 -in cli-rsa.csr -out $@
all_final += cli-rsa-sha256.crt

server2-rsa.csr: server2.key
	$(OPENSSL) req -new -key server2.key -passin "pass:$(test_ca_pwd_rsa)" -subj "/C=NL/O=PolarSSL/CN=localhost" -out $@
all_intermediate += server2-rsa.csr
server2-sha256.crt: server2-rsa.csr
	$(OPENSSL) x509 -req -extfile $(cli_crt_extensions_file) -extensions cli-rsa -CA test-ca-sha256.crt -CAkey $(test_ca_key_file_rsa) -passin "pass:$(test_ca_pwd_rsa)" -set_serial 4 -days 3653 -sha256 -in server2-rsa.csr -out $@
all_final += server2-sha256.crt



################################################################
#### Meta targets
################################################################

all_final: $(all_final)
all: $(all_intermediate) $(all_final)

.PHONY: default all_final all

# These files should not be committed to the repository.
list_intermediate:
	@printf '%s\n' $(all_intermediate) | sort
# These files should be committed to the repository so that the test data is
# available upon checkout without running a randomized process depending on
# third-party tools.
list_final:
	@printf '%s\n' $(all_final) | sort
.PHONY: list_intermediate list_final

## Remove intermediate files
clean:
	rm -f $(all_intermediate)
## Remove all build products, even the ones that are committed
neat: clean
	rm -f $(all_final)
.PHONY: clean neat
