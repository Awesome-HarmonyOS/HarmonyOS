# mbed TLS

mbed TLS makes it trivially easy for developers to include cryptographic and SSL/TLS capabilities in their embedded products, with a minimal code footprint. It offers an SSL library with an intuitive API and readable source code.

**Note:** The current release is beta, and implements no secure source of random numbers, weakening its security.

Currently the only supported yotta targets are:
- `frdm-k64f-gcc`
- `frdm-k64f-armcc`
- `x86-linux-native`
- `x86-osx-native`

## Sample programs

This release includes the following examples:

1. [**Self test:**](https://github.com/ARMmbed/mbedtls/blob/development/yotta/data/example-selftest) Tests different basic functions in the mbed TLS library.

2. [**Benchmark:**](https://github.com/ARMmbed/mbedtls/blob/development/yotta/data/example-benchmark) Measures the time taken to perform basic cryptographic functions used in the library.

3. [**Hashing:**](https://github.com/ARMmbed/mbedtls/blob/development/yotta/data/example-hashing) Demonstrates the various APIs for computing hashes of data (also known as message digests) with SHA-256.

4. [**Authenticated encryption:**](https://github.com/ARMmbed/mbedtls/blob/development/yotta/data/example-authcrypt) Demonstrates usage of the Cipher API for encrypting and authenticating data with AES-CCM.

These examples are integrated as yotta tests, so that they are built automatically when you build mbed TLS. Each of them comes with complete usage instructions as a Readme file in the repository.

## Performing TLS and DTLS connections

A high-level API for performing TLS and DTLS connections with mbed TLS in mbed OS is provided in a separate yotta module: [mbed-tls-sockets](https://github.com/ARMmbed/mbed-tls-sockets). We recommend this API for TLS and DTLS connections. It is very similar to the API provided by the [sockets](https://github.com/ARMmbed/sockets) module for unencrypted TCP and UDP connections.

The `mbed-tls-sockets` module includes a complete [example TLS client](https://github.com/ARMmbed/mbed-tls-sockets/blob/master/test/tls-client/main.cpp) with [usage instructions](https://github.com/ARMmbed/mbed-tls-sockets/blob/master/test/tls-client/README.md).

## Configuring mbed TLS features

mbed TLS makes it easy to disable any feature during compilation, if that feature isn't required for a particular project. The default configuration enables all modern and widely-used features, which should meet the needs of new projects, and disables all features that are older or less common, to minimize the code footprint.

The list of available compilation flags is available in the fully documented [config.h file](https://github.com/ARMmbed/mbedtls/blob/development/include/mbedtls/config.h).

If you need to adjust those flags, you can provide your own configuration-adjustment file with suitable `#define` and `#undef` statements. These will be included between the default definitions and the sanity checks. Your configuration file should be in your application's include directory, and can be named freely; you just need to let mbed TLS know the file's name. To do that, use yotta's [configuration system](http://docs.yottabuild.org/reference/config.html). The file's name should be in your `config.json` file, under mbedtls, as the key `user-config-file`.

For example, in an application called `myapp`, if you want to enable the EC J-PAKE key exchange and disable the CBC cipher mode, you can create a file named  `mbedtls-config-changes.h` in the `myapp` directory containing the following lines:

    #define MBEDTLS_ECJPAKE_C
    #define MBEDTLS_KEY_EXCHANGE_ECJPAKE_ENABLED

    #undef MBEDTLS_CIPHER_MODE_CBC

And then create a file named `config.json` at the root of your application with the following contents:

    {
       "mbedtls": {
          "user-config-file": "\"myapp/mbedtls-config-changes.h\""
       }
    }

Please note: you need to provide the exact name that will be used in the `#include` directive, including the `<>` or quotes around the name.

## Getting mbed TLS from GitHub

Like most components of mbed OS, mbed TLS is developed in the open and its source can be found on GitHub: [ARMmbed/mbedtls](https://github.com/ARMmbed/mbedtls). Unlike most other mbed OS components, however, you cannot just clone the repository and run `yotta build` from its root. This is because mbed TLS also exists as an independent component, so its repository includes things that are not relevant for mbed OS, as well as other build systems.

The way to use mbed TLS from a clone of the GitHub repository is to run the following commands from the root of a checkout:

    yotta/create-module.sh
    cd yotta/module

You can then run any yotta command you would normally run, such as `yotta build` or `yotta link`.

## Differences between the standalone and mbed OS editions

While the two editions share the same code base, there are still a number of differences, mainly in configuration and integration. You should keep in mind those differences when reading some articles in our [knowledge base](https://tls.mbed.org/kb), as currently all the articles are about the standalone edition.

* The mbed OS edition has a smaller set of features enabled by default in `config.h`, in order to reduce footprint. While the default configuration of the standalone edition puts more emphasize on maintaining interoperability with old peers, the mbed OS edition only enables the most modern ciphers and the latest version of (D)TLS.

* The following components of mbed TLS are disabled in the mbed OS edition: `net_sockets.c` and `timing.c`. This is because mbed OS include their equivalents.

* The mbed OS edition comes with a fully integrated API for (D)TLS connections in a companion module: [mbed-tls-sockets](https://github.com/ARMmbed/mbed-tls-sockets). See "Performing TLS and DTLS connections" above.

## Other resources

The [mbed TLS website](https://tls.mbed.org) contains many other useful
resources for the developer, such as [developer
documentation](https://tls.mbed.org/dev-corner), [knowledgebase
articles](https://tls.mbed.org/kb), and a [support forum](https://tls.mbed.org/discussions).

## Contributing

We gratefully accept bug reports and contributions from the community. There are some requirements we need to fulfill in order to be able to integrate contributions:

* Simple bug fixes to existing code do not contain copyright themselves and we can integrate without issue. The same is true of trivial contributions.

* For larger contributions, such as a new feature, the code can possibly fall under copyright law. We then need your consent to share in the ownership of the copyright. We have a form for this, which we will send to you in case you submit a contribution or pull request that we deem this necessary for.

To contribute, please:

* [Check for open issues](https://github.com/ARMmbed/mbedtls/issues) or [start a discussion](https://tls.mbed.org/discussions) around a feature idea or a bug.

* Fork the [mbed TLS repository on GitHub](https://github.com/ARMmbed/mbedtls) to start making your changes. As a general rule, you should use the "development" branch as a basis.

* Write a test that shows that the bug was fixed or that the feature works as expected.

* Send a pull request and bug us until it gets merged and published. We will include your name in the ChangeLog.

