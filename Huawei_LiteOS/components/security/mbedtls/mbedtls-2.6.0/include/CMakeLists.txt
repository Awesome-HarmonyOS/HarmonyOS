option(INSTALL_MBEDTLS_HEADERS "Install mbed TLS headers." ON)

if(INSTALL_MBEDTLS_HEADERS)

    file(GLOB headers "mbedtls/*.h")

    install(FILES ${headers}
        DESTINATION include/mbedtls
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

endif(INSTALL_MBEDTLS_HEADERS)
