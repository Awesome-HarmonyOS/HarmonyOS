set(libs
    mbedtls
)

if(USE_PKCS11_HELPER_LIBRARY)
    set(libs ${libs} pkcs11-helper)
endif(USE_PKCS11_HELPER_LIBRARY)

if(ENABLE_ZLIB_SUPPORT)
    set(libs ${libs} ${ZLIB_LIBRARIES})
endif(ENABLE_ZLIB_SUPPORT)

add_executable(selftest selftest.c)
target_link_libraries(selftest ${libs})

add_executable(benchmark benchmark.c)
target_link_libraries(benchmark ${libs})

add_executable(ssl_cert_test ssl_cert_test.c)
target_link_libraries(ssl_cert_test ${libs})

add_executable(udp_proxy udp_proxy.c)
target_link_libraries(udp_proxy ${libs})

install(TARGETS selftest benchmark ssl_cert_test udp_proxy
        DESTINATION "bin"
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
