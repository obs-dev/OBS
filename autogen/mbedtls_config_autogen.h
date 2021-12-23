// This is an autogenerated config file, any changes to this file will be overwritten

#ifndef MBEDTLS_CONFIG_AUTOGEN_H
#define MBEDTLS_CONFIG_AUTOGEN_H





#define MBEDTLS_AES_C
#define MBEDTLS_CIPHER_C
#define MBEDTLS_ENTROPY_HARDWARE_ALT
#define MBEDTLS_ENTROPY_C
#define MBEDTLS_ENTROPY_FORCE_SHA256
#define MBEDTLS_ENTROPY_MAX_SOURCES  2
#define MBEDTLS_NO_PLATFORM_ENTROPY
#define MBEDTLS_CTR_DRBG_C
#define MBEDTLS_SHA256_C
#define MBEDTLS_PSA_CRYPTO_C
#define MBEDTLS_PSA_CRYPTO_CONFIG
#define MBEDTLS_PSA_CRYPTO_DRIVERS
#define MBEDTLS_PSA_CRYPTO_STORAGE_C


#include "config-device-acceleration.h"

#if !defined(TEST_SUITE_MEMORY_BUFFER_ALLOC)
#include "sl_malloc.h"

#define MBEDTLS_PLATFORM_FREE_MACRO    sl_free
#define MBEDTLS_PLATFORM_CALLOC_MACRO  sl_calloc
#endif

#define MBEDTLS_PLATFORM_MEMORY
#define MBEDTLS_PLATFORM_C

#endif
