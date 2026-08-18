/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/**
 * @file  services_test_key_management.c
 * @brief Key management services test harness
 * @ingroup services
 * @par
 */

/******************************************************************************
 *  I N C L U D E   F I L E S
 *****************************************************************************/
#include "services_lib_api.h"
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef A32_LINUX
#include "services_lib_linux.h"
#else
#include "services_lib_interface.h"
#endif

char *fill_print_buf(uint8_t *buf, uint32_t length);

/*******************************************************************************
 *  M A C R O   D E F I N E S
 ******************************************************************************/

/*******************************************************************************
 *  T Y P E D E F S
 ******************************************************************************/

/*******************************************************************************
 *  G L O B A L   V A R I A B L E S
 ******************************************************************************/

// Various test vectors per NIST SP 800-38A F.5.5 / F.5.6
static const uint8_t KEY_AES256[] = {
    0x60, 0x3D, 0xEB, 0x10, 0x15, 0xCA, 0x71, 0xBE, 0x2B, 0x73, 0xAE,
    0xF0, 0x85, 0x7D, 0x77, 0x81, 0x1F, 0x35, 0x2C, 0x07, 0x3B, 0x61,
    0x08, 0xD7, 0x2D, 0x98, 0x10, 0xA3, 0x09, 0x14, 0xDF, 0xF4};

static const uint8_t PLAIN[] = {0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96,
				0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A};

static const uint8_t IV_CBC_OFB[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
					     0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
					     0x0C, 0x0D, 0x0E, 0x0F};
static const uint8_t IV_GCM[] = {0xCA, 0xFE, 0xBA, 0xBE, 0xFA, 0xCE,
					 0xDB, 0xAD, 0xDE, 0xCA, 0xF8, 0x88};

/*******************************************************************************
 *  C O D E
 ******************************************************************************/

static const char *se_key_type_to_str(se_key_type_t key_type)
{
  switch (key_type) {
  case SE_KEY_TYPE_NONE:
    return "No Key";
  case SE_KEY_TYPE_AES_128:
    return "AES-128";
  case SE_KEY_TYPE_AES_256:
    return "AES-256";
  case SE_KEY_TYPE_ECC_P256_ECDSA:
    return "ECC P-256 ECDSA";
  case SE_KEY_TYPE_ECC_P384_ECDSA:
    return "ECC P-384 ECDSA";
  case SE_KEY_TYPE_ECC_P256_ECDH:
    return "ECC P-256 ECDH";
  case SE_KEY_TYPE_ECC_P384_ECDH:
    return "ECC P-384 ECDH";
  case SE_KEY_TYPE_HMAC_256:
    return "HMAC-256";
  case SE_KEY_TYPE_HMAC_384:
    return "HMAC-384";
  default:
    return "Unknown";
  }
}

/**
 * @brief Dump Key Management slot table
 * @param services_handle
 */
static void display_key_table(uint32_t services_handle)
{
  uint32_t error_code = 0;
  key_attributes_t key_attr;

  TEST_print(services_handle, "-- Display Key_slots %3u slots ---\n",
	     SE_NUM_KEY_SLOTS);
  for (int key_index = 0; key_index < SE_NUM_KEY_SLOTS; key_index++) {
	SERVICES_key_get_key_attributes(services_handle, key_index, &key_attr,
					&error_code);
    TEST_print(services_handle, "-- Slot %d:  [0x%04x] (%s)\n", key_index,
	       key_attr.key_type, se_key_type_to_str(key_attr.key_type));
  }
  TEST_print(services_handle, "----------------------------------\n");
}

static void test_key_mgmt_import_clear_key(uint32_t services_handle)
{
  uint32_t key_handle = 0;
  uint32_t error_code = SERVICES_REQ_SUCCESS;

  SERVICES_key_mgmt_import_key(services_handle, SE_KEY_TYPE_AES_256, KEY_AES256,
			       sizeof(KEY_AES256), &key_handle, &error_code);

  TEST_print(services_handle,
	     "== TEST SERVICES_key_mgmt_import_key        key handle: 0x%X    "
	     "service_resp=0x%08X\n",
	     key_handle, error_code);

  SERVICES_key_mgmt_clear_key(services_handle, key_handle, &error_code);
  TEST_print(services_handle,
	     "== TEST SERVICES_key_mgmt_clear_key                              "
	     "service_resp=0x%08X\n",
	     error_code);

  SERVICES_key_mgmt_clear_key(services_handle, key_handle, &error_code);
  TEST_print(services_handle,
	     "== TEST SERVICES_key_mgmt_clear_key using invalid key handle     "
	     "service_resp=0x%08X\n",
	     error_code);
}

static void test_key_mgmt_aes_crypt_by_handle_gcm(uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;

  static const uint8_t key[] = {0xFE, 0xFF, 0xE9, 0x92, 0x86, 0x65, 0x73, 0x1C,
				0x6D, 0x6A, 0x8F, 0x94, 0x67, 0x30, 0x83, 0x08};
  // const int key_len = sizeof(key) * 8;

  const size_t iv_len = sizeof(IV_GCM);
  static const uint8_t additional[] = {
      0x3A, 0xD7, 0x7B, 0xB4, 0x0D, 0x7A, 0x36, 0x60, 0xA8, 0x9E, 0xCA,
      0xF3, 0x24, 0x66, 0xEF, 0x97, 0xF5, 0xD3, 0xD5, 0x85, 0x03, 0xB9,
      0x69, 0x9D, 0xE7, 0x85, 0x89, 0x5A, 0x96, 0xFD, 0xBA, 0xAF, 0x43,
      0xB1, 0xCD, 0x7F, 0x59, 0x8E, 0xCE, 0x23, 0x88, 0x1B, 0x00, 0xE3,
      0xED, 0x03, 0x06, 0x88, 0x7B, 0x0C, 0x78, 0x5E, 0x27, 0xE8, 0xAD,
      0x3F, 0x82, 0x23, 0x20, 0x71, 0x04, 0x72, 0x5D, 0xD4};
  const size_t add_len = sizeof(additional);
  static const uint8_t pt[] = {
      0xD9, 0x31, 0x32, 0x25, 0xF8, 0x84, 0x06, 0xE5, 0xA5, 0x59, 0x09,
      0xC5, 0xAF, 0xF5, 0x26, 0x9A, 0x86, 0xA7, 0xA9, 0x53, 0x15, 0x34,
      0xF7, 0xDA, 0x2E, 0x4C, 0x30, 0x3D, 0x8A, 0x31, 0x8A, 0x72, 0x1C,
      0x3C, 0x0C, 0x95, 0x95, 0x68, 0x09, 0x53, 0x2F, 0xCF, 0x0E, 0x24,
      0x49, 0xA6, 0xB5, 0x25, 0xB1, 0x6A, 0xED, 0xF5, 0xAA, 0x0D, 0xE6,
      0x57, 0xBA, 0x63, 0x7B, 0x39, 0x1A, 0xAF, 0xD2, 0x55};
  const size_t pt_len = sizeof(pt);
  // static const uint8_t ct[] =
  // {0x42,0x83,0x1E,0xC2,0x21,0x77,0x74,0x24,0x4B,0x72,0x21,0xB7,0x84,0xD0,0xD4,0x9C,0xE3,0xAA,0x21,0x2F,0x2C,0x02,0xA4,0xE0,0x35,0xC1,0x7E,0x23,0x29,0xAC,0xA1,0x2E,0x21,0xD5,0x14,0xB2,0x54,0x66,0x93,0x1C,0x7D,0x8F,0x6A,0x5A,0xAC,0x84,0xAA,0x05,0x1B,0xA3,0x0B,0x39,0x6A,0x0A,0xAC,0x97,0x3D,0x58,0xE0,0x91,0x47,0x3F,0x59,0x85};
  // static const size_t ct_len = sizeof(ct);
  static const uint8_t tag[] = {0x64, 0xC0, 0x23, 0x29, 0x04, 0xAF, 0x39, 0x8A,
				0x5B, 0x67, 0xC1, 0x0B, 0x53, 0xA5, 0x02, 0x4D};
  const size_t tag_len = sizeof(tag);

  uint32_t key_handle;
  SERVICES_key_mgmt_import_key(services_handle, SE_KEY_TYPE_AES_128, key,
			       sizeof(key), &key_handle, &error_code);
  // TEST_print(services_handle, "Key Handle: %X  Error code: %0X\n",
  // key_handle, error_code);

  uint8_t iv_buf[12];
  memcpy(iv_buf, IV_GCM, iv_len);
  uint8_t add_buf[64];
  memcpy(add_buf, additional, add_len);
  uint8_t plain_buf[64];
  memcpy(plain_buf, pt, pt_len);
  uint8_t tag_buf[16];
  memset(tag_buf, 0, sizeof(tag_buf));
  uint8_t cipher_buf[64] = {0};
  memset(cipher_buf, 0, sizeof(cipher_buf));

  uint32_t crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_aes_crypt_by_handle(
      services_handle, key_handle, SE_CRYPT_DIRECTION_ENCRYPT,
      SE_CRYPT_TYPE_GCM, iv_buf, 12, plain_buf, 64, cipher_buf, add_buf,
      add_len, tag_buf, tag_len, &error_code, &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_aes_crypt_by_handle - GCM  "
	     "service_resp=0x%08X crypto_resp=0x%08X\n",
	     error_code, crypto_error_code);

  char *print_buf;
  // print_buf = fill_print_buf(iv_buf, 12);
  // TEST_print(services_handle, "IV: %s\n", print_buf);
  // print_buf = fill_print_buf(plain_buf, 40);
  // TEST_print(services_handle, "Plain data: %s\n", print_buf);
  print_buf = fill_print_buf(cipher_buf, MBEDTLS_AES_BLOCK_SIZE);
  TEST_print(services_handle, "Encrypted data: %s\n", print_buf);
  print_buf = fill_print_buf(tag_buf, tag_len);
  TEST_print(services_handle, "Tag: %s\n", print_buf);

  memset(plain_buf, 0, sizeof(plain_buf));
  memcpy(tag_buf, tag, tag_len);
  memcpy(iv_buf, IV_GCM, iv_len);
  memcpy(add_buf, additional, add_len);
  // memcpy(cipher_buf, ct, ct_len);

  crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_aes_crypt_by_handle(
      services_handle, key_handle, SE_CRYPT_DIRECTION_DECRYPT,
      SE_CRYPT_TYPE_GCM, iv_buf, 12, cipher_buf, 64, plain_buf, add_buf,
      add_len, tag_buf, tag_len, &error_code, &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_aes_crypt_by_handle - GCM  "
	     "service_resp=0x%08X crypto_resp=0x%08X\n",
	     error_code, crypto_error_code);

  print_buf = fill_print_buf(plain_buf, MBEDTLS_AES_BLOCK_SIZE);
  TEST_print(services_handle, "Decrypted data: %s\n", print_buf);

  SERVICES_key_mgmt_clear_key(services_handle, key_handle, &error_code);
}

/**
 * @fn void test_key_mgmt_aes_crypt_by_handle_kcp(uint32_t)
 * @param services_handle
 */
static void test_key_mgmt_aes_crypt_by_handle_kcp(uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;

  uint8_t buf[MBEDTLS_AES_BLOCK_SIZE];
  uint8_t iv[MBEDTLS_AES_BLOCK_SIZE];

  memcpy(buf, PLAIN, sizeof(buf));
  memcpy(iv, IV_CBC_OFB, sizeof(IV_CBC_OFB));

  uint32_t crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_aes_crypt_by_handle(
      services_handle, SE_KEY_ID_KCP, SE_CRYPT_DIRECTION_ENCRYPT,
      MBEDTLS_AES_CRYPT_CBC, iv, MBEDTLS_AES_BLOCK_SIZE, buf,
      MBEDTLS_AES_BLOCK_SIZE, buf, 0, 0, 0, 0, &error_code, &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_aes_crypt_by_handle - KCP  "
	     "service_resp=0x%08X crypto_resp=0x%08X\n",
	     error_code, crypto_error_code);

  char *print_buf = fill_print_buf(buf, MBEDTLS_AES_BLOCK_SIZE);
  TEST_print(services_handle, "Encrypted data: %s\n", print_buf);

  memcpy(iv, IV_CBC_OFB, sizeof(IV_CBC_OFB));

  crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_aes_crypt_by_handle(
      services_handle, SE_KEY_ID_KCP, SE_CRYPT_DIRECTION_DECRYPT,
      MBEDTLS_AES_CRYPT_CBC, iv, MBEDTLS_AES_BLOCK_SIZE, buf,
      MBEDTLS_AES_BLOCK_SIZE, buf, 0, 0, 0, 0, &error_code, &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_aes_crypt_by_handle - KCP  "
	     "service_resp=0x%08X crypto_resp=0x%08X\n",
	     error_code, crypto_error_code);

  print_buf = fill_print_buf(buf, MBEDTLS_AES_BLOCK_SIZE);
  TEST_print(services_handle, "Decrypted data: %s\n", print_buf);
}

static void test_key_mgmt_aes_crypt_by_handle(uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;

  uint32_t key_handle;
  SERVICES_key_mgmt_import_key(services_handle, SE_KEY_TYPE_AES_256, KEY_AES256,
			       sizeof(KEY_AES256), &key_handle, &error_code);

  // TEST_print(services_handle, "Key Handle: %X  Error code: %0X\n",
  // key_handle, error_code);

  // uint8_t key[MBEDTLS_AES_KEY_256 / 8];
  uint8_t buf[MBEDTLS_AES_BLOCK_SIZE];
  uint8_t iv[MBEDTLS_AES_BLOCK_SIZE];

  memcpy(buf, PLAIN, sizeof(buf));
  memcpy(iv, IV_CBC_OFB, sizeof(IV_CBC_OFB));

  // KM AES test #1 - unauthenticated AES
  uint32_t crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_aes_crypt_by_handle(
      services_handle, key_handle, SE_CRYPT_DIRECTION_ENCRYPT,
      MBEDTLS_AES_CRYPT_CBC, iv, MBEDTLS_AES_BLOCK_SIZE, buf,
      MBEDTLS_AES_BLOCK_SIZE, buf, 0, 0, 0, 0, &error_code, &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_aes_crypt_by_handle - AES  "
	     "service_resp=0x%08X crypto_resp=0x%08X\n",
	     error_code, crypto_error_code);

  char *print_buf = fill_print_buf(buf, MBEDTLS_AES_BLOCK_SIZE);
  TEST_print(services_handle, "Encrypted data: %s\n", print_buf);

  memcpy(iv, IV_CBC_OFB, sizeof(IV_CBC_OFB));

  crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_aes_crypt_by_handle(
      services_handle, key_handle, SE_CRYPT_DIRECTION_DECRYPT,
      MBEDTLS_AES_CRYPT_CBC, iv, MBEDTLS_AES_BLOCK_SIZE, buf,
      MBEDTLS_AES_BLOCK_SIZE, buf, 0, 0, 0, 0, &error_code, &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_aes_crypt_by_handle - AES  "
	     "service_resp=0x%08X crypto_resp=0x%08X\n",
	     error_code, crypto_error_code);

  print_buf = fill_print_buf(buf, MBEDTLS_AES_BLOCK_SIZE);
  TEST_print(services_handle, "Decrypted data: %s\n", print_buf);

  // KM AES test #2 - authenticated AES
  test_key_mgmt_aes_crypt_by_handle_gcm(services_handle);
  // KM AES test #3 - AES using the KCP key
  test_key_mgmt_aes_crypt_by_handle_kcp(services_handle);

  SERVICES_key_mgmt_clear_key(services_handle, key_handle, &error_code);
}

static void test_key_mgmt_generate_get_ecc_key(uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t crypto_error_code = SERVICES_REQ_SUCCESS;

  uint8_t public_key[SE_ECC_P384_PUBKEY_SIZE];
  uint32_t public_key_len;
  uint32_t key_handle;
  SERVICES_key_mgmt_generate_ecc_key(
      services_handle, SE_KEY_TYPE_ECC_P384_ECDSA, &key_handle, public_key,
      &public_key_len, &error_code, &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_generate_ecc_key  key_len: %d    "
	     "handle: 0x%X  service_resp=0x%08X, crypto_resp=0x%08X\n",
	     public_key_len, key_handle, error_code, crypto_error_code);

  char *print_buf = fill_print_buf(public_key, 16);
  TEST_print(services_handle, "Generated public key: %s...\n", print_buf);

  SERVICES_key_mgmt_get_ecc_public_key(services_handle, key_handle, public_key,
				       &public_key_len, &error_code,
				       &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_get_ecc_public_key  key_len: %d  "
	     "handle: 0x%X  service_resp=0x%08X crypto_resp=0x%08X\n",
	     public_key_len, key_handle, error_code, crypto_error_code);

  print_buf = fill_print_buf(public_key, 16);
  TEST_print(services_handle, "Derived public key: %s...\n", print_buf);

  // Cleanup
  SERVICES_key_mgmt_clear_key(services_handle, key_handle, &error_code);
}

static void test_key_mgmt_hmac_by_handle(uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t crypto_error_code = SERVICES_REQ_SUCCESS;

  static uint8_t hmacTest_Key[] = {
      // We don't use 64-byte keys, this is the original test key from the
      // cryptocell code
      //{ 0x15, 0xb2, 0x9a, 0xd8, 0xae, 0x2a, 0xad, 0x73, 0xa7, 0x26, 0x43,
      // 0x50, 0x70, 0xe8, 0xe9, 0xda, 0x9b, 0x47, 0x69, 0xc3, 0xe3, 0xa4, 0xee,
      // 0x99, 0x6e, 0x20, 0x6a, 0x9b, 0x4f, 0x0c, 0x35, 0xca, 0x4f, 0xa2, 0xf7,
      // 0x43, 0xed, 0xf2, 0xc7, 0xcb, 0xa3, 0x1e, 0x94, 0xac, 0x6b, 0xca, 0xc4,
      // 0xc0, 0x82, 0xcf, 0x1c, 0xcb, 0x6c, 0x2f, 0xe0, 0x0d, 0x38, 0x4e, 0x3b,
      // 0x18, 0x05, 0x5f, 0xe0, 0xe0 };
      // We use this key instead -
      0x15, 0xb2, 0x9a, 0xd8, 0xae, 0x2a, 0xad, 0x73, 0xa7, 0x26, 0x43, 0x50,
      0x70, 0xe8, 0xe9, 0xda, 0x9b, 0x47, 0x69, 0xc3, 0xe3, 0xa4, 0xee, 0x99,
      0x6e, 0x20, 0x6a, 0x9b, 0x4f, 0x0c, 0x35, 0xca, 0x4f, 0xa2, 0xf7, 0x43,
      0xed, 0xf2, 0xc7, 0xcb, 0xa3, 0x1e, 0x94, 0xac, 0x6b, 0xca, 0xc4, 0xc0};

  static const uint16_t hmacTest_KeySize = sizeof(hmacTest_Key);
  static uint8_t hmacTest_InputData[] = {
      0x99, 0xfd, 0x18, 0xa3, 0x5d, 0x50, 0x81, 0x84, 0xa6, 0xf3, 0x61,
      0xc6, 0x7c, 0xd9, 0xb1, 0x0b, 0x4c, 0xd1, 0xd8, 0xb2, 0x46, 0x57,
      0x2a, 0x4d, 0x03, 0xb0, 0xae, 0x55, 0x6b, 0x36, 0x24, 0x1d, 0xd6,
      0xf0, 0x46, 0x05, 0x71, 0x65, 0x4f, 0xf0, 0xe4, 0xb2, 0xba, 0xf8,
      0x31, 0xdb, 0x4c, 0x60, 0xdf, 0x5f, 0x54, 0xc9, 0x59, 0x0f, 0x32,
      0xa9, 0x91, 0x1f, 0x16, 0xfa, 0xe8, 0x7e, 0x0a, 0x2f, 0x52};
  static const uint32_t hmacTest_InputDataSize = sizeof(hmacTest_InputData);

  // Expected result for SHA256 -
  // 83C272E39397A5FA793B96DC37ABC461A16F5ECB8A2D2A1FD904B7C6C1855433
  // Expected result for SHA384 -
  // 98b8310b2025ab9d69b72d5fbd51d64b1660d45cfe626a714380595d9c60dbc8f741762b808ce241e28d00ad6dc3bc4d

  unsigned char hmacOutBuff[SE_HMAC_MAX_OUTPUT_SIZE] = {0};
  uint32_t hmacOutLen;

  uint32_t key_handle;
  SERVICES_key_mgmt_import_key(services_handle, SE_KEY_TYPE_HMAC_384,
			       hmacTest_Key, hmacTest_KeySize, &key_handle,
			       &error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_import_key        key handle: 0x%X    "
	     "service_resp=0x%08X\n",
	     key_handle, error_code);

  SERVICES_key_mgmt_hmac_by_handle(services_handle, key_handle, SE_HMAC_SHA384,
				   hmacTest_InputData, hmacTest_InputDataSize,
				   hmacOutBuff, &hmacOutLen, &error_code,
				   &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_hmac_by_handle                        "
	     "service_resp=0x%08X crypto_resp=0x%08X\n",
	     error_code, crypto_error_code);

  char *print_buf = fill_print_buf(hmacOutBuff, hmacTest_KeySize);
  TEST_print(services_handle, "Generated HMAC: %s  len: %d\n", print_buf,
	     hmacTest_KeySize);

  // Cleanup
  SERVICES_key_mgmt_clear_key(services_handle, key_handle, &error_code);
}

static void test_key_mgmt_wrap_unwrap_key(uint32_t services_handle)
{
  /*
  static uint8_t KEY[] = {
    0x15, 0xb2, 0x9a, 0xd8, 0xae, 0x2a, 0xad, 0x73,
    0xa7, 0x26, 0x43, 0x50, 0x70, 0xe8, 0xe9, 0xda,
    0x9b, 0x47, 0x69, 0xc3, 0xe3, 0xa4, 0xee, 0x99,
    0x6e, 0x20, 0x6a, 0x9b, 0x4f, 0x0c, 0x35, 0xca,
    0x4f, 0xa2, 0xf7, 0x43, 0xed, 0xf2, 0xc7, 0xcb,
    0xa3, 0x1e, 0x94, 0xac, 0x6b, 0xca, 0xc4, 0xc0
  };
  */
  uint32_t key_len = sizeof(KEY_AES256);

  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t crypto_error_code = SERVICES_REQ_SUCCESS;

  uint8_t wrapped_key[SE_WRAPPED_KEY_MAX_SIZE] = {0};
  uint32_t wrapped_key_len;

  uint32_t key_handle;
  SERVICES_key_mgmt_import_key(services_handle, SE_KEY_TYPE_AES_256, KEY_AES256,
			       key_len, &key_handle, &error_code);
  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_import_key        key handle: 0x%X    "
	     "service_resp=0x%08X\n",
	     key_handle, error_code);

  // Use the session key (AES key) to encrypt the same data on both local and
  // peer sides
  uint8_t iv_buf[MBEDTLS_AES_BLOCK_SIZE];
  uint8_t input_buf[MBEDTLS_AES_BLOCK_SIZE];
  uint8_t cipher_buf[MBEDTLS_AES_BLOCK_SIZE];

  memset(iv_buf, 0, sizeof(iv_buf));
  memset(input_buf, 0, sizeof(input_buf));
  memset(cipher_buf, 0, sizeof(cipher_buf));

  // AES encrypt using the original key
  crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_aes_crypt_by_handle(
      services_handle, key_handle, SE_CRYPT_DIRECTION_ENCRYPT,
      MBEDTLS_AES_CRYPT_CBC, iv_buf, sizeof(iv_buf), input_buf,
      sizeof(input_buf), cipher_buf, 0, 0, 0, 0, &error_code,
      &crypto_error_code);

  char *print_buf = fill_print_buf(cipher_buf, 16);
  TEST_print(services_handle, "AES Encrypt using the original key: %s...\n",
	     print_buf);

  crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_wrap_key(services_handle, key_handle, wrapped_key,
			     &wrapped_key_len, &error_code, &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_wrap_key                              "
	     "service_resp=0x%08X crypto_resp=0x%08X\n",
	     error_code, crypto_error_code);

  // Printing the Key Type, IV and TAG is for diagnostic purposes.
  // Client code should not try to parse the wrapped key blob
  print_buf = fill_print_buf(wrapped_key, 32);
  TEST_print(services_handle, "Wrapped Key: %s... length: %d\n", print_buf,
	     wrapped_key_len);

  uint32_t offset = key_len + sizeof(uint32_t) + WRAPPED_KEY_METADATA_LEN;
  print_buf = fill_print_buf(wrapped_key + offset, WRAPPED_KEY_IV_LEN);
  TEST_print(services_handle, "IV: %s\n", print_buf);
  offset += WRAPPED_KEY_IV_LEN;
  print_buf = fill_print_buf(wrapped_key + offset, WRAPPED_KEY_TAG_LEN);
  TEST_print(services_handle, "TAG: %s\n", print_buf);

  crypto_error_code = SERVICES_REQ_SUCCESS;
  uint32_t unwrapped_key_handle;
  SERVICES_key_mgmt_unwrap_key(services_handle, wrapped_key,
			       wrapped_key_len, // SE_WRAPPED_KEY_MAX_SIZE,
			       &unwrapped_key_handle, &error_code,
			       &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_unwrap_key   Key handle: 0x%X         "
	     "service_resp=0x%08X crypto_resp=0x%08X\n",
	     key_handle, error_code, crypto_error_code);

  memset(iv_buf, 0, sizeof(iv_buf));
  memset(input_buf, 0, sizeof(input_buf));
  memset(cipher_buf, 0, sizeof(cipher_buf));

  // AES Encrypt using the unwrapped key
  crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_aes_crypt_by_handle(
      services_handle, unwrapped_key_handle, SE_CRYPT_DIRECTION_ENCRYPT,
      MBEDTLS_AES_CRYPT_CBC, iv_buf, sizeof(iv_buf), input_buf,
      sizeof(input_buf), cipher_buf, 0, 0, 0, 0, &error_code,
      &crypto_error_code);

  print_buf = fill_print_buf(cipher_buf, 16);
  TEST_print(services_handle, "AES Encrypt using the unwrapped key: %s...\n",
	     print_buf);

  // Cleanup
  SERVICES_key_mgmt_clear_key(services_handle, key_handle, &error_code);
  SERVICES_key_mgmt_clear_key(services_handle, unwrapped_key_handle,
			      &error_code);
}

static void test_key_mgmt_ecdh(uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t crypto_error_code = SERVICES_REQ_SUCCESS;

  // Generate local ECC key pair
  uint8_t public_key[SE_ECC_P384_PUBKEY_SIZE];
  uint32_t public_key_len;
  uint32_t key_handle;
  SERVICES_key_mgmt_generate_ecc_key(services_handle, SE_KEY_TYPE_ECC_P384_ECDH,
				     &key_handle, public_key, &public_key_len,
				     &error_code, &crypto_error_code);
  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_generate_ecc_key  key_len: %d       "
	     "handle: 0x%X  service_resp=0x%08X crypto_resp=0x%08X\n",
	     public_key_len, key_handle, error_code, crypto_error_code);
  char *print_buf = fill_print_buf(public_key, 16);
  TEST_print(services_handle, "Generated public key: %s...\n", print_buf);

  // Generate peer ECC key pair
  uint8_t peer_public_key[SE_ECC_P384_PUBKEY_SIZE];
  uint32_t peer_public_key_len;
  uint32_t peer_key_handle;
  crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_generate_ecc_key(
      services_handle, SE_KEY_TYPE_ECC_P384_ECDH, &peer_key_handle,
      peer_public_key, &peer_public_key_len, &error_code, &crypto_error_code);
  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_generate_ecc_key  PEER key_len: %d  "
	     "handle: 0x%X  service_resp=0x%08X crypto_resp=0x%08X\n",
	     peer_public_key_len, peer_key_handle, error_code,
	     crypto_error_code);
  print_buf = fill_print_buf(peer_public_key, 16);
  TEST_print(services_handle, "Generated PEER public key: %s...\n", print_buf);

  // Run local ECDH function
  const uint8_t salt[32] = {0};
  uint32_t salt_len = 32;
  uint32_t session_key_handle;
  crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_ecdh(services_handle, key_handle, SE_ECC_CURVE_P384,
			 peer_public_key, peer_public_key_len, SE_HKDF_SHA384,
			 SE_KEY_TYPE_AES_256, salt, salt_len,
			 &session_key_handle, &error_code, &crypto_error_code);
  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_ecdh   Session Key handle: 0x%X        "
	     "           service_resp=0x%08X crypto_resp=0x%08X\n",
	     session_key_handle, error_code, crypto_error_code);

  // Run PEER ECDH function
  crypto_error_code = SERVICES_REQ_SUCCESS;
  uint32_t peer_session_key_handle;
  SERVICES_key_mgmt_ecdh(
      services_handle, peer_key_handle, SE_ECC_CURVE_P384, public_key,
      public_key_len, SE_HKDF_SHA384, SE_KEY_TYPE_AES_256, salt, salt_len,
      &peer_session_key_handle, &error_code, &crypto_error_code);
  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_ecdh   Peer Session Key handle: 0x%X   "
	     "           service_resp=0x%08X crypto_resp=0x%08X\n",
	     peer_session_key_handle, error_code, crypto_error_code);

  // Use the session key (AES key) to encrypt the same data on both local and
  // peer sides
  uint8_t iv_buf[MBEDTLS_AES_BLOCK_SIZE];
  uint8_t input_buf[MBEDTLS_AES_BLOCK_SIZE];
  uint8_t cipher_buf[MBEDTLS_AES_BLOCK_SIZE];

  memset(iv_buf, 0, sizeof(iv_buf));
  memset(input_buf, 0, sizeof(input_buf));
  memset(cipher_buf, 0, sizeof(cipher_buf));

  // ECDH on local side
  crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_aes_crypt_by_handle(
      services_handle, session_key_handle, SE_CRYPT_DIRECTION_ENCRYPT,
      MBEDTLS_AES_CRYPT_CBC, iv_buf, sizeof(iv_buf), input_buf,
      sizeof(input_buf), cipher_buf, 0, 0, 0, 0, &error_code,
      &crypto_error_code);

  print_buf = fill_print_buf(cipher_buf, 16);
  TEST_print(services_handle, "Ciphertext: %s...\n", print_buf);

  memset(iv_buf, 0, sizeof(iv_buf));
  memset(input_buf, 0, sizeof(input_buf));
  memset(cipher_buf, 0, sizeof(cipher_buf));

  // ECDH on peer side
  crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_aes_crypt_by_handle(
      services_handle, peer_session_key_handle, SE_CRYPT_DIRECTION_ENCRYPT,
      MBEDTLS_AES_CRYPT_CBC, iv_buf, sizeof(iv_buf), input_buf,
      sizeof(input_buf), cipher_buf, 0, 0, 0, 0, &error_code,
      &crypto_error_code);

  print_buf = fill_print_buf(cipher_buf, 16);
  TEST_print(services_handle, "PEER Ciphertext: %s...\n", print_buf);

  // Cleanup
  SERVICES_key_mgmt_clear_key(services_handle, key_handle, &error_code);
  SERVICES_key_mgmt_clear_key(services_handle, peer_key_handle, &error_code);
  SERVICES_key_mgmt_clear_key(services_handle, session_key_handle, &error_code);
  SERVICES_key_mgmt_clear_key(services_handle, peer_session_key_handle,
			      &error_code);
}

static void test_key_mgmt_ecdsa(uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t crypto_error_code = SERVICES_REQ_SUCCESS;

  // Generate an ECC key pair for signing
  uint8_t public_key[SE_ECC_P384_PUBKEY_SIZE];
  uint32_t public_key_len;
  uint32_t key_handle;
  SERVICES_key_mgmt_generate_ecc_key(
      services_handle, SE_KEY_TYPE_ECC_P256_ECDSA, &key_handle, public_key,
      &public_key_len, &error_code, &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_generate_ecc_key  key_len: %d  handle: "
	     "0x%X service_resp=0x%08X crypto_resp=0x%08X\n",
	     public_key_len, key_handle, error_code, crypto_error_code);

  uint8_t digest[48] = {0x12}; // ECDSA Verify fails if the digect is all 0s
  uint8_t signature[96] = {0x0};
  uint32_t signature_len;
  crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_ecdsa_sign(services_handle, key_handle, digest, 32,
			       signature, &signature_len, &error_code,
			       &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_ecdsa_sign  Signature length: %d       "
	     "     service_resp=0x%08X crypto_resp=0x%08X\n",
	     signature_len, error_code, crypto_error_code);

  char *print_buf = fill_print_buf(signature, 16);
  TEST_print(services_handle, "Signature: %s...\n", print_buf);

  // Verify signature
  crypto_error_code = SERVICES_REQ_SUCCESS;
  SERVICES_key_mgmt_ecdsa_verify(services_handle, SE_ECC_CURVE_P256, digest, 32,
				 public_key, public_key_len, signature,
				 signature_len, &error_code,
				 &crypto_error_code);
  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_ecdsa_verify                           "
	     "     service_resp=0x%08X crypto_resp=0x%08X\n",
	     error_code, crypto_error_code);

  // Cleanup
  SERVICES_key_mgmt_clear_key(services_handle, key_handle, &error_code);
}

static void test_key_mgmt_derive_key(uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t crypto_error_code = SERVICES_REQ_SUCCESS;

  uint32_t key_handle;
  SERVICES_key_mgmt_import_key(services_handle, SE_KEY_TYPE_AES_256, KEY_AES256,
			       sizeof(KEY_AES256), &key_handle, &error_code);
  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_import_key   base key handle: 0x%X     "
	     "          service_resp=0x%08X\n",
	     key_handle, error_code);

  uint8_t nonce[16] = {0x0};
  uint8_t label[16] = {0x0};
  uint32_t derived_key_handle;
  SERVICES_key_mgmt_derive_key(
      services_handle, key_handle, SE_HKDF_SHA256, SE_KEY_TYPE_AES_256, nonce,
      16, label, 16, &derived_key_handle, &error_code, &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_derive_key   derived key handle: 0x%X  "
	     "          service_resp=0x%08X crypto_resp=0x%08X\n",
	     derived_key_handle, error_code, crypto_error_code);

  crypto_error_code = SERVICES_REQ_SUCCESS;
  uint32_t root_derived_key_handle;
  SERVICES_key_mgmt_derive_key(services_handle, SE_KEY_ID_HUK, SE_HKDF_SHA256,
			       SE_KEY_TYPE_AES_256, nonce, 16, label, 16,
			       &root_derived_key_handle, &error_code,
			       &crypto_error_code);

  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_derive_key FROM ROOT  derived key "
	     "handle: 0x%X   service_resp=0x%08X crypto_resp=0x%08X\n",
	     root_derived_key_handle, error_code, crypto_error_code);

  // Cleanup
  SERVICES_key_mgmt_clear_key(services_handle, key_handle, &error_code);
  SERVICES_key_mgmt_clear_key(services_handle, derived_key_handle, &error_code);
  SERVICES_key_mgmt_clear_key(services_handle, root_derived_key_handle,
			      &error_code);
}

static void test_key_mgmt_disable_key_services(uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t crypto_error_code = SERVICES_REQ_SUCCESS;

  SERVICES_key_mgmt_disable_key_services(services_handle, &error_code);
  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_disable_key_services                   "
	     " service_resp=0x%08X\n",
	     error_code);

  uint32_t key_handle;
  SERVICES_key_mgmt_import_key(services_handle, SE_KEY_TYPE_AES_256, KEY_AES256,
			       sizeof(KEY_AES256), &key_handle, &error_code);
  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_import_key   base key handle: 0x%X     "
	     " service_resp=0x%08X\n",
	     key_handle, error_code);

  uint8_t nonce[16] = {0x0};
  uint8_t label[16] = {0x0};
  uint32_t derived_key_handle;
  SERVICES_key_mgmt_derive_key(
      services_handle, key_handle, SE_HKDF_SHA256, SE_KEY_TYPE_AES_256, nonce,
      16, label, 16, &derived_key_handle, &error_code, &crypto_error_code);
  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_derive_key   derived key handle: 0x%X  "
	     " service_resp=0x%08X crypto_resp=0x%08X\n",
	     derived_key_handle, error_code, crypto_error_code);

  uint8_t wrapped_key[SE_WRAPPED_KEY_MAX_SIZE] = {0};
  uint32_t wrapped_key_len;

  SERVICES_key_mgmt_wrap_key(services_handle, key_handle, wrapped_key,
			     &wrapped_key_len, &error_code, &crypto_error_code);
  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_wrap_key                               "
	     " service_resp=0x%08X crypto_resp=0x%08X\n",
	     error_code, crypto_error_code);

  uint32_t unwrapped_key_handle;
  SERVICES_key_mgmt_unwrap_key(services_handle, wrapped_key, wrapped_key_len,
			       &unwrapped_key_handle, &error_code,
			       &crypto_error_code);
  TEST_print(services_handle,
	     "** TEST SERVICES_key_mgmt_unwrap_key                             "
	     " service_resp=0x%08X crypto_resp=0x%08X\n",
	     error_code, crypto_error_code);

  // Cleanup
  SERVICES_key_mgmt_clear_key(services_handle, key_handle, &error_code);
  SERVICES_key_mgmt_clear_key(services_handle, derived_key_handle, &error_code);
  SERVICES_key_mgmt_clear_key(services_handle, unwrapped_key_handle,
			      &error_code);

  // test AES using KCP when in 'services disabled' state
  test_key_mgmt_aes_crypt_by_handle_kcp(services_handle);
}

/**
 * @brief Runs all key management crypto tests
 *
 * @param services_handle
 */
void test_key_management(uint32_t services_handle)
{
  TEST_print(services_handle, "== STARTING KEY MGMT TESTS      ==\n");

  TEST_print(
      services_handle,
      "----- TEST SE STORAGE ---------------------------------------------\n");
  test_key_mgmt_import_clear_key(services_handle);
  TEST_print(
      services_handle,
      "----- TEST AES CRYPT BY HANDLE ------------------------------------\n");
  test_key_mgmt_aes_crypt_by_handle(services_handle);
  TEST_print(
      services_handle,
      "----- TEST ECC GENERATE/DERIVE ------------------------------------\n");
  test_key_mgmt_generate_get_ecc_key(services_handle);
  TEST_print(
      services_handle,
      "----- TEST HMAC ---------------------------------------------------\n");
  test_key_mgmt_hmac_by_handle(services_handle);
  TEST_print(
      services_handle,
      "----- TEST KEY WRAPPING -------------------------------------------\n");
  test_key_mgmt_wrap_unwrap_key(services_handle);
  TEST_print(
      services_handle,
      "----- TEST ECDH ----------------------------------------------------\n");
  test_key_mgmt_ecdh(services_handle);
  TEST_print(
      services_handle,
      "----- TEST ECDSA --------------------------------------------------\n");
  test_key_mgmt_ecdsa(services_handle);
  TEST_print(
      services_handle,
      "----- TEST Derive Key ---------------------------------------------\n");
  test_key_mgmt_derive_key(services_handle);
  TEST_print(
      services_handle,
      "----- TEST Disable Key Services -----------------------------------\n");
  test_key_mgmt_disable_key_services(services_handle);

  TEST_print(services_handle, "== FINISHED KEY MGMT TESTS      ==\n");
  display_key_table(services_handle);
}
