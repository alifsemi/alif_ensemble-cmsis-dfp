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
 * @file  services_host_key_management.c
 * @brief Key management services test harness
 * @ingroup services
 * @par
 */

/******************************************************************************
 *  I N C L U D E   F I L E S
 *****************************************************************************/
#include "services_lib_api.h"
#include "services_lib_ids.h"
#include "services_lib_protocol.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#if defined(A32_LINUX)
#include "a32_linux.h"
#else
//#include "system_utils.h"
#include "sys_utils.h"
#endif

/*******************************************************************************
 *  M A C R O   D E F I N E S
 ******************************************************************************/

/*******************************************************************************
 *  T Y P E D E F S
 ******************************************************************************/

/*******************************************************************************
 *  G L O B A L   V A R I A B L E S
 ******************************************************************************/

/*******************************************************************************
 *  C O D E
 ******************************************************************************/

static bool km_request_ok(uint32_t ret, uint32_t error_code)
{
	return (ret == SERVICES_REQ_SUCCESS) && (error_code == SE_KM_SUCCESS);
}

/**
 * @brief Import a plain text key
 *
 * @param services_handle
 * @param key_type[in]     se_key_type_t
 * @param key_addr[in]     key address
 * @param key_len[in]      key length
 * @param key_handle[out]  se_key_handle_t
 * @param error_code
 * @return
 * @ingroup services-host-crypto
 */
uint32_t SERVICES_key_mgmt_import_key(uint32_t services_handle,
					      se_key_type_t key_type,
					      const uint8_t *key, uint32_t key_len,
					      se_key_handle_t *key_handle,
					      uint32_t *error_code)
{
	key_mgmt_import_key_svc_t *p_svc =
		(key_mgmt_import_key_svc_t *)SERVICES_prepare_packet_buffer(
			sizeof(key_mgmt_import_key_svc_t));

  p_svc->send_key_type = key_type;
  p_svc->send_key_addr = LocalToGlobal(key);
  p_svc->send_key_len = key_len;

  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_KEY_MGMT_IMPORT_KEY, DEFAULT_TIMEOUT);

  if (km_request_ok(ret, p_svc->resp_error_code)) {
    *key_handle = p_svc->resp_key_handle;
  }
  *error_code = p_svc->resp_error_code;
  return ret;
}

/**
 * @brief Clear a key slot in SE
 *
 * @param services_handle
 * @param key_handle[in]  se_key_handle_t
 * @param error_code
 * @return
 * @ingroup services-host-crypto
 */
uint32_t SERVICES_key_mgmt_clear_key(uint32_t services_handle,
					     se_key_handle_t key_handle,
					     uint32_t *error_code)
{
	key_mgmt_clear_key_svc_t *p_svc =
		(key_mgmt_clear_key_svc_t *)SERVICES_prepare_packet_buffer(
			sizeof(key_mgmt_clear_key_svc_t));

  p_svc->send_key_handle = key_handle;
  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_KEY_MGMT_CLEAR_KEY, DEFAULT_TIMEOUT);
  *error_code = p_svc->resp_error_code;
  return ret;
}

/**
 * @brief Service for AES encryption/decryption by handle
 *
 * @param services_handle
 * @param key_handle[in]     se_key_handle_t
 * @param direction[in]      se_crypt_direction_t (encrypt/decrypt)
 * @param crypt_type[in]     se_crypt_type_t (ECB/CBC/CTR/OFB/CCM/CCM STAR/GCM)
 * @param iv[in]             Initialization Vector/Nonce
 * @param iv_len[in]         IV length
 * @param input[in]          Input data
 * @param input_len[in]      Input length
 * @param output[out]        Output buffer
 * @param aad[in]            Additional authenticated data (AAD)
 * @param aad_len[in]        AAD length
 * @param tag[out]           Buffer for generated TAG
 * @param tag_len[in]        TAG buffer length
 * @param error_code         Error code from the Key Management logic
 * @param crypto_error_code  Error code from the Crypto library
 * @return
 */
uint32_t SERVICES_key_mgmt_aes_crypt_by_handle(
    uint32_t services_handle, se_key_handle_t key_handle,
    se_crypt_direction_t direction, se_crypt_type_t crypt_type, uint8_t *iv,
    uint32_t iv_len, const uint8_t *input, uint32_t input_len, uint8_t *output,
    const uint8_t *aad, uint32_t aad_len, uint8_t *tag, uint32_t tag_len,
    uint32_t *error_code, uint32_t *crypto_error_code)
{
	key_mgmt_aes_crypt_by_handle_svc_t *p_svc =
		(key_mgmt_aes_crypt_by_handle_svc_t *)SERVICES_prepare_packet_buffer(
			sizeof(key_mgmt_aes_crypt_by_handle_svc_t));

  p_svc->send_key_handle = key_handle;
  p_svc->send_direction = direction;
  p_svc->send_crypt_type = crypt_type;
  p_svc->send_iv_addr = LocalToGlobal(iv);
  p_svc->send_iv_len = iv_len;
  p_svc->send_input_addr = LocalToGlobal(input);
  p_svc->send_input_len = input_len;
  p_svc->send_output_addr = LocalToGlobal(output);
  p_svc->send_aad_addr = LocalToGlobal(aad);
  p_svc->send_aad_len = aad_len;
  p_svc->send_tag_addr = LocalToGlobal(tag);
  p_svc->send_tag_len = tag_len;

  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_KEY_MGMT_AES_CRYPT_BY_HANDLE, DEFAULT_TIMEOUT);

  *error_code = p_svc->resp_error_code;
  *crypto_error_code = p_svc->resp_crypto_error_code;
  return ret;
}

/**
 * @brief Service to generate an ECC key
 *
 * @param services_handle
 * @param key_type[int]        se_key_type_t (must be an ECC type)
 * @param key_handle[out]      Handle of generated private key
 * @param public_key[out]      Generated public key
 * @param public_key_len[out]  Public key length
 * @param error_code           Error code from the Key Management logic
 * @param crypto_error_code    Error code from the Crypto library
 * @return
 */
uint32_t SERVICES_key_mgmt_generate_ecc_key(
    uint32_t services_handle, se_key_type_t key_type, uint32_t *key_handle,
    uint8_t *public_key, uint32_t *public_key_len, uint32_t *error_code,
    uint32_t *crypto_error_code)
{
	key_mgmt_generate_ecc_key_svc_t *p_svc =
		(key_mgmt_generate_ecc_key_svc_t *)SERVICES_prepare_packet_buffer(
			sizeof(key_mgmt_generate_ecc_key_svc_t));

  p_svc->send_key_type = key_type;
  // p_svc->public_key = (uint32_t)public_key;

  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_KEY_MGMT_GENERATE_ECC_KEY, DEFAULT_TIMEOUT);

  if (km_request_ok(ret, p_svc->resp_error_code)) {
	memcpy(public_key, (const void *)p_svc->public_key,
	       p_svc->resp_public_key_len);
    *key_handle = p_svc->resp_key_handle;
    *public_key_len = p_svc->resp_public_key_len;
  }

  *error_code = p_svc->resp_error_code;
  *crypto_error_code = p_svc->resp_crypto_error_code;
  return ret;
}

/**
 * @brief Service to get an ECC public key
 *
 * @param services_handle
 * @param key_handle[in]       Handle of ECC private key in storage
 * @param public_key[out]      Public key derived from the private key
 * @param public_key_len[out]  Public key length
 * @param error_code           Error code from the Key Management logic
 * @param crypto_error_code    Error code from the Crypto library
 * @return
 */
uint32_t SERVICES_key_mgmt_get_ecc_public_key(uint32_t services_handle,
						      uint32_t key_handle,
						      uint8_t *public_key,
						      uint32_t *public_key_len,
						      uint32_t *error_code,
						      uint32_t *crypto_error_code)
{
	key_mgmt_get_ecc_pubkey_svc_t *p_svc =
		(key_mgmt_get_ecc_pubkey_svc_t *)SERVICES_prepare_packet_buffer(
			sizeof(key_mgmt_get_ecc_pubkey_svc_t));

  p_svc->send_key_handle = key_handle;
  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_KEY_MGMT_GET_ECC_PUBLIC_KEY, DEFAULT_TIMEOUT);

  if (km_request_ok(ret, p_svc->resp_error_code)) {
	memcpy((void *)public_key, (void *)p_svc->resp_public_key,
	       p_svc->resp_public_key_len);
    *public_key_len = p_svc->resp_public_key_len;
  }

  *error_code = p_svc->resp_error_code;
  *crypto_error_code = p_svc->resp_crypto_error_code;
  return ret;
}

/**
 * @brief Service for HMAC generation/verification by handle
 *
 * @param services_handle
 * @param key_handle[in]     Handle of ECC private key in storage
 * @param hmac_algo[in]      se_hmac_algo_t
 * @param input[in]          Input data
 * @param input_len[in]      Input data length
 * @param mac[out]           Generated HMAC
 * @param mac_len[out]       HMAC length
 * @param error_code         Error code from the Key Management logic
 * @param crypto_error_code  Error code from the Crypto library
 * @return
 */
uint32_t SERVICES_key_mgmt_hmac_by_handle(
    uint32_t services_handle, uint32_t key_handle, se_hmac_algo_t hmac_algo,
    const uint8_t *input, uint32_t input_len, uint8_t *mac, uint32_t *mac_len,
    uint32_t *error_code, uint32_t *crypto_error_code)
{
	key_mgmt_hmac_by_handle_svc_t *p_svc =
		(key_mgmt_hmac_by_handle_svc_t *)SERVICES_prepare_packet_buffer(
			sizeof(key_mgmt_hmac_by_handle_svc_t));

  p_svc->send_key_handle = key_handle;
  p_svc->send_hmac_algo = hmac_algo;
  p_svc->send_input_addr = LocalToGlobal(input);
  p_svc->send_input_len = input_len;

  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_KEY_MGMT_HMAC_BY_HANDLE, DEFAULT_TIMEOUT);

  if (km_request_ok(ret, p_svc->resp_error_code)) {
    memcpy((void *)mac, (void *)p_svc->resp_mac, p_svc->resp_mac_len);
    *mac_len = p_svc->resp_mac_len;
  }

  *error_code = p_svc->resp_error_code;
  *crypto_error_code = p_svc->resp_crypto_error_code;
  return ret;
}

/**
 * @brief Service to wrap (export) a key
 *
 * @param services_handle
 * @param key_handle[in]           Handle of ECC private key in storage
 * @param wrapped_key[out]         Wrapped key blob
 * @param wrapped_key_len[output]  Wrapped key blob length
 * @param error_code               Error code from the Key Management logic
 * @param crypto_error_code        Error code from the Crypto library
 * @return
 */
uint32_t SERVICES_key_mgmt_wrap_key(uint32_t services_handle,
					    uint32_t key_handle, uint8_t *wrapped_key,
					    uint32_t *wrapped_key_len,
					    uint32_t *error_code,
					    uint32_t *crypto_error_code)
{
	key_mgmt_wrap_key_svc_t *p_svc =
		(key_mgmt_wrap_key_svc_t *)SERVICES_prepare_packet_buffer(
			sizeof(key_mgmt_wrap_key_svc_t));

  p_svc->send_key_handle = key_handle;
  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_KEY_MGMT_WRAP_KEY, DEFAULT_TIMEOUT);

  if (km_request_ok(ret, p_svc->resp_error_code)) {
	memcpy(wrapped_key, (void *)p_svc->resp_wrapped_key,
	       p_svc->resp_wrapped_key_len);
    *wrapped_key_len = p_svc->resp_wrapped_key_len;
  }

  *error_code = p_svc->resp_error_code;
  *crypto_error_code = p_svc->resp_crypto_error_code;
  return ret;
}

/**
 * @brief Service to unwrap (import) a wrapped key into an empty slot in storage
 *
 * @param services_handle
 * @param wrapped_key[in]      Wrapped key blob
 * @param wrapped_key_len[in]  Wrapped key blob length
 * @param key_handle[out]      Handle of unwrapped key in storage
 * @param error_code           Error code from the Key Management logic
 * @param crypto_error_code    Error code from the Crypto library
 * @return
 */
uint32_t SERVICES_key_mgmt_unwrap_key(uint32_t services_handle,
					      const uint8_t *wrapped_key,
					      uint32_t wrapped_key_len,
					      uint32_t *key_handle,
					      uint32_t *error_code,
					      uint32_t *crypto_error_code)
{
	key_mgmt_unwrap_key_svc_t *p_svc =
		(key_mgmt_unwrap_key_svc_t *)SERVICES_prepare_packet_buffer(
			sizeof(key_mgmt_unwrap_key_svc_t));

  if (wrapped_key_len > SE_WRAPPED_KEY_MAX_SIZE) {
    *error_code = SE_KM_ERROR_INVALID_KEY_SIZE;
    return SERVICES_REQ_SUCCESS;
  }

  memcpy((void *)p_svc->send_wrapped_key, wrapped_key, wrapped_key_len);
  p_svc->send_wrapped_key_len = wrapped_key_len;

  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_KEY_MGMT_UNWRAP_KEY, DEFAULT_TIMEOUT);

  if (km_request_ok(ret, p_svc->resp_error_code)) {
    *key_handle = p_svc->resp_key_handle;
  }

  *error_code = p_svc->resp_error_code;
  *crypto_error_code = p_svc->resp_crypto_error_code;
  return ret;
}

/**
 * @brief Service for ECDH key agreement
 *
 * @param services_handle
 * @param key_handle[in]           Handle of ECDH private key in storage
 * @param curve[in]                Curve type
 * (SE_ECC_CURVE_P256/SE_ECC_CURVE_P384)
 * @param peer_pubkey[in]          Public ECDH key of peer
 * @param peer_pubkey_len[in]      Public ECDH key of peer length
 * @param kdf_algo[in]             KDF algorithm (SE_HKDF_SHA256/SE_HKDF_SHA384)
 * @param derived_key_type[in]     se_key_type_t for output
 * @param salt[in]                 Salt
 * @param salt_len[in]             Salt length
 * @param session_key_handle[out]  Agreed secret AES key
 * @param error_code               Error code from the Key Management logic
 * @param crypto_error_code        Error code from the Crypto library
 * @return
 */
uint32_t
SERVICES_key_mgmt_ecdh(uint32_t services_handle, uint32_t key_handle,
		       se_ecc_curve_t curve, const uint8_t *peer_pubkey,
		       uint32_t peer_pubkey_len, se_hkdf_algo_t kdf_algo,
		       se_key_type_t derived_key_type, const uint8_t *salt,
		       uint32_t salt_len,
		       uint32_t *session_key_handle, // output
		       uint32_t *error_code, uint32_t *crypto_error_code)
{
	key_mgmt_ecdh_svc_t *p_svc =
		(key_mgmt_ecdh_svc_t *)SERVICES_prepare_packet_buffer(
			sizeof(key_mgmt_ecdh_svc_t));

  p_svc->send_private_key_handle = key_handle;
  p_svc->send_curve = curve;
  p_svc->send_peer_pubkey_addr = LocalToGlobal(peer_pubkey);
  p_svc->send_peer_pubkey_len = peer_pubkey_len;
  p_svc->send_kdf_algo = kdf_algo;
  p_svc->send_derived_key_type = derived_key_type;
  p_svc->send_salt_addr = LocalToGlobal(salt);
  p_svc->send_salt_len = salt_len;

  uint32_t ret = SERVICES_send_request(services_handle, SERVICE_KEY_MGMT_ECDH,
				       DEFAULT_TIMEOUT);

  if (km_request_ok(ret, p_svc->resp_error_code)) {
    *session_key_handle = p_svc->resp_session_key_handle;
  }

  *error_code = p_svc->resp_error_code;
  *crypto_error_code = p_svc->resp_crypto_error_code;
  return ret;
}

/**
 * @brief Service for ECDSA sign
 *
 * @param services_handle
 * @param key_handle[in]      Handle of ECDSA private key in storage
 * @param digest[in]          Digest of message to sign
 * @param digest_len[in]      Digest length
 * @param signature[out]      Salt
 * @param signature_len[out]  Salt length
 * @param error_code          Error code from the Key Management logic
 * @param crypto_error_code   Error code from the Crypto library
 * @return
 */
uint32_t SERVICES_key_mgmt_ecdsa_sign(
    uint32_t services_handle, uint32_t key_handle, const uint8_t *digest,
    uint32_t digest_len, uint8_t *signature, uint32_t *signature_len,
    uint32_t *error_code, uint32_t *crypto_error_code)
{
	key_mgmt_ecdsa_sign_svc_t *p_svc =
		(key_mgmt_ecdsa_sign_svc_t *)SERVICES_prepare_packet_buffer(
			sizeof(key_mgmt_ecdsa_sign_svc_t));

  p_svc->send_key_handle = key_handle;
  p_svc->send_digest_addr = LocalToGlobal(digest);
  p_svc->send_digest_len = digest_len;
  p_svc->send_signature_addr = LocalToGlobal(signature);

  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_KEY_MGMT_ECDSA_SIGN, DEFAULT_TIMEOUT);

  if (km_request_ok(ret, p_svc->resp_error_code)) {
    *signature_len = p_svc->resp_signature_len;
  }

  *error_code = p_svc->resp_error_code;
  *crypto_error_code = p_svc->resp_crypto_error_code;
  return ret;
}

/**
 * @brief Service for ECDSA verify
 *
 * @param services_handle
 * @param se_ecc_curve_t[in]  se_ecc_curve_t
 * @param digest[in]          Digest of message to verify
 * @param digest_len[in]      Digest length
 * @param pubkey[out]         Public ECDSSA key to use for verification
 * @param pubkey_len[out]     Public ECDSSA key length
 * @param signature[out]      Signature of message to verify
 * @param signature_len[out]  Signature length
 * @param error_code          Error code from the Key Management logic
 * @param crypto_error_code   Error code from the Crypto library
 * @return
 */
uint32_t SERVICES_key_mgmt_ecdsa_verify(
    uint32_t services_handle, se_ecc_curve_t curve, const uint8_t *digest,
    uint32_t digest_len, const uint8_t *pubkey, uint32_t pubkey_len,
    const uint8_t *signature, uint32_t signature_len, uint32_t *error_code,
    uint32_t *crypto_error_code)
{
	key_mgmt_ecdsa_verify_svc_t *p_svc =
		(key_mgmt_ecdsa_verify_svc_t *)SERVICES_prepare_packet_buffer(
			sizeof(key_mgmt_ecdsa_verify_svc_t));

  p_svc->send_curve = curve;
  p_svc->send_digest_addr = LocalToGlobal(digest);
  p_svc->send_digest_len = digest_len;
  p_svc->send_pubkey_addr = LocalToGlobal(pubkey);
  p_svc->send_pubkey_len = pubkey_len;
  p_svc->send_signature_addr = LocalToGlobal(signature);
  p_svc->send_signature_len = signature_len;

  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_KEY_MGMT_ECDSA_VERIFY, DEFAULT_TIMEOUT);

  *error_code = p_svc->resp_error_code;
  *crypto_error_code = p_svc->resp_crypto_error_code;
  return ret;
}

/**
 * @brief Service to derive a key
 *
 * @param services_handle
 * @param base_key_handle[in]      Handle of base key to derive from
 * @param kdf_algo[in]             KDF algorithm (SE_HMAC_SHA256/SE_HMAC_SHA384)
 * @param derived_key_type[in]     Derived key type
 * @param nonce[in]                NONCE
 * @param nonce_len[in]            NONCE length
 * @param label[in]                Label
 * @param label_len[in]            Label length
 * @param derived_key_handle[out]  Handle of derived key
 * @param error_code               Error code from the Key Management logic
 * @param crypto_error_code        Error code from the Crypto library
 * @return
 */
uint32_t SERVICES_key_mgmt_derive_key(
    uint32_t services_handle, uint32_t base_key_handle, se_hkdf_algo_t kdf_algo,
    se_key_type_t derived_key_type, const uint8_t *nonce, uint32_t nonce_len,
    const uint8_t *label, uint32_t label_len, uint32_t *derived_key_handle,
    uint32_t *error_code, uint32_t *crypto_error_code)
{
	key_mgmt_derive_key_svc_t *p_svc =
		(key_mgmt_derive_key_svc_t *)SERVICES_prepare_packet_buffer(
			sizeof(key_mgmt_derive_key_svc_t));

  p_svc->send_base_key_handle = base_key_handle;
  p_svc->send_kdf_algo = kdf_algo;
  p_svc->send_derived_key_type = derived_key_type;
  p_svc->send_nonce_addr = LocalToGlobal(nonce);
  p_svc->send_nonce_len = nonce_len;
  p_svc->send_label_addr = LocalToGlobal(label);
  p_svc->send_label_len = label_len;
  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_KEY_MGMT_DERIVE_KEY, DEFAULT_TIMEOUT);

  if (km_request_ok(ret, p_svc->resp_error_code)) {
    *derived_key_handle = p_svc->resp_derived_key_handle;
  }

  *error_code = p_svc->resp_error_code;
  *crypto_error_code = p_svc->resp_crypto_error_code;
  return ret;
}

/**
 * @brief Service to disable key services
 *
 * @param services_handle
 * @param error_code
 * @return
 */
uint32_t SERVICES_key_mgmt_disable_key_services(uint32_t services_handle,
							uint32_t *error_code)
{
	generic_svc_t *p_svc =
		(generic_svc_t *)SERVICES_prepare_packet_buffer(sizeof(generic_svc_t));

  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_KEY_MGMT_DISABLE_KEY_SERVICES, DEFAULT_TIMEOUT);

  *error_code = p_svc->resp_error_code;
  return ret;
}

/**
 * @brief Get Key slot details based on a key_slot index
 * @param services_handle
 * @param key_index
 * @param key_attr
 * @param error_code
 * @return
 */
uint32_t SERVICES_key_get_key_attributes(uint32_t services_handle,
						 uint32_t key_index,
						 key_attributes_t *key_attr,
						 uint32_t *error_code)
{
	key_mgmt_get_attributes_t *p_svc =
		(key_mgmt_get_attributes_t *)SERVICES_prepare_packet_buffer(
			sizeof(key_mgmt_get_attributes_t));

  p_svc->send_key_handle = key_index;

  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_KEY_MGMT_GET_ATTRIBUTES, DEFAULT_TIMEOUT);
  /* unpack response */
  key_attr->algorithm = p_svc->resp_algorithm;
  key_attr->key_identifier = p_svc->resp_key_identifier;
  key_attr->key_size = p_svc->resp_key_size;
  key_attr->key_type = p_svc->resp_key_type;
  key_attr->lifetime = p_svc->resp_lifetime;
  key_attr->usage_flags = p_svc->resp_usage_flags;

  *error_code = p_svc->resp_error_code;
  return ret;
}

/**
 * @brief Get Key slot details based on a key_slot handle
 * @param services_handle
 * @param key_handle
 * @param key_attr
 * @param error_code
 * @return
 */
uint32_t SERVICES_key_get_key_handle_attributes(uint32_t services_handle,
							uint32_t key_handle,
							key_attributes_t *key_attr,
							uint32_t *error_code)
{
	key_mgmt_get_attributes_t *p_svc =
		(key_mgmt_get_attributes_t *)SERVICES_prepare_packet_buffer(
			sizeof(key_mgmt_get_attributes_t));

  p_svc->send_key_handle = key_handle;

  uint32_t ret = SERVICES_send_request(
      services_handle, SERVICE_KEY_MGMT_GET_ATTRIBUTES_HANDLE, DEFAULT_TIMEOUT);
  /* unpack response */
  key_attr->algorithm = p_svc->resp_algorithm;
  key_attr->key_identifier = p_svc->resp_key_identifier;
  key_attr->key_size = p_svc->resp_key_size;
  key_attr->key_type = p_svc->resp_key_type;
  key_attr->lifetime = p_svc->resp_lifetime;
  key_attr->usage_flags = p_svc->resp_usage_flags;

  *error_code = p_svc->resp_error_code;
  return ret;
}
