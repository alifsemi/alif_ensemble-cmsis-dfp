/**
 * @file services_lib_api.h
 *
 * @brief Services library public API header file
 * @defgroup host_services host_services
 * @par
 *
 * Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
#ifndef __SERVICES_LIB_API_H__
#define __SERVICES_LIB_API_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 *  I N C L U D E   F I L E S
 *****************************************************************************/
#include <stddef.h>
#include <stdbool.h>
#include "services_lib_protocol.h"
#include "aipm.h"

/*******************************************************************************
 *  M A C R O   D E F I N E S
 ******************************************************************************/

/**
 * Default service call timeout
 */
#define DEFAULT_TIMEOUT                              (0)

/**
 * Maximum size of a Services packet
 */
#define SERVICES_MAX_PACKET_BUFFER_SIZE              600

/**
 * Common Service error codes - follow the pattern from the PLL services
 */
#define SERVICE_SUCCESS                              0x0
#define SERVICE_FAIL                                 0x200
#define SERVICE_INVALID_PARAMETER                    0x201

/**
 * Pin muxing/pad control error codes
 */
#define PINMUX_SUCCESS                               0x0
#define PINMUX_ERROR_INVALID_PARAMETER               0x200

/**
 * OSPI Write Key error codes
 */
#define OSPI_WRITE_KEY_SUCCESS                       0x0
#define OSPI_WRITE_KEY_ERROR_INVALID_PARAMETER       0x200
#define OSPI_WRITE_KEY_ERROR_OTP_READ_FAILED         0x201

/**
 * Crypto services error codes - use values not used by MbedTLS
 */
#define CRYPTOCELL_SUCCESS                           0x0
#define CRYPTOCELL_ERROR_INVALID_CRYPT_TYPE          0xFFFFFFFFul
#define CRYPTOCELL_ERROR_INVALID_SHA_TYPE            0xFFFFFFFEul
#define CRYPTOCELL_ERROR_INVALID_KEY_TYPE            0xFFFFFFFDul
#define CRYPTOCELL_ERROR_INVALID_SEND_DIRECTION      0xFFFFFFFCul

/**
 * Clocks services error codes
 */
#define PLL_SUCCESS                                  0x0
#define PLL_ERROR_INVALID_PARAMETER                  0x200
#define PLL_ERROR_PLL_NOT_RUNNING                    0x201
#define PLL_ERROR_PLL_ALREADY_RUNNING                0x202
#define PLL_ERROR_XTAL_NOT_RUNNING                   0x203
#define PLL_ERROR_XTAL_ALREADY_RUNNING               0x204

/**
 * Boot services error codes
 */
#define BL_STATUS_OK                                 0x00
#define BL_ERROR_APP_INVALID_TOC_ADDRESS             0x01
#define BL_ERROR_APP_INVALID_TOC_OFFSET              0x02
#define BL_ERROR_UNALIGNED_ADDRESS                   0x03
#define BL_ERROR_INVALID_TOC_ADDRESS_RANGE           0x04
#define BL_ERROR_INVALID_TOC_FLAGS                   0x05
#define BL_ERROR_INVALID_ADDRESS                     0x06
#define BL_ERROR_CERTIFICATE_NO_VERIFY_IN_MEMORY     0x07
#define BL_ERROR_CERTIFICATE_NO_VERIFY_IN_FLASH      0x08
#define BL_ERROR_CERTIFICATE_INVALID_LOAD_ADDRESS    0x09
#define BL_ERROR_CERTIFICATE_INVALID_CHAIN           0x0A
#define BL_ERROR_CERTIFICATE_STORAGE_ADDRESS_INVALID 0x0B
#define BL_ERROR_DEVICE_ADDRESS_INVALID              0x0C
#define BL_ERROR_UNCOMPRESS_FAILED                   0x0D
#define BL_ERROR_SIGNATURE_VERIFY_FAILED             0x0E
#define BL_ERROR_APP_ACCESSING_PROTECTED_AREA        0x0F
#define BL_ERROR_ICV_ACCESSING_PROTECTED_AREA        0x10
#define BL_ERROR_FAILED_TOC_CRC32                    0x11
#define BL_ERROR_INVALID_TOC                         0x12
#define BL_ERROR_EXCEED_MAXIMUM_TOC_ENTRIES          0x13
#define BL_ERROR_NOT_IMAGE_NOT_DEVICE_CFG            0x14
#define BL_ERROR_INVALID_TOC_ENTRY_ID                0x15
#define BL_ERROR_INVALID_CPU_ID                      0x16
#define BL_ERROR_ENTRY_NOT_SIGNED                    0x17
#define BL_ERROR_LOAD_TO_MRAM_NOT_ALLOWED            0x18
#define BL_ERROR_NO_FREE_SLOTS                       0x19
#define BL_ERROR_INVALID_M55_BOOT_ADDRESS            0x1A
#define BL_TOC_OBJECT_NOT_FOUND                      0x1B
#define BL_TOC_OBJECT_FOR_CPU_NOT_FOUND              0x1C
#define BL_TOC_IMAGE_NOT_BOOTABLE                    0x1D
#define BL_TOC_IMAGE_NOT_FOUND                       0x1E
#define BL_ERROR_COMPRESSION_NOT_SUPPORTED           0x1F
#define BL_TOC_IMAGE_DEVICE_MISMATCH                 0x20
#define BL_ERROR_UPD_SIGNATURE_INCORRECT             0x21
#define BL_ERROR_UPD_IMG_IN_MRAM_NOT_SUPPORTED       0x22
#define BL_ERROR_HAVE_FAILED_ATOC_IMAGES             0x23
#define BL_ERROR_UPD_IMG_IN_EXT_MEM_NOT_SUPPORTED    0x24
/**
 * OTP Offsets
 */
#define OTP_CUSTOM_AREA_START                        0x70

/**
 * MBED TLS
 */
#define MBEDTLS_OP_DECRYPT                           0
#define MBEDTLS_OP_ENCRYPT                           1

#define MBEDTLS_AES_BLOCK_SIZE                       16

#define MBEDTLS_AES_CRYPT_ECB                        0
#define MBEDTLS_AES_CRYPT_CBC                        1
#define MBEDTLS_AES_CRYPT_CTR                        2
#define MBEDTLS_AES_CRYPT_OFB                        3

#define MBEDTLS_AES_KEY_128                          128
#define MBEDTLS_AES_KEY_192                          192
#define MBEDTLS_AES_KEY_256                          256

#define MBEDTLS_HASH_SHA1                            0
#define MBEDTLS_HASH_SHA224                          1
#define MBEDTLS_HASH_SHA256                          2

#define MBEDTLS_CCM_KEY                              0
#define MBEDTLS_GCM_KEY                              1

#define MBEDTLS_CCM_ENCRYPT_AND_TAG                  0
#define MBEDTLS_CCM_AUTH_DECRYPT                     1
#define MBEDTLS_CCM_STAR_ENCRYPT_AND_TAG             2
#define MBEDTLS_CCM_STAR_AUTH_DECRYPT                3
#define MBEDTLS_GCM_ENCRYPT_AND_TAG                  4
#define MBEDTLS_GCM_DECRYPT_AND_TAG                  5
#define MBEDTLS_GCM_AUTH_DECRYPT                     6

#define MBEDTLS_CHACHAPOLY_ENCRYPT_AND_TAG           0
#define MBEDTLS_CHACHAPOLY_AUTH_DECRYPT              1

/**
 * Key Management
 */

/**
 * Error codes specific to key management services.
 */
#define SE_KM_SUCCESS 0x0
#define SE_KM_ERROR_NO_FREE_SLOTS 0x1001
#define SE_KM_ERROR_INVALID_HANDLE 0x1002
#define SE_KM_ERROR_INVALID_KEY_TYPE 0x1003
#define SE_KM_ERROR_INVALID_KEY_SIZE 0x1004
#define SE_KM_ERROR_INVALID_KEY_FOR_OPERATION 0x1005
#define SE_KM_ERROR_INVALID_CURVE_TYPE 0x1006
#define SE_KM_ERROR_INVALID_KDF_TYPE 0x1007
#define SE_KM_ERROR_INVALID_CRYPT_TYPE 0x1008
#define SE_KM_ERROR_INVALID_HMAC_TYPE 0x1009
#define SE_KM_ERROR_INVALID_LABEL_LEN 0x100A
#define SE_KM_ERROR_INVALID_NONCE_LEN 0x100B
#define SE_KM_ERROR_KEY_SERVICES_DISABLED                                      \
  0x100C /* After DISABLE_KEY_SERVICES */
#define SE_KM_ERROR_AES_CRYPT_FAILED 0x100D
#define SE_KM_ERROR_GENERATE_ECC_KEY_FAILED 0x100E
#define SE_KM_ERROR_GET_ECC_PUBLIC_KEY_FAILED 0x100F
#define SE_KM_ERROR_HMAC_BY_HANDLE_FAILED 0x1010
#define SE_KM_ERROR_WRAP_FAILED 0x1011
#define SE_KM_ERROR_UNWRAP_AUTH_FAILED 0x1012 /* Tampered or wrong device */
#define SE_KM_ERROR_ECDH_FAILED 0x1013
#define SE_KM_ERROR_ECDSA_SIGN_FAILED 0x1014
#define SE_KM_ERROR_ECDSA_VERIFY_FAILED 0x1015
#define SE_KM_ERROR_DERIVE_KEY_FAILED 0x1016

typedef uint32_t se_key_handle_t;
#define SE_KEY_HANDLE_INVALID ((se_key_handle_t)0xFFFFFFFF)
#define SE_KEY_HANDLE_DEVICE_ROOT ((se_key_handle_t)0x00000000)

// CC312-controlled keys
#define SE_KEY_ID_HUK ((se_key_handle_t)0x00000000)
#define SE_KEY_ID_RTL ((se_key_handle_t)0x00000001)
#define SE_KEY_ID_KCP ((se_key_handle_t)0x00000002)
#define SE_KEY_ID_KCE ((se_key_handle_t)0x00000003)
#define SE_KEY_ID_KPICV ((se_key_handle_t)0x00000004)
#define SE_KEY_ID_KCEICV ((se_key_handle_t)0x00000005)
// Other OTP keys
#define SE_KEY_ID_DEVICE_ECC ((se_key_handle_t)0x00000010)
#define SE_KEY_ID_AES1 ((se_key_handle_t)0x00000011)
#define SE_KEY_ID_AES2 ((se_key_handle_t)0x00000012)
// Keys from SE key storage
#define SE_KEY_STORAGE_BASE 0x00000100

#define SE_NUM_KEY_SLOTS 8

#define WRAPPED_KEY_KEYTYPE_LEN 4
#define WRAPPED_KEY_METADATA_LEN 16
#define WRAPPED_KEY_IV_LEN 12
#define WRAPPED_KEY_TAG_LEN 16

#define SE_ECDSA_P256_SIG_LEN 64 /* 32B r + 32B s */
#define SE_ECDSA_P384_SIG_LEN 96 /* 48B r + 48B s */

#define SE_LABEL_MAX_LEN 64
#define SE_NONCE_MAX_LEN 64

/**
 * Key types supported by the SE key management services.
 */
typedef enum {
  SE_KEY_TYPE_NONE = 0x0000,           /* No key */
  SE_KEY_TYPE_AES_128 = 0x0001,        /* 128-bit AES key */
  SE_KEY_TYPE_AES_256 = 0x0002,        /* 256-bit AES key */
  SE_KEY_TYPE_ECC_P256_ECDSA = 0x0010, /* NIST P-256 for ECDSA */
  SE_KEY_TYPE_ECC_P384_ECDSA = 0x0011, /* NIST P-384 for ECDSA */
  SE_KEY_TYPE_ECC_P256_ECDH = 0x0020,  /* NIST P-256 for ECDH */
  SE_KEY_TYPE_ECC_P384_ECDH = 0x0021,  /* NIST P-384 for ECDH */
  SE_KEY_TYPE_HMAC_256 = 0x0030,       /* 256-bit HMAC key */
  SE_KEY_TYPE_HMAC_384 = 0x0031,       /* 384-bit HMAC key */
} se_key_type_t;

typedef enum {
  SE_CRYPT_DIRECTION_DECRYPT,
  SE_CRYPT_DIRECTION_ENCRYPT
} se_crypt_direction_t;

typedef enum {
  SE_CRYPT_TYPE_ECB,
  SE_CRYPT_TYPE_CBC,
  SE_CRYPT_TYPE_CTR,
  SE_CRYPT_TYPE_OFB,
  SE_CRYPT_TYPE_CCM,
  SE_CRYPT_TYPE_CCM_STAR,
  SE_CRYPT_TYPE_GCM
} se_crypt_type_t;

/**
 * ECC curve identifiers.
 */
typedef enum {
  SE_ECC_CURVE_P256 = 0,
  SE_ECC_CURVE_P384 = 1,
} se_ecc_curve_t;

/**
 * Hash algorithms for HMAC.
 */
typedef enum {
  SE_HMAC_SHA256 = 0,
  SE_HMAC_SHA384 = 1,
} se_hmac_algo_t;

/**
 * Hash algorithms for HKDF.
 */
typedef enum {
  SE_HKDF_SHA256 = 0,
  SE_HKDF_SHA384 = 1,
} se_hkdf_algo_t;

/**
 * secure key management
 * @enum key_attributes_t
 * @brief Secure key attributes
 */
typedef struct {
  uint32_t lifetime;       /*!< key storage     */
  uint32_t key_identifier; /*!< key handle      */
  uint32_t key_type;       /*!< Key slot type   */
  uint32_t key_size;       /*!< key size        */
  uint32_t usage_flags;    /*!< key usage flags */
  uint32_t algorithm;      /*!< key algorithm   */
} key_attributes_t;

/**
 * OSPI write key commands
 */
#define OSPI_WRITE_OTP_KEY_OSPI0                     0
#define OSPI_WRITE_OTP_KEY_OSPI1                     1
#define OSPI_WRITE_EXTERNAL_KEY_OSPI0                2
#define OSPI_WRITE_EXTERNAL_KEY_OSPI1                3

#define OSPI_KEY_LENGTH_BYTES                        16

/**
 * TOC related
 */
#define TOC_NAME_LENGTH                              8
#define SERVICES_NUMBER_OF_TOC_ENTRIES               15

/**
 * Global standby configuration macros
 */

/*
 * Host CPU Cluster Power Request HOST_CPU_CLUS_PWR_REQ
 */
// MEM_RET_REQ
#define MEM_RET_REQ_LAST_LEVEL_CACHE_RET_OFF         0x0
#define MEM_RET_REQ_LAST_LEVEL_CACHE_RET_ON          0x1
// PWR_REQ
#define PWR_REQ_CLUSTOP_LOW_POWER_ON                 0x0
#define PWR_REQ_CLUSTOP_FUNC_RET_ON                  0x1

/*
 * Base System Power Request BSYS_PWR_REQ
 */
// SYSTOP_PWR_REQ
#define SYSTOP_PWR_REQ_LOGIC_OFF_MEM_OFF             0x0
#define SYSTOP_PWR_REQ_LOGIC_OFF_MEM_RET             0x1
#define SYSTOP_PWR_REQ_LOGIC_ON_MEM_ON_OR_RET        0x2
#define SYSTOP_PWR_REQ_LOGIC_ON_MEM_ON               0x4
// DBGTOP_PWR_REQ
#define DBGTOP_PWR_REQ_OFF                           0x0
#define DBGTOP_PWR_REQ_ON                            0x1
// REFCLK_REQ
#define REFCLK_REQ_OFF                               0x0
#define REFCLK_REQ_ON                                0x1
// WAKEUP_EN
#define WAKEUP_EN_SE_OFF                             0x0
#define WAKEUP_EN_SE_ON                              0x1

/**
 * @brief Power / retention error codes
 */
#define ERROR_POWER_SRAM_RETENTION_INVALID           0x100

/**
 * Memory SRAM 0 1 MRAM Power configuration bit encoding
 */
#define POWER_MEM_SRAM_0_ENABLE                      (1 << 0)
#define POWER_MEM_SRAM_1_ENABLE                      (1 << 1)
#define POWER_MEM_SRAM_0_ISOLATION_ENABLE            (1 << 2)
#define POWER_MEM_SRAM_1_ISOLATION_ENABLE            (1 << 3)
#define POWER_MEM_MRAM_ENABLE                        (1 << 4)

/**
 * SYSTOP power configuration
 */
#define SYSTOP_LOGIC_OFF_POWER_OFF                   0x0
#define SYSTOP_LOGIC_OFF_RETENTION_ON                0x1
#define SYSTOP_LOGIC_ON_POWER_X_RET_X                0x2  // can be powered/retained
#define SYSTOP_LOGIC_ON_POWER_ON                     0x4

/*******************************************************************************
 *  T Y P E D E F S
 ******************************************************************************/

typedef int32_t (*wait_ms_t)(uint32_t wait_time_ms);
typedef int (*print_msg_t)(const char *fmt, ...);

/**
 * @enum SERVICES_cpuid_t
 * @brief CPU names
 */
typedef enum {
    HOST_CPU_0 = 0, /*!< A32_0 CPU               HOST_CPU_0 */
    HOST_CPU_1 = 1, /*!< A32_1 CPU               HOST_CPU_1 */
    EXTSYS_0   = 2, /*!< M55 HP CPU or other CPU EXTSYS_0   */
    EXTSYS_1   = 3, /*!< M55 HE CPU              EXTSYS_1   */
} SERVICES_cpuid_t;

/**
 * @struct SERVICES_toc_info_t
 * @brief user facing TOC information
 */
typedef struct {
    uint8_t  image_identifier[TOC_NAME_LENGTH]; /*!< TOC name         */
    uint32_t version;                           /*!< TOC Version      */
    uint32_t cpu;                               /*!< TOC Cpu ID       */
    uint32_t store_address;                     /*!< TOC MRAM address */
    uint32_t load_address;                      /*!< TOC load         */
    uint32_t boot_address;                      /*!< TOC boot address */
    uint32_t image_size;                        /*!< TOC image size   */
    uint32_t processing_time;                   /*!< TOC process time */
    uint32_t flags;                             /*!< TOC flag state   */
    uint8_t  flags_string[FLAG_STRING_SIZE];    /*!< TOC flag string  */
} SERVICES_toc_info_t;

/**
 * @struct SERVICES_version_data_t
 * @brief  user facing device details, including internal OTP
 */
typedef struct {
    uint32_t revision_id;        /*!< SoC revision          */
    uint8_t  version[4];         /*!< @todo deprecate       */
    uint8_t  ALIF_PN[16];        /*!< SoC part number       */
    uint8_t  HBK0[16];           /*!< ALIF Key              */
    uint8_t  HBK1[16];           /*!< ALIF Key              */
    uint8_t  HBK_FW[20];         /*!< ALIF Firmware version */
    uint8_t  config[4];          /*!< Wounding data         */
    uint8_t  DCU[16];            /*!< DCU settings          */
    uint8_t  MfgData[32];        /*!< Manufacturing data    */
    uint8_t  SerialN[8];         /*!< SoC Serial number     */
    uint8_t  LCS;                /*!< SoC lifecycle state   */
    uint32_t external_config[4]; /*!< External mem   */
    uint32_t flags2;             /*!< Alt path options      */
} SERVICES_version_data_t;

/**
 * @struct SERVICES_toc_data_t
 * @brief user facing structure for all TOC data
 */
typedef struct {
    uint32_t            number_of_toc_entries; /*!< Number of real TOC objects */
    SERVICES_toc_info_t toc_entry[SERVICES_NUMBER_OF_TOC_ENTRIES]; /* TOC details */
} SERVICES_toc_data_t;

/**
 * @enum services_power_profile_t
 * @brief Power profiles
 */
typedef enum {
    OFF_PROFILE = 0,         /*!< OFF_PROFILE                    */
    RUN_PROFILE,             /*!< HIGH_PERFORMANCE_POWER_PROFILE */
    NUMBER_OF_POWER_PROFILES /*!< NUMBER_OF_POWER_PROFILES       */
} services_power_profile_t;

/**
 * Clocks Services definitions
 */

/**
 * @enum oscillator_source_t
 * @brief Oscillator clock selectors
 */
typedef enum {
    OSCILLATOR_SOURCE_RC,   // use RC as oscillator clock
    OSCILLATOR_SOURCE_XTAL  // use XTAL  as oscillator clock
} oscillator_source_t;

/**
 * @enum oscillator_target_t
 * @brief Oscillator target selectors
 */
typedef enum {
    OSCILLATOR_TARGET_SYS_CLOCKS,     // various system clocks
    OSCILLATOR_TARGET_PERIPH_CLOCKS,  // clock for peripherrals
    OSCILLATOR_TARGET_S32K_CLOCK      // 32K low frequency clock
} oscillator_target_t;

/**
 * @enum pll_source_t
 * @brief PLL clock selectors
 */
typedef enum {
    PLL_SOURCE_PLL,  // use the PLL clocks
    PLL_SOURCE_OSC   // use the OCS clocks (can be RC or XTAL)
} pll_source_t;

// ES0 CPU clock frequencies
#define ES0_CLOCK_16MHZ 0
#define ES0_CLOCK_24MHZ 4
#define ES0_CLOCK_48MHZ 0xC

// ES0 configuration bit field
#define ES0_CONFIG_NONE 0x0
#define ES0_CONFIG_HPA  0x1
#define ES0_CONFIG_CSP  0x2

/**
 * @struct net_proc_boot_args_t
 * @brief ExtSys0 Boot arguments
 */
typedef struct {
    uint32_t nvds_src_addr;
    uint32_t nvds_dst_addr;
    uint32_t nvds_copy_len;
    uint32_t trng_dst_addr;
    uint32_t trng_len;
    uint32_t es0_clock_select;
    uint32_t configuration;
} net_proc_boot_args_t;

/**
 * @enum pll_target_t
 * @brief PLL Target selectors
 */
typedef enum {
    PLL_TARGET_SYSREFCLK, /**< PLL_TARGET_SYSREFCLK */
    PLL_TARGET_SYSCLK,    /**< PLL_TARGET_SYSCLK */
    PLL_TARGET_UART,      /**< PLL_TARGET_UART */
    PLL_TARGET_ES0,       /**< PLL_TARGET_ES0 */
    PLL_TARGET_ES1,       /**< PLL_TARGET_ES1 */
    PLL_TARGET_SECENC,    /**< PLL_TARGET_SECENC */
    PLL_TARGET_PD4_SRAM   /**< PLL_TARGET_PD4_SRAM */
} pll_target_t;

/**
 * @enum clock_enable_t
 * @brief Clock selectors
 */
typedef enum {
    CLKEN_SYSPLL,   /**< CLKEN_SYSPLL */
    CLKEN_CPUPLL,   /**< CLKEN_CPUPLL */
    CLKEN_ES0,      /**< CLKEN_ES0 */
    CLKEN_ES1,      /**< CLKEN_ES1 */
    CLKEN_HFXO_OUT, /**< CLKEN_HFXO_OUT*/
    CLKEN_CLK_160M, /**< CLKEN_CLK_160M */
    CLKEN_CLK_100M, /**< CLKEN_CLK_100M */
    CLKEN_CLK_20M,  /**< Renamed from CLKEN_USB */
    CLKEN_HFOSC,    /**< CLKEN_HFOSC */
    CLKEN_SRAM0,    /**< CLKEN_SRAM0 */
    CLKEN_SRAM1,    /**< CLKEN_SRAM1 */
    CLKEN_HFOSCx2,  /**< 76.8MHz, double the frequency of HFOSC above */
    CLKEN_CLK_10M,
    CLKEN_CLK_25M,
    CLKEN_CLK_50M,
    CLKEN_CLK_80M,
    CLKEN_CLK_200M,
    CLKEN_CLK_266M,
    CLKEN_CLK_400M,
    CLKEN_MRAM,
    CLKEN_APB,
    CLKEN_AHB,
    CLKEN_ISP,
    CLKEN_JPEG,
    CLKEN_NPU
} clock_enable_t;

/**
 * @enum a32_source_t
 * @brief A32 clock sources
 */
typedef enum {
    A32_CLOCK_GATE = 0, /**< A32_CLOCK_GATE */
    A32_REFCLK     = 1, /**< A32_REFCLK */
    A32_SYSPLL     = 2, /**< A32_SYSPLL */
    A32_CPUPLL     = 4  /**< A32_CPUPLL */
} a32_source_t;

/**
 * @enum aclk_source_t
 * @brief Clock sources
 */
typedef enum {
    ACLK_CLOCK_GATE = 0, /**< ACLK_CLOCK_GATE */
    ACLK_REFCLK     = 1, /**< ACLK_REFCLK */
    ACLK_SYSPLL     = 2  /**< ACLK_SYSPLL */
} aclk_source_t;

/**
 * @enum clock_divider_t
 * @brief Clock divider selectors
 */
typedef enum {
    DIVIDER_CPUPLL, /**< DIVIDER_CPUPLL */
    DIVIDER_SYSPLL, /**< DIVIDER_SYSPLL */
    DIVIDER_ACLK,   /**< DIVIDER_ACLK */
    DIVIDER_HCLK,   /**< DIVIDER_HCLK */
    DIVIDER_PCLK    /**< DIVIDER_PCLK */
} clock_divider_t;

/**
 * @enum power_setting_t
 * @brief Power setting selectors
 */
typedef enum {
    POWER_SETTING_BOR_EN,         /**< POWER_SETTING_BOR_EN */
    POWER_SETTING_SCALED_CLK_FREQ, /**< POWER_SETTING_SCALED_CLK_FREQ */
    POWER_SETTING_ANA_PERIPH_EN   /**< POWER_SETTING_ANA_PERIPH_EN */
} power_setting_t;

/**
 * @enum clock_setting_t
 * @brief Clock frequency selectors
 */
typedef enum {
    CLOCK_SETTING_HFOSC_FREQ,   /**< CLOCK_SETTING_HFOSC_FREQ */
    CLOCK_SETTING_EXTSYS0_FREQ, /**< CLOCK_SETTING_EXTSYS0_FREQ */
    CLOCK_SETTING_EXTSYS1_FREQ, /**< CLOCK_SETTING_EXTSYS1_FREQ */
    CLOCK_SETTING_AXI_FREQ,     /**< CLOCK_SETTING_AXI_FREQ */
    CLOCK_SETTING_AHB_FREQ,     /**< CLOCK_SETTING_AHB_FREQ */
    CLOCK_SETTING_APB_FREQ,     /**< CLOCK_SETTING_APB_FREQ */
    CLOCK_SETTING_SYSREF_FREQ,  /**< CLOCK_SETTING_SYSREF_FREQ */
    CLOCK_SETTING_ACLK_FORCE_EN,    /**< CLOCK_SETTING_ACLK_FORCE_EN */
    CLOCK_SETTING_ACLK_ENTRY_DELAY, /**< CLOCK_SETTING_ACLK_ENTRY_DELAY */
    CLOCK_SETTING_LF_FREQ,          /**< CLOCK_SETTING_LF_FREQ */
    CLOCK_SETTING_LF_SOURCE         /**< CLOCK_SETTING_LF_SOURCE */
} clock_setting_t;

/**
 * @struct clock_get_t
 * @brief  return structure for clocks
 */
typedef struct {
  uint32_t status;
  float se_frequency_mhz;
  uint32_t hostcpuclk_ctrl;
  uint32_t hostcpuclk_div0;
  uint32_t hostcpuclk_div1;
  uint32_t aclk_ctrl;
  uint32_t aclk_div0;
  uint32_t cgu_osc_ctrl;
  uint32_t cgu_pll_lock_ctrl;
  uint32_t cgu_pll_sel;
  uint32_t cgu_escclk_sel;
  uint32_t cgu_clk_ena;
  uint32_t systop_clk_div;
  uint32_t misc_reg1;
  uint32_t xo_reg1;
  uint32_t pd4_clk_sel;
  uint32_t pd4_clk_pll;
  uint32_t misc_ctrl;
  uint32_t dcdc_reg1;
  uint32_t dcdc_reg2;
  uint32_t vbat_ana_reg1;
  uint32_t vbat_ana_reg2;
  uint32_t lf_oscillator_source;
  uint32_t lf_frequency_hz;
} clock_get_t;

/*******************************************************************************
 *  G L O B A L   D E F I N E S
 ******************************************************************************/

/*******************************************************************************
 *  F U N C T I O N   P R O T O T Y P E S
 ******************************************************************************/

// Services infrastructure APIs
uint32_t SERVICES_register_channel(uint32_t mhu_id, uint32_t channel_number);
void     SERVICES_unregister_channel(uint32_t mhu_id, uint32_t channel_number);

const char *SERVICES_version(void);
char       *SERVICES_error_to_string(uint32_t error_code);

// Services functional APIs
uint32_t SERVICES_heartbeat(uint32_t services_handle);
uint32_t SERVICES_uart_write(uint32_t services_handle, size_t size, const uint8_t *uart_data);
uint32_t SERVICES_pinmux(uint32_t services_handle, uint8_t port_number, uint8_t pin_number,
                         uint8_t config_data, uint32_t *error_code);
uint32_t SERVICES_padcontrol(uint32_t services_handle, uint8_t port_number, uint8_t pin_number,
                             uint8_t configuration_value, uint32_t *error_code);
uint32_t SERVICES_application_ospi_write_key(uint32_t services_handle, uint32_t command,
                                             uint8_t *key, uint32_t *error_code);
uint32_t SERVICES_application_verify_image(uint32_t services_handle,
               uint32_t image_address,
               uint32_t cert_chain_address,
               uint32_t *error_code);
uint32_t SERVICES_application_dmpu(uint32_t services_handle,
               uint32_t assets_address,
               uint32_t *error_code);
uint32_t SERVICES_cryptocell_get_rnd(uint32_t services_handle, uint16_t rnd_len, void *rnd_value,
                                     int32_t *error_code);

uint32_t SERVICES_application_configure_lpcmp(
    uint32_t services_handle, uint8_t lpcmp_hyst, uint8_t lpcmp_in_m_sel,
    uint8_t lpcmp_in_p_sel, uint8_t lpcmp_en, uint8_t lpcmp_clk_sel,
    uint8_t lpcmp_clk32_en, uint32_t *error_code);

uint32_t SERVICES_cryptocell_get_rnd(uint32_t services_handle, uint16_t rnd_len,
				     void *rnd_value, int32_t *error_code);
uint32_t SERVICES_cryptocell_get_lcs(uint32_t services_handle,
					uint32_t *lcs_state,
					int32_t *error_code);
// MbedTLS macros and APIs
uint32_t SERVICES_cryptocell_mbedtls_hardware_poll(uint32_t services_handle, uint32_t *error_code,
                                                   uint32_t data, uint32_t output, uint32_t len,
                                                   uint32_t olen);
uint32_t SERVICES_cryptocell_mbedtls_aes_init(uint32_t services_handle, uint32_t *error_code,
                                              uint32_t ctx);
uint32_t SERVICES_cryptocell_mbedtls_aes_set_key(uint32_t services_handle, uint32_t *error_code,
                                                 uint32_t ctx, uint32_t key, uint32_t keybits,
                                                 uint32_t dir);
uint32_t SERVICES_cryptocell_mbedtls_aes_crypt(uint32_t services_handle, uint32_t *error_code,
                                               uint32_t ctx, uint32_t crypt_type, uint32_t mode,
                                               uint32_t length, uint32_t iv, uint32_t input,
                                               uint32_t output);
uint32_t SERVICES_cryptocell_mbedtls_aes(uint32_t services_handle, uint32_t *error_code,
                                         uint32_t key, uint32_t keybits, uint32_t direction,
                                         uint32_t crypt_type, uint32_t iv, uint32_t length,
                                         uint32_t input, uint32_t output);

uint32_t SERVICES_cryptocell_mbedtls_sha_starts(uint32_t services_handle, uint32_t *error_code,
                                                uint32_t ctx, uint32_t sha_type);
uint32_t SERVICES_cryptocell_mbedtls_sha_process(uint32_t services_handle, uint32_t *error_code,
                                                 uint32_t ctx, uint32_t sha_type, uint32_t data);
uint32_t SERVICES_cryptocell_mbedtls_sha_update(uint32_t services_handle, uint32_t *error_code,
                                                uint32_t ctx, uint32_t sha_type, uint32_t data,
                                                uint32_t data_length);
uint32_t SERVICES_cryptocell_mbedtls_sha_finish(uint32_t services_handle, uint32_t *error_code,
                                                uint32_t ctx, uint32_t sha_type, uint32_t data);
uint32_t SERVICES_cryptocell_mbedtls_sha(uint32_t services_handle, uint32_t *error_code,
                                         uint32_t sha_type, uint32_t data, uint32_t data_length,
                                         uint32_t sha_sum);

uint32_t SERVICES_cryptocell_mbedtls_ccm_gcm_set_key(uint32_t services_handle, uint32_t *error_code,
                                                     uint32_t context_addr, uint32_t key_type,
                                                     uint32_t cipher, uint32_t key_addr,
                                                     uint32_t key_bits);
uint32_t SERVICES_cryptocell_mbedtls_ccm_gcm_crypt(
    uint32_t services_handle, uint32_t *error_code, uint32_t context_addr, uint32_t crypt_type,
    uint32_t length, uint32_t iv_addr, uint32_t iv_length, uint32_t add_addr, uint32_t add_length,
    uint32_t input_addr, uint32_t output_addr, uint32_t tag_addr, uint32_t tag_length);
uint32_t SERVICES_cryptocell_mbedtls_ccm_gcm(uint32_t services_handle, uint32_t *error_code,
                                             uint32_t crypt_type, uint32_t key_addr,
                                             uint32_t key_bits, uint32_t length, uint32_t iv_addr,
                                             uint32_t iv_length, uint32_t add_addr,
                                             uint32_t add_length, uint32_t input_addr,
                                             uint32_t output_addr, uint32_t tag_addr,
                                             uint32_t tag_length);

uint32_t SERVICES_cryptocell_mbedtls_chacha20_crypt(uint32_t services_handle, uint32_t *error_code,
                                                    uint32_t key_addr, uint32_t nonce_addr,
                                                    uint32_t counter, uint32_t data_len,
                                                    uint32_t input_addr, uint32_t output_addr);
uint32_t SERVICES_cryptocell_mbedtls_chachapoly_crypt(uint32_t  services_handle,
                                                      uint32_t *error_code, uint32_t context_addr,
                                                      uint32_t crypt_type, uint32_t length,
                                                      uint32_t nonce_addr, uint32_t aad_addr,
                                                      uint32_t aad_len, uint32_t tag_addr,
                                                      uint32_t input_addr, uint32_t output_addr);
uint32_t SERVICES_cryptocell_mbedtls_poly1305_crypt(uint32_t services_handle, uint32_t *error_code,
                                                    uint32_t key_addr, uint32_t input_addr,
                                                    uint32_t ilen, uint32_t mac_addr);
uint32_t SERVICES_cryptocell_mbedtls_cmac_init_setkey(uint32_t  services_handle,
                                                      uint32_t *error_code, uint32_t ctx,
                                                      uint32_t key, uint32_t keybits);
uint32_t SERVICES_cryptocell_mbedtls_cmac_update(uint32_t services_handle, uint32_t *error_code,
                                                 uint32_t ctx, uint32_t input, uint32_t length);
uint32_t SERVICES_cryptocell_mbedtls_cmac_finish(uint32_t services_handle, uint32_t *error_code,
                                                 uint32_t ctx, uint32_t output);
uint32_t SERVICES_cryptocell_mbedtls_cmac_reset(uint32_t services_handle, uint32_t *error_code,
                                                uint32_t ctx);
uint32_t SERVICES_cryptocell_mbedtls_cmac(uint32_t services_handle, uint32_t *error_code,
                                          uint32_t key, uint32_t keybits, uint32_t input,
                                          uint32_t length, uint32_t output);

uint32_t SERVICES_system_get_toc_version(uint32_t services_handle, uint32_t *toc_version,
                                         uint32_t *error_code);
uint32_t SERVICES_system_get_toc_number(uint32_t services_handle, uint32_t *toc_number,
                                        uint32_t *error_code);
uint32_t SERVICES_system_get_toc_via_name(uint32_t services_handle, const uint8_t *cpu_name,
                                          uint32_t *error_code);
uint32_t SERVICES_system_get_toc_via_cpuid(uint32_t services_handle, SERVICES_cpuid_t cpuid,
                                           SERVICES_toc_data_t *toc_info, uint32_t *error_code);
uint32_t SERVICES_system_get_toc_data(uint32_t services_handle, SERVICES_toc_data_t *toc_data,
                                      uint32_t *error_code);
uint32_t SERVICES_system_get_device_part_number(uint32_t  services_handle,
                                                uint32_t *device_part_number, uint32_t *error_code);
uint32_t SERVICES_system_set_services_debug(uint32_t services_handle, bool debug_enable,
                                            uint32_t *error_code);
uint32_t SERVICES_system_get_device_data(uint32_t                 services_handle,
                                         SERVICES_version_data_t *device_info,
                                         uint32_t                *error_code);
uint32_t SERVICES_get_se_revision(uint32_t services_handle, uint8_t *revision_data,
                                  uint32_t *error_code);
uint32_t SERVICES_system_read_otp(uint32_t services_handle, uint32_t otp_offset,
                                  uint32_t *otp_value_word, uint32_t *error_code);
uint32_t SERVICES_system_write_otp(uint32_t services_handle, uint32_t otp_offset,
                                   uint32_t otp_value_word, uint32_t *error_code);
uint32_t SERVICES_system_get_ecc_public_key(uint32_t services_handle, uint8_t *ecc_pubkey_buffer,
                                            uint32_t *error_code);

uint32_t SERVICES_system_get_eui_extension(uint32_t services_handle, bool is_eui48,
                                           uint8_t *eui_extension, uint32_t *error_code);
uint32_t SERVICES_system_get_device_id64(uint32_t services_handle, uint8_t *device_id,
                                         uint32_t *error_code);

uint32_t SERVICES_system_get_eui_extension(uint32_t services_handle,
					bool is_eui48,
					uint8_t *eui_extension,
					uint32_t *error_code);
uint32_t SERVICES_system_get_device_id64(uint32_t services_handle,
                    uint8_t *device_id,
                    uint32_t *error_code);

uint32_t SERVICES_boot_process_toc_entry(uint32_t services_handle,
					 const uint8_t *image_id,
					 uint32_t *error_code);
uint32_t SERVICES_boot_cpu(uint32_t services_handle,
			   uint32_t cpu_id,
			   uint32_t address,
			   uint32_t *error_code);
uint32_t SERVICES_boot_set_vtor(uint32_t services_handle,
				uint32_t cpu_id,
				uint32_t address,
				uint32_t *error_code);
uint32_t SERVICES_boot_reset_cpu(uint32_t services_handle,
				 uint32_t cpu_id,
				 uint32_t *error_code);
uint32_t SERVICES_boot_release_cpu(uint32_t services_handle,
				   uint32_t cpu_id,
				   uint32_t *error_code);
uint32_t SERVICES_boot_reset_soc(uint32_t services_handle);

uint32_t SERVICES_power_stop_mode_req(uint32_t                 services_handle,
                                      services_power_profile_t power_profile, bool override);
uint32_t SERVICES_power_ewic_config(uint32_t services_handle, uint32_t ewic_source,
                                    services_power_profile_t power_profile);
uint32_t SERVICES_power_wakeup_config(uint32_t services_handle, uint32_t vbat_wakeup_source,
                                      services_power_profile_t power_profile);
uint32_t SERVICES_power_memory_req(uint32_t services_handle, uint32_t memory_request,
                                   uint32_t *error_code);
uint32_t SERVICES_power_se_sleep_req(uint32_t services_handle, uint32_t se_param,
                                     uint32_t *error_code);
uint32_t SERVICES_power_mem_retention_config(uint32_t services_handle, uint32_t mem_retention,
                                             services_power_profile_t power_profile);
uint32_t SERVICES_corstone_standby_mode(uint32_t                services_handle,
                                        host_cpu_clus_pwr_req_t host_cpu_clus_pwr_req,
                                        bsys_pwr_req_t bsys_pwr_req, uint32_t *error_code);
uint32_t SERVICES_power_m55_he_vtor_save(uint32_t services_handle, uint32_t ns_vtor_addr,
                                         uint32_t                 se_vtor_addr,
                                         services_power_profile_t power_profile);
uint32_t SERVICES_power_m55_hp_vtor_save(uint32_t services_handle, uint32_t ns_vtor_addr,
                                         uint32_t                 se_vtor_addr,
                                         services_power_profile_t power_profile);

uint32_t SERVICES_power_dcdc_voltage_control(uint32_t services_handle, uint32_t dcdc_vout_sel,
                                             uint32_t dcdc_vout_trim, uint32_t *error_code);

uint32_t SERVICES_power_ldo_voltage_control(uint32_t services_handle, uint32_t ret_ldo_voltage,
                                            uint32_t aon_ldo_voltage, uint32_t *error_code);

uint32_t SERVICES_power_setting_configure(uint32_t services_handle, power_setting_t setting_type,
                                          uint32_t value, uint32_t *error_code);
uint32_t SERVICES_power_setting_get(uint32_t services_handle, power_setting_t setting_type,
                                    uint32_t *value, uint32_t *error_code);

uint32_t SERVICES_power_stop_mode_raw_req(uint32_t services_handle, uint32_t *error_code);
uint32_t SERVICES_power_ewic_config_raw(uint32_t services_handle, uint32_t ewic_source,
                                        uint32_t *error_code);
uint32_t SERVICES_power_wakeup_config_raw(uint32_t services_handle, uint32_t vbat_wakeup_source,
                                          uint32_t *error_code);
uint32_t SERVICES_power_mem_retention_config_raw(uint32_t services_handle, uint32_t mem_retention,
                                                 uint32_t *error_code);
uint32_t SERVICES_power_m55_he_vtor_save_raw(uint32_t services_handle, uint32_t ns_vtor_addr,
                                             uint32_t se_vtor_addr, uint32_t *error_code);

// Clocks services
uint32_t SERVICES_clocks_select_osc_source(uint32_t services_handle, oscillator_source_t source,
                                           oscillator_target_t target, uint32_t *error_code);
uint32_t SERVICES_clocks_select_pll_source(uint32_t services_handle, pll_source_t source,
                                           pll_target_t target, uint32_t *error_code);
uint32_t SERVICES_clocks_enable_clock(uint32_t services_handle, clock_enable_t clock, bool enable,
                                      uint32_t *error_code);
uint32_t SERVICES_clocks_set_ES0_frequency(uint32_t services_handle, clock_frequency_t frequency,
                                           uint32_t *error_code);
uint32_t SERVICES_clocks_set_ES1_frequency(uint32_t services_handle, clock_frequency_t frequency,
                                           uint32_t *error_code);
uint32_t SERVICES_clocks_select_a32_source(uint32_t services_handle, a32_source_t source,
                                           uint32_t *error_code);
uint32_t SERVICES_clocks_select_aclk_source(uint32_t services_handle, aclk_source_t source,
                                            uint32_t *error_code);
uint32_t SERVICES_clocks_set_divider(uint32_t services_handle, clock_divider_t divider,
                                     uint32_t value, uint32_t *error_code);
uint32_t SERVICES_clocks_setting_get(uint32_t services_handle, clock_setting_t setting_type,
                                     uint32_t *value, uint32_t *error_code);
uint32_t SERVICES_clocks_set_aclk(uint32_t services_handle, uint32_t *aclk_entry_delay,
                                  uint32_t *aclk_force_en, uint32_t *error_code);
uint32_t SERVICES_clocks_set_vbat_clk(uint32_t services_handle, uint32_t *vbat_fast_clk_en,
					      uint32_t *error_code);
uint32_t SERVICES_clocks_get_data(uint32_t services_handle, clock_get_t *clk_settings,
				  uint32_t *error_code);

// PLL services
uint32_t SERVICES_pll_initialize(uint32_t services_handle, uint32_t *error_code);
uint32_t SERVICES_pll_deinit(uint32_t services_handle, uint32_t *error_code);
uint32_t SERVICES_pll_xtal_start(uint32_t services_handle, bool faststart, bool boost,
                                 uint32_t delay_count, uint32_t *error_code);
uint32_t SERVICES_pll_xtal_stop(uint32_t services_handle, uint32_t *error_code);
uint32_t SERVICES_pll_xtal_is_started(uint32_t services_handle, bool *is_started,
                                      uint32_t *error_code);
uint32_t SERVICES_pll_clkpll_start(uint32_t services_handle, bool faststart, uint32_t delay_count,
                                   uint32_t *error_code);
uint32_t SERVICES_pll_clkpll_stop(uint32_t services_handle, uint32_t *error_code);
uint32_t SERVICES_pll_clkpll_is_locked(uint32_t services_handle, bool *is_locked,
                                       uint32_t *error_code);

// External System 0 Services
uint32_t SERVICES_Boot_Net_Proc(uint32_t services_handle, net_proc_boot_args_t *boot_args,
                                uint32_t *error_code);
uint32_t SERVICES_Shutdown_Net_Proc(uint32_t services_handle, uint32_t *error_code);

// Update services
uint32_t SERVICES_update_stoc(uint32_t services_handle, uint32_t image_address, uint32_t image_size,
                              uint32_t *error_code);

// Key management services
uint32_t SERVICES_key_mgmt_import_key(uint32_t services_handle,
					      se_key_type_t key_type,
					      const uint8_t *key, uint32_t key_len,
					      se_key_handle_t *key_handle,
					      uint32_t *error_code);

uint32_t SERVICES_key_mgmt_clear_key(uint32_t services_handle,
					     se_key_handle_t key_handle,
					     uint32_t *error_code);

uint32_t SERVICES_key_mgmt_aes_crypt_by_handle(
    uint32_t services_handle, se_key_handle_t key_handle,
    se_crypt_direction_t direction, se_crypt_type_t crypt_type, uint8_t *iv,
    uint32_t iv_len, const uint8_t *input, uint32_t input_len, uint8_t *output,
    const uint8_t *aad, uint32_t aad_len, uint8_t *tag, uint32_t tag_len,
    uint32_t *error_code, uint32_t *crypto_error_code);

uint32_t SERVICES_key_mgmt_generate_ecc_key(
    uint32_t services_handle, se_key_type_t key_type, uint32_t *key_handle,
    uint8_t *public_key, uint32_t *public_key_len, uint32_t *error_code,
    uint32_t *crypto_error_code);

uint32_t SERVICES_key_mgmt_get_ecc_public_key(uint32_t services_handle,
						      uint32_t key_handle,
						      uint8_t *public_key,
						      uint32_t *public_key_len,
						      uint32_t *error_code,
						      uint32_t *crypto_error_code);

uint32_t SERVICES_key_mgmt_hmac_by_handle(
    uint32_t services_handle, uint32_t key_handle, se_hmac_algo_t hmac_algo,
    const uint8_t *input, uint32_t input_len, uint8_t *resp_mac,
    uint32_t *resp_mac_len, uint32_t *error_code, uint32_t *crypto_error_code);

uint32_t SERVICES_key_mgmt_wrap_key(uint32_t services_handle,
					    uint32_t key_handle, uint8_t *wrapped_key,
					    uint32_t *wrapped_key_len,
					    uint32_t *error_code,
					    uint32_t *crypto_error_code);
uint32_t SERVICES_key_mgmt_unwrap_key(uint32_t services_handle,
					      const uint8_t *wrapped_key,
					      uint32_t wrapped_key_len,
					      uint32_t *key_handle,
					      uint32_t *error_code,
					      uint32_t *crypto_error_code);

uint32_t
SERVICES_key_mgmt_ecdh(uint32_t services_handle, uint32_t key_handle,
		       se_ecc_curve_t curve, const uint8_t *peer_pubkey,
		       uint32_t peer_pubkey_len, se_hkdf_algo_t kdf_algo,
		       se_key_type_t derived_key_type, const uint8_t *salt,
		       uint32_t salt_len, uint32_t *session_key_handle,
		       uint32_t *error_code, uint32_t *crypto_error_code);

uint32_t
SERVICES_key_mgmt_ecdsa_sign(uint32_t services_handle, uint32_t key_handle,
			     const uint8_t *digest, uint32_t digest_len,
			     uint8_t *signature, uint32_t *signature_len,
			     uint32_t *error_code, uint32_t *crypto_error_code);

uint32_t SERVICES_key_mgmt_ecdsa_verify(
    uint32_t services_handle, se_ecc_curve_t curve, const uint8_t *digest,
    uint32_t digest_len, const uint8_t *pubkey, uint32_t pubkey_len,
    const uint8_t *signature, uint32_t signature_len, uint32_t *error_code,
    uint32_t *crypto_error_code);

uint32_t SERVICES_key_mgmt_derive_key(
    uint32_t services_handle, uint32_t base_key_handle, se_hkdf_algo_t kdf_algo,
    se_key_type_t derived_key_type, const uint8_t *nonce, uint32_t nonce_len,
    const uint8_t *label, uint32_t label_len, uint32_t *derived_key_handle,
    uint32_t *error_code, uint32_t *crypto_error_code);

uint32_t SERVICES_key_mgmt_disable_key_services(uint32_t services_handle,
							uint32_t *error_code);

uint32_t SERVICES_key_get_key_attributes(uint32_t services_handle,
						 uint32_t key_index,
						 key_attributes_t *key_attr,
						 uint32_t *error_code);
uint32_t SERVICES_key_get_key_handle_attributes(uint32_t services_handle,
							uint32_t key_index,
							key_attributes_t *key_attr,
							uint32_t *error_code);

#ifdef __cplusplus
}
#endif
#endif /* __SERVICES_LIB_API_H__ */
