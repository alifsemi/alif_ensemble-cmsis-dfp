/**
 * @file services_host_system.c
 *
 * @brief System Management services source file
 *
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
 * @ingroup host_services
 */

/******************************************************************************
 *  I N C L U D E   F I L E S
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "services_lib_api.h"
#include "services_lib_protocol.h"
#include "services_lib_ids.h"

/*******************************************************************************
 *  M A C R O   D E F I N E S
 ******************************************************************************/

#define UNUSED(x) (void)(x)

/*******************************************************************************
 *  T Y P E D E F S
 ******************************************************************************/

/*******************************************************************************
 *  G L O B A L   V A R I A B L E S
 ******************************************************************************/

/*******************************************************************************
 *  C O D E
 ******************************************************************************/

/**
 * @fn      uint32_t SERVICES_system_get_toc_version(uint32_t * toc_version)
 * @brief   get the TOC
 * @param   services_handle
 * @param   toc_version
 * @return
 */
uint32_t SERVICES_system_get_toc_version(uint32_t services_handle, 
                                         uint32_t * toc_version,
                                         uint32_t * error_code)
{
  get_toc_version_svc_t * p_svc = (get_toc_version_svc_t *)
      SERVICES_prepare_packet_buffer(sizeof(get_toc_version_svc_t));

  uint32_t ret = SERVICES_send_request(services_handle, 
                                       SERVICE_SYSTEM_MGMT_GET_TOC_VERSION, 
                                       NULL);

  *toc_version = p_svc->resp_version;
  *error_code = p_svc->resp_error_code;

  return ret;
}

/**
 * @fn      uint32_t SERVICES_system_get_toc_number(uint32_t toc_version)
 * @brief   get the TOC number
 * @param   services_handle
 * @param   toc_number
 * @return
 */
uint32_t SERVICES_system_get_toc_number(uint32_t services_handle, 
                                        uint32_t * toc_number,
                                        uint32_t * error_code)
{
  get_toc_number_svc_t * p_svc = (get_toc_number_svc_t *)
      SERVICES_prepare_packet_buffer(sizeof(get_toc_number_svc_t));
	
  uint32_t ret = SERVICES_send_request(services_handle, 
                                       SERVICE_SYSTEM_MGMT_GET_TOC_NUMBER, 
                                       NULL);

  *toc_number = p_svc->resp_number_of_toc;
  *error_code = p_svc->resp_error_code;
  return ret;
}

/**
 * @brief   get toc info via name
 * @param   services_handle
 * @param   cpu_name
 * @return
 */
uint32_t SERVICES_system_get_toc_via_name(uint32_t services_handle, 
                                          const uint8_t * cpu_name,
                                          uint32_t * error_code)
{
  get_toc_via_name_svc_t * p_svc = (get_toc_via_name_svc_t *)
      SERVICES_prepare_packet_buffer(sizeof(get_toc_via_name_svc_t));

  memcpy((void *)p_svc->send_cpu_name, cpu_name, IMAGE_NAME_LENGTH);

  uint32_t ret = SERVICES_send_request(services_handle,
                               SERVICE_SYSTEM_MGMT_GET_TOC_VIA_CPU_NAME, 
                               NULL);
  *error_code = p_svc->resp_error_code;
  return ret;
}

/**
 * @brief     get toc info via cpuid
 * @param     services_handle
 * @param[in] cpuid
 * @param     toc_info
 * @return
 */
uint32_t SERVICES_system_get_toc_via_cpuid(uint32_t services_handle,
                                           SERVICES_cpuid_t cpuid, 
                                           SERVICES_toc_data_t *toc_info,
                                           uint32_t *error_code)
{
  get_toc_data_t *p_svc = (get_toc_data_t *)
     SERVICES_prepare_packet_buffer(sizeof(get_toc_data_t)); /* Packet bucket */
  uint32_t return_code; /* returned error from SERVICES */

  p_svc->send_cpu_id = cpuid;  /* which CPU do we want */

 return_code = SERVICES_send_request(services_handle,
                                     SERVICE_SYSTEM_MGMT_GET_TOC_VIA_CPU_ID,
                                     NULL);

 /* Unpack results */
 toc_info->number_of_toc_entries = p_svc->resp_number_of_toc_entries;
 memcpy((SERVICES_toc_data_t*)&toc_info->toc_entry[0],
        (get_toc_data_t *)&p_svc->resp_toc_entry[0],
        sizeof(p_svc->resp_toc_entry));
  *error_code = p_svc->resp_error_code;

  return return_code;
}

/**
 * @brief Obtain all TOC data from SE
 * @param services_handle
 * @param toc_info
 * @param error_code
 * @return
 */
uint32_t SERVICES_system_get_toc_data (uint32_t services_handle,
                                       SERVICES_toc_data_t *toc_info,
                                       uint32_t * error_code)
{
  get_toc_data_t *p_svc = (get_toc_data_t *)
      SERVICES_prepare_packet_buffer(sizeof(get_toc_data_t));
  uint32_t return_code;

 return_code = SERVICES_send_request(services_handle,
                                     SERVICE_SYSTEM_MGMT_GET_TOC_INFO,
                                     NULL);

 toc_info->number_of_toc_entries = p_svc->resp_number_of_toc_entries;
 memcpy((SERVICES_toc_data_t*)&toc_info->toc_entry[0],
        (get_toc_data_t *)&p_svc->resp_toc_entry[0],
        sizeof(p_svc->resp_toc_entry));
  *error_code = p_svc->resp_error_code;

  return return_code;
}

/**
 * @fn    uint32_t SERVICES_system_get_device_part_number(
 *                                uint32_t services_handle,
                                  uint32_t *device_part_number)
 * @param[in]   services_handle
 * @param[out]  device_part_number
 * @return
 */
uint32_t SERVICES_system_get_device_part_number(uint32_t services_handle, 
                                                uint32_t * device_part_number,
                                                uint32_t * error_code)
{
  get_device_part_svc_t * p_svc = (get_device_part_svc_t *)
      SERVICES_prepare_packet_buffer(sizeof(get_device_part_svc_t));

  uint32_t ret = SERVICES_send_request(services_handle, 
                                       SERVICE_SYSTEM_MGMT_GET_DEVICE_PART_NUMBER, 
                                       NULL);

  *device_part_number = p_svc->resp_device_string;
  *error_code = p_svc->resp_error_code;
  return ret;
}

/**
 * @fn    uint32_t SERVICES_system_set_services_debug (uint32_t services_handle,
 *                                                     uint32_t * error_code)
 * @brief Set debug capability will
 * @param services_handle
 * @param error_code
 * @return
 */
uint32_t SERVICES_system_set_services_debug (uint32_t services_handle,
                                             bool debug_enable,
                                             uint32_t *error_code)
{
  set_services_capabilities_t * p_svc = (set_services_capabilities_t *)
      SERVICES_prepare_packet_buffer(sizeof(set_services_capabilities_t));

  p_svc->send_services_debug = debug_enable;

  uint32_t ret = SERVICES_send_request(services_handle,
                                       SERVICE_SYSTEM_MGMT_SET_CAPABILITIES_DEBUG,
                                       NULL);

  *error_code = p_svc->resp_error_code;
  return ret;
}

/**
 * @brief obtain the OTP data
 * @param services_handle
 * @param toc_info
 * @param error_code
 * @return
 */
#if 0
uint32_t SERVICES_system_get_otp_data (uint32_t services_handle,
                                       SERVICES_otp_data_t *device_info,
                                       uint32_t * error_code)
{
  get_otp_data_t *p_svc = (get_otp_data_t *)
      SERVICES_prepare_packet_buffer(sizeof(get_otp_data_t));
  uint32_t return_code;

 return_code = SERVICES_send_request(services_handle,
                                     SERVICE_SYSTEM_MGMT_GET_OTP_INFO,
                                     NULL);
 /**
  * @todo
  */
 *error_code = p_svc->resp_error_code;

  UNUSED(device_info);

  return return_code;
}
#endif

/**
 * @brief
 *
 * @param services_handle
 * @param device_info
 * @param error_code
 * @return
 */
uint32_t SERVICES_system_get_device_data(uint32_t services_handle,
                                         SERVICES_version_data_t *device_info,
                                         uint32_t * error_code)
{
  get_device_revision_data_t *p_svc = (get_device_revision_data_t *)
      SERVICES_prepare_packet_buffer(sizeof(get_device_revision_data_t));
  uint32_t return_code;

  return_code = SERVICES_send_request(services_handle,
                                      SERVICE_SYSTEM_MGMT_GET_DEVICE_REVISION_DATA,
                                      NULL);

  /* unpack and return */
  device_info->revision_id = p_svc->revision_id;
  memcpy((uint8_t*)&device_info->SerialN[0], (uint8_t*)p_svc->SerialN,
         sizeof(device_info->SerialN));
  memcpy((uint8_t*)&device_info->ALIF_PN[0], (uint8_t*)p_svc->ALIF_PN,
         sizeof(device_info->ALIF_PN));
  memcpy((uint8_t*)&device_info->HBK0[0], (uint8_t*)p_svc->HBK0,
          sizeof(device_info->HBK0));
  memcpy((uint8_t*)&device_info->DCU[0], (uint8_t*)p_svc->DCU,
         sizeof(device_info->DCU));
  memcpy((uint8_t*)&device_info->config[0], (uint8_t*)p_svc->config,
         sizeof(device_info->config));
  memcpy((uint8_t*)&device_info->HBK1[0], (uint8_t*)p_svc->HBK1,
         sizeof(device_info->HBK1));
  memcpy((uint8_t*)&device_info->HBK_FW[0],(uint8_t*)p_svc->HBK_FW,
         sizeof(device_info->HBK_FW));
  memcpy((uint8_t*)&device_info->MfgData[0],(uint8_t*)p_svc->MfgData,
         sizeof(device_info->MfgData));
  device_info->LCS = p_svc->LCS,

  *error_code = p_svc->resp_error_code;

  return return_code;
}

/**
 * @fn  uint32_t SERVICES_system_read_otp     (uint32_t services_handle,
 *                                             uint32_t otp_offset,
 *                                             uint32_t *otp_value_word,
 *                                             uint32_t *error_code)
 * @brief read OTP data
 * @param services_handle
 * @param otp_offset
 * @param otp_value_word
 * @param error_code
 * @return
 */
uint32_t SERVICES_system_read_otp     (uint32_t services_handle,
                                       uint32_t otp_offset,
                                       uint32_t *otp_value_word,
                                       uint32_t *error_code)
{
  read_otp_data_t *p_svc = (read_otp_data_t *)
      SERVICES_prepare_packet_buffer(sizeof(read_otp_data_t));
  uint32_t return_code;

 p_svc->send_offset = otp_offset;
 return_code = SERVICES_send_request(services_handle,
                                     SERVICE_SYSTEM_MGMT_READ_OTP,
                                     NULL);
 *otp_value_word = p_svc->resp_otp_word;
 *error_code = p_svc->resp_error_code;

  return return_code;
}
