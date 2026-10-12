/*
 * Copyright 2026 Arm Limited and/or its affiliates.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "RTE_Components.h"
#include CMSIS_device_header

#include "app_mem_regions.h"
#include "ethosu_driver.h"

typedef struct {
  uint32_t start_addr; // Base address of a block
  uint32_t end_addr;   // End address of a block (inclusive)
} mem_block_t;

const mem_block_t non_cached_memory[] = {
    {APP_ITCM_BASE,    APP_ITCM_BASE    + APP_HP_ITCM_SIZE - 1},
    {APP_DTCM_BASE,    APP_DTCM_BASE    + APP_HP_DTCM_SIZE - 1},
    {APP_MRAM_HP_BASE, APP_MRAM_HP_BASE + APP_MRAM_HP_SIZE - 1}};

/**
  \brief Check if the memory region needs to be invalidated.
  \param[in] p      Pointer to the memory region start address.
  \param[in] bytes  Size of memory region in bytes.
  \return           true if an invalidate is required, false otherwise
*/
static bool check_mem_region(const void *p, size_t bytes) {
  uint32_t n_blocks;
  uint32_t block;
  uint32_t mem_start;
  uint32_t mem_end;

  mem_start = (uint32_t)p;
  mem_end = mem_start + bytes - 1;

  n_blocks = sizeof(non_cached_memory) / sizeof(non_cached_memory[0]);

  for (block = 0; block < n_blocks; block++) {
    /* Is current block within non-cacheable memory region */
    if (mem_start >= non_cached_memory[block].start_addr &&
        mem_end <= non_cached_memory[block].end_addr) {
      /* No need for cache management */
      return false;
    }
  }

  return true;
}

/**
  \brief Cleans data-cache contents associated with buffers accessed by the NPU.

  Ensures that CPU writes to cacheable buffers are committed to memory before
  the NPU starts accessing them. Non-cacheable buffers require no cache clean,
  but the function ensures that all preceding memory accesses have completed
  before it returns.

  \param[in] base_addr       Array of buffer base addresses.
  \param[in] base_addr_size  Array containing the size, in bytes, of each buffer.
  \param[in] num_base_addr   Number of entries in the address and size arrays.
*/
void ethosu_flush_dcache(const uint64_t *base_addr, const size_t *base_addr_size, int num_base_addr) {

  for (int i = 0; i < num_base_addr; i++) {
    if (check_mem_region((const void *)(uintptr_t)base_addr[i], base_addr_size[i])) {
      /* Call CleanDCache instead of CleanDCache_by_Addr to avoid delays.        */
      /* Memory regions size is usually large and calling by_Addr consumes time. */
      SCB_CleanDCache();

      /* Cache is cleaned, return immediately */
      return;
    }
  }

  /* No cleaning, just ensure all memory accesses are completed before continuing */
  __DSB();
}

/**
  \brief Invalidates data-cache contents associated with buffers accessed by the NPU.

  Ensures that subsequent CPU reads observe data written by the NPU. Dirty
  cache lines may be cleaned before invalidation to preserve pending CPU
  writes. For non-cacheable buffers, no cache invalidation is required,
  but the function ensures that all preceding memory accesses have completed
  before it returns.

  \param[in] base_addr       Array of buffer base addresses.
  \param[in] base_addr_size  Array containing the size, in bytes, of each buffer.
  \param[in] num_base_addr   Number of entries in the address and size arrays.
*/
void ethosu_invalidate_dcache(const uint64_t *base_addr, const size_t *base_addr_size, int num_base_addr) {

  for (int i = 0; i < num_base_addr; i++) {
    if (check_mem_region((const void *)(uintptr_t)base_addr[i], base_addr_size[i])) {
      /* Call CleanInvalidateDCache instead of SCB_InvalidateDCache to avoid silently losing data. */
      /* This avoids losing dirty lines that the CPU has written but not yet committed to memory.  */
      SCB_CleanInvalidateDCache();

      /* Cache is cleaned and invalidated, return immediately */
      return;
    }
  }

  /* No invalidation, just ensure all memory accesses are completed before continuing */
  __DSB();
}
