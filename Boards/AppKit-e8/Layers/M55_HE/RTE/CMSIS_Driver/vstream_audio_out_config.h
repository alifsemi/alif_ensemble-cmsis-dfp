/*
 * Copyright 2026 Arm Limited and/or its affiliates.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef VSTREAM_AUDIO_OUT_CONFIG_H_
#define VSTREAM_AUDIO_OUT_CONFIG_H_

/* Board driver configuration. Independent of the application configuration. */
/* Buffer addresses align to the sample container (2 or 4 bytes), and block sizes
 * contain whole stereo frames (4 or 8 bytes). Trailing buffer bytes are ignored.
 * Continuous underflow sends 64-frame silence transfers and reports UNDERFLOW
 * per missing-payload demand. Silence produces no DATA. Later commits resume at
 * the next transfer boundary, without Start or a status read. Single completion
 * leaves TX enabled for FIFO drainage; Stop/Uninitialize provides safe teardown.
 */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------
// <h> AppKit-E8 Audio Output Driver
//   <o> Sample Rate [Hz] <8000=> 8 kHz <16000=> 16 kHz <48000=> 48 kHz
//   <i> Rates supported with the board's 12.288 MHz codec clock.
#ifndef VSTREAM_AUDIO_OUT_SAMPLE_RATE
#define VSTREAM_AUDIO_OUT_SAMPLE_RATE 16000U
#endif

//   <o> Bits per Sample <16=> 16-bit <24=> 24-bit <32=> 32-bit
//   <i> 16-bit samples use int16_t; 24-bit and 32-bit samples use int32_t.
//   <i> Stereo interleaved left/right samples; 24-bit data is right-aligned.
#ifndef VSTREAM_AUDIO_OUT_SAMPLE_BITS
#define VSTREAM_AUDIO_OUT_SAMPLE_BITS 16U
#endif

//   <o> Headphone Volume <0-100>
#ifndef VSTREAM_AUDIO_OUT_VOLUME
#define VSTREAM_AUDIO_OUT_VOLUME      80U
#endif
// </h>
//------------- <<< end of configuration section >>> ---------------------------

#if (VSTREAM_AUDIO_OUT_SAMPLE_RATE != 8000U) && \
    (VSTREAM_AUDIO_OUT_SAMPLE_RATE != 16000U) && \
    (VSTREAM_AUDIO_OUT_SAMPLE_RATE != 48000U)
#error "AppKit-E8 AudioOut supports 8, 16 or 48 kHz"
#endif

#if (VSTREAM_AUDIO_OUT_SAMPLE_BITS != 16U) && \
    (VSTREAM_AUDIO_OUT_SAMPLE_BITS != 24U) && \
    (VSTREAM_AUDIO_OUT_SAMPLE_BITS != 32U)
#error "AppKit-E8 AudioOut supports 16, 24 or 32-bit samples"
#endif

#if VSTREAM_AUDIO_OUT_VOLUME > 100U
#error "AppKit-E8 AudioOut volume must be in the range 0 to 100"
#endif

#endif /* VSTREAM_AUDIO_OUT_CONFIG_H_ */
