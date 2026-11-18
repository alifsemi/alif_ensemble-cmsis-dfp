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

#ifndef VSTREAM_AUDIO_OUT_CONFIG_H_
#define VSTREAM_AUDIO_OUT_CONFIG_H_

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

// <h> AppKit-E8 Audio Output Driver
//   <o> Sample Rate [Hz] <8000=> 8 kHz <16000=> 16 kHz <48000=> 48 kHz
//   <i> Rates supported with the board's 12.288 MHz codec clock.
//   <i> Default: 16000
#ifndef VSTREAM_AUDIO_OUT_SAMPLE_RATE
#define VSTREAM_AUDIO_OUT_SAMPLE_RATE   16000U
#endif

//   <o> Bits per Sample <16=> 16-bit <24=> 24-bit <32=> 32-bit
//   <i> 16-bit samples use int16_t; 24-bit and 32-bit samples use int32_t.
//   <i> Stereo interleaved left/right samples; 24-bit data is right-aligned.
//   <i> Default: 16
#ifndef VSTREAM_AUDIO_OUT_SAMPLE_BITS
#define VSTREAM_AUDIO_OUT_SAMPLE_BITS   16U
#endif

//   <o> Headphone Volume <0-100>
//   <i> Default: 80
#ifndef VSTREAM_AUDIO_OUT_VOLUME
#define VSTREAM_AUDIO_OUT_VOLUME        80U
#endif
// </h>
//------------- <<< end of configuration section >>> ---------------------------

#endif /* VSTREAM_AUDIO_OUT_CONFIG_H_ */
