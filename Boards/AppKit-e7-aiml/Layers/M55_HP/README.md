# Board: AlifSemiconductor AppKit

## Board Layer for M55 High Performance Core

Device: AE722F80F55D5LS:M55_HP

This setup is configured using **Conductor Tool**, an interactive tool provided by Alif Semiconductor for device configuration.
Refer to ["Alif Conductor Tool Manual"](https://conductor.alifsemi.com/Alif_HTML_DCT_User_Help/Content/Help%20Manual.htm) for additional information.

### System Configuration

| System Component        | Setting
|:------------------------|:----------------------------------------
| Heap                    | 32 kB (configured in M55_HP linker file)
| Stack (MSP)             |  8 kB (configured in M55_HP linker file)

### STDIO mapping

**STDIO** is routed to Virtual COM port via **UART4** peripheral

### CMSIS-Driver mapping

| CMSIS-Driver           | Peripheral | Board connector/component  | Connection
|:-----------------------|:-----------|:---------------------------|:----------------------
| Driver_USART4          | UART4      | PRG USB connector (J3)     | STDIN, STDOUT, STDERR
| CMSIS-Driver VIO       | GPIO       | Buttons, RGB LED, Joystick | CMSIS_VIO
| Driver_vStreamAudioIn  | SAI3       | I2S Microphone (U13, U15)  | CMSIS_VSTREAM_AUDIO_IN
| Driver_vStreamVideoIn  | MIPI CSI   | MIPI Camera (J10, J12)     | CMSIS_VSTREAM_VIDEO_IN
| Driver_vStreamVideoOut | MIPI DSI   | GLCD Display (J18)         | CMSIS_VSTREAM_VIDEO_OUT

### CMSIS-Driver Virtual I/O mapping

| CMSIS-Driver VIO | Board component
|:-----------------|:----------------------------
|vioBUTTON0        | Joystick Select Button
|vioJOYup          | Joystick Up
|vioJOYdown        | Joystick Down
|vioJOYleft        | Joystick Left
|vioJOYright       | Joystick Right
|vioJOYselect      | Joystick Select Button
|vioLED0           | RGB LED Red
|vioLED1           | RGB LED Green
|vioLED2           | RGB LED Blue

### CMSIS-Driver vStream configuration

| Driver                 | Stream Format Description
|:-----------------------|:----------------------------------------------------
| Driver_vStreamAudioIn  | 16-bit PCM audio,      16000 samples/second
| Driver_vStreamVideoIn  | RAW8 Bayer GBRG video, resolution 1280 x 720 (W x H)
| Driver_vStreamVideoOut | RGB888 video,          resolution  480 x 800 (W x H)
