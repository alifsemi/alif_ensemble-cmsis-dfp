# Create a bare-metal application

The following template can be used to create applications for custom hardware that is based
on [Alif Essemble](https://www.keil.arm.com/packs/ensemble-alifsemiconductor) devices.

## Steps to create an application

- Select from the CMSIS View the menu command: Create a Solution
- Choose a Target Device or Target Board that is based on an Alif Semicondutor Essemble device.
- Select a folder for the CMSIS Solution project.
- Add hardware abstraction layer (i.e. copy a Board layer from the pack).
- Use from the CMSIS View the menu command: Manage Solution Settings and configure a Debug Adapter.
- Use from the CMSIS View the menu command: Build solution.
- Configure the device using the [Alif SETools](https://www.keil.arm.com/packs/ensemble-alifsemiconductor)
- Use from the CMSIS View the menu command: Load & Run or Load & Debug application.

## Copy a Board layer

The [board layer](https://open-cmsis-pack.github.io/cmsis-toolbox/ReferenceApplications/#board-layer) provides system startup, board/device hardware 
initialization, and transfers control to the application. It also exposes various drivers and interfaces.

Ready to use board layers are provided in the directory ${CMSIS_PACK_ROOT}/AlifSemiconductor/Ensemble/2.x.x/Boards.

- Copy the content of a layer directory to CMSIS Solution project directory, for example: `cp .\DevKit-e7\Layers\M55_HP\*.* \Workspace\Alif_CMSIS\Board\*.*`.
- Open the `*.cproject.yml` and add the reference to the laye as shown below.
- Then configure the layer for your target application.

```yml
project:
  groups:
    - group: AppMain
      files:
        - file: README.md
        - file: app_main.c

  layers:
    - layer: $SolutionDir()$/Board/Board_HP.clayer.yml   
#  device: :M55_HP           # remove this node when using a layer with device: specified
```

> _**NOTE**_<br>
> Some layers require CMSIS-RTOS2 functions. In this case add an RTOS kernel, for example RTX or FreeRTOS.

## Toolchains supported

- GCC, ARMCLANG

