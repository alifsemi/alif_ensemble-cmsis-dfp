# CMake Configurations

This document contains all the CMake files necessary for including header directories and source files required to compile and generate binaries.

## Directory Structure

**rtss**

- **device_cmake**  
  - Contains the CMake file with details related to devices, including **RTSS_HP/RTSS_HE** and **M55_HP/M55_HE**.

- **drivers_cmake**  
  - Includes the CMake file that specifies the source files and header files for all supporting drivers.

- **os_cmake**  
  - Contains CMake files with specifics on OS-related source and header files, including details about test applications for each supported operating system.

- **toolchains**  
  - Contains the CMake file with details about the compiler toolchain and configure default compiler arguments for the RTSS environment.

## Source Files 

- **CMakeLists.txt**  
  - The primary CMake file that manages the integration of all the aforementioned CMake files based on the project's configuration and requirements. It also includes source files for test applications within the build process.

- **utilities_func.cmake**  
  - A CMake script file containing helper functions and macro definitions vital for the build process.

- **CMakePresets.json.**  
  - A collection of CMake presets .

- **run.sh.**  
  - A wrapper bash script which can used to run all avaiable presets i.e. workflow or reuse existing preset and compile as per user arguments

- **build_stats.sh.**  
  - A script which gives build statistics of any given configuration.

- **se_flashing.sh.**  
  - A bash script which will handle/control SE flashing.
