# Documentation rules {#rules_page}

These are the CMSIS conventions from the [CMSIS 6 introduction](https://arm-software.github.io/CMSIS_6/main/General/index.html), applied to this pack. The coding rules there are the rules for the C sources. The page rules are how this HTML book is written.

## Software

- C sources are C99. C++ at an API boundary is C++03.
- Use the types in `<stdint.h>`. Parameters and variables have a complete type.
- Put parentheses around expressions in `#define` constants.
- `CAPITAL` names identify core registers, peripheral registers, and CPU instructions.
- `CamelCase` names identify functions and interrupt handlers.
- A prefix, usually the peripheral name, keeps symbols out of the application namespace.
- The sources follow MISRA C:2012 practice and record deviations. The pack does not claim MISRA compliance. CMSIS states the same limit: there is no guideline-enforcement plan, and the C++ sources are not checked against MISRA C++.

## Comments that become the API reference

A public function uses a Doxygen documentation comment. The comment must start with `/**` or `/*!`. An ordinary `/* ... */` or `//` comment is not a documentation comment.

`docs/Doxyfile` sets `EXTRACT_ALL` to `NO` and `HIDE_UNDOC_MEMBERS` to `YES`. A function that has only an ordinary comment is left out of the API reference. `JAVADOC_AUTOBRIEF` is `NO`, so the first sentence is not taken as the brief description. Write an explicit `@brief`.

    /**
     * @brief Enable the interrupt in the NVIC.
     * @param IRQn Interrupt number.
     * @return None.
     *
     * Other interrupt settings, including priority, are unchanged.
     */

`@param` names each argument. `@return` describes the result. Headers that only have a file banner do not produce a function page. I2C, UART, CRC, CDC, PDM, and ADC already follow this more closely than CAN, GPIO, SPI, and I2S.

## Pages in this book

- User guides are Markdown under `docs/General/`.
- Every page except the home page has a Doxygen id in the title, for example `# Getting started {#gs_page}`.
- The home page lists child pages with `@subpage`, which builds the left-hand tree.
- Cross-references use `@ref`.
- Clocks, memory sizes, and part numbers come from `AlifSemiconductor.Ensemble.pdsc`.
- Product descriptions come from [alifsemi.com](https://alifsemi.com/) and the [Ensemble family page](https://alifsemi.com/products/ensemble/), with the datasheet linked beside them.
- When a SETOOLS part string and the pack memory map differ, both are shown. The datasheet is the source to settle them.
