Basic Proteus Information  {#proteus}
=======

Introduction {#proteus_intro}
===
The Proteus is controller with a powerful Freescale ARM Cortex M4 processor, a touch screen, digital/analog I/O, and motor and servo outputs.  It is programmable in C++, using VSCode with the FEHVSCode extension. For more information, see the Hello, World! tutorial [here](@ref hello_world).

The name "Proteus" is taken from the god of rivers in Greek mythology.  The adjective "protean" means capable of assuming many forms. The Proteus was developed to replace the MIT Handyboard, which was used previously.

In order to extend the life of the Proteus, and to avoid possible fees at the end of the semester, please adhere to the following:

- Do not drop the Proteus. Handle with care!
- Do not remove the screen protector from the screen of the Proteus.  If the Proteus' screen protector starts to come off, return it to the FEH store for servicing.
- Do not put Velcro on the face of the Proteus with the screen or the face with the motor ports.
- Do not disassemble the Proteus.
- Do not wrap the power adapter cable around the power adapter, as this will cause it to break.

Proteus Kit Contents {#proteus_kit_contents}
===
The Proteus is stored in a plastic container, containing:
- The Proteus controller
- MicroSD to USB adapter
- MicroSD card
- Stylus
- Power adapter

The Proteus should always be stored in the plastic container.  It should *never* be stored on a robot.

All components, and the plastic container, should be labelled with the same number.  Take care to ensure that only components with the same number are stored together.

Charging {#proteus_charging}
===
To charge the Proteus, plug the power adapter into the port highlighted in the image below.
TODO

There are three LED colors that indicate the charging status of the Proteus. The LED is highlighted on the image below.

- Green: Proteus is fully charged.
- Orange: Proteus is charging.
- Red: Proteus needs to be returned to the FEH store.

![](@ref proteus_charging.png)

Technical Specifications {#proteus_tech_specs}
===
- 4.5" x 4.5" x ?
- Freescale Kinetis K60 (ARM Cortex M4) processor
    - 96 MHz single core
- 128 kB RAM
- 8 core parallax propeller chip
- Programmable I/O
    - 32 programmable 3.3V I/O ports, capable of:
        - Digital input
        - Digital output
        - Digital encoders
        - Analog input
        - Analog encoders
    - 8 servo ports (5V)
    - 4 DC motor ports (12V)
- 320 x 240 (2.79" x 2.12") color touchscreen LCD display
- Micro-SD slot
- Mini USB programming port
- XBee wireless module

![](@ref proteus_internals.png)

Proteus ASEE Paper {#proteus_paper}
===
More detailed information can be obtained by reading the [ASEE](http://www.asee.org/) (American Society for Engineering Education) paper written about the proteus, located [here](https://osu.box.com/s/1j9602pfb1m47n4u2jlvmduz5o7l4quj).