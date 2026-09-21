Duck House Controller Rev 1.8 - Wiring Pass 1

This is the first wiring pass using the uploaded 38-pin ESP32-DevKitC footprint.

Completed in this pass:
- ESP32 DevKitC symbol/pin mapping installed.
- GPIO net labels terminate at the corresponding ESP32 pins.
- BTS7960 connector signal nets labeled and connected by net name.
- Limit switch connector nets labeled.
- Pushbutton connector nets labeled.
- Status LED circuit added (GPIO4 / 330R / blue LED).
- Pulldowns added for GPIO5, GPIO12 and GPIO17.
- Unused ESP32 pins are marked No Connect.

Still to complete and verify:
- Full limit-switch RC networks.
- Full pushbutton RC networks.
- Photosensor RC/input network.
- Travel-pot networks.
- Battery ADC divider and filter.
- 5V/3V3 power symbols and distribution.
- Reverse-polarity MOSFET and TVS implementation.
- BTS7960 logic power switch component-level wiring.
- ERC power flags and final ERC review.

The schematic is intentionally still a work-in-progress and should not yet be used as a PCB manufacturing source.
