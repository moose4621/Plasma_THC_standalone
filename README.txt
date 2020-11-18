Torch height controller, stand alone.

Requires a plasma machine with an ark ok output and an arc voltage output such as the hypertherm machines.

This uses a ATmega328 based board to control the height of the plasma torch from the work by monitoring the voltage and injecting pulses into the z axis step line as well as controlling the dir.

When a cut is initiated, the torch should probe the work surface and reset z height accordingly. The rise to pierce height and fire the torch. When a stable arc is detected, the plasma cutting machine sends a "arc ok" to the controller which then enables the torch height control routine. The torch voltage is monitored an the z axis is adjusted to maintain the required voltage set via a pot and lcd screen. At the end of the cut, the "arc ok" signal is lost and so the thc routine becomes inactive, and the "pass through" routine becomes active.

In pass through mode, all control of the z axis from the cnc controller are passed through to the stepper driver without alteration.

The torch must probe again before the next torch fire because any adjustments made to the z axis during a cut are independant of the cnc controller and therefore untracked.

The response speed, or frequency of steps during thc control can be adjusted by a pot to regulate the stepper speed during corrections.

Schematic and board layout of the controller are in EasyEda format.
