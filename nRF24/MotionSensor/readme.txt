the PCB design is using pin 4 (PCINT11) as 2nd motion sensor input. This pin can only be used
as input after disabling the external interrupt by setting the hfuse to 0x5F which disables further programming

changes for a possible new version:

- change battery connector to 2.54mil and larger thru-holes
- select different pin for 2nd motion sensor
