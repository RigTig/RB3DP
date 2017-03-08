\ to move 28BYJ-48 stepper motor using ULN2003 driver

\ The 28BYJ-48 motor is a 4-phase, 8-beat motor, geared down by
\ a factor of approximately 64. One bipolar winding is on motor pins 1 & 3 and
\ the other on motor pins 2 & 4.  
\ Blue   - 28BYJ48 wire 1
\ Pink   - 28BYJ48 wire 2
\ Yellow - 28BYJ48 wire 3
\ Orange - 28BYJ48 wire 4
\ Red    - 28BYJ48 wire 5 (VCC)
\ Each motor is connected to a ULN2003 driver

\ connect wires 1,2,3,4 to D1, D2, D3, D4
4 byte-array: pin-motor
5  0 pin-motor c! \ D1
4  1 pin-motor c! \ D2
0  2 pin-motor c! \ D3
2  3 pin-motor c! \ D4

: set-motor-pins-out ( -- )
    4 0 do i pin-motor c@ GPIO_OUT gpio-mode loop ;

\ lookup for mask is simpler than calculation
4 byte-array: pin-mask
1  0 pin-mask c! \ D1
2  1 pin-mask c! \ D2
4  2 pin-mask c! \ D3
8  3 pin-mask c! \ D4


\ sequence of microsteps (one bit per wire)
\ {B00001, B00011, B00010, B00110, B00100, B01100, B01000, B01001}
8 byte-array: microstep 
\ vertical_voltage horizontal_voltage +> magnetic direction = relative to vertical, clockwise
 1 0 microstep c! \ +5  0 => |^ =0=360
 3 1 microstep c! \ +5 +5 => /> =45
 2 2 microstep c! \  0 +5 => -> =90
 6 3 microstep c! \ -5 +5 => \> =135
 4 4 microstep c! \ -5  0 => | = 180
12 5 microstep c! \ -5 -5 => </ =225
 8 6 microstep c! \  0 -5 => <- =270
 9 7 microstep c! \ +5 -5 => <\ =315


\ \ sequence of microsteps (one bit per wire)
\ \ {B01110, B01100, B01101, B01001, B01011, B00011, B00111, B00110}
\ 8 byte-array: microstep
\ 14 0 microstep c!
\ 12 1 microstep c!
\ 13 2 microstep c!
\  9 3 microstep c!
\ 11 4 microstep c!
\  3 5 microstep c!
\  7 6 microstep c!
\  6 7 microstep c!

: microstep2pin ( microstep@ pin-motor# -- ) \ just one pin
    dup pin-mask c@ >r pin-motor c@ swap r> and if GPIO_HIGH else GPIO_LOW then gpio-write ;
  
: microstep2pins ( microstep# -- ) \ all 4 pins
    microstep c@ 4 0 do dup i microstep2pin loop drop ;

\ The microstep angle is 5.625/64 degrees
512 constant: STEPS_PER_REV

\ No-load pull-in frequency: > 500 or 600 Hz (depends on mfr)
\ No-load pull-out frequency: > 900 or 1000 Hz (depends on mfr)
\ if can reliably microstep at 1000 Hz, then maximum rpm 
\ = 1000 microstep/sec x 60 sec/min / 512 step/revolution / 8 microsteps/step = 14.6 rev/min
\   equivalent to 20 seconds per meter
\   = 20 mSec/mm = 20,000 uSec/mm

\ calibration for motor
399 init-variable: um_per_step \ assuming shaft diameter is 65 mm

\ there are 8 microsteps per step
: step-forward ( -- )
    8 0 do i microstep2pins ( non-blocking delay needed for timing ) loop ;

