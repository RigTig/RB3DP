\ bp_ESP.forth
\ ESP8266 part of ESP14 needs to handle 
\ * receiving GCODE via wifi and sending to STM8
\ receiving and processing commands from STM8

: init_ ( -- )
  \ next two lines are automatic in ESP8266 based on previous activity
  172 16 0 10 >ipv4 wifi-set-ip
  4 3 0 AUTH_WPA2_PSK str: "1234567890" str: "AVAGO" wifi-softap
  1 172 16 0 1 >ipv4 dhcpd-start
  ;

\ 8088 wifi-ip netcon-tcp-server init-variable: nc-local
\ nc-local @ netcon-accept init-variable: nc-rem
\ 128 buffer: inline
\ nc-rem @ 128 inline netcon-readln
\ cr inline type
\ nc-rem @ str: " >" netcon-write


: msgMotor ( n a -- )
  \ temp; just echoing command effect
  swap cr . space type 
;

\ commands expected from STM8

: motorsOn ( -- )
  \ tell all motors to get ready to move
  4 0 do i str: " set-motor-pins-out" msgMotor loop 
  ;
  
: motorsOff ( -- )
  \ tell all motors to ignore any moves until turned on
  4 0 do i str: " set-motor-pins-in" msgMotor loop
  ;
  
: moveMsg ( l ms n -- )
  \ message motor n to move to absolute position l
  \ in the time specified in ms
  \ temp; just echoing command effect
  cr print: " motor " .   print: " :" space . space .  print: " move"
  ;


  
  
\ ' boot is: main
\ turnkey abort
