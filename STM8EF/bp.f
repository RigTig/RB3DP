FILE
\ gcode interpreter in eForth
NVM

\ string supports must be on a horizontal plane 
\  (i.e. at same height above a horizontal ground)
\ calibration and length calculations are based on
\ - the three support points are called S1, S2 and S3, 
\  in an anticlockwise direction
\ - S1 has coordinates of [0,0,0] = machine origin
\ - S2 = [x2,0,0], so the s1 to S2 line is the X axis
\ - S3 = [x3,y3,0]
variable x2 \ mm, say 3000
variable x3 \ mm, say 1500
variable y3 \ mm, say 2000

variable zmax 
\ height from lowest horizontal surface to 
\  plane of string supports in mm, say 2000 


\ where is object origin relative to 
\  machine origin (NB z increases down) [mm]
variable objx \ say, the middle @ 1500
variable objy \ say, the middle @ 1500
variable objz \ say, at lowest level @ zmax

\ effector position, relative to 
\  object origin (NB z increases up) [mm]
variable posx \ 0
variable posy \ 0
variable posz \ 0
\ extruder
variable pose \ 0

variable absMode 
\ absolute (true) or incremental (false) 
\  programming model; TRUE

: pos@ ( -- x y z e )
  posx @ posy @ posz @ pose @ 
  ;
: pos+ ( x1 y1 z1 e1 -- x2 y2 z2 e2 )
  pose @ + >r 
  posz @ + rot ( yy zz xx )
  posx @ + rot ( zz xx yy )
  posy @ + rot ( xx yy zz )
  r> ( xx yy zz ee )
  ;
: pos! ( x y z e -- )
  pose !
  posz !
  posy !
  posx !
;

\ factored out of IK
\ machine coordinates
: mx ( mx my mz -- mx my mz n )
  3 pick ;
: my ( mx my mz d -- mx my mz d n )
  4 pick ;
: mz ( mx my mz d -- mx my mz d n )
  3 pick ;
  
: sd+z2+rt ( mx my mz d n -- mx my mz n )
  sq ud+ mz sq ud+ isqrt ;

\ Inverse Kinematics - turns XYZ object coordinates [mm] 
\  into thread lengths l1,l2 & l3 [mm]
\ currently limited to integer input of 
\  < sqrt(2^32 - 1 = 65,535 mm =~ 65 metres
: IK ( x y z -- l1 l2 l3)
  \ translate object coordinates into machine coordinates
  rot objx @ + 
  rot objy @ +
  rot objz @ swap -
  ( mx my mz -- )
  \ find string lengths in mm
  mx x3 @ - sq my y3 @ - sd+z2+rt >r \ l3
  mx x2 @ - sq my sd+z2+rt >r \ l2
  mx sq my sd+z2+rt >r \ l1
  drop 2drop ( -- )
  r> r> r> ( l1 l2 l3 -- )
;

\ Forward Kinematics - turns lengths of string (in mm) 
\  into XYZ object coordinates
: FK ( l1 l2 l3 -- x y z )
  3 pick ( l1 ) sq 4 pick ( l2 ) sq d- x2 @ sq ud+
  ( l1 l2 l3 d ) x2 @ 2* m/mod nip ( l1 l2 l3 x )
  4 pick sq 4 pick ( l3 ) sq d- 
  3 pick ( x ) sq d- ( l1 l2 l3 x d )
  3 pick ( x ) x3 @ - sq ud+
  y3 @ sq ud+
  y3 @ 2* m/mod nip ( l1 l2 l3 x y )
  5 pick sq 4 pick ( x ) sq d-
  3 pick ( y ) sq d-  isqrt
  ( l1 l2 l3 x y z  in machine space )
  >r >r >r drop 2drop ( clean up the stack )
  \ translate to object space
  r> objx @ - ( x )
  r> objy @ - ( x y )
  objz @ r> - ( x y z )
;
 
variable xyzeRate \ mm/sec
variable eFactor


: motorsOn ( -- )
  .$" motorsOn" CR \ just pass on to ESP8266
;
: motorsOff ( -- )
  .$" motorsOff" CR \ just pass on to ESP8266
;

: moveMsg ( l ms n -- )
  \ just pass on to ESP8266
  rot . space swap . space . space .$" moveMsg" CR
  ;

: line ( x y z e -- )
  \ starting at {posx, posy, posz, pose}
  \  go to {x, y, z, e} at speed stored in globals
  
  \ calculate time to move; need distance and rate
  4 pick ( x ) posx @ - sq ( x y z e d )
  5 pick ( y ) posy @ - sq ud+
  4 pick ( z ) posz @ - sq ud+ 
  isqrt \ distance to move in mm
  2 over <  if \ move if at least 2mm, otherwise ignore
    1000 xyzeRate */ \ ms for move (xyzeRate is mm/sec)
    ( x y z e ms )
  
    \ do the moves by telling slaves what to do
    \ 2do: handle extruder here
    5 pick 5 pick 5 pick IK ( x y z e ms l1 l2 l3 )
    4 pick ( ms ) 2 moveMsg 
    3 pick ( ms ) 1 moveMsg
    swap 0 moveMsg
  
    \ update position records
    pos!
  else ( x y z e n ) drop 2drop 2drop
  then
  ;

\ parse next code and integer
: cint ( b u -- c n )
  over c@ rot rot ( c b u ) 1+
  1 do 
    dup i + c@ $2f $3a within not if
      i leave then 
    loop ( c b u ) 1-
  over c! number? drop
  ;

: moreParam? ( b u -- f )
  dup 0= if ( end of line ) nip else 
    \ 
    -1 swap \ assume true
    \ ; =$3B is end of GCODE
    0 do over i + c@ $3B = 
      if drop 0 leave then
    loop nip 
  then 
  ;

\ GCODES

: G00 \ move in a straight line
  absMode @ if
    pos@ else
    0 0 0 0 then ( xx yy zz ee )
  begin $20 parse ( b u ) 
    2dup >r >r 
    dup 0= if 2drop else
      cint ( xx yy zz ee c n )
      over ( 'F'= ) $46 = if 
        xyzeRate ! drop else
      over ( 'E'= ) $45 = if
        \ replace e on stack with n
        nip nip else
      over ( 'Z'= ) $5A = if 
        \ replace z on stack with n
        nip swap >r nip r> else
      over ( 'Y'= ) $59 = if 
        \ replace y on stack with n
        nip swap >r swap >r nip r> r> else
      over ( 'X'= ) $58 = if 
        \ replace x on stack with n
        nip swap >r swap >r swap >r nip r> r> r> else
      2drop \ ignore unknown char
    then then then then then
    then
    ( xx yy zz ee ) 
    r> r> ( b u ) moreParam? while  
    repeat
  absMode @ not if
    pos+
  then 
  line
  ;
: G01 G00 ;
: G0 G00 ;
: G1 G00 ;

\ G04 \ dwell
\   [XUP]n

: G20; \ set input to be in inches
  cr ." \ sorry, input must be in mm"
  ;
: G20  G20; ; \ just in case no semicolon  
  
\ doesn't do anything, so can simply be ignored  
\ : G21; \ set input to be in millimeters
\   ;
\ : G21 G21; ; \ just in case no semicolon

: G28 \ self-check and move to home
  \ 2do: self-check
  0 0 0 pose @ line \ no extrusion
  0 pose ! \ reset extruder position
  ;

: G90; \ absolute mode
  -1 absMode !
  ;
: G90 G90; ; \ just in case no semicolon

: G91; \ relative (or incremental) mode
  0 absMode !
  ;
: G91 G91; ; \ just in case no semicolon

: G92 \ set current position to values
  \ and re-map the object origin accordingly
  \ does not validate if the virtual move is valid
  begin $20 parse ( b u ) 
    2dup >r >r 
    dup 0= if 2drop else
      cint ( c n )
      over ( 'E'= ) $45 = if pose ! drop else
      over ( 'Z'= ) $5A = if 
        dup posz @ - objz +!   posz !  drop else
      over ( 'Y'= ) $59 = if 
        dup posy @ - objy +!   posy !  drop else
      over ( 'X'= ) $58 = if 
        dup posx @ - objx +!   posx !  drop else
      2drop \ ignore unknown char
    then then then then
    then
    r> r> ( b u ) moreParam? while  
    repeat
  ;

: M17; \ 
  \ enable motors
  motorsOn
  ;
: M17 M17; ; \ just in case no semicolon

: M18; \ disable motors
  \ no parameters, so usually has semicolon
  motorsOff
  ;
: M18 M18; ; \ just in case no semicolon

: M114 \ current status, diagnostic
  CR
  ." \ Object: X" posx @ 1 U.R
  space ." ,Y" posy @ 1 U.R  space ." ,Z" posz @ 1 U.R
  space ." ,E" pose @ 1 U.R  space ." ,F" xyzeRate @ 1 U.R
  CR
  ." \ Offsets from machine origin ("
  objx ? ." ," objy ? ." ," objz ? ." )"
  CR
  ." \ Motors: M1=( 0, 0, 0) M2=(" x2 ? 
  ." , 0, 0) M3=(" x3 ? ." ," y3 ? ." , 0)"
  ;

\ Non-standard GCODE commands

: CONFIG \ lengths between supports, and height
  x2 @ \ l12
  y3 @ sq  x2 @ x3 @ - sq  ud+ isqrt \ l23
  x3 @ sq  y3 @ sq  ud+  isqrt \ l13
  ( l12 l23 l13 )
  begin $20 parse ( b u ) 
    2dup >r >r 
    dup 0= if 2drop else
      cint ( l12 l23 l13 c n )
      over ( 'P'= ) $50 = if \ new l12
        nip swap >r swap >r nip r> r> else
      over ( 'Q'= ) $51 = if \ new l23
        nip swap >r nip r> else
      over ( 'R'= ) $52 = if \ new l13
        nip nip else
      over ( 'H'= ) $48 = if zmax ! drop else
      2drop \ ignore unknown char
    then then then then
    then
    ( l12 l23 l13 ) 
    r> r> ( b u ) moreParam? while  
    repeat
  ( l12 l23 l13 )
  CR ." \ l12=" 3 pick . 
  ." , l23=" 2 pick . 
  ." , l13=" dup . 
  ( l12 l23 l13 )
  3 pick x2 !
  3 pick ( l12 ) sq  3 pick ( l13 ) sq  ud+ 
    4 pick ( l23 ) sq  d-
    5 pick ( l12 ) 2*  um/mod nip  x3 !
  ( l13 ) sq  x3 @ sq  d- isqrt  y3 !
  ( l12 l23  ) 2drop
;  



\ GCODE interpreter
\ ignore lines starting with undefined words
: GEV ( -- )
  NAME? IF EXECUTE ELSE DROP THEN 
  0 PARSE 2DROP ; \ bypass rest of the line
  
: setvars
  \ defaults for variables to assist in testing
  3000 x2 !
  1500 x3 !
  2000 y3 !
  2000 zmax !
  1500 objx !
  1000 objy !
  2000 objz !
  -1 absMode ! \ true
  \ start as fast as practical
  5000 xyzeRate !
  10 eFactor !
  \ initialize the plotter position.
  0 0 0 0 pos!
  ;
  
: init_ ( -- )
  \ " AVAGO" UID $!
  setvars
  
  \ ready motors
  motorsOn
  
  \ change interpreter over for GCODE
  ' GEV 'EVAL !
  FILE \ prompt for new line with ascii VT
  ;

RAM
  
