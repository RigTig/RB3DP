FILE
\ some maths utility words

NVM

\ eForth + is u+
: u+ ( n n -- u )
   +
;

\ : DNEGATE ( d -- -d )
\   \ fix for bug in v2.2
\   NOT >R NOT 1 UM+ R> + ;
\ fixed in v2.2+VARIABLE

: ud+ ( d d -- ud )
  ( l h l h ) 
  >r rot um+ ( h d ) rot u+ ( d ) r> u+
;
: d- ( d d -- d )
 \ 2do: ensure works on full range
 dnegate ud+
;
: d> ( d d -- f )
  d- 2dup or 0= ( d f ) rot rot 
  0< swap drop or not 
;

: sq ( n -- d )
  abs dup um*
;

: isqrt ( d -- n )
 \ see Fixed Point sqrt_02 for C code
 $8000 ( d c ) $8000 ( d c g )
 begin
  \ cr 2dup . . 
  dup sq ( d c g g^2)
  6 pick 6 pick ( d c g g^2 d )
  d> if ( d c g ) over xor then ( d c g )
  swap 2/ $7fff and ( d g c )
  dup 0= if 
    drop rot rot 2drop -1 ( g true ) 
  else
    swap over ( d c g c ) or 0 
  then
 until
;

RAM
