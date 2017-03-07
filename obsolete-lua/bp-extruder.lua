-- 3D drawing/printing robot - Supports 28BYJ-48
-- CPU intensive routines delegated from Arduino  

-- needs to know (at least until add something to motor code
local STEPS_PER_REV = 512 -- see D01

--[[ string supports must be on a horizontal plane (i.e. at same height above a horizontal ground)
 calibration and length calculations are based on
 - the three support points are called S1, S2 and S3, in an anticlockwise direction
 - S1 has coordinates of [0,0,0] = machine origin
 - S2 = [x2,0,0], so the S1 to S2 line is the X axis
 - S3 = [x3,y3,0]
]]
x2, x3, y3 = 3000.0, 1500.0, 2000.0 -- mm

zmax = 2800.0 -- mm height from lowest horizontal surface to plane of string supports  

-- where is object origin relative to machine origin (NB z increases down) [mm]
objx = 1500.0 -- say, the middle
objy = 1500.0 -- say, the middle
objz = zmax -- say, at lowest level

-- effector position, relative to object origin (NB z increases up) [mm]
posx = 0.0
posy = 0.0
posz = 0.0
--extruder
pose = 0.0

absolute_mode = true -- absolute (true) or incremental (false) mode

mmPerUnit = 1.0 -- conversion factor
inputUnit_Name = 'mm' -- what units is input using

UID = 'SSID' -- name of printer, same as access point SSID

-- --------------------------------------------------------------------------------------
-- Inverse Kinematics - turns XYZ object coordinates into thread lengths l1,l2 & l3 in um
function IK(x, y, z)
  -- translate object coordinates into machine coordinates
  local mx = x + objx
  local my = y + objy
  local mz = objz - z
  
  -- find string lengths in um, i.e. mm * 1000 um/mm
  local mzmz = mz*mz
  l1 = math.floor( math.sqrt(mx*mx + my*my + mzmz) * 1000.0 + 0.5) 
  l2 = math.floor( math.sqrt((mx-x2)*(mx-x2) + my*my + mzmz) * 1000.0 + 0.5)
  l3 = math.floor( math.sqrt((mx-x3)*(mx-x3) + (my-y3)*(my-y3) + mzmz) * 1000.0 + 0.5)
  return l1, l2, l3
end

-- Forward Kinematics - turns lengths of string (in um) into XYZ object coordinates
function FK(l1, l2, l3)
  -- scale um to mm
  local a = l1 * 0.001
  local b = l2 * 0.001
  local c = l3 * 0.001

  local x = (a*a - b*b + x2*x2)/(2*x2)
  local y = (a^2 - c^2 - x^2 + (x-x3)^2 + y3^2)/(2*y3)
  local z = math.sqrt(a^2 - x^2 - y^2)
  
  --in object space
  x = x - objx
  y = y - objy
  z = objz - z
  
  return x, y, z
end

function msgMotor(n, msg)
  -- stub for now
  print('motor ' .. n .. ' >' .. msg)
  
end

eRate = 10.0 -- speed for extruder
xyzSpeed = 5000 -- um/sec = speed for effector (head) in 3D space

function line(x, y, z, e) 
  -- starting at {posx, posy, posz, pose}
  --  go to {x, y, z, e} at speeds stored in globals
  local tmp = (x-posx)^2 + (y-posy)^2 + (z-posz)^2
  if(tmp < .1 and math.abs(pose - e) < 1) then return end -- already there (within 10 um)
  
  -- calculate string lengths at end posn
  local l1,l2,l3,l4;  
  l1, l2, l3 = IK(x,y,z);
  l4 = e * eRate; --2do: define interface and conversion for extruder
  
  -- calculate time for movement: distance to move / rate of movement
  -- 2do check maths for length
  local dist = math.sqrt(tmp) * 1000 -- um
  local duration = 1000000 * dist / xyzSpeed -- microseconds
  
  -- send target length and duration to get there to each motor
  msgMotor(0, string.format('%d', duration) .. ' ' .. (l1) .. ' move')
  msgMotor(1, string.format('%d', duration) .. ' ' .. (l2) .. ' move')
  msgMotor(2, string.format('%d', duration) .. ' ' .. (l3) .. ' move')
  msgMotor(3, string.format('%d', duration) .. ' ' .. (l4) .. ' move')
  
  -- in object space
  posx=x;
  posy=y;
  posz=z;
  pose=e;
end


function setPos(x, y, z, e)
  -- re-map object origin 
  objx = objx + x - posx
  objy = objy + y - posy
  objz = objz + z - posz
  -- set current position
  posx = x;
  posy = y;
  posz = z;
  -- set extruder to current position
  pose = e;
end

function GoHome()
  pose = 0 -- reset extruder position to start again
  line(0, 0, 0, 0) 
end
  
function MotorsOn()
  msgMotor(0, 'set-motor-pins-out' )
  msgMotor(1, 'set-motor-pins-out' )
  msgMotor(2, 'set-motor-pins-out' )
  msgMotor(3, 'set-motor-pins-out' )
end

function help() 
  print("++ RigTig's Big 3D Printer ++")
  print("HELP;  - display this message")
  print("CONFIG [Ax.x] [Bx.x] [Cx.x] [Px.x] [Qx.x] [Rx.x] [Hx.x];")
  print(" - set current physical position as object origin")
  print(" - can set distances between supports, string lengths.")
  print("UID str; - set the identifier used to distinguish printers.")
  print("D00 n um; - move motor n in um (+/-).")
  print("D01 n diam; - adjust shaft diameter (mm); set extruder factor")
  print("D02 ; - show shaft diameter and extruder factor")
  print("As well as the following G-codes (http://en.wikipedia.org/wiki/G-code):")
  print("G00,G01,G02,G03,G04,G20,G21,G28,G90,G91,G92,M17,M18,M114")
end


function where() 
  print("Object using "..inputUnit_Name..
    ": X"..string.format('%.2f', posx/mmPerUnit)..
    " Y"..string.format('%.2f', posy/mmPerUnit)..
    " Z"..string.format('%.2f', posz/mmPerUnit)..
    " E"..string.format('%.2f', pose/mmPerUnit)
  )
  print("Offsets from machine origin:"..
    string.format('%.2f',objx)..
    ", "..string.format('%.2f', objy)..
    ", "..string.format('%.2f', objz))
  
  printFeedRate()
  
  if(absolute_mode) then
    print('Absolute mode')
  else
    print('Relative mode')
  end
      
end

function printConfig() 
  print("Using "..(inputUnit_Name))
  print("S1=[0,0,0]")
  print("S2=["..(x2/mmPerUnit)..",0,0]") 
  print("S3=["..(x3/mmPerUnit)..","..(y3/mmPerUnit)..",0]") 
  print((pose/mmPerUnit).." extruder ")
end

function printFeedRate()
  print('F'..string.format('%d',60 * xyzSpeed / mmPerUnit)..'  [ '..(inputUnit_Name).."/min ]")
end

function G00_G01(buffer)
  local xx, yy, zz, ee

  if(absolute_mode) then
    xx=posx
    yy=posy
    zz=posz
    ee=pose
  else -- relative mode
    xx=0
    yy=0
    zz=0
    ee=0
  end

  -- overwrite local config values when found
  local sStart, sEnd, sCapture
  -- target for move, in millimetres 
  sStart, sEnd, sCapture = string.find(buffer, 'X(%-?%d[%.%d]*)') --
  if(sStart) then -- got a value for X
    xx=tonumber(sCapture)*mmPerUnit -- mm, can be negative
  end
  sStart, sEnd, sCapture = string.find(buffer, 'Y(%-?%d[%.%d]*)') --
  if(sStart) then -- got a value for Y
    yy=tonumber(sCapture)*mmPerUnit -- mm, can be negative
  end
  sStart, sEnd, sCapture = string.find(buffer, 'Z(%-?%d[%.%d]*)') --
  if(sStart) then -- got a value for Z
    zz=tonumber(sCapture)*mmPerUnit -- mm, can be negative
  end
  -- target for extruder
  sStart, sEnd, sCapture = string.find(buffer, 'E(%-?%d[%.%d]*)') --
  if(sStart) then -- got a value for E
    zz=tonumber(sCapture)*eRate -- can be negative
  end
  -- move at this speed until another speed is set
  sStart, sEnd, sCapture = string.find(buffer, 'F(%d[%.%d]*)') --
  if(sStart) then -- got a value for F (units/minute)
    xyzSpeed=tonumber(sCapture)*mmPerUnit/60 -- mm/sec
  end

  if(not absolute_mode) then
    xx = xx + posx
    yy = yy + posy
    zz = zz + posz
    ee = ee + pose
  end

  line(xx,yy,zz,ee)
end

function G02_G03(buffer)
  local xx, yy, zz, ee, dd
  if(string.sub(buffer,1,4) == "G02 " or
    string.sub(buffer,1,3) == "G2 ") then -- 2do: check direction
    dd = 1
  else
    dd = -1
  end
  local ii = 0;
  local jj = 0;

  if(absolute_mode) then
    xx=posx
    yy=posy
    zz=posz
    ee=pose
  else -- relative mode
    xx=0
    yy=0
    zz=0
    ee=0
  end

  -- overwrite local config values when found
  local sStart, sEnd, sCapture
  -- centre of rotation
  -- 2do: can cor be negative input?
  sStart, sEnd, sCapture = string.find(buffer, 'I(%-?%d[%.%d]*)') --
  if(sStart) then -- got a value for I
    ii=tonumber(sCapture)*mmPerUnit -- mm, can be negative
  end
  sStart, sEnd, sCapture = string.find(buffer, 'J(%-?%d[%.%d]*)') --
  if(sStart) then -- got a value for J
    jj=tonumber(sCapture)*mmPerUnit -- mm, can be negative
  end
  -- target for move, in millimetres 
  sStart, sEnd, sCapture = string.find(buffer, 'X(%-?%d[%.%d]*)') --
  if(sStart) then -- got a value for X
    xx=tonumber(sCapture)*mmPerUnit -- mm, can be negative
  end
  sStart, sEnd, sCapture = string.find(buffer, 'Y(%-?%d[%.%d]*)') --
  if(sStart) then -- got a value for Y
    yy=tonumber(sCapture)*mmPerUnit -- mm, can be negative
  end
  sStart, sEnd, sCapture = string.find(buffer, 'Z(%-?%d[%.%d]*)') --
  if(sStart) then -- got a value for Z
    zz=tonumber(sCapture)*mmPerUnit -- mm, can be negative
  end
  -- target for extruder
  sStart, sEnd, sCapture = string.find(buffer, 'E(%-?%d[%.%d]*)') --
  if(sStart) then -- got a value for E
    zz=tonumber(sCapture)*eRate -- can be negative
  end
  -- move at this speed until another speed is set
  sStart, sEnd, sCapture = string.find(buffer, 'F(%d[%.%d]*)') --
  if(sStart) then -- got a value for F (units/minute)
    xyzSpeed=tonumber(sCapture)*mmPerUnit/60 -- mm/sec
  end

  if(not absolute_mode) then
    xx = xx + posx
    yy = yy + posy
    zz = zz + posz
    ee = ee + pose
  end

  -- 2do arc(posx+ii,posy+jj,xx,yy,zz,dd,ee);
  line(xx,yy,zz,ee) -- just a straight line for now

end

function G04(buffer)    
  local sStart, sEnd, sCapture
  sStart, sEnd, sCapture = string.find(buffer, '[XUP](%d[%.%d]*)') --
  if(sStart) then -- got a value for X, U or P, so hang about
    -- 2do: delay(tonumber(sCapture))
  end
end

function G92(buffer)
  local xx=posx
  local yy=posy
  local zz=posz
  local ee=pose

  local sStart, sEnd, sCapture
  sStart, sEnd, sCapture = string.find(buffer, 'X(%d[%.%d]*)') --

  if(sStart) then -- got a value for X
    xx=tonumber(sCapture) * mmPerUnit
  end
  sStart, sEnd, sCapture = string.find(buffer, 'Y(%d[%.%d]*)') --
  if(sStart) then -- got a value for Y
    yy=tonumber(sCapture) * mmPerUnit
  end
  sStart, sEnd, sCapture = string.find(buffer, 'Z(%d[%.%d]*)') --
  if(sStart) then -- got a value for Z
    zz=tonumber(sCapture) * mmPerUnit
  end
  sStart, sEnd, sCapture = string.find(buffer, 'E(%d[%.%d]*)') --
  if(sStart) then -- got a value for E
    ee=tonumber(sCapture) * eRate
  end

  setPos(xx,yy,zz,ee)
end

function CONFIG_(buffer)

  -- save all existing config values locally
  local l12=x2
  local l23=math.sqrt(y3*y3 + (x2-x3)*(x2-x3))
  local l13=math.sqrt(x3*x3 + y3*y3)
  local h=zmax
  local l1,l2,l3
  l1, l2, l3 = IK(posx, posy, posz)

  -- overwrite local config values when found
  local sStart, sEnd, sCapture
  -- current lengths of strings in units converted to micrometres 
  sStart, sEnd, sCapture = string.find(buffer, 'A(%d[%.%d]*)') --
  if(sStart) then -- got a value for A, motor at S1
    l1=tonumber(sCapture)*mmPerUnit*1000 -- um
  end
  sStart, sEnd, sCapture = string.find(buffer, 'B(%d[%.%d]*)') --
  if(sStart) then -- got a value for B, motor at S2
    l2=tonumber(sCapture)*mmPerUnit*1000 -- um
  end
  sStart, sEnd, sCapture = string.find(buffer, 'C(%d[%.%d]*)') --
  if(sStart) then -- got a value for C, motor at S3
    l3=tonumber(sCapture)*mmPerUnit*1000 -- um
  end

  -- lengths between supports in units converted to mm
  sStart, sEnd, sCapture = string.find(buffer, 'P(%d[%.%d]*)') --
  if(sStart) then -- got a value for P, distance from S1 to S2
    l12=tonumber(sCapture)*mmPerUnit -- mm
  end
  sStart, sEnd, sCapture = string.find(buffer, 'Q(%d[%.%d]*)') --
  if(sStart) then -- got a value for Q, distance from S2 to S3
    l23=tonumber(sCapture)*mmPerUnit -- mm
  end
  sStart, sEnd, sCapture = string.find(buffer, 'R(%d[%.%d]*)') --
  if(sStart) then -- got a value for R, distance from S1 to S3
    l13=tonumber(sCapture)*mmPerUnit -- mm
  end

  -- height
  sStart, sEnd, sCapture = string.find(buffer, 'H(%d[%.%d]*)') --
  if(sStart) then -- got a value for H, height of S1, S2 and S3
    h=tonumber(sCapture)*mmPerUnit -- mm
  end

  -- use local values to set globals
  -- x1=y1=y2=0
  x2 = l12
  x3 = (l12*l12 + l13*l13 - l23*l23)/(2*l12)
  y3 = math.sqrt(l13*l13 - x3*x3)
  zmax = h;

  -- update current position of effector
  -- start by assuming no offset for object from machine space
  objx = 0
  objy = 0
  objz = zmax
  FK(l1, l2, l3, posx, posy, posz) -- where is the effector?
  -- adjust offsets and set effector at object origin
  objx = posx
  objy = posy
  objz = zmax - posz 
  posx = 0 -- set current pos as home, including extruder
  posy = 0
  posz = 0
  pose = 0  

  where();
  printConfig();
end

function processCommand(buffer)
  -- G00 G01 = move in a straight line
  if(string.sub(buffer,1,4) == "G01 "
  ) then
  G01(buffer)
  return end
  
  -- G02 = move in an arc
  if(string.sub(buffer,1,4) == "G02 "
  ) then
  G02(buffer)
  return end

  -- G28    
  if(string.sub(buffer,1,3) == "G28") then
    GoHome()
    return end
    
 -- G90 = absolute mode
  if(string.sub(buffer,1,3) == "G90") then
    absolute_mode = true
    return end
    
  -- G91 = relative (or incremental) mode
  if(string.sub(buffer,1,3) == "G91") then
    absolute_mode = false
    return end
    
  -- G92    
  if(string.sub(buffer,1,4) == "G92 ") then
    G92(buffer)
    return end

  -- M17
  if(string.sub(buffer,1,3) == "M17") then
    -- enable motors
    MotorsOn()
    return end
    
  -- M18
  if(string.sub(buffer,1,3) == "M18") then
    -- disable motors
    msgMotor(0, 'set-motor-pins-in' )
    msgMotor(1, 'set-motor-pins-in' )
    msgMotor(2, 'set-motor-pins-in' )
    msgMotor(3, 'set-motor-pins-in' )
    return end
    
  -- UID    
  if(string.sub(buffer,1,4) == 'UID ') then --
    UID=string.match(string.sub(buffer,5), '%a%w+') -- 
    -- 2do: set AP as UID, and change all connected clients
    return end
    
  -- 2do: get data back from motors
    
  -- default
  print("Invalid command '"..buffer.."'")
end

function getline()
  if(FILE_INPUT) then
     
  else -- input from terminal
    
  end
end


function init_()

  UID = 'AVAGO'
  
  -- ready motors
  MotorsOn()
  
  -- start as fast as practical
  xyzRate = 5000
  eRate = 10
  
  -- initialize the plotter position.
  setPos(0,0,0,0)
  
end

function loop_()
  -- loop forever waiting for routine calls
  while(true) do
      processCommand(buffer)

  end
end

init_()
loop_()



