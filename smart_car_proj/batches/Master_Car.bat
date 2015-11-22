@echo off


: for Indoor
:start D:\MobileRobots\Aria\examples\master_car\bin\master_car.exe -rp COM5 -rb 9600 -sport 9999 -rn 1 -sensor 5 5 0 0 0 0 1 1 COM6


: for Indoor, test on Simulator
:start D:\MobileRobots\Aria\examples\master_car\bin\master_car.exe -rh 127.0.0.1 -rrtp 8101 -sport 9999 -rn 1 -sensor 5 5 0 2 0 0 1 1 COM6


: for Outdoor, test on Simulator
:start D:\MobileRobots\Aria\examples\master_car\bin\master_car.exe -rh 127.0.0.1 -rrtp 8101 -sport 9999 -rn 2 -sleepTime 0 -masterGpsTest 0


: for Outdoor
start D:\MobileRobots\Aria\examples\master_car\bin\master_car.exe -rp COM5 -rb 9600 -sport 9999 -rn 2 -gps COM6 -sleepTime 5000 -masterGpsTest 0

