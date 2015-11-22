@echo off


: for Indoor, test on Simulator
:start D:\MobileRobots\Aria\examples\slave_car\bin\slave_car.exe -rh 127.0.0.1 -rrtp 8101 -server 127.0.0.1 -sport 9999 -sensor 5 5 0 0 0 0 1 1 COM6

: for Indoor
:start D:\MobileRobots\Aria\examples\slave_car\bin\slave_car.exe -rp COM4 -rb 9600 -server 172.20.85.109 -sport 9999 -sensor 5 5 0 0 0 0 1 1 COM6


: for Outdoor, test on Simulator
start D:\MobileRobots\Aria\examples\slave_car\bin\slave_car.exe -rh 127.0.0.1 -rrtp 8101 -server 127.0.0.1 -sport 9999 -hasClient -slaveServerPort 9998 -sleepTime 0 -slaveGpsTest 0 -delayGo 0


: for Outdoor, act as Master and Slave
:start D:\MobileRobots\Aria\examples\slave_car\bin\slave_car.exe -rp COM4 -rb 9600 -server 172.20.85.109 -sport 9999 -hasClient -slaveServerPort 9998 -gps COM6 -sleepTime 5000 -slaveGpsTest 0 -delayGo 7000

