: 测试跟车（叶子节点） 室内测试无GPS
start slave_car_indoor_test.exe -rp COM4 -rb 9600 -server 10.0.126.114 -sport 9999 -sleepTime 5000 -slaveGpsTest 0 -delayGo 7000
:start slave_car.exe -rh 127.0.0.1 -rrtp 8101 -server 127.0.0.1 -sport 9999 -sleepTime 5000 -slaveGpsTest 0 -delayGo 7000

: 测试跟车 叶子节点 室外测试带GPS
:start slave_car_outdoor_test.exe -rp COM4 -rb 9600 -server 10.0.126.114 -sport 9999 -gps COM6 -sleepTime 5000 -slaveGpsTest 0 -delayGo 7000

: 测试跟车 中继节点 室外测试带GPS
: start slave_car_outdoor_test.exe -rp COM4 -rb 9600 -server 10.0.126.144