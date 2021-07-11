# Arduino Self-Balancing Robot

A self balancing robot using NodeMCU

This repo includes the library needed to build the robot.

//================================================================================
// + Connections
//     NodeMCU              GY-521          L298N
//      3V3        <----->   VCC
//      GND        <----->   GND   <----->   GND
//      D1,GPIO5   <----->   SDL
//      D2,GPIO4   <----->   SDA
//      D3,GPIO0   <--------------------->   ENA
//      D4,GPIO2   <--------------------->   IN1
//      D5,GPIO14  <--------------------->   IN2
//      D6,GPIO12  <--------------------->   IN3
//      D7,GPIO13  <--------------------->   IN4
//      D8,GPIO15  <--------------------->   ENB
//      SD3,GPIO10 <----->   INT
//
//================================================================================