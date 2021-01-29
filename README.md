stm32 based board design and firmware for robot basketball

Tu run the codes you need processing environment, download from processing.org
 Then add libraries from library manager, only one needed to add is processing.video
Connect camera and stm board. 
Press run. 
One code is for serial communication and keyboard controll.
One is for camera and serial comunication
Last one is for webserver setub and webcontrol of robot.

For mainboard firmware there is main.c file. It is meant for stm cube ide
Configure the stm board as described on wiki and paste the main.c code in there.
