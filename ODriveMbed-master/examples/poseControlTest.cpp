#include "ODriveMbed.h"
#include "mbed.h"

#define ODRIVE2TX D1
#define ODRIVE2RX D0

Serial pc(USBTX, USBRX);
Serial odrive_serial(ODRIVE2TX, ODRIVE2RX);
ODriveMbed odrive(odrive_serial);
InterruptIn userButton(USER_BUTTON);

bool flag = false;

void raiseFlag(){
	flag = true;
}

void waitForButtonPress(){
	while(1){
		wait_ms(10);
		if(flag ==1)
			break;
	}
	flag = false;

}

int main(){
	pc.baud(115200);
    odrive_serial.baud(115200);
	userButton.rise(&raiseFlag);

	pc.puts("press the user button once the odrive is calibrated to continue\n");
	waitForButtonPress();

	pc.puts("great, thanks\n");

	int axis = 0;
	float positionSetpoint = 0;
		pc.puts("setting position control mode\n");
	if(!odrive.setControlMode(axis, ODriveMbed::CTRL_MODE_POSITION_CONTROL, true))
		pc.printf("something went wrong and the control mode was not successfully set, current mode is a %d\n", odrive.readControlMode(axis));
	else{
		pc.printf("control mode set to: %d\n", odrive.readControlMode(axis));
		
		odrive.setPosition(axis, positionSetpoint);
		pc.puts("position set to 0, waiting for button press to start \n");

		for(positionSetpoint = 0; positionSetpoint < 25000; positionSetpoint+=2000.0f){
			waitForButtonPress();
			odrive.setPosition(axis, positionSetpoint);
			wait_ms(100);
			pc.printf("Setpoint: %f\tPosition Estimate: %f\n", positionSetpoint, odrive.getPositionEstimate(axis));
		}
	}

}

