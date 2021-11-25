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

	pc.puts("press the user button once the odrive id calibrated to continue\n");
	waitForButtonPress();

	pc.puts("great, thanks\n");

	int axis = 0;
	float currentSetpoint = 0.0f;

	pc.puts("setting current control mode\n");
	if(!odrive.setControlMode(axis, ODriveMbed::CTRL_MODE_CURRENT_CONTROL, true))
		pc.printf("something went wrong and the control mode was not successfully set, current mode is a %d\n", odrive.readControlMode(axis));
	else{
		pc.printf("control mode set to: %d\n", odrive.readControlMode(axis));
		odrive.setPosition(axis, currentSetpoint);
		pc.puts("current set to 0, waiting for button press to start \n");
		for(currentSetpoint = 0.0f; currentSetpoint < 6.0f; currentSetpoint+=0.3f){
			waitForButtonPress();
			odrive.setCurrent(axis, currentSetpoint);
			wait_ms(100);
			pc.printf("Setpoint: %f\tCurrent Estimate: %f\n", currentSetpoint, odrive.getCurrentEstimate(axis));
		}
		odrive.setCurrent(axis, 0.0f);
	}

	pc.puts("done\n");

}