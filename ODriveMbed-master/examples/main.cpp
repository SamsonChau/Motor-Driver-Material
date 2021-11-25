#include "ODriveMbed.h"
#include <mbed.h>
// TODO: set these pins to be the correct ones for UART with ODrive
#define ODRIVE2TX PG_14
#define ODRIVE2RX PG_9

Serial pc(USBTX, USBRX);
Serial odrive_serial(ODRIVE2TX, ODRIVE2RX);
ODriveMbed odrive(odrive_serial);

int main()
{
    // wait(5);
    pc.printf("hello");

    odrive_serial.baud(115200);

    pc.baud(115200);
    pc.puts("ODriveArduino\n");
    pc.puts("Setting parameters...\n");

    /* Already verified that these commands print out to serial if
       odrive takes in pc serial object
    
    odrive.SetPosition(0,2000.0f);
    odrive.SetVelocity(1,45.0f);

    odrive.run_state(0,odrive.AXIS_STATE_FULL_CALIBRATION_SEQUENCE,1);
    odrive.run_state(0,odrive.AXIS_STATE_ENCODER_INDEX_SEARCH,0);
    
    */

    pc.puts("ODriveArduino\n");
    pc.puts("Setting parameters...\n");

    // In this example we set the same parameters to both motors.
    // You can of course set them different if you want.
    // See the documentation or play around in odrivetool to see the available parameters
    for (int axis = 0; axis < 2; ++axis)
    {
        // TODO: rewrite the library to include all these states in the enumeration instead of manually sending them over serial
        odrive_serial.printf("w axis%d.controller.config.vel_limit %f\n",axis, 22000.0f);
        odrive_serial.printf("w axis%d.motor.config.current_lim %f\n",axis, 11.0f);
        // odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
        // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
    }

    pc.puts("Ready!\n");
    pc.puts("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)\n");
    pc.puts("Send the character 's' to exectue test move\n");
    pc.puts("Send the character 'b' to read bus voltage\n");
    pc.puts("Send the character 'p' to read motor positions in a 10s loop\n");
    char c;
    do 
    {
        if (pc.readable())
        {
            c = pc.getc();
            // Run calibration sequence
            if (c == '0' || c == '1')
            {
                int motornum = c - '0';
                int requested_state;

                requested_state = ODriveMbed::AXIS_STATE_MOTOR_CALIBRATION;
                // Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
                pc.printf("Axis%c: Requesting State %d\n", c, requested_state);
                odrive.run_state(motornum, requested_state, true);

                requested_state = ODriveMbed::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
                // Serial << "Axis" << c << ": Requesting State " << requested_state << '\n';
                pc.printf("Axis%c: Requesting State %d\n", c, requested_state);
                odrive.run_state(motornum, requested_state, true);

                requested_state = ODriveMbed::AXIS_STATE_CLOSED_LOOP_CONTROL;
                // Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
                pc.printf("Axis%c: Requesting State %d\n", c, requested_state);
                odrive.run_state(motornum, requested_state, false); // don't wait
            }

            // Sinusoidal test move
            if (c == 's')
            {
                int requested_state = ODriveMbed::AXIS_STATE_CLOSED_LOOP_CONTROL;
                // Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
                pc.printf("Axis%c: Requesting State %d\n", c, requested_state);
                odrive.run_state(0, requested_state, false); // don't wait
                pc.puts("Executing test move\n");
                /*
                for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f)
                {
                    float pos_m0 = 20000.0f * cos(ph);
                    float pos_m1 = 20000.0f * sin(ph);
                    odrive.SetPosition(0, pos_m0);
                    odrive.SetPosition(1, pos_m1);
                    wait_ms(50);
                }
                */
                odrive.SetPosition(0, 400000);
                pc.puts("Test Move Completed\n");
            }

            // Read bus voltage
            if (c == 'b')
            {
                odrive_serial.puts("r vbus_voltage\n");
                // Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
                pc.printf("Vbus voltage: %f\n", odrive.readFloat());
            }

            // print motor positions in a 10s loop
            if (c == 'p')
            {
                static const unsigned int duration = 10000;
                Timer t;
                t.start();
                unsigned int start = t.read_ms();

                while (t.read_ms() - start < duration)
                {
                    for (int motor = 0; motor < 2; ++motor)
                    {
                        // odrive_serial << "r axis" << motor << ".encoder.pos_estimate\n";
                        odrive_serial.printf("r axis%d.encoder.pos_estimate\n", motor);
                        // Serial << odrive.readFloat() << '\t';
                        pc.printf("%f\t", odrive.readFloat());
                    }
                    // Serial << '\n';
                    pc.putc('\n');
                }
            }
        }
    } while (c != 'q');
}