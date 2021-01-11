package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Reicher Robotics on 3/4/2018.
 */

public class OuttakeRamp {
    // SERVO POSITIONS
    private final double UP = 0.0;
    private final double MID = 0.5;
    private final double DOWN = 1.0;

    // SERVOS
    private int numServos = 1;
    private int OUTTAKE_RAMP_SERVO = 0;

    public Servo[] servos = new Servo[numServos];

    public OuttakeRamp(HardwareMap hwMap, String[] servoNames){
        for(int i = 0; i < numServos; i++){
            servos[i] = hwMap.servo.get(servoNames[i]);
        }
    }

    public void up() {
        servos[OUTTAKE_RAMP_SERVO].setPosition(UP);
    }

    public void mid(){
        servos[OUTTAKE_RAMP_SERVO].setPosition(MID);
    }

    public void down(){
        servos[OUTTAKE_RAMP_SERVO].setPosition(DOWN);
    }
}
