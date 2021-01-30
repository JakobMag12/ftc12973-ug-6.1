package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Reicher Robotics on 3/4/2018.
 */

public class WobbleAutoClaw {
    // SERVO POSITIONS
    private final double OPEN = 0.0;
    private final double MID = 0.5;
    private final double CLOSE = 1.0;

    // SERVOS
    private int numServos = 1;
    private int WOBBLE_AUTO_CLAW_SERVO = 0;

    public Servo[] servos = new Servo[numServos];

    public WobbleAutoClaw(HardwareMap hwMap, String[] servoNames){
        for(int i = 0; i < numServos; i++){
            servos[i] = hwMap.servo.get(servoNames[i]);
        }
    }

    public void open() {
        servos[WOBBLE_AUTO_CLAW_SERVO].setPosition(OPEN);
    }

    public void mid(){
        servos[WOBBLE_AUTO_CLAW_SERVO].setPosition(MID);
    }

    public void close() {
        servos[WOBBLE_AUTO_CLAW_SERVO].setPosition(CLOSE);
    }
}
