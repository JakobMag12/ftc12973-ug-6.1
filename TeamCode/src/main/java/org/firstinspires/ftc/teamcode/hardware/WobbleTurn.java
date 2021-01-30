package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Reicher Robotics on 3/4/2018.
 */

public class WobbleTurn {
    // SERVO POSITIONS
    private final double IN = 0.0;
    private final double MID = 0.5;
    private final double OUT = 1.0;

    // SERVOS
    private int numServos = 1;
    private int WOBBLE_TURN_SERVO = 0;

    public Servo[] servos = new Servo[numServos];

    public WobbleTurn(HardwareMap hwMap, String[] servoNames){
        for(int i = 0; i < numServos; i++){
            servos[i] = hwMap.servo.get(servoNames[i]);
        }
    }

    public void in() {
        servos[WOBBLE_TURN_SERVO].setPosition(IN);
    }

    public void mid(){
        servos[WOBBLE_TURN_SERVO].setPosition(MID);
    }

    public void out(){
        servos[WOBBLE_TURN_SERVO].setPosition(OUT);
    }
}
