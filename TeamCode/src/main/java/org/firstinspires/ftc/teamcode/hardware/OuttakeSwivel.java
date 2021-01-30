package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Reicher Robotics on 3/4/2018.
 */

public class OuttakeSwivel {
    // SERVO POSITIONS
    private final double UP = 0.0;
    private final double MID = 0.5;
    private final double DOWN = 1.0;

    // SERVOS
    private int numServos = 1;
    private int OUTTAKE_SWIVEL = 0;

    public Servo[] servos = new Servo[numServos];

    public OuttakeSwivel(HardwareMap hwMap, String[] servoNames){
        for(int i = 0; i < numServos; i++){
            servos[i] = hwMap.servo.get(servoNames[i]);
        }
    }

    public void up() {
        servos[OUTTAKE_SWIVEL].setPosition(UP);
    }

    public void mid(){
        servos[OUTTAKE_SWIVEL].setPosition(MID);
    }

    public void down(){
        servos[OUTTAKE_SWIVEL].setPosition(DOWN);
    }
}
