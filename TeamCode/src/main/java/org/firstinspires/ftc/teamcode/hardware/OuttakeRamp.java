package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Reicher Robotics on 3/4/2018.
 */

public class OuttakeRamp {
    // SERVO POSITIONS
    public final double TOP = 0.430;
    public final double POWER = 0.470;
    public final double MID = 0.520;
    public final double BOTTOM = 0.600;

    // SERVOS
    private int numServos = 1;
    private int OUTTAKE_RAMP_SERVO = 0;

    public Servo[] servos = new Servo[numServos];

    public OuttakeRamp(HardwareMap hwMap, String[] servoNames){
        for(int i = 0; i < numServos; i++){
            servos[i] = hwMap.servo.get(servoNames[i]);
        }
    }

    public void position(double position) {
        servos[OUTTAKE_RAMP_SERVO].setPosition(position);
    }
    public void top() {
        servos[OUTTAKE_RAMP_SERVO].setPosition(TOP);
    }
    public void power() {
        servos[OUTTAKE_RAMP_SERVO].setPosition(POWER);
    }
    public void mid(){
        servos[OUTTAKE_RAMP_SERVO].setPosition(MID);
    }

    public void bottom(){
        servos[OUTTAKE_RAMP_SERVO].setPosition(BOTTOM);
    }
}
