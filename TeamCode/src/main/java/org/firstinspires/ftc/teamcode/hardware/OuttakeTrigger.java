package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Reicher Robotics on 3/4/2018.
 */

public class OuttakeTrigger {

    public enum TRIGGER_STATE {
        FIRST_SHOT,
        FIRST_IN,
        SECOND_SHOT,
        SECOND_IN,
        THIRD_SHOT,
        THIRD_IN,
        UNKNOWN
    }

    // SERVO POSITIONS
    public final double SHOOT = 0.570;
    public final double IN = 0.375;
    public double SHOOT_TIME = 100;
    public double IN_TIME = 200;

    public TRIGGER_STATE rapidFireState;
    private double rapidFireTime;

    // SERVOS
    private int numServos = 1;
    private int OUTTAKE_TRIGGER = 0;

    public Servo[] servos = new Servo[numServos];

    public OuttakeTrigger(HardwareMap hwMap, String[] servoNames){
        for(int i = 0; i < numServos; i++){
            servos[i] = hwMap.servo.get(servoNames[i]);
        }
    }

    public double getPosition() {
        return servos[OUTTAKE_TRIGGER].getPosition();
    }

    public void position(double position) {
        servos[OUTTAKE_TRIGGER].setPosition(position);
    }

    public void shoot() {
        servos[OUTTAKE_TRIGGER].setPosition(SHOOT);
    }

    public void in(){
        servos[OUTTAKE_TRIGGER].setPosition(IN);
    }

    public void setRapidFireTime(double time) {
        rapidFireTime = time;
    }

    public void setShootTime(double time) {
        SHOOT_TIME = time;
    }

    public void setInTime(double time) {
        IN_TIME = time;
    }

    public void rapidFire(double time) {
        switch(rapidFireState) {
            case FIRST_SHOT:
                if(time - rapidFireTime < SHOOT_TIME) {
                    shoot();
                } else {
                    rapidFireTime = time;
                    rapidFireState = TRIGGER_STATE.FIRST_IN;
                }
                break;
            case FIRST_IN:
                if(time - rapidFireTime < IN_TIME) {
                    in();
                } else {
                    rapidFireTime = time;
                    rapidFireState = TRIGGER_STATE.SECOND_SHOT;
                }
                break;
            case SECOND_SHOT:
                if(time - rapidFireTime < SHOOT_TIME) {
                    shoot();
                } else {
                    rapidFireTime = time;
                    rapidFireState = TRIGGER_STATE.SECOND_IN;
                }
                break;
            case SECOND_IN:
                if(time - rapidFireTime < IN_TIME) {
                    in();
                } else {
                    rapidFireTime = time;
                    rapidFireState = TRIGGER_STATE.THIRD_SHOT;
                }
                break;
            case THIRD_SHOT:
                if(time - rapidFireTime < SHOOT_TIME) {
                    shoot();
                } else {
                    rapidFireTime = time;
                    rapidFireState = TRIGGER_STATE.THIRD_IN;
                }
                break;
            case THIRD_IN:
                if(time - rapidFireTime < IN_TIME) {
                    in();
                } else {
                    rapidFireTime = time;
                    rapidFireState = TRIGGER_STATE.UNKNOWN;
                }
                break;
            default:
                break;
        }
    }
}
