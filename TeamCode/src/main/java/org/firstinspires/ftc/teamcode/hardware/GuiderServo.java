package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Reicher Robotics on 3/4/2018.
 */

public class GuiderServo {

    public enum GuiderPosition {
        IN,
        OUT,
        UNKNOWN;
    }

    private final static double MIN_IN_LIMIT = 0.6;
    private final static double MAX_IN_LIMIT = 2.5;
    private final static double MIN_OUT_LIMIT = 1.05;
    private final static double MAX_OUT_LIMIT = 1.25;


    private int numGuiderCRServos = 1;

    public CRServo guiderCRServos[] = new CRServo[numGuiderCRServos];

    private int numGuiderAnalogSensors = 1;
    public GuiderServo(HardwareMap hwMap, String crServoNames[], CRServo.Direction crServoDirections[]){
        for(int i = 0; i < numGuiderCRServos; i++){
            guiderCRServos[i] = hwMap.crservo.get(crServoNames[i]);
            guiderCRServos[i].setDirection(crServoDirections[i]);
        }
    }

    public void stop(){
        guiderCRServos[0].setPower(0.0);
    }

    public void in(){
        guiderCRServos[0].setPower(-1.0);
    }

    public void out(){
        guiderCRServos[0].setPower(1.0);
    }
}
