package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Reicher Robotics on 3/4/2018.
 */

public class GuiderServo {

    private int numGuiderCRServos = 1;

    public CRServo guiderCRServos[] = new CRServo[numGuiderCRServos];

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
