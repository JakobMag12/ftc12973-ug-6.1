package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Reicher Robotics on 2/22/2018.
 */

public class Intake {
    // MOTORS
    private int numMotors = 1;
    private int INTAKE_MOTOR = 0;

    public DcMotor[] motors = new DcMotor[numMotors];

    public Intake(HardwareMap hwMap, String[] motorNames, DcMotorSimple.Direction[] motorDirections){
        for(int i = 0; i < numMotors; i++){
            motors[i] = hwMap.dcMotor.get(motorNames[i]);
            motors[i].setDirection(motorDirections[i]);
        }
    }

    public boolean isMotorBusy(){
        boolean isBusy = false;
        for (DcMotor _motor : motors){
            if(_motor.isBusy()){
                isBusy = true;
            }
        }
        return isBusy;
    }

    public void setMotorModes(DcMotor.RunMode mode){
        for (DcMotor _motor : motors){
            _motor.setMode(mode);
        }
    }

    public void setMotorBreak(DcMotor.ZeroPowerBehavior mode){
        for (DcMotor _motor : motors){
            _motor.setZeroPowerBehavior(mode);
        }
    }

    public void setIntakePowers(double leftPower){
        motors[0].setPower(leftPower);
    }

    public void in(double forwardPower){
        setIntakePowers(forwardPower);
    }

    public void out(double backwardPower){
        in(-backwardPower);
    }

    public void stop(){
        in(0.0);
    }
}
