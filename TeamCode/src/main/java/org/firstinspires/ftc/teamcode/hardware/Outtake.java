package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Reicher Robotics on 2/22/2018.
 */

public class Outtake {
    // MOTORS
    private int numMotors = 2;
    private int OUTTAKE_ONE = 0;
    private int OUTTAKE_TWO = 1;

    public DcMotor[] motors = new DcMotor[numMotors];

    public Outtake(HardwareMap hwMap, String[] motorNames, DcMotorSimple.Direction[] motorDirections){
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

    public void setOuttakePowers(double leftPower){
        motors[0].setPower(leftPower);
        motors[1].setPower(leftPower);
    }

    public void in(double forwardPower){
        setOuttakePowers(forwardPower);
    }

    public void out(double backwardPower){
        in(-backwardPower);
    }

    public void stop(){
        in(0.0);
    }
}
