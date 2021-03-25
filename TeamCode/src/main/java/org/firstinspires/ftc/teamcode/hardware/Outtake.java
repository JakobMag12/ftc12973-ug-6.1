package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    public final int VELOCITY = 1100;

    public DcMotorEx[] motors = new DcMotorEx[numMotors];

    public Outtake(HardwareMap hwMap, String[] motorNames, DcMotorSimple.Direction[] motorDirections){
        for(int i = 0; i < numMotors; i++){
            motors[i] = hwMap.get(DcMotorEx.class, motorNames[i]);
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

    public void setOuttakePowers(double power){
        motors[0].setPower(power);
        motors[1].setPower(power);
    }

    public void shoot(double velocity) {
        motors[0].setVelocity(velocity);
        motors[1].setVelocity(velocity);
    }

    public double getVelocity() {
        return (motors[0].getVelocity() + motors[1].getVelocity()) / 2;
    }

    public void in(double power){
        setOuttakePowers(power);
    }

    public void out(double power){
        in(-power);
    }

    public void stop(){
        in(0.0);
    }
}
