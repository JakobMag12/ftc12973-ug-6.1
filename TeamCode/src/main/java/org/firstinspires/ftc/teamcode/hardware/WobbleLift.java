package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;


/**
 * Created by Reicher Robotics on 3/4/2018.
 */

public class WobbleLift {
    private double TICKS_PER_REV = 103.6;
    private double LIFT_PER_REV = 1.375 * Math.PI;
    private double TICKS_PER_LIFT = TICKS_PER_REV / LIFT_PER_REV;

    private double MAX_LIFT = 900.0;
    private double MAX_LIFT_TICKS = convertLiftToTicks(MAX_LIFT);
    private double MIN_LIFT = 0.5;
    private double MIN_LIFT_TICKS = convertLiftToTicks(MIN_LIFT);

    private int numLiftMotors = 1;

    // 0 = Extension
    public DcMotor[] liftMotors = new DcMotor[numLiftMotors];

    public WobbleLift(HardwareMap hwMap, String[] motorNames, DcMotorSimple.Direction[] motorDirections) {
        for (int i = 0; i < numLiftMotors; i++) {
            liftMotors[i] = hwMap.dcMotor.get(motorNames[i]);
            liftMotors[i].setDirection(motorDirections[i]);
            liftMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public boolean isMotorBusy() {
        boolean isBusy = false;
        for (DcMotor _motor : liftMotors) {
            if (_motor.isBusy()) {
                isBusy = true;
            }
        }
        return isBusy;
    }

    public void setMotorModes(DcMotor.RunMode mode) {
        for (DcMotor _motor : liftMotors) {
            _motor.setMode(mode);
        }
    }

    public void setMotorBreak(DcMotor.ZeroPowerBehavior behavior) {
        for (DcMotor _motor : liftMotors) {
            _motor.setZeroPowerBehavior(behavior);
        }
    }

    public void stop() {
        liftMotors[0].setPower(0.0);
    }

    public void up() {
        liftMotors[0].setPower(1.0);
    }

    public void up(double power) {
        liftMotors[0].setPower(power);
    }

    public void down() {
        liftMotors[0].setPower(-1.0);
    }

    public void down(double power) {
        liftMotors[0].setPower(-power);
    }

    public void setLiftTargets(int target) {
        if (target != 0) {
            liftMotors[0].setTargetPosition(target);
        }
    }

    public void setUpTarget(int upTarget) {
        setLiftTargets(upTarget);
    }

    public void setDownTarget(int downTarget) {
        setLiftTargets(-downTarget);
    }

    public int getLiftPosition() {
        return liftMotors[0].getCurrentPosition();
    }

    public void liftWithLimits(double power) {
        if (power > 0.0) {
            up(power);
        } else if (power < 0.0) {
            down(-power);
        } else {
            stop();
        }
    }

    public void overrideLimits() {
        MIN_LIFT_TICKS = -MAX_LIFT_TICKS;
    }

    public void resetLimits() {
        liftMotors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int convertLiftToTicks(double lift) {
        return (int) (lift * TICKS_PER_LIFT);
    }

}

