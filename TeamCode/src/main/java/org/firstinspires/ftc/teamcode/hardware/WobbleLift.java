package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
 * Created by Reicher Robotics on 3/4/2018.
 */

public class WobbleLift {
    private double P = 2.0;

    public double ZERO_VOLTAGE = 2.95;
    public double GRAB_VOLTAGE = 0.8;
    private double PID_VOLTAGE_RANGE = 0.3;

    private int numLiftMotors = 1;

    // 0 = Extension
    public DcMotor[] liftMotors = new DcMotor[numLiftMotors];

    public AnalogInput limit = null;

    public WobbleLift(HardwareMap hwMap, String[] motorNames, DcMotorSimple.Direction[] motorDirections, String limitName) {
        for (int i = 0; i < numLiftMotors; i++) {
            liftMotors[i] = hwMap.dcMotor.get(motorNames[i]);
            liftMotors[i].setDirection(motorDirections[i]);
            liftMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        limit = hwMap.analogInput.get(limitName);
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

    public void upP(double power) {
        double voltageError = (ZERO_VOLTAGE - getLimitVoltage()) / PID_VOLTAGE_RANGE;
        up(P * voltageError * power);
    }

    public void upPLimit(double power, double limit) {
        double voltageError = (limit - getLimitVoltage()) / PID_VOLTAGE_RANGE;
        up(P * voltageError * power);
    }

    public void down() {
        liftMotors[0].setPower(-1.0);
    }

    public void down(double power) {
        liftMotors[0].setPower(-power);
    }

    public void downP(double power) {
        double voltageError = (getLimitVoltage() - GRAB_VOLTAGE) / PID_VOLTAGE_RANGE;
        down(P * voltageError * power);
    }

    public void downPLimit(double power, double limit) {
        double voltageError = (limit - getLimitVoltage()) / PID_VOLTAGE_RANGE;
        up(P * voltageError * power);
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

    // TODO: CHECK IF MOTOR ENCODER TICKS ARE GOING UP WHEN THE MOTOR IS GOING FORWARD
    // IF TICKS GOING UP, RETURN POSITIVE
    // IF TICKS GOING DOWN, ADD NEGATIVE SIGN TO FIX WRONG TICK SIGN
    public int getLiftPosition() {
        return -liftMotors[0].getCurrentPosition();
    }

    public void liftWithLimits(double power) {
        if (power > 0.0) {
            if(!atZeroPosition()) {
                upP(power);
            }
        } else if (!atGrabPosition()) {
           downP(-power);
        } else {
            stop();
        }
    }

    public void resetLimits() {
        liftMotors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getLimitVoltage() {
        return limit.getVoltage();
    }

    public boolean atZeroPosition() {
        return getLimitVoltage() > ZERO_VOLTAGE;
    }

    public boolean atGrabPosition() {
        return getLimitVoltage() < GRAB_VOLTAGE;
    }

    public int convertAngleToTicks(double angle) {
        return (int) (angle / 360 * 383.6);
    }
}

