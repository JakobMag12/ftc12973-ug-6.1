package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Reicher Robotics on 2/22/2018.
 */

public class DriveMecanum {

    public double P = 0.5;
    public double I = 0.0;
    public double D = 0.9;

    public double TICKS_PER_REV = 537.6;
    public double WHEEL_DIAMETER = 96.0 / 25.4;
    public double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_DIAMETER / 3.14159;

    private int numDriveMecMotors = 4;
    // 0 = Left Front
    // 1 = Left Rear
    // 2 = Right Front
    // 3 = Right Rear
    public DcMotor[] driveMecMotors = new DcMotor[numDriveMecMotors];

    public DriveMecanum(HardwareMap driveMap, String motorNames[], DcMotorSimple.Direction motorDirections[]) {
        for (int i = 0; i < numDriveMecMotors; i++) {
            driveMecMotors[i] = driveMap.dcMotor.get(motorNames[i]);
            driveMecMotors[i].setDirection(motorDirections[i]);
        }
    }

    public boolean isMotorBusy() {
        boolean isBusy = false;
        for (DcMotor _motor : driveMecMotors) {
            if (_motor.isBusy()) {
                isBusy = true;
            }
        }
        return isBusy;
    }

    public boolean is4MotorsBusy() {
        boolean isBusy = false;
        if (driveMecMotors[0].isBusy() && driveMecMotors[1].isBusy() && driveMecMotors[2].isBusy() && driveMecMotors[3].isBusy()) {
            isBusy = true;
        }
        return isBusy;
    }

    public void setMotorModes(DcMotor.RunMode mode) {
        for (DcMotor _motor : driveMecMotors) {
            _motor.setMode(mode);
        }
    }

    public void setMotorBreak(DcMotor.ZeroPowerBehavior mode) {
        for (DcMotor _motor : driveMecMotors) {
            _motor.setZeroPowerBehavior(mode);
        }
    }

    public void setDrivePowers(double leftFrontPower, double leftRearPower, double rightFrontPower, double rightRearPower) {
        driveMecMotors[0].setPower(leftFrontPower);
        driveMecMotors[1].setPower(leftRearPower);
        driveMecMotors[2].setPower(rightFrontPower);
        driveMecMotors[3].setPower(rightRearPower);
    }

    public int getMotorTicks(int driveMotor) {
        return driveMecMotors[driveMotor].getCurrentPosition();
    }

    public void driveStop() {
        setDrivePowers(0.0, 0.0, 0.0, 0.0);
    }

    public void setDriveXYTargets(int yTarget, int xTarget){
        driveMecMotors[0].setTargetPosition(yTarget - xTarget);
        driveMecMotors[1].setTargetPosition(yTarget + xTarget);
        driveMecMotors[2].setTargetPosition(yTarget - xTarget);
        driveMecMotors[3].setTargetPosition(yTarget + xTarget);
    }

    public void mecanumDrive(double maxDrivePower, double drive, double strafe, double turn) {
        double leftFrontPower = drive + strafe + turn;
        double leftRearPower = drive - strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double rightRearPower = drive + strafe - turn;
/*
        double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightRearPower));
        maxPower /= maxDrivePower;

        leftFrontPower = leftFrontPower * Math.abs(leftFrontPower) / maxPower;
        leftRearPower = leftRearPower * Math.abs(leftRearPower) / maxPower;
        rightFrontPower = rightFrontPower * Math.abs(rightFrontPower) / maxPower;
        rightRearPower = rightRearPower * Math.abs(rightRearPower) / maxPower;
 */

        setDrivePowers(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);
    }

    public int convertYInchToTicks(double inches) { return (int) (inches * TICKS_PER_INCH) ; }
    public int convertXInchToTicks(double inches) { return (int) (inches * TICKS_PER_INCH / Math.sqrt(2.0)) ; }

}
