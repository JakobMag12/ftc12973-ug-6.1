package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;

/**
 * Created by Reicher Robotics on 3/25/2018.
 */

public class AutoComWDDrive extends AutoCommandMain {
    private double speed;
    private double maxDistance;
    private double yDistance;
    private int yTargetTicks = 0;
    private double ySpeed = 0.0;
    private double xDistance;
    private int xTargetTicks = 0;
    private double xSpeed = 0.0;
    private double distance;
    private double angle = 0.0;

    private double error;
    private double correction;
    private double targetHeading;
    private double prevTime;
    private double integral;
    private double derivative;
    private double prevError;
    private double currentTime;
    private double timeLapsed;

    ElapsedTime time = new ElapsedTime();

    public AutoComWDDrive(RobotOpModes opMode, double distance, double speed, double angle){
        super(opMode);

        this.speed = speed;
        this.distance = distance;
        yTargetTicks = bot.driveMecanum.convertYInchToTicks(yDistance);
        xTargetTicks = bot.driveMecanum.convertXInchToTicks(xDistance);
        this.angle = angle;
        yTargetTicks = bot.driveMecanum.convertYInchToTicks(yDistance);
        xTargetTicks = bot.driveMecanum.convertXInchToTicks(xDistance);
        maxDistance = Math.max(Math.abs(yDistance), Math.abs(xDistance));
        ySpeed = yDistance / maxDistance;
        xSpeed = xDistance / maxDistance;
    }

    @Override
    public void Start(){
        bot.driveMecanum.setDriveXYTargets(yTargetTicks, xTargetTicks);
        bot.driveMecanum.setMotorBreak(DcMotor.ZeroPowerBehavior.BRAKE);
        bot.driveMecanum.setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.driveMecanum.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        bot.wobbleLift.setDownTarget(bot.wobbleLift.convertAngleToTicks(angle));
        bot.wobbleLift.setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.wobbleLift.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        targetHeading = bot.gyroIMU.getHeading();
        prevError = 0.0;
        prevTime = 0.0;
        integral = 0.0;
        time.reset();
    }

    @Override
    public void Loop(){
        currentTime = time.milliseconds();
        timeLapsed = currentTime - prevTime;
        error = bot.gyroIMU.getError(targetHeading);
        integral += error * timeLapsed;
        derivative = (error - prevError) / timeLapsed;
        correction = bot.driveMecanum.P * error + bot.driveMecanum.I * integral + bot.driveMecanum.D * derivative;
        prevTime = currentTime;
        prevError = error;
        bot.driveMecanum.mecanumDrive(speed, ySpeed, xSpeed, -correction);
        bot.wobbleLift.down();
    }

    @Override
    public void Stop(){
        bot.driveMecanum.driveStop();
        bot.wobbleLift.stop();
        bot.driveMecanum.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.wobbleLift.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public boolean IsTaskRunning(){
        return bot.driveMecanum.is4MotorsBusy();
    }
}