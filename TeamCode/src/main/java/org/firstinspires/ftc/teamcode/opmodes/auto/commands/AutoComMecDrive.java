package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.DriveMecanum;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Reicher Robotics on 3/25/2018.
 */

public class AutoComMecDrive extends AutoCommandMain {
    private DriveMecanum.DriveDirection direction;
    private double speed = 0.0;
    private double speedIncrement = 0.0;
    private double minSpeed = 0.05;
    private double maxSpeed;
    private double distance;
    private int targetTicks = 0;
    private int rampTargetTicks = 0;
    private int maxRampTargetTicks = (int) bot.driveMecanum.TICKS_PER_REV;

    private int currentTick = 0;

    public AutoComMecDrive(RobotOpModes opMode, DriveMecanum.DriveDirection direction, double speed, double distance){
        super(opMode);

        this.direction = direction;
        maxSpeed = speed;
        this.distance = distance;
    }

    @Override
    public void Start(){
        targetTicks = bot.driveMecanum.convertYInchToTicks(distance);
        rampTargetTicks = Range.clip(targetTicks / 2, 0, maxRampTargetTicks);
        speedIncrement = maxSpeed / maxRampTargetTicks;
        bot.driveMecanum.setDriveDirectionTarget(direction, targetTicks);
        bot.driveMecanum.setMotorBreak(DcMotor.ZeroPowerBehavior.BRAKE);
        bot.driveMecanum.setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.driveMecanum.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void Loop() {
        currentTick = Math.abs(bot.driveMecanum.driveMecMotors[0].getCurrentPosition());
        if (currentTick <= rampTargetTicks) {
            speed = currentTick * speedIncrement;
        } else if (currentTick >= (targetTicks - rampTargetTicks)) {
            speed = (targetTicks - currentTick) * speedIncrement;
        } else {
            speed = maxSpeed;
        }
        speed = Range.clip(speed, minSpeed, maxSpeed);
        bot.driveMecanum.directionDrive(direction, maxSpeed);
    }

    @Override
    public void Stop(){
        bot.driveMecanum.driveStop();
        bot.driveMecanum.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public boolean IsTaskRunning(){ return bot.driveMecanum.is4MotorsBusy(); }
}