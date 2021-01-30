package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;


/**
 * Created by Reicher Robotics on 3/25/2018.
 */

public class AutoComOdomTurn extends AutoCommandMain {
    private double maxSpeed = 0.0;
    private double minSpeed = 0.1;
    private double targetHeading = 0.0;
    private double tolerance = 2.5;
    private double maxTime;
    private boolean tooLong = false;
    ElapsedTime endTime = new ElapsedTime();

    public AutoComOdomTurn(RobotOpModes opMode, double speed, double angle){
        super(opMode);

        this.maxSpeed = speed;
        this.targetHeading = angle;
        maxTime = 100.0;
    }

    public AutoComOdomTurn(RobotOpModes opMode, double speed, double angle, double time){
        super(opMode);

        this.maxSpeed = speed;
        this.targetHeading = angle;
        this.maxTime = time;
    }

    @Override
    public void Start(){endTime.reset();}

    @Override
    public void Loop(){
        if(endTime.time() > maxTime){
            tooLong = true;
        }

    }

    @Override
    public void Stop(){

    }

    @Override
    public boolean IsTaskRunning(){
        return !onHeading() && !opMode.isStopRequested() && !tooLong;
    }

    private boolean onHeading() {
        boolean onTarget = false;
        double currentHeading;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double finalError;

        bot.odometry.updatePosition(bot.leftEncoder.getDistance(), bot.rightEncoder.getDistance());
        currentHeading = bot.odometry.getPose().getHeading();
        if (Math.abs(targetHeading - currentHeading) <= tolerance) {
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            finalError = bot.gyroIMU.getError(targetHeading, currentHeading) / 150;
            if (finalError > 0.0) {
                steer = Range.clip(finalError, 0.01, 1.0);
            } else {
                steer = Range.clip(finalError, -1.0, -0.01);
            }
            rightSpeed = maxSpeed * steer;
            leftSpeed = -rightSpeed;
        }
        bot.diffy.tankDrive(leftSpeed, rightSpeed);
        return onTarget;
    }
}
