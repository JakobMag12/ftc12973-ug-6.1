package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;

/**
 * Created by Reicher Robotics on 3/25/2018.
 */

/**
public class AutoComOdomDifDrive extends AutoCommandMain {
    private double distance;
    private int targetTicks = 0;
    private double speed = 0.0;
    private Pose2d currentPose, targetPose;
    private double xDistance, yDistance;

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

    public AutoComOdomDifDrive(RobotOpModes opMode, double distance, double speed){
        super(opMode);

        this.speed = speed;
        this.distance = distance;
        targetTicks = (int)bot.differentialDriveTrain.convertInchToTicks(distance);
    }

    @Override
    public void Start(){
        currentPose = bot.drive.getPoseEstimate();
        targetHeading = currentPose.getHeading();
        targetPose = new Pose2d(currentPose.getX() + distance, currentPose.getY(), targetHeading);
        prevError = 0.0;
        prevTime = 0.0;
        integral = 0.0;
        time.reset();
    }

    @Override
    public void Loop(){
        currentTime = time.milliseconds();
        timeLapsed = currentTime - prevTime;
        currentPose = bot.drive.getPoseEstimate();
        error = currentPose.getHeading();
        integral += error * timeLapsed;
        derivative = (error - prevError) / timeLapsed;
        correction = bot.differentialDriveTrain.P * error + bot.differentialDriveTrain.I * integral + bot.differentialDriveTrain.D * derivative;
        prevTime = currentTime;
        prevError = error;
        bot.drive.driveArcade(speed, -correction);
    }

    @Override
    public void Stop(){
        bot.drive.setMotorPowers(0.0, 0.0);
        bot.differentialDriveTrain.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public boolean IsTaskRunning(){
        return (Math.sqrt(Math.pow(xDistance, 2.0) + Math.pow(yDistance, 2.0)) > error);
    }
}
 */