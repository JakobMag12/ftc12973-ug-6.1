package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;

/**
 * Created by Reicher Robotics on 3/25/2018.
 */

/**
public class AutoComGyroDifDrive extends AutoCommandMain {
    private double distance;
    private int targetTicks = 0;
    private double speed = 0.0;

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

    public AutoComGyroDifDrive(RobotOpModes opMode, double distance, double speed){
        super(opMode);

        this.speed = speed;
        this.distance = distance;
        targetTicks = (int)bot.differentialDriveTrain.convertInchToTicks(distance);
    }

    @Override
    public void Start(){
        bot.differentialDriveTrain.setDriveTargets(targetTicks, targetTicks);
        bot.differentialDriveTrain.setMotorBreak(DcMotor.ZeroPowerBehavior.BRAKE);
        bot.differentialDriveTrain.setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.differentialDriveTrain.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
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
        correction = bot.differentialDriveTrain.P * error + bot.differentialDriveTrain.I * integral + bot.differentialDriveTrain.D * derivative;
        prevTime = currentTime;
        prevError = error;
        bot.differentialDriveTrain.differentialDrive(speed, -correction);
    }

    @Override
    public void Stop(){
        bot.differentialDriveTrain.driveStop();
        bot.differentialDriveTrain.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public boolean IsTaskRunning(){
        return bot.differentialDriveTrain.is4MotorsBusy();
    }
}
 */