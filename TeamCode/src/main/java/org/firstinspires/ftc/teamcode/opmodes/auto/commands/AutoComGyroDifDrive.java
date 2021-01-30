package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.DifferentialDrive;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;

/**
 * Created by Reicher Robotics on 3/25/2018.
 */

public class AutoComGyroDifDrive extends AutoCommandMain {


    public double P = 0.5;
    public double I = 0.0;
    public double D = 0.9;
    private double speed;
    private double maxDistance;
    private double distance;
    private int targetTicks = 0;

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
        targetTicks = convertInchToTicks(distance);
    }

    @Override
    public void Start(){
        bot.diffySub.resetEncoders();
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
        correction = P * error + I * integral + D * derivative;
        prevTime = currentTime;
        prevError = error;
        bot.diffySub.drive(speed, -correction);
    }

    @Override
    public void Stop(){
        bot.diffySub.drive(0, 0);
    }

    @Override
    public boolean IsTaskRunning(){
        return Math.abs(bot.diffySub.getAverageEncoderDistance()) > Math.abs(targetTicks);
    }

    public double TICKS_PER_REV = 8192;
    public double WHEEL_DIAMETER = 38 / 25.4;
    public double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_DIAMETER / 3.14159;

    public int convertInchToTicks(double inches) { return (int) (inches * TICKS_PER_INCH) ; }
}