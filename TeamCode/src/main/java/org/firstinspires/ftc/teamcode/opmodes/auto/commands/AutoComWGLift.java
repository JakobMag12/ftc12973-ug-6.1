package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;

/**
 * Created by Reicher Robotics on 3/25/2018.
 */


public class AutoComWGLift extends AutoCommandMain {
    private double angle = 0.0;

    public AutoComWGLift(RobotOpModes opMode, double angle){
        super(opMode);

        this.angle = angle;
    }

    @Override
    public void Start(){
        //bot.wobbleLift.setDownTarget(bot.wobbleLifts.convertAngleToTicks(angle));
        bot.wobbleLift.setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.wobbleLift.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void Loop(){
        bot.wobbleLift.down();
    }

    @Override
    public void Stop(){
        bot.wobbleLift.stop();
        bot.wobbleLift.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public boolean IsTaskRunning(){
        return bot.wobbleLift.isMotorBusy();
    }
}