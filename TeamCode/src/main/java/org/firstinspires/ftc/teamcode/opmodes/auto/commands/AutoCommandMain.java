package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;

/**
 * Created by Reicher Robotics on 3/19/2018.
 */

public abstract class AutoCommandMain {
    public Robot bot;
    public RobotOpModes opMode;
    public boolean overrideLoop = false;

    public AutoCommandMain(RobotOpModes opMode){
        this.opMode = opMode;
        this.bot = opMode.bot;
    }

    public void setOverrideLoop(boolean va){
        overrideLoop = overrideLoop;
    }

    public void Run(){
        Start();
        if(!overrideLoop){
            while(canRunLoop()){
                Loop();
            }
            Stop();
        }

    }

    public abstract void Start();
    public abstract void Loop();
    public abstract void Stop();
    public abstract boolean IsTaskRunning();

    public boolean canRunLoop(){
        return !opMode.isStopRequested() && opMode.opModeIsActive() && IsTaskRunning();
    }
}
