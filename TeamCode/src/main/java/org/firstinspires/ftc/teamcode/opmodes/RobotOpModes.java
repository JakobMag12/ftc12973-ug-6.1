package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Created by Reicher Robotics on 3/19/2018.
 */

public abstract class RobotOpModes extends LinearOpMode {

    public Robot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Robot();
    }
}
