package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotDrive;

/**
 * Created by Reicher Robotics on 3/19/2018.
 */

public abstract class RobotOpModes extends LinearOpMode {

    public Robot bot;
    public RobotDrive driveBot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Robot(hardwareMap);
        driveBot = new RobotDrive(hardwareMap);
    }
}
