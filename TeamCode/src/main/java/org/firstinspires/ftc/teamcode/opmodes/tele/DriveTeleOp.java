package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;

/**
 * Created by Reicher Robotics on 2/26/2018.
 */

@TeleOp(name="MecDrive", group="Version 1")
public class DriveTeleOp extends RobotOpModes {

    private Controller controller1;
    private Controller controller2;

    private double maxDrivePower;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        maxDrivePower = 0.0;

        // Controllers
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        // Init
        telemetry.addLine("Status Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            controller1.update();
            controller2.update();

            /**********************************************************************************************
             * Controller 1 Controls
             **********************************************************************************************/

            // Mecanum Drive Joysticks
            // Left Joystick Up/Down - drive forward/backward
            // Left Joystick Left/Right - turn right/left
            // Right Joystick Left/Right - drive left/right
            // Either Joystick Button - slows down max speed, press slows
            if (controller1.leftTrigger == Controller.ButtonState.PRESSED) {
                maxDrivePower = 0.4;
            } else {
                maxDrivePower = 1.0;
            }
            driveBot.driveMecanum.mecanumDrive(maxDrivePower, controller1.leftJoystickYValue, controller1.leftJoystickXValue, controller1.rightJoystickXValue);
        }
    }
}

