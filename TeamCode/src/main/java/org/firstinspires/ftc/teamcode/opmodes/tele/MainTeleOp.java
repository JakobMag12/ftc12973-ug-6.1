package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.hardware.DifferentialDrive;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;

/**
 * Created by Reicher Robotics on 2/26/2018.
 */

@TeleOp(name="Main", group="Version 2.1")
public class MainTeleOp extends RobotOpModes {

    private Controller controller1;
    private Controller controller2;

    private boolean TRIGGERED;
    private boolean INTAKE;
    private boolean LAUNCHED;
    private boolean TRAP;
    private double rampHeight;

    private double maxDrivePower;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        // Flags
        TRIGGERED = false;
        INTAKE = false;
        LAUNCHED = false;
        TRAP = false;

        maxDrivePower = 0.0;

        // Controllers
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        // Init
        //bot.initTeleOpServo();
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
            bot.diffy.arcadeDrive(0.8, 0.8);

            if (controller1.yButton == Controller.ButtonState.PRESSED) {
                bot.outtakeRamp.top();
            }
            if (controller1.bButton == Controller.ButtonState.PRESSED) {
                bot.outtakeRamp.mid();
            }
            if (controller1.aButton == Controller.ButtonState.PRESSED) {
                bot.outtakeRamp.power();
            }

            if (controller1.rightTrigger == Controller.ButtonState.PRESSED) {
                bot.outtake.out(1.0);
            } else {
                bot.outtake.stop();
            }

            if (controller1.leftBumper == Controller.ButtonState.PRESSED) {
                bot.intake.in(1.0);
                bot.guiderServo.in();
            }
            if (controller1.leftTrigger == Controller.ButtonState.PRESSED) {
                bot.intake.out(1.0);
                bot.guiderServo.out();
            }

            if (controller1.leftBumper == Controller.ButtonState.PRESSED) {
                TRIGGERED = !TRIGGERED;
                if (TRIGGERED) {
                    bot.outtakeTrigger.shoot();
                } else {
                    bot.outtakeTrigger.in();
                }
            }

            /**********************************************************************************************
             * Controller 2 Controls
             **********************************************************************************************/

            bot.wobbleLift.liftWithLimits(controller2.rightJoystickYValue);

            if (controller2.leftTrigger == Controller.ButtonState.PRESSED) {
                bot.outtake.out(1.0);
            } else {
                bot.outtake.stop();
            }

            if (controller2.leftBumper == Controller.ButtonState.PRESSED) {
                bot.wobbleTurn.in();
                TRAP = !TRAP;
                if (TRAP) {
                    bot.trapDoor.down();
                } else {
                    bot.trapDoor.up();
                }
            }

            if (controller2.leftBumper == Controller.ButtonState.PRESSED) {
                TRIGGERED = !TRIGGERED;
                if (TRIGGERED) {
                    bot.outtakeTrigger.shoot();
                } else {
                    bot.outtakeTrigger.in();
                }
            }

            if (controller2.bButton == Controller.ButtonState.PRESSED) {
                bot.trapDoor.up();
            }
            if (controller2.aButton == Controller.ButtonState.PRESSED) {
                bot.trapDoor.down();
            }
        }
    }
}


