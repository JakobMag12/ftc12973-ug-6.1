package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;

/**
 * Created by Reicher Robotics on 2/26/2018.
 */

@TeleOp(name="Main", group="Version 2.1")
public class MainTeleOp extends RobotOpModes {

    private Controller controller1;
    private Controller controller2;

    private boolean TRIGGER;
    private boolean INTAKE;
    private boolean RAMP;
    private boolean EXTENDED;
    private boolean LAUNCHED;
    private double rampHeight;

    private double maxDrivePower;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        // Flags
        TRIGGER = false;
        INTAKE = false;
        EXTENDED = false;

        maxDrivePower = 0.0;

        // Controllers
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        // Init
        bot.initTeleOpServos();
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
            maxDrivePower = 1.0;
            bot.WobbleTurn.tankDrive(maxDrivePower, controller1.leftJoystickYValue, controller1.leftJoystickXValue, controller1.rightJoystickXValue);

            // Intake Wheels

            if (controller1.yButton == Controller.ButtonState.PRESSED) {
                bot.intakeDown.down();
            }
            if (controller1.xButton == Controller.ButtonState.PRESSED) {
                bot.intakeDown.up();
            }

            if (controller1.leftBumper == Controller.ButtonState.ON_PRESS) {
                INTAKE = !INTAKE;
            }
            if (controller1.aButton == Controller.ButtonState.PRESSED) {
                bot.intake.out(0.5);
                INTAKE = false;
                RAMP = false;
            } else if (INTAKE) {
                bot.intake.in(0.8);
            } else {
                bot.intake.stop();
            }

            // Trigger
            // Right Trigger
            if (controller1.rightTrigger == Controller.ButtonState.ON_PRESS) {
                bot.trigger.out();
            }

            if (controller1.rightTrigger == Controller.ButtonState.ON_RELEASE) {
                bot.trigger.in();
            }

            if (controller1.leftTrigger == Controller.ButtonState.PRESSED) {
                bot.launcher.out(1.0);
            } else {
                bot.launcher.stop();
            }
            /*
            if(controller1.dPadUp == Controller.ButtonState.PRESSED) {
                if(rampHeight < 0.98) {
                    rampHeight += 0.02;
                }
            }

            if(controller1.dPadDown == Controller.ButtonState.PRESSED) {
                if(rampHeight > 0.02) {
                    rampHeight -= 0.02;
                }
            }

            bot.ramp.setHeight(rampHeight);
            */

            if (controller1.dPadUp == Controller.ButtonState.ON_PRESS) {
                bot.ramp.top();
            }

            if (controller1.dPadLeft == Controller.ButtonState.ON_PRESS) {
                bot.ramp.mid();
            }

            if (controller1.dPadRight == Controller.ButtonState.ON_PRESS) {
                bot.ramp.bottom();
            }

            if (controller1.dPadDown == Controller.ButtonState.ON_PRESS) {
                bot.ramp.power();
            }

            /**********************************************************************************************
             * Controller 2 Controls
             **********************************************************************************************/

            // Lift Control
            // Left Joystick Y
            bot.lift.liftWithLimits(controller2.rightJoystickYValue);

            // Lift Control
            // Left Joystick Y
            bot.extender.extend(controller2.leftJoystickYValue);
        }
    }
}


