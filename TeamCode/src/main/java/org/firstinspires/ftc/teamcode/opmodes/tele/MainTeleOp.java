package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.hardware.OuttakeTrigger;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.lib.Controller;

import java.util.concurrent.TimeUnit;

/**
 * Created by Reicher Robotics on 2/26/2018.
 */

@TeleOp(name="Main", group="Version 1.0")
public class MainTeleOp extends LinearOpMode {

    private Controller controller1;
    private Controller controller2;

    private boolean TRIGGERED;
    private boolean INTAKE;
    private boolean LAUNCHED;
    private boolean TRAP;
    private boolean SLOW;
    private boolean CLAW_OPEN;
    private boolean SHOOT;

    private int velocity;
    private double rampPosition;
    private double triggerPosition;
    private double shootTime;
    private double inTime;

    private double maxDrivePower;
    private Robot bot = new Robot();
    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Flags
        TRIGGERED = false;
        INTAKE = false;
        LAUNCHED = false;
        TRAP = false;
        SLOW = false;
        CLAW_OPEN = false;
        SHOOT = false;

        maxDrivePower = 0.0;

        // Controllers
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

         //Init
        bot.init(hardwareMap);
        rampPosition = bot.outtakeRamp.TOP;
        triggerPosition = bot.outtakeTrigger.SHOOT;
        velocity = bot.outtake.VELOCITY;
        shootTime = bot.outtakeTrigger.SHOOT_TIME;
        inTime = bot.outtakeTrigger.IN_TIME;
        bot.initTeleOpServos();
        telemetry.addLine("Status Initialized");
        telemetry.update();
        waitForStart();

        time.reset();
        while (opModeIsActive()) {
            controller1.update();
            controller2.update();

            /**********************************************************************************************
             * Controller 1 Controls
             **********************************************************************************************/

            // Differential Drive Joysticks
            // Add
            // To
            // Later
            // Either Joystick Button - slows down max speed, press slows
            if (controller1.leftJoystickButton == Controller.ButtonState.PRESSED || controller1.rightJoystickButton == Controller.ButtonState.PRESSED) {
                SLOW = !SLOW;
            } if (SLOW) {
                maxDrivePower = 0.4;
            } else {
                maxDrivePower = 1.0;
            }
            bot.driveMecanum.mecanumDrive(maxDrivePower, controller1.leftJoystickYValue, controller1.rightJoystickXValue, controller1.leftJoystickXValue);

            if (controller1.yButton == Controller.ButtonState.PRESSED) {
                rampPosition = bot.outtakeRamp.TOP;
            }
            if (controller1.bButton == Controller.ButtonState.PRESSED) {
                rampPosition = bot.outtakeRamp.POWER;
            }
            if (controller1.aButton == Controller.ButtonState.PRESSED) {
                rampPosition = bot.outtakeRamp.MID;
            }

            if (controller1.leftBumper == Controller.ButtonState.ON_PRESS) {
                INTAKE = !INTAKE;
                if (INTAKE) {
                    bot.intake.in(1.0);
                    bot.guiderServo.in();
                } else {
                    bot.intake.stop();
                    bot.guiderServo.stop();
                }
            }
            if (controller1.leftTrigger == Controller.ButtonState.PRESSED) {
                bot.intake.out(1.0);
                bot.guiderServo.out();
            }
            if(controller1.leftTrigger == Controller.ButtonState.ON_RELEASE) {
                INTAKE = false;
                bot.intake.stop();
                bot.guiderServo.stop();
            }

            if (controller1.rightTrigger == Controller.ButtonState.ON_PRESS) {
                bot.outtakeTrigger.position(triggerPosition);
            }
            if (controller1.rightTrigger == Controller.ButtonState.ON_RELEASE) {
                bot.outtakeTrigger.in();
            }

            if (controller1.rightBumper == Controller.ButtonState.ON_PRESS) {
                bot.outtakeTrigger.rapidFireState = OuttakeTrigger.TRIGGER_STATE.FIRST_SHOT;
                bot.outtakeTrigger.setRapidFireTime(time.now(TimeUnit.MILLISECONDS));
            }
            if (controller1.rightBumper == Controller.ButtonState.PRESSED) {
                bot.outtakeTrigger.rapidFire(time.now(TimeUnit.MILLISECONDS));
            }
            if (controller1.rightBumper == Controller.ButtonState.ON_RELEASE) {
                bot.outtakeTrigger.rapidFireState = OuttakeTrigger.TRIGGER_STATE.UNKNOWN;
                bot.outtakeTrigger.in();
            }

            if (controller1.dPadUp == Controller.ButtonState.ON_PRESS) {
                shootTime += 10;
            }
            if (controller1.dPadDown == Controller.ButtonState.ON_PRESS) {
                shootTime -= 10;
            }

            if (controller1.dPadLeft == Controller.ButtonState.ON_PRESS) {
                inTime += 10;
            }
            if (controller1.dPadRight == Controller.ButtonState.ON_PRESS) {
                inTime -= 10;
            }

            /**********************************************************************************************
             * Controller 2 Controls
             **********************************************************************************************/

            bot.wobbleLift.liftWithLimits(controller2.rightJoystickYValue);

            if (controller2.leftTrigger == Controller.ButtonState.PRESSED) {
                SHOOT = true;
            }

            if (controller2.leftTrigger == Controller.ButtonState.ON_RELEASE) {
                SHOOT = false;
            }

            if (controller2.leftBumper == Controller.ButtonState.ON_PRESS) {
                SHOOT = !SHOOT;
            }

            if (SHOOT) {
                bot.outtake.shoot(-velocity);
            } else {
                bot.outtake.stop();
            }

            if (controller2.rightTrigger == Controller.ButtonState.ON_PRESS) {
                bot.outtakeTrigger.position(triggerPosition);
            }
            if (controller2.rightTrigger == Controller.ButtonState.ON_RELEASE) {
                bot.outtakeTrigger.in();
            }

            if (controller2.rightBumper == Controller.ButtonState.ON_PRESS) {
                bot.outtakeTrigger.rapidFireState = OuttakeTrigger.TRIGGER_STATE.FIRST_SHOT;
                bot.outtakeTrigger.setRapidFireTime(time.now(TimeUnit.MILLISECONDS));
            }
            if (controller2.rightBumper == Controller.ButtonState.PRESSED) {
                bot.outtakeTrigger.rapidFire(time.now(TimeUnit.MILLISECONDS));
            }
            if (controller1.rightBumper == Controller.ButtonState.ON_RELEASE) {
                bot.outtakeTrigger.rapidFireState = OuttakeTrigger.TRIGGER_STATE.UNKNOWN;
                bot.outtakeTrigger.in();
            }

            /*
            if (controller2.xButton == Controller.ButtonState.PRESSED) {
                bot.wobbleClaw.open();
            }
            if (controller2.yButton == Controller.ButtonState.PRESSED) {
                bot.wobbleClaw.close();
            }
            */

            if (controller2.aButton == Controller.ButtonState.ON_PRESS) {
                CLAW_OPEN = !CLAW_OPEN;
                if (CLAW_OPEN) {
                    bot.wobbleClaw.open();
                } else {
                    bot.wobbleClaw.close();
                }
            }

            if (controller2.dPadUp == Controller.ButtonState.ON_PRESS) {
                velocity += 50;
            }
            if (controller2.dPadDown == Controller.ButtonState.ON_PRESS) {
                velocity -= 50;
            }

            if (controller2.dPadLeft == Controller.ButtonState.ON_PRESS) {
                triggerPosition += 0.005;
            }
            if (controller2.dPadRight == Controller.ButtonState.ON_PRESS) {
                triggerPosition -= 0.005;
            }

            if (controller2.yButton == Controller.ButtonState.ON_PRESS) {
                rampPosition -= 0.005;
            }
            if (controller2.xButton == Controller.ButtonState.ON_PRESS) {
                rampPosition += 0.005;
            }

            bot.outtakeTrigger.setShootTime(shootTime);
            bot.outtakeTrigger.setInTime(inTime);
            bot.outtakeRamp.position(rampPosition);

            /**********************************************************************************************
             * Telemetry
             **********************************************************************************************/
            telemetry.addData("Set Velocity", velocity);
            telemetry.addData("Shooter Velocity", "%.1f", -bot.outtake.getVelocity());
            telemetry.addData("Ramp Position", "%.3f", rampPosition);
            telemetry.addData("Trigger Position", "%.3f", triggerPosition);
            telemetry.addData("Shoot Time", shootTime);
            telemetry.addData("In Time", inTime);
            telemetry.addData("Lift", "%.2f", bot.wobbleLift.getLimitVoltage());
            telemetry.update();
        }
    }
}


