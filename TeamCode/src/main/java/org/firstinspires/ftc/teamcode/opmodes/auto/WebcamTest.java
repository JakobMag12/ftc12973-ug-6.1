package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;

/**
 * Created by Reicher Robotics on 3/24/2018.
 */

@Autonomous(name="Webcam Test", group="Test")
//@Disabled
public class WebcamTest extends RobotOpModes {


    private Controller controller1;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        controller1 = new Controller(gamepad1);
        // Init
        bot.initAutoServos();
        telemetry.update();

        waitForStart();


        pause(1.0);
        bot.initTeleOpServos();
    }

    ElapsedTime waitTime = new ElapsedTime();
    public void pause(double time) {
        waitTime.reset();
        while (waitTime.time() < time && this.opModeIsActive()) {
        }
    }
}