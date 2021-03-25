package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;
//import org.firstinspires.ftc.teamcode.opmodes.auto.commands.AutoComGyroDifDrive;

/**
 * Created by Reicher Robotics on 3/24/2018.
 */

@Autonomous(name="Red Right Top Wobble", group="RedRight")
//@Disabled
public class RedRightTopWobble extends RobotOpModes {

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        // Init
        bot.initAutoServos();
        /*while(!isStarted() && !bot.wobbleLift.limitReached()) {
            bot.wobbleLift.up();
        }*/
        bot.wobbleLift.stop();
        telemetry.update();

        waitForStart();
        //new AutoComGyroDifDrive(this, 24.0, 0.5).Run();
    }

    ElapsedTime waitTime = new ElapsedTime();
    public void pause(double time) {
        waitTime.reset();
        while (waitTime.time() < time && this.opModeIsActive()) {
        }
    }
}