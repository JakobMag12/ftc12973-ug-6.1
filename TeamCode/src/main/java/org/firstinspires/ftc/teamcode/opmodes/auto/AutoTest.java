/**
package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DriveMecanum;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.AutoComGyroMecDrive;


/**
 * Created by Reicher Robotics on 3/24/2018.
 */
/**
@Autonomous(name="Test", group="Test")
//@Disabled
public class AutoTest extends RobotOpModes {

    private int currentTick = 0;
    private double speed = 0.5;
    private int frontLeftTicks, frontRightTicks, rearLeftTicks, rearRightTicks;
    private DriveMecanum.DriveDirection[] directions = new DriveMecanum.DriveDirection[3];

    private Controller controller1;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        // Init
        bot.initAutoServos();


        waitForStart();

        new AutoComGyroMecDrive(this, 48.0, 0.0, 0.75).Run();


        bot.driveMecanum.driveStop();
        pause(10.0);

    }

    ElapsedTime waitTime = new ElapsedTime();
    public void pause(double time) {
        waitTime.reset();
        while (waitTime.time() < time && this.opModeIsActive()) {
        }
    }
}
 */