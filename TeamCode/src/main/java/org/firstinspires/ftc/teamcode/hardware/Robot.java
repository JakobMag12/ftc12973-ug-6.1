package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

import java.util.List;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Robot
{

    public static final double TRACKWIDTH = 15.5;
    public static final double WHEEL_DIAMETER = 1.5;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    /* Public OpMode members. */
    public DcMotor  leftFrontDrive = null;
    public DcMotor  leftRearDrive = null;
    public DcMotor  rightFrontDrive = null;
    public DcMotor  rightRearDrive = null;

    public DriveMecanum      driveMecanum = null;
    public Intake            intake = null;
    public WobbleLift        wobbleLift = null;
    public Outtake           outtake = null;
    public WobbleClaw        wobbleClaw = null;
    public WobbleAutoClaw    wobbleAutoClaw = null;
    public OuttakeRamp       outtakeRamp = null;
    public OuttakeTrigger    outtakeTrigger = null;
    public GuiderServo       guiderServo = null;
    public IMU               gyroIMU = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Robot() { }

    public void init(HardwareMap hwM) {
        hwMap = hwM;

        // Define and Initialize Motors
        String[] driveMotorNames = new String[]{"lf", "lr", "rf", "rr"};
        DcMotorSimple.Direction[] driveMotorDirections = new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD};
        String[] outtakeMotorNames = new String[]{"fw1", "fw2"};
        DcMotorSimple.Direction[] outtakeMotorDirections = new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD};
        String[] intakeMotorNames = new String[]{"intake"};
        DcMotorSimple.Direction[] intakeMotorDirections = new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD};
        String[] liftMotorNames = new String[]{"lift"};
        DcMotorSimple.Direction[] liftMotorDirections = new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD};
        gyroIMU = new IMU(hwMap, "imu");

        // Set all motors to zero power
        driveMecanum = new DriveMecanum(hwMap, driveMotorNames, driveMotorDirections);
        driveMecanum.setMotorBreak(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleLift = new WobbleLift(hwMap, liftMotorNames, liftMotorDirections, "wglimit");
        intake = new Intake(hwMap, intakeMotorNames, intakeMotorDirections);
        outtake = new Outtake (hwMap, outtakeMotorNames, outtakeMotorDirections);
        outtake.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        // Define and initialize ALL installed servos.
        String[] wobbleClawServoNames = new String[]{"wclaw"};
        String[] wobbleAutoClawServoNames = new String[]{"waclaw"};
        String[] outtakeRampServoNames = new String[]{"ramp"};
        String[] outtakeTriggerServoNames = new String[]{"trigger"};
        String[] guiderCRServoNames = new String[] {"guide"};
        CRServo.Direction[] guiderCRServoDirections = new CRServo.Direction[]{CRServo.Direction.REVERSE};

        wobbleClaw = new WobbleClaw(hwMap, wobbleClawServoNames);
        wobbleAutoClaw = new WobbleAutoClaw(hwMap, wobbleAutoClawServoNames);
        outtakeRamp = new OuttakeRamp(hwMap, outtakeRampServoNames);
        outtakeTrigger = new OuttakeTrigger(hwMap, outtakeTriggerServoNames);
        guiderServo = new GuiderServo (hwMap, guiderCRServoNames, guiderCRServoDirections);

        List<LynxModule> hubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void initAutoServos() {
        wobbleClaw.close();
        wobbleAutoClaw.close();
        outtakeRamp.top();
        outtakeTrigger.in();
    }

    public void initTeleOpServos() {
        wobbleClaw.close();
        wobbleAutoClaw.close();
        outtakeRamp.top();
        outtakeTrigger.in();
    }
}