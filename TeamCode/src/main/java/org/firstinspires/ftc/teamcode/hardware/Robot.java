package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public MotorEx leftEncoder = null;
    public MotorEx rightEncoder = null;
    public com.arcrobotics.ftclib.kinematics.DifferentialOdometry odometry = null;

    /* Public OpMode members. */
    public DcMotor  leftFrontDrive = null;
    public DcMotor  leftRearDrive = null;
    public DcMotor  rightFrontDrive = null;
    public DcMotor  rightRearDrive = null;

    public DifferentialDrive diffy = null;
    public DriveSubsystem diffySub = null;
    public Intake            intake = null;
    public WobbleLift        wobbleLift = null;
    public Outtake           outtake = null;
    public WobbleClaw        wobbleClaw = null;
    public WobbleTurn        wobbleTurn = null;
    public WobbleAutoClaw    wobbleAutoClaw = null;
    public OuttakeRamp       outtakeRamp = null;
    public OuttakeTrigger    outtakeTrigger = null;
    public OuttakeSwivel     outtakeSwivel = null;
    public TrapDoor          trapDoor = null;
    public GuiderServo       guiderServo = null;
    public IMU               gyroIMU = null;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Robot(HardwareMap hwM) {
        hwMap = hwM;
    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {

        // Define and Initialize Motors

        leftFrontDrive  = hwMap.get(DcMotor.class, "lf");
        leftRearDrive   = hwMap.get(DcMotor.class, "lr");
        rightFrontDrive = hwMap.get(DcMotor.class, "rf");
        rightRearDrive  = hwMap.get(DcMotor.class, "rr");
        Motor leftFrontDrive = new Motor(hwMap, "lf", Motor.GoBILDA.RPM_435);
        Motor rightFrontDrive = new Motor(hwMap, "rf", Motor.GoBILDA.RPM_435);
        Motor leftRearDrive = new Motor(hwMap, "lr", Motor.GoBILDA.RPM_435);
        Motor rightRearDrive = new Motor(hwMap, "rr", Motor.GoBILDA.RPM_435);
        MotorGroup leftSideMotors = new MotorGroup(leftFrontDrive, leftRearDrive);
        MotorGroup rightSideMotors = new MotorGroup(rightFrontDrive, rightRearDrive);
        DifferentialDrive diffy = new DifferentialDrive(leftSideMotors, rightSideMotors);
        DriveSubsystem diffySub = new DriveSubsystem(leftEncoder, rightEncoder, 96/25.4);
        String[] outtakeMotorNames = new String[]{"fw1", "fw2"};
        DcMotorSimple.Direction[] outtakeMotorDirections = new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD};
        String[] intakeMotorNames = new String[]{"intake"};
        DcMotorSimple.Direction[] intakeMotorDirections = new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD};
        String[] liftMotorNames = new String[]{"lift"};
        DcMotorSimple.Direction[] liftMotorDirections = new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD};
        gyroIMU = new IMU(hwMap, "imu");

        leftEncoder = new MotorEx(hwMap, "rf");
        rightEncoder = new MotorEx(hwMap, "rr");

        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        odometry = new com.arcrobotics.ftclib.kinematics.DifferentialOdometry(leftEncoder::getDistance, rightEncoder::getDistance, TRACKWIDTH);
        odometry.updatePose();
        // Set all motors to zero power

        wobbleLift = new WobbleLift(hwMap, liftMotorNames, liftMotorDirections);
        intake = new Intake(hwMap, intakeMotorNames, intakeMotorDirections);
        outtake = new Outtake (hwMap, outtakeMotorNames, outtakeMotorDirections);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        // Define and initialize ALL installed servos.
        String[] wobbleClawServoNames = new String[]{"wclaw"};
        String[] wobbleTurnServoNames = new String[]{"wturn"};
        String[] wobbleAutoClawServoNames = new String[]{"waclaw"};
        String[] outtakeRampServoNames = new String[]{"oramp"};
        String[] outtakeTriggerServoNames = new String[]{"otrigger"};
        String[] outtakeSwivelServoNames = new String[]{"oswivel"};
        String[] guiderCRServoNames = new String[] {"guide"};
        String[] trapDoorServoNames = new String[] {"td"};
        CRServo.Direction[] guiderCRServoDirections = new CRServo.Direction[]{CRServo.Direction.REVERSE};


        wobbleClaw = new WobbleClaw(hwMap, wobbleClawServoNames);
        wobbleTurn = new WobbleTurn(hwMap, wobbleTurnServoNames);
        wobbleAutoClaw = new WobbleAutoClaw(hwMap, wobbleAutoClawServoNames);
        outtakeRamp = new OuttakeRamp(hwMap, outtakeRampServoNames);
        outtakeTrigger = new OuttakeTrigger(hwMap, outtakeTriggerServoNames);
        outtakeSwivel = new OuttakeSwivel(hwMap, outtakeSwivelServoNames);
        trapDoor = new TrapDoor(hwMap, trapDoorServoNames);
        guiderServo = new GuiderServo (hwMap, guiderCRServoNames, guiderCRServoDirections);

        List<LynxModule> hubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    public void initAutoServos() {

    }
    public void initTeleOpServos() {

    }
}