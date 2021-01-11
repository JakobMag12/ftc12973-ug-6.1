package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    /* Public OpMode members. */
    public DcMotor  leftFrontDrive = null;
    public DcMotor  leftBackDrive = null;
    public DcMotor  rightFrontDrive = null;
    public DcMotor  rightBackDrive = null;
    public DcMotor  intake  = null;
    public DcMotor  flywheelOne  = null;
    public DcMotor  flywheelTwo  = null;
    public DcMotor  wobbleLift     = null;
    public Servo    wobbleClaw    = null;
    public Servo    wobbleTurn   = null;
    public Servo    wobbleAutoClaw  = null;
    public Servo    outtakeRamp  = null;
    public Servo    outtakeTrigger  = null;
    public Servo    outtakeSwivel  = null;


    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

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

        leftFrontDrive  = hwMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hwMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hwMap.get(DcMotor.class, "right_back_drive");
        intake          = hwMap.get(DcMotor.class, "intake");
        flywheelOne     = hwMap.get(DcMotor.class, "flywheel_one");
        flywheelTwo     = hwMap.get(DcMotor.class, "flywheel_two");
        wobbleLift      = hwMap.get(DcMotor.class, "wobble_lift");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        flywheelOne.setDirection(DcMotor.Direction.FORWARD);
        flywheelTwo.setDirection(DcMotor.Direction.FORWARD);
        wobbleLift.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        intake.setPower(0);
        flywheelOne.setPower(0);
        flywheelTwo.setPower(0);
        wobbleLift.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        wobbleClaw     = hwMap.get(Servo.class, "wobble_claw");
        wobbleTurn     = hwMap.get(Servo.class, "wobble_turn");
        wobbleAutoClaw = hwMap.get(Servo.class, "wobble_auto_claw");
        outtakeRamp    = hwMap.get(Servo.class, "outtake_ramp");
        outtakeTrigger = hwMap.get(Servo.class, "right_hand");
        outtakeSwivel  = hwMap.get(Servo.class, "right_hand");
        wobbleClaw.setPosition(MID_SERVO);
        wobbleTurn.setPosition(MID_SERVO);
        wobbleAutoClaw.setPosition(MID_SERVO);
        outtakeRamp.setPosition(MID_SERVO);
        outtakeTrigger.setPosition(MID_SERVO);
        outtakeSwivel.setPosition(MID_SERVO);
    }
 }
