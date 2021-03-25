/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.OuttakeTrigger;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpModes;
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.AutoComGyroMecDrive;
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.AutoComGyroTurn;
//import org.firstinspires.ftc.teamcode.opmodes.auto.commands.AutoComWDDrive;
//import org.firstinspires.ftc.teamcode.opmodes.auto.commands.AutoComWDDrive;
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.AutoComWDDrive;
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.AutoComWGLift;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red Right Top Wobble Vision", group = "Test")
public class
VisionTopSingleWobble extends RobotOpModes
{
    OpenCvWebcam webcam;
    SkystoneDeterminationPipeline pipeline;
    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        bot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webcam.setViewportRenderingPolicy(OpenCvWebcam.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(640, 480,  OpenCvCameraRotation.UPRIGHT));

        bot.init(hardwareMap);
        bot.initAutoServos();
        while(!isStarted() && !bot.wobbleLift.atZeroPosition()) {
            bot.wobbleLift.upP(1.0);
        }
        bot.wobbleLift.stop();

        while (!isStarted())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.addData("IMU", bot.gyroIMU.getXHeading());
            telemetry.addData("IMU", bot.gyroIMU.getYHeading());
            telemetry.addData("IMU", bot.gyroIMU.getZHeading());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(1000);
        }

        waitForStart();
        time.reset();

        SkystoneDeterminationPipeline.RingPosition rings = pipeline.position;


        bot.outtake.shoot(bot.outtake.VELOCITY);
        //new AutoComWDDrive(this, 52.0, 0.5, 90.0).Run();
        new AutoComGyroMecDrive(this, 52.0, 0.0, 0.5).Run();
        new AutoComGyroTurn(this, 0.6, 93.0, 8.0).Run();
        new AutoComGyroMecDrive(this, 18.0, 0.0, 0.5).Run();
        while(!bot.wobbleLift.atGrabPosition()) {
            bot.wobbleLift.downP(1.0);
        }
        bot.wobbleLift.stop();
        new AutoComGyroTurn(this, 0.6, 0.0, 8.0).Run();

        bot.outtakeTrigger.rapidFireState = OuttakeTrigger.TRIGGER_STATE.FIRST_SHOT;
        bot.outtakeTrigger.setRapidFireTime(time.now(TimeUnit.MILLISECONDS));
        while (opModeIsActive() && bot.outtakeTrigger.rapidFireState != OuttakeTrigger.TRIGGER_STATE.UNKNOWN) {
            bot.outtakeTrigger.rapidFire(time.now(TimeUnit.MILLISECONDS));
        }

        if(rings == SkystoneDeterminationPipeline.RingPosition.NONE) {
            new AutoComGyroMecDrive(this, 11.0, 0.0, 0.0).Run();
            new AutoComGyroTurn(this, 0.6, 270.0, 8.0).Run();
            new AutoComGyroMecDrive(this, 12.0, 0.0, 0.5).Run();
            bot.wobbleAutoClaw.open();
            sleep(1000);
            new AutoComGyroMecDrive(this, -6.0, 0.0, 0.5).Run();
            new AutoComWGLift(this, 270.0).Run();
            new AutoComGyroTurn(this, 0.6,0,8.0).Run();
        } else if(rings == SkystoneDeterminationPipeline.RingPosition.ONE) {
            new AutoComGyroTurn(this, 0.6, -12.0, 8.0).Run();
            new AutoComGyroMecDrive(this, 20.0, 0.0, 0.5).Run();
            bot.wobbleAutoClaw.open();
            sleep(500);
            new AutoComGyroMecDrive(this, -8.0, 0.0, 0.5).Run();
            new AutoComGyroTurn(this, 0.6,0,8.0).Run();
        } else if (rings == SkystoneDeterminationPipeline.RingPosition.FOUR) {
            new AutoComGyroTurn(this, 0.75, -25.0, 4.0).Run();
            new AutoComGyroMecDrive(this, 50.0, 0.0, 0.5).Run();
            bot.wobbleAutoClaw.open();
            sleep(500);
            new AutoComGyroMecDrive(this, -30.0, 0.0, 0.5).Run();
            new AutoComGyroTurn(this, 0.6,0,8.0).Run();
        }


        bot.wobbleClaw.open();
        bot.wobbleAutoClaw.close();
        new AutoComWGLift(this, 153.0).Run();
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(320,240);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 50;

        final int FOUR_RING_THRESHOLD = 158;
        final int ONE_RING_THRESHOLD = 137;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}