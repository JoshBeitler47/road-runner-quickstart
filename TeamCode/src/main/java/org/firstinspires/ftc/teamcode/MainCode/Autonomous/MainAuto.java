package org.firstinspires.ftc.teamcode.MainCode.Autonomous;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision.VisionHandler;
import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Constants.Spike;
import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Constants.Alliance;
import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Constants.Side;
import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Constants.Park;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
@Config
@TeleOp(name="Autonomous", group="Linear Opmode")


public final class MainAuto extends LinearOpMode {
    public static Side start = Side.AUDIENCE;
    public static Spike lcr;
    public static Alliance color = Alliance.RED;
    public static Park park = Park.CORNER;

    //for dashboard
    public static String startValue = "";
    public static String lcrValue = "";
    public static String colorValue = "";
    public static String parkValue = "";

    //AprilTagStuff
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    private static final boolean USE_WEBCAM = true;
    private static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    boolean readyToGo = false;


    public void runOpMode() throws InterruptedException {
        Pose2d startingPose;
        Pose2d nextPose;
        double xOffset = 0;
        double yOffset = 0;
        MecanumDrive drive;
        int reflect;
        int LCRNUM = 0;
        ConfigDashboard();

        //AprilTagStuff
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double desX = 0;
        double desY = 0;
        double desHeading = 0;
        initAprilTag();
        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        while(!isStarted()){
            if (gamepad1.right_bumper){
                if(color.equals(Alliance.RED)){
                    color = Alliance.BLUE;
                } else {
                    color = Alliance.RED;
                }
            }
            if (gamepad1.left_bumper){
                if(park.equals(Park.CORNER)){
                    park = Park.STAGE;
                } else {
                    park = Park.CORNER;
                }
            }
            if (gamepad1.a){
                if(start.equals(Side.AUDIENCE)){
                    start = Side.BACKSTAGE;
                } else {
                    start = Side.AUDIENCE;
                }
            }
            telemetry.addData("Color: ", color.name());
            telemetry.addData("Side: ", start.name());
            telemetry.addData("Parking: ", park.name());
            telemetry.update();
        }
        waitForStart();

//        visionHandler.init(hardwareMap);
//        waitForStart();
//
//        if(color.equals(Alliance.RED)){
//            visionHandler.setRed();
//        }else{
//            visionHandler.setBlue();
//        }
//        visionHandler.setLeft();
//        double left = visionHandler.read();
//        visionHandler.setMiddle();
//        double mid = visionHandler.read();
//        visionHandler.setRight();
//        double right = visionHandler.read();
//        if(left >= mid && left >= right)
//            lcr = Spike.LEFT;
//        if(mid >= right && mid >= left)
//            lcr = Spike.CENTER;
//        if(right >= left && right >= mid)
//            lcr = Spike.RIGHT;

        if (color.equals(Alliance.RED)) {
            reflect = 1;
        } else {
            reflect = -1;
        }
        switch (lcr){
            case LEFT:
                LCRNUM = -1*reflect;
                break;
            case CENTER:
                LCRNUM = 0;
                break;
            case RIGHT:
                LCRNUM = 1*reflect;
                break;
        }
        if (start.equals(Side.BACKSTAGE)){ //BackstageSide
            startingPose = new Pose2d(12, -64*reflect, Math.toRadians(90*reflect));
            drive = new MecanumDrive(hardwareMap, startingPose);
            switch (LCRNUM) {
                case -1:
                    nextPose = new Pose2d(2 + xOffset, -30*reflect + yOffset, Math.toRadians(90*reflect));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
                case 0:
                    nextPose = new Pose2d(12 + xOffset, -26*reflect + yOffset, Math.toRadians(90*reflect));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
                case 1:
                    nextPose = new Pose2d(22 + xOffset, -30*reflect + yOffset, Math.toRadians(90*reflect));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
            }
        } else { //AudienceSide
            startingPose = new Pose2d(-36, -64, Math.PI / 2);
            drive = new MecanumDrive(hardwareMap, startingPose);
            switch (LCRNUM) {
                case -1:
                    nextPose = new Pose2d(-46 + xOffset, -30*reflect + yOffset, Math.toRadians(90*reflect));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
                case 0:
                    nextPose = new Pose2d(-36 + xOffset, -26*reflect + yOffset, Math.toRadians(90*reflect));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
                case 1:
                    nextPose = new Pose2d(-26 + xOffset, -30*reflect + yOffset, Math.toRadians(90*reflect));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;

            }
        }
        //go to backboard
        if (start.equals(Side.BACKSTAGE)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(Math.toRadians(90*reflect))
                            .lineToY(-48*reflect)
                            .build());
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setTangent(0)
                            .turnTo(0)
                            .splineToConstantHeading(new Vector2d(45, -36*reflect), 0)
                            .build());
        } else if (start.equals(Side.AUDIENCE)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(Math.toRadians(90*reflect))
                            .lineToY(-60*reflect)
                            .build());
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setTangent(0)
                            .turnTo(0)
                            .lineToX(24)
                            .splineToConstantHeading(new Vector2d(45, -36*reflect), 0)
                            .build());
        }
        setDesAprilTag();
        while (opModeIsActive()) {
            targetFound = false;
            desiredTag = null;
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        targetFound = true;
                        desiredTag = detection;
                        readyToGo = true;
                        break;
                    }
                }
            }
        }

            if (readyToGo && targetFound) {
                desX = desiredTag.ftcPose.x + DESIRED_DISTANCE * Math.cos(desiredTag.ftcPose.yaw - (Math.PI / 2));
                desY = desiredTag.ftcPose.y + DESIRED_DISTANCE * Math.sin(desiredTag.ftcPose.yaw - (Math.PI / 2));
                desHeading = desiredTag.ftcPose.yaw;
                nextPose = new Pose2d(desX, desY, desHeading);
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(nextPose.position, nextPose.heading)
                                .build());
        }
    }



    private static void ConfigDashboard() {
        switch (startValue){
            case "AUDIENCE":
                start = Side.AUDIENCE;
                break;
            case "BACKSTAGE":
                start = Side.BACKSTAGE;
                break;
        }
        switch (colorValue) {
            case "RED":
                color = Alliance.RED;
                break;
            case "BLUE":
                color = Alliance.BLUE;
                break;
        }
        switch (lcrValue) {
            case "LEFT":
                lcr = Spike.LEFT;
                break;
            case "CENTER":
                lcr = Spike.CENTER;
                break;
            case "RIGHT":
                lcr = Spike.RIGHT;
                break;
        }
        switch (parkValue) {
            case "CORNER":
                park = Park.CORNER;
                break;
            case "STAGE":
                park = Park.STAGE;
        }
    }
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    private void setDesAprilTag(){
        if (color.equals(Alliance.RED)){
            switch (lcr){
                case LEFT:
                    DESIRED_TAG_ID = 4;
                    break;
                case CENTER:
                    DESIRED_TAG_ID = 5;
                    break;
                case RIGHT:
                    DESIRED_TAG_ID = 6;
                    break;
            }
        } else {
            switch (lcr){
                case LEFT:
                    DESIRED_TAG_ID = 1;
                    break;
                case CENTER:
                    DESIRED_TAG_ID = 2;
                    break;
                case RIGHT:
                    DESIRED_TAG_ID = 3;
                    break;
            }
        }
        }
}
