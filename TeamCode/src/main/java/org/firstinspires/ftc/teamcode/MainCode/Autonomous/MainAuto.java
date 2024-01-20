package org.firstinspires.ftc.teamcode.MainCode.Autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision.VisionHandler;
import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision.VisionParameters;
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

//Intake Pixel goes closer to truss
//Outtake pixel is on right side

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
    VisionHandler visionHandler;
    DcMotorEx intake_elbow, outtake_elbow, hang_arm;
    DcMotor intake_grabber;
    Servo left_intake, right_intake, outtake_wrist, drone_launcher;


    private PIDController controller;
    public static double p = 0.02, i = 0, d = 0.0002;
    public static double f = -0.15;
    private final double ticks_in_degree = 144.0 / 180.0;
    public static double offset = -25;
    int armPos;
    double pid, targetArmAngle, ff, currentArmAngle, intakeArmPower, outtakeArmPower;

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        setupRobot();
        Pose2d startingPose;
        Pose2d nextPose;
        double xOffset = 3;
        double yOffset = -5;
        double outtakeOffset = 2;
        MecanumDrive drive;
        int reflect;
        int LCRNUM = 0;
        visionHandler = new VisionHandler();
        ConfigDashboard();
        gamepadSetValues();
        visionHandler.init(hardwareMap);

        telemetry.addData("Status", "Wait For start");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Wait For start complete");
        telemetry.update();

        lookForTeamElement();
//Set Reflect and things
        {
            if (color.equals(Alliance.RED)) {
                reflect = 1;
            } else {
                reflect = -1;
            }
            xOffset *= reflect;
            yOffset *= reflect;
            if (start == Side.BACKSTAGE) {
                xOffset *= -1;
            }
            switch (lcr) {
                case LEFT:
                    LCRNUM = -1 * reflect;
                    break;
                case CENTER:
                    LCRNUM = 0;
                    break;
                case RIGHT:
                    LCRNUM = 1 * reflect;
                    break;
            }
        }
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        while (intake_elbow.getCurrentPosition() > -18 && !isStopRequested())
        {
            SetIntakePIDTarget(-20);
        }
        if (start.equals(Side.BACKSTAGE)){      //BackstageSide
            {
                startingPose = new Pose2d(12 - (4*reflect), -64 * reflect, Math.toRadians(90 * reflect));
                drive = new MecanumDrive(hardwareMap, startingPose);
                switch (LCRNUM) {
                    case -1:
                        nextPose = new Pose2d(2 + xOffset, -30 * reflect + yOffset, Math.toRadians(90 * reflect));
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case 0:
                        nextPose = new Pose2d(12 + xOffset, -26 * reflect + yOffset, Math.toRadians(90 * reflect));
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case 1:
                        nextPose = new Pose2d(22 + xOffset, -30 * reflect + yOffset, Math.toRadians(90 * reflect));
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                }
            }
        } else {                                          //AudienceSide
            {
                startingPose = new Pose2d(-36 - (4*reflect), -64 * reflect, Math.toRadians(90 * reflect));
                drive = new MecanumDrive(hardwareMap, startingPose);
                switch (LCRNUM) {
                    case -1:
                        nextPose = new Pose2d(-46 + xOffset, -30 * reflect + yOffset, Math.toRadians(90 * reflect));
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case 0:
                        nextPose = new Pose2d(-36 + xOffset, -26 * reflect + yOffset, Math.toRadians(90 * reflect));
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case 1:
                        nextPose = new Pose2d(-26 + xOffset, -30 * reflect + yOffset, Math.toRadians(90 * reflect));
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                }
            }
            while (intake_elbow.getCurrentPosition() > -18 && !isStopRequested())
            {
                SetIntakePIDTarget(-20);
            }
        }
                                        //Go To Backboard!!
        if (start.equals(Side.BACKSTAGE)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(Math.toRadians(90*reflect))
                            .lineToY(-60*reflect)
                            .build());
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setTangent(0)
                            .turnTo(0)
                            .splineToConstantHeading(new Vector2d(40, -36*reflect), 0)
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
                            .splineToConstantHeading(new Vector2d(40, -36*reflect), 0)
                            .build());
        }
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(180))
                        .build());
        switch (lcr){
            case LEFT:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(new Vector2d(45, (-30*reflect)+outtakeOffset), 0)
                                .build());
                break;
            case CENTER:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(new Vector2d(45, (-36*reflect)+outtakeOffset), 0)
                                .build());
                break;
            case RIGHT:
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(new Vector2d(45, (-42*reflect)+outtakeOffset), 0)
                                .build());
                break;
        }
        while (outtake_elbow.getCurrentPosition() < 2300 && !isStopRequested())
        {
            SetOuttakePIDTarget(2500);
        }
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(180))
                        .lineToX(50)
                        .build());
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(180))
                        .lineToX(45)
                        .build());
        while (outtake_elbow.getCurrentPosition() > 50 && !isStopRequested())
        {
            SetOuttakePIDTarget(0);
        }
        if(park == Park.CORNER);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(50, -64*reflect), 0)
                        .build());
    }

    private void lookForTeamElement() throws InterruptedException {
        if(color.equals(Alliance.RED)){
            visionHandler.setRed();
        }else{
            visionHandler.setBlue();
        }
        visionHandler.setLeft();
        double left = visionHandler.read();
        visionHandler.setMiddle();
        double mid = visionHandler.read();
        if(left >= mid && left >= 0.5)
            lcr = Spike.LEFT;
        if(mid >= left && mid >= 0.5)
            lcr = Spike.CENTER;
        if(left <= 0.5 && mid <= 0.5)
            lcr = Spike.RIGHT;
    }

    private void gamepadSetValues() {
        while(!isStarted()){
            if (gamepad1.right_bumper){
                if(color == Alliance.RED){
                    color = Alliance.BLUE;
                } else {
                    color = Alliance.RED;
                }
            }
            if (gamepad1.left_bumper){
                if(park == Park.CORNER){
                    park = Park.STAGE;
                } else {
                    park = Park.CORNER;
                }
            }
            if (gamepad1.a){
                if(start == Side.AUDIENCE){
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
    private void setupRobot(){
        intake_elbow = hardwareMap.get(DcMotorEx.class, "intake_elbow");
        outtake_elbow = hardwareMap.get(DcMotorEx.class, "outtake_elbow");
        hang_arm = hardwareMap.get(DcMotorEx.class, "hang_arm");

        intake_grabber = hardwareMap.get(DcMotor.class, "intake_grabber");

        MotorInit(intake_elbow);
        MotorInit(hang_arm);
        MotorInit(outtake_elbow);

        outtake_elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        outtake_elbow.setDirection(DcMotorSimple.Direction.REVERSE);

        left_intake = hardwareMap.get(Servo.class, "left_intake");
        right_intake = hardwareMap.get(Servo.class, "right_intake");
        outtake_wrist = hardwareMap.get(Servo.class, "outtake_wrist");
        drone_launcher = hardwareMap.get(Servo.class, "drone_launcher");
    }
    private void MotorInit(DcMotorEx motor) {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    private void SetIntakePIDTarget(int target)
    {
        controller.setPID(p, i, d);
        armPos = intake_elbow.getCurrentPosition();
        pid = controller.calculate(armPos, target);
        targetArmAngle = Math.toRadians((target - offset) / ticks_in_degree);
        ff = Math.cos(targetArmAngle) * f;
        currentArmAngle = Math.toRadians((armPos - offset) / ticks_in_degree);

        intakeArmPower = pid + ff;

        intake_elbow.setPower(intakeArmPower);
        hang_arm.setPower(intakeArmPower);
    }
    private void SetOuttakePIDTarget(int target)
    {
        controller.setPID(p, i, d);
        armPos = outtake_elbow.getCurrentPosition();
        pid = controller.calculate(armPos, target);
        targetArmAngle = Math.toRadians((target - offset) / ticks_in_degree);
        ff = Math.cos(targetArmAngle) * f;
        currentArmAngle = Math.toRadians((armPos - offset) / ticks_in_degree);

        outtakeArmPower = pid + ff;

        outtake_elbow.setPower(outtakeArmPower);
    }
    private void Unfold()
    {
        //Find motor positions at unfold levels
    }
}
