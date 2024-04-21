package org.firstinspires.ftc.teamcode.MainCode.CommunityDay;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.ThreeDeadWheelLocalizer;
@Config
@Autonomous(name="CDMain", group="Linear Opmode")

public final class CDMain extends LinearOpMode {
    public MecanumDrive drive;
    public Pose2d currentPose;
    public int sq = 24;
    public int neg = 1;
    public double currentHeading = 90;

    public void runOpMode() throws InterruptedException {
        double xStart = -60;
        double yStart = -60;
        double headingStart = currentHeading;

        currentPose = new Pose2d(xStart, yStart, headingStart);
        drive = new MecanumDrive(hardwareMap, currentPose);

        waitForStart();


///////
        //Type Your Code Below!
        //driveForward(1);
        //driveBackward(2);
        //turnRight();
        //turnLeft();







//////
    }

    public void driveForward(int amnt) {
        if ((currentHeading == 90) || (currentHeading == 270)) {
            if (currentHeading == 270) {
                neg = -1;
            }
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToYConstantHeading(currentPose.position.y + (sq * amnt * neg))
                            .build());
            neg = 1;
        } else if ((currentHeading == 180) || (currentHeading == 0)) {
            if (currentHeading == 180) {
                neg = -1;
            }
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToXConstantHeading(currentPose.position.x + (sq * amnt * neg))
                            .build());
            neg = 1;
        }
    }
    public void driveBackward(int amnt) {
        if ((currentHeading == 90) || (currentHeading == 270)) {
            if (currentHeading == 90) {
                neg = -1;
            }
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToYConstantHeading(currentPose.position.y + (sq * amnt * neg))
                            .build());
            neg = 1;
        } else if ((currentHeading == 180) || (currentHeading == 0)) {
            if (currentHeading == 0) {
                neg = -1;
            }
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToXConstantHeading(currentPose.position.x + (sq * amnt * neg))
                            .build());
            neg = 1;
        }
    }
    public void turnRight() {
        if (currentHeading == 0){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(270)
                            .build());
            currentHeading = 270;
        } else if (currentHeading == 90){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(0)
                            .build());
            currentHeading = 0;
        } else if (currentHeading == 180){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(90)
                            .build());
            currentHeading = 90;
        } else if (currentHeading == 270){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(180)
                            .build());
            currentHeading = 180;
        }

    }
    public void turnLeft(){
        if (currentHeading == 0){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(90)
                            .build());
            currentHeading = 90;
        } else if (currentHeading == 90){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(180)
                            .build());
            currentHeading = 180;
        } else if (currentHeading == 180){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(270)
                            .build());
            currentHeading = 270;
        } else if (currentHeading == 270){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(0)
                            .build());
            currentHeading = 0;
        }

    }
}
