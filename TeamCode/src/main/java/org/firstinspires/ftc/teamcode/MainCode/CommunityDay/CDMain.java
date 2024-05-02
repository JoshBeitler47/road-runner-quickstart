package org.firstinspires.ftc.teamcode.MainCode.CommunityDay;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
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
    public int sq = 12;

    public void runOpMode() throws InterruptedException {
        double xStart = 60;
        double yStart = -60;
        double headingStart = Math.toRadians(90);

        currentPose = new Pose2d(xStart, yStart, headingStart);
        drive = new MecanumDrive(hardwareMap, currentPose);

        waitForStart();


///////
        //Type Your Code Below!
        Forward(2);
        Right(1);
        Backward(1);
        Forward(2);
        Left(1);

//////
    }

    public void Forward(int amnt) {
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToYConstantHeading(drive.pose.position.y + (sq * amnt))

                        .build());
    }

    public void Backward(int amnt) {
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToConstantHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y - (sq * amnt)))
                        .build());
    }

    public void Right(int amnt) {
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToConstantHeading(new Vector2d(drive.pose.position.x + (sq * amnt), drive.pose.position.y))
                        .build());
    }

    public void Left(int amnt) {
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToConstantHeading(new Vector2d(drive.pose.position.x - (sq * amnt), drive.pose.position.y))
                        .build());
    }
}
