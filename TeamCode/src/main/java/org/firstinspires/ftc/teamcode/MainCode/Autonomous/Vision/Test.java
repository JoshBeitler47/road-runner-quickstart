package org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test OpenCV", group="Linear Opmode")
public class Test extends LinearOpMode {

    public void runOpMode(){
        VisionHandler visionHandler = new VisionHandler();
        visionHandler.init(hardwareMap);

        waitForStart();

        try {
            visionHandler.setBlue();
            visionHandler.setLeft();
            telemetry.addData("Blue Left: ", visionHandler.read());
            visionHandler.setMiddle();
            telemetry.addData("Blue Middle: ", visionHandler.read());

            visionHandler.setRed();
            visionHandler.setLeft();
            telemetry.addData("Red Left: ", visionHandler.read());
            visionHandler.setMiddle();
            telemetry.addData("Red Middle: ", visionHandler.read());

            telemetry.update();

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        while(opModeIsActive()){

        }
    }
}
