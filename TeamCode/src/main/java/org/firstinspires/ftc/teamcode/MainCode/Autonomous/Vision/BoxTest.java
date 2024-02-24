package org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Configure BoxTest", group="Linear Opmode")
public class BoxTest extends LinearOpMode {
    enum Parameter {
        HUE,
        SAT,
        VAL,
        POS,
        HUE2,
        SAT2,
        VAL2,
        POS2
    }


    Parameter parameterToModify = Parameter.HUE;

    boolean canSwitch = true;

    public static double MODIFY_SPEED = 8;
    public static double MOVE_SPEED = 8;

    public void runOpMode(){

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        FtcDashboard.getInstance().startCameraStream(camera, 30);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                camera.startStreaming(VisionParameters.resX,VisionParameters.resY, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode) {}
        });

        ElementDetectionPipeline elementDetectionPipeline1 = new ElementDetectionPipeline();
        ElementDetectionPipeline elementDetectionPipeline2 = new ElementDetectionPipeline();
        camera.setPipeline(elementDetectionPipeline1);

        waitForStart();

        elementDetectionPipeline1.setPositionParametersLeft(VisionParameters.leftStartX, VisionParameters.leftStartY, VisionParameters.leftEndX, VisionParameters.leftEndY);
        elementDetectionPipeline2.setPositionParametersLeft(VisionParameters.middleStartX, VisionParameters.middleStartY, VisionParameters.middleEndX, VisionParameters.middleEndY);

        while(opModeIsActive()){
            // Loading config with x, b, y, a
            {
                if (gamepad1.x) {
                    elementDetectionPipeline1.setColorParameters(
                            VisionParameters.blueHueMin,
                            VisionParameters.blueHueMax,
                            VisionParameters.blueSatMin,
                            VisionParameters.blueSatMax,
                            VisionParameters.blueValMin,
                            VisionParameters.blueValMax
                    );
                }
                if (gamepad1.a){
                    elementDetectionPipeline2.setColorParameters(
                            VisionParameters.blueHueMin,
                            VisionParameters.blueHueMax,
                            VisionParameters.blueSatMin,
                            VisionParameters.blueSatMax,
                            VisionParameters.blueValMin,
                            VisionParameters.blueValMax
                    );
                }
                if (gamepad1.b) {
                    elementDetectionPipeline1.setColorParameters(
                            VisionParameters.redHueMin,
                            VisionParameters.redHueMax,
                            VisionParameters.redSatMin,
                            VisionParameters.redSatMax,
                            VisionParameters.redValMin,
                            VisionParameters.redValMax
                    );
                }
                if (gamepad1.y){
                    elementDetectionPipeline2.setColorParameters(
                            VisionParameters.redHueMin,
                            VisionParameters.redHueMax,
                            VisionParameters.redSatMin,
                            VisionParameters.redSatMax,
                            VisionParameters.redValMin,
                            VisionParameters.redValMax
                    );
                }
                if (gamepad1.right_bumper){
                    camera.setPipeline(elementDetectionPipeline2);
                }
                if (gamepad1.left_bumper){
                    camera.setPipeline(elementDetectionPipeline1);
                }
            }

            // Change parameter to edit with dpad
            {
                if(canSwitch){
                    if(gamepad1.dpad_right){
                        switch(parameterToModify){
                            case HUE:
                                parameterToModify = Parameter.SAT;
                                break;
                            case SAT:
                                parameterToModify = Parameter.VAL;
                                break;
                            case VAL:
                                parameterToModify = Parameter.POS;
                                break;
                            case POS:
                                parameterToModify = Parameter.HUE2;
                                break;
                            case HUE2:
                                parameterToModify = Parameter.SAT2;
                                break;
                            case SAT2:
                                parameterToModify = Parameter.VAL2;
                                break;
                            case VAL2:
                                parameterToModify = Parameter.POS2;
                                break;
                            case POS2:
                                parameterToModify = Parameter.HUE;
                                break;
                        }
                    }
                }
                canSwitch = !gamepad1.dpad_left && !gamepad1.dpad_right;
            }

            // Modify parameters with sticks
            {
                switch(parameterToModify){
                    case HUE:
                        elementDetectionPipeline1.minHue -= MODIFY_SPEED*gamepad1.left_stick_y;
                        elementDetectionPipeline1.maxHue -= MODIFY_SPEED*gamepad1.right_stick_y;
                        break;
                    case SAT:
                        elementDetectionPipeline1.minSat -= MODIFY_SPEED*gamepad1.left_stick_y;
                        elementDetectionPipeline1.maxSat -= MODIFY_SPEED*gamepad1.right_stick_y;
                        break;
                    case VAL:
                        elementDetectionPipeline1.minVal -= MODIFY_SPEED*gamepad1.left_stick_y;
                        elementDetectionPipeline1.maxVal -= MODIFY_SPEED*gamepad1.right_stick_y;
                        break;
                    case POS:
                        elementDetectionPipeline1.leftStartX += MOVE_SPEED*gamepad1.left_stick_x;
                        elementDetectionPipeline1.leftStartY += MOVE_SPEED*gamepad1.left_stick_y;
                        elementDetectionPipeline1.leftEndX += MOVE_SPEED*gamepad1.right_stick_x;
                        elementDetectionPipeline1.leftEndY += MOVE_SPEED*gamepad1.right_stick_y;
                        break;
                    case HUE2:
                        elementDetectionPipeline2.minHue -= MODIFY_SPEED*gamepad1.left_stick_y;
                        elementDetectionPipeline2.maxHue -= MODIFY_SPEED*gamepad1.right_stick_y;
                        break;
                    case SAT2:
                        elementDetectionPipeline2.minSat -= MODIFY_SPEED*gamepad1.left_stick_y;
                        elementDetectionPipeline2.maxSat -= MODIFY_SPEED*gamepad1.right_stick_y;
                        break;
                    case VAL2:
                        elementDetectionPipeline2.minVal -= MODIFY_SPEED*gamepad1.left_stick_y;
                        elementDetectionPipeline2.maxVal -= MODIFY_SPEED*gamepad1.right_stick_y;
                        break;
                    case POS2:
                        elementDetectionPipeline2.leftStartX += MOVE_SPEED*gamepad1.left_stick_x;
                        elementDetectionPipeline2.leftStartY += MOVE_SPEED*gamepad1.left_stick_y;
                        elementDetectionPipeline2.leftEndX += MOVE_SPEED*gamepad1.right_stick_x;
                        elementDetectionPipeline2.leftEndY += MOVE_SPEED*gamepad1.right_stick_y;
                        break;
                }
            }

            // Logging parameters to telemetry
            {
                dashboardTelemetry.addData("Modifying parameter", parameterToModify);
                dashboardTelemetry.addData("Amount", elementDetectionPipeline1.amountLeft);
                dashboardTelemetry.addData("Amount2", elementDetectionPipeline2.amountLeft);
                dashboardTelemetry.addData("hueMin", elementDetectionPipeline1.minHue);
                dashboardTelemetry.addData("hueMin2", elementDetectionPipeline2.minHue);
                dashboardTelemetry.addData("hueMax", elementDetectionPipeline1.maxHue);
                dashboardTelemetry.addData("hueMax2", elementDetectionPipeline2.maxHue);
                dashboardTelemetry.addData("satMin", elementDetectionPipeline1.minSat);
                dashboardTelemetry.addData("satMax2", elementDetectionPipeline2.maxSat);
                dashboardTelemetry.addData("valMin", elementDetectionPipeline1.minVal);
                dashboardTelemetry.addData("valMin2", elementDetectionPipeline2.minVal);
                dashboardTelemetry.addData("valMax", elementDetectionPipeline1.maxVal);
                dashboardTelemetry.addData("valMax2", elementDetectionPipeline2.maxVal);
                dashboardTelemetry.addData("startX", elementDetectionPipeline1.leftStartX);
                dashboardTelemetry.addData("startX2", elementDetectionPipeline2.leftStartX);
                dashboardTelemetry.addData("startY", elementDetectionPipeline1.leftStartY);
                dashboardTelemetry.addData("startY2", elementDetectionPipeline2.leftStartY);
                dashboardTelemetry.addData("endX", elementDetectionPipeline1.leftEndX);
                dashboardTelemetry.addData("endX2", elementDetectionPipeline2.leftEndX);
                dashboardTelemetry.update();
            }

            sleep(100);
        }


    }
}
