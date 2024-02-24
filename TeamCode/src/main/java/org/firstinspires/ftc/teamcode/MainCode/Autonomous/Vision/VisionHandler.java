package org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class VisionHandler {
    public ElementDetectionPipeline pipeline = new ElementDetectionPipeline();
    OpenCvCamera camera;
    public boolean ready = false;
    public void init(HardwareMap hardwareMap){

        // I have no clue why this works
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                camera.startStreaming(VisionParameters.resX, VisionParameters.resY, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode) {}
        });
    }

    public void setLeft(){
        pipeline.setPositionParametersLeft(
                VisionParameters.leftStartX,
                VisionParameters.leftStartY,
                VisionParameters.leftEndX,
                VisionParameters.leftEndY
        );
    }
    public void setMiddle(){
        pipeline.setPositionParametersRight(
                VisionParameters.middleStartX,
                VisionParameters.middleStartY,
                VisionParameters.middleEndX,
                VisionParameters.middleEndY
        );
    }
    public void setRed(){
        pipeline.setColorParameters(
                VisionParameters.redHueMin,
                VisionParameters.redHueMax,
                VisionParameters.redSatMin,
                VisionParameters.redSatMax,
                VisionParameters.redValMin,
                VisionParameters.redValMax
        );
    }
    public void setBlue(){
        pipeline.setColorParameters(
                VisionParameters.blueHueMin,
                VisionParameters.blueHueMax,
                VisionParameters.blueSatMin,
                VisionParameters.blueSatMax,
                VisionParameters.blueValMin,
                VisionParameters.blueValMax
        );
    }
    public double read() throws InterruptedException {

        camera.startStreaming(VisionParameters.resX, VisionParameters.resY, OpenCvCameraRotation.UPRIGHT);
        Thread.sleep(VisionParameters.readTime);
        double amt = pipeline.amountLeft;
        camera.stopStreaming();
        return amt;
    }
}
