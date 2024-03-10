package org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision;

import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ElementDetectionPipeline extends OpenCvPipeline {

    public enum Position {
        LEFT,
        CENTER,
        RIGHT
    }

    int width = VisionParameters.resX;
    int height = VisionParameters.resY;

    Position position = Position.CENTER;

    int minHue = 0;
    int maxHue = 255;
    int minSat = 0;
    int maxSat = 255;
    int minVal = 0;
    int maxVal = 255;

    int readX = 0;
    int readY = 0;

    int readHue = 0;
    int readSat = 0;
    int readVal = 0;

    void setColorParameters(
            int minHue,
            int maxHue,
            int minSat,
            int maxSat,
            int minVal,
            int maxVal
    ){
        this.minHue = minHue;
        this.maxHue = maxHue;
        this.minSat = minSat;
        this.maxSat = maxSat;
        this.minVal = minVal;
        this.maxVal = maxVal;
    }

    int leftStartX = 0;
    int leftStartY = 0;
    int leftEndX = 0;
    int leftEndY = 0;
    int rightStartX = 0;
    int rightStartY = 0;
    int rightEndX = 0;
    int rightEndY = 0;

    void setPositionParametersLeft(
            int xStart,
            int yStart,
            int xEnd,
            int yEnd
    ){
        this.leftStartX = xStart;
        this.leftEndX = xEnd;
        this.leftStartY = yStart;
        this.leftEndY = yEnd;
    }
    void setPositionParametersRight(
            int xStart,
            int yStart,
            int xEnd,
            int yEnd
    ){
        this.rightStartX = xStart;
        this.rightEndX = xEnd;
        this.rightStartY = yStart;
        this.rightEndY = yEnd;
    }



    public double amountLeft;
    public double amountRight;


    @Override
    public Mat processFrame(Mat image) {

        Mat converted = image.clone();
        Imgproc.cvtColor(image, converted, Imgproc.COLOR_BGR2HSV);

        double[] readPix = converted.get(readY, readX);
        readHue = (int) readPix[0];
        readSat = (int) readPix[1];
        readVal = (int) readPix[2];

        Mat convertedSubmatLeft = converted.submat(leftStartY, leftEndY, leftStartX, leftEndX);
        Mat threshLeft = convertedSubmatLeft.clone();

        Mat convertedSubmatRight = converted.submat(rightStartY, rightEndY, rightStartX, rightEndX);
        Mat threshRight = convertedSubmatRight.clone();

        Core.inRange(threshLeft, new Scalar(minHue, minSat, minVal), new Scalar(maxHue, maxSat, maxVal), threshLeft);
        amountLeft = Core.sumElems(threshLeft).val[0]/255./(leftEndX - leftStartX)/(leftEndY - leftStartY);

        Core.inRange(threshRight, new Scalar(minHue, minSat, minVal), new Scalar(maxHue, maxSat, maxVal), threshRight);
        amountRight = Core.sumElems(threshRight).val[0]/255./(rightEndX - rightStartX)/(rightEndY - rightStartY);


        Mat thresh4Left = new Mat();
        Imgproc.cvtColor(threshLeft, thresh4Left, Imgproc.COLOR_GRAY2BGRA);
        Mat submatLeft = image.submat(leftStartY, leftEndY, leftStartX, leftEndX);
        Core.add(submatLeft, thresh4Left, submatLeft);

        Mat thresh4Right = new Mat();
        Imgproc.cvtColor(threshRight, thresh4Right, Imgproc.COLOR_GRAY2BGRA);
        Mat submatRight = image.submat(rightStartY, rightEndY, rightStartX, rightEndX);
        Core.add(submatRight, thresh4Right, submatRight);

        converted.release();

        threshLeft.release();
        thresh4Left.release();

        threshRight.release();
        thresh4Right.release();

        if ((amountLeft > amountRight) && (amountLeft >= 0.12)) {
            position = Position.LEFT;
        } else if ((amountRight > amountLeft) && (amountRight >= 0.12)) {
            position = Position.CENTER;
        } else {
            position = Position.RIGHT;
        }


        Imgproc.rectangle(
                image,
                new Point(leftStartX, leftStartY),
                new Point(leftEndX, leftEndY),
                new Scalar(255, 255, 255),
                3
        );

        Imgproc.rectangle(
                image,
                new Point(rightStartX, rightStartY),
                new Point(rightEndX, rightEndY),
                new Scalar(255, 255, 255),
                3
        );

        Imgproc.circle(
                image,
                new Point(readX, readY),
                3,
                new Scalar(0, 0, 0),
                2
        );

        return image;
    }
    public Position GetAnalysis() {
        return position;
    }
}
