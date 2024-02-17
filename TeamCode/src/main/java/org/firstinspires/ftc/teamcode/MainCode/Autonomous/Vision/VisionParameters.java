package org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionParameters {
    public static int readTime = 500;
    public static int resX = 320;
    public static int resY = 176;

    public static int blueHueMax = 33;
    public static int blueHueMin = -38;
    public static int blueSatMax = 255;
    public static int blueSatMin = 95;
    public static int blueValMax = 215;
    public static int blueValMin = 0;

    public static int redHueMax = 218;
    public static int redHueMin = 78;
    public static int redSatMax = 255;
    public static int redSatMin = 55;
    public static int redValMax = 255;
    public static int redValMin = 10;

    public static int leftStartX = 16;
    public static int leftStartY = 50;
    public static int leftEndX = 90;
    public static int leftEndY = 129;

    public static int middleStartX = 190;
    public static int middleStartY = 20;
    public static int middleEndX = 239;
    public static int middleEndY = 77;
}
