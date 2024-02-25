package org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionParameters {
    public static int readTime = 500;
    public static int resX = 320;
    public static int resY = 176;

    public static int blueHueMax = 186;
    public static int blueHueMin = -8;
    public static int blueSatMax = 241;
    public static int blueSatMin = 127;
    public static int blueValMax = 263;
    public static int blueValMin = 134;

    public static int redHueMax = 218;
    public static int redHueMin = 78;
    public static int redSatMax = 255;
    public static int redSatMin = 55;
    public static int redValMax = 255;
    public static int redValMin = 10;

    public static int leftStartX = 8;
    public static int leftStartY = 42;
    public static int leftEndX = 105;
    public static int leftEndY = 110;

    public static int middleStartX = 167;
    public static int middleStartY = 39;
    public static int middleEndX = 251;
    public static int middleEndY = 93;
}
