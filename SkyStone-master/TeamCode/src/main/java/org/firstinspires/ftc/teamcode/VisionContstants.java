package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Rect;
import org.opencv.core.Scalar;

@Config
public class VisionContstants {
    public static final int IMAGE_WIDTH = 320;
    public static final int IMAGE_HEIGHT = 240;
    public static Scalar HSV_LOW = new Scalar(10, 20, 70);
    public static Scalar HSV_HIGH = new Scalar(30, 255, 255);
    public static Rect rectCrop0 = new Rect(240, 120, 60, 45);
    public static Rect rectCrop1 = new Rect(140, 120, 60, 45);
    public static Rect rectCrop2 = new Rect(60, 120, 60, 45);
}