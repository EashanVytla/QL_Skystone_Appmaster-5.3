package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

@Config
public class DriveConstants {
    public static double kV = 0.0;
    public static double kA = 0.0;
    public static double kStatic = 0.0;

    public static double TRACK_WIDTH = 9.0;

    public static PIDCoefficients TRANSLATIONAL_PID_X;
    public static PIDCoefficients TRANSLATIONAL_PID_Y;
    public static PIDCoefficients HEADING_PID;

    public static DriveConstraints constraints = new DriveConstraints(79.2, 30, 30, Math.PI, Math.PI, Math.PI);
}
