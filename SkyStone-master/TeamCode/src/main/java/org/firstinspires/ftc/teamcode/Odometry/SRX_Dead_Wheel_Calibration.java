package org.firstinspires.ftc.teamcode.Odometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Universal.Math.Pose;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

@Autonomous(name = "Dead Wheel Calibration", group = "Odometry")
public class SRX_Dead_Wheel_Calibration extends OpMode {
    SRX_Three_Wheel_Localizer localizer;
    private ExpansionHubEx hub;
    Pose2d currentPos = new Pose2d(0.0,0.0,0.0);
    SRX_Odometry SRX;
    ThreeTrackingWheelLocalizer odos;

    public void init() {
        RevExtensions2.init();
        localizer = new SRX_Three_Wheel_Localizer(new SRX_Encoder("intake_left", hardwareMap), new SRX_Encoder("intake_right", hardwareMap), new SRX_Encoder("lift_2", hardwareMap), hardwareMap, telemetry);
        SRX = new SRX_Odometry(new SRX_Encoder("intake_right", hardwareMap), telemetry, hardwareMap);
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        odos = new ThreeWheelTrackingLocalizer(this.hardwareMap);
    }

    public void loop() {
        odos.update();
        telemetry.addData("POSE",odos.getPoseEstimate());
        currentPos = localizer.ktrack();
        telemetry.addData("ANGLE: ", localizer.getTrackWidthAngle());
        localizer.OutputRaw();
        telemetry.update();
    }
}
