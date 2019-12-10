package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Dead_Wheel;
import org.firstinspires.ftc.teamcode.LynxOptimizedI2cFactory;
import org.firstinspires.ftc.teamcode.MA3_Encoder;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

@Autonomous(name = "Dead Wheel Calibration", group = "Odometry")
public class Dead_Wheel_Calibration extends OpMode {
    Dead_Wheel leftWheel;
    Dead_Wheel rightWheel;
    Dead_Wheel strafeWheel;

    BNO055IMU imu;

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;

    public void init(){
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        leftWheel = new Dead_Wheel(new MA3_Encoder("a3", hardwareMap, 0.495));
        rightWheel = new Dead_Wheel(new MA3_Encoder("a4", hardwareMap, 1.365));
        strafeWheel = new Dead_Wheel(new MA3_Encoder("a1", hardwareMap, 2.464));
        rightWheel.getEncoder().reverse();
        strafeWheel.getEncoder().reverse();
        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();
        leftWheel.getEncoder().calibrate(data);
        rightWheel.getEncoder().calibrate(data2);
        strafeWheel.getEncoder().calibrate(data);

        leftWheel.setBehavior(1.5385 * 2 * 0.797, -0.319237); //1.5144 0.0361262
        rightWheel.setBehavior(1.5385 * 2 * 0.797, -0.319237); //1.5204 -0.00305571
        strafeWheel.setBehavior(1.53642 * 2 * 0.797, 0.0); //1.50608 -0.221642

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void loop(){
        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();

        leftWheel.update(data);
        rightWheel.update(data2);
        strafeWheel.update(data);

        telemetry.addData("Angle: ", imu.getAngularOrientation().firstAngle);
        telemetry.addData("Left Wheel Voltage: ", leftWheel.getEncoder().getPos());
        telemetry.addData("Right Wheel Voltage: ", rightWheel.getEncoder().getPos());
        telemetry.addData("Strafe Wheel Voltage: ", strafeWheel.getEncoder().getPos());
        telemetry.addData("Left Wheel Dist: ", leftWheel.getDistance());
        telemetry.addData("Left Wheel POC: ", leftWheel.getPOC());
        telemetry.addData("Right Wheel Dist: ", rightWheel.getDistance());
        telemetry.addData("Right Wheel POC: ", rightWheel.getPOC());
        telemetry.addData("Strafe Wheel Dist: ", strafeWheel.getDistance());
        telemetry.addData("Strafe Wheel POC: ", strafeWheel.getPOC());
    }
}