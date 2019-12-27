package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Dead_Wheel;
import org.firstinspires.ftc.teamcode.LynxOptimizedI2cFactory;
import org.firstinspires.ftc.teamcode.MA3_Encoder;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

@Autonomous(name = "PID_tuner", group = "Odometry")
public class PID_tuner extends OpMode {
    Dead_Wheel leftWheel;
    Dead_Wheel rightWheel;
    Dead_Wheel strafeWheel;
    Mecanum_Drive drive;
    PID pid;

    double dt = 0.0;
    long prev_time = System.currentTimeMillis();
    double power = 0.0;
    double headingerror = 0.0;
    double prevheading = 0.0;
    double integralError = 0.0;
    double angle = 0.0;
    double targetangle = 0.0;

    final double TRACK_WIDTH = 14.85006069;

    /*
    final double kp = 0.3;
    final double ki = 0.0;
    final double kd = 0.02; //0.02

    final double kpr = 1.0;
    final double kir = 0.0;
    final double kdr = 0.0;

     */

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
        pid = new PID(hardwareMap, telemetry);

        drive = new Mecanum_Drive(hardwareMap, telemetry);

        leftWheel.setBehavior(1.5385 * 2 * 0.797, -0.319237); //1.5144 0.0361262
        rightWheel.setBehavior(1.5385 * 2 * 0.797, -0.319237); //1.5204 -0.00305571
        strafeWheel.setBehavior(1.53642 * 2 * 0.797, 0.0); //1.50608 -0.221642

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        pid.reset();

    }

    public void loop(){
        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();

        drive.straight(-35, -getForwardDist(), telemetry);

        /*
        dt = (System.currentTimeMillis() - prev_time);
        prev_time = System.currentTimeMillis();
        angle = drive.angleWrap(drive.getExternalHeading());
        headingerror =  targetangle - angle;

        if (Math.abs(headingerror) > Math.toRadians(180.0)){
            if (headingerror > 0) {
                headingerror = -((Math.PI * 2) - Math.abs(headingerror));
            }
            else{
                headingerror = ((Math.PI * 2) - Math.abs(headingerror));
            }
        }

        double prop = headingerror * kpr;
        double integral = integralError * kir;
        double deriv = (headingerror - prevheading) * kdr / dt;

        double power = prop + integral + deriv;
        if (Math.abs(power) < 0.3) {
            integralError += headingerror;
        }
        prevheading = headingerror;

        pid.setTargetPosition(75);



        drive.setPower(-(pid.update(getForwardDist())), 0.0, Range.clip(power, -1.0, 1.0));

         */

        leftWheel.update(data);
        rightWheel.update(data2);

        drive.write();
        //telemetry.addData("Angle: ", imu.getAngularOrientation().firstAngle);
        //telemetry.addData("Left Wheel Voltage: ", leftWheel.getEncoder().getPos());
        //telemetry.addData("Right Wheel Voltage: ", rightWheel.getEncoder().getPos());
        //telemetry.addData("Strafe Wheel Voltage: ", strafeWheel.getEncoder().getPos());
        telemetry.addData("Left Wheel Dist: ", getLeftDist());
        //telemetry.addData("Left Wheel POC: ", leftWheel.getPOC());
        telemetry.addData("Right Wheel Dist: ", getRightDist());
        telemetry.addData("Foward Dist: ", rightWheel.getDistance() * (23 / 37.678));
        telemetry.addData("power", power);
        //telemetry.addData("Right Wheel POC: ", rightWheel.getPOC());
        //telemetry.addData("Strafe Wheel Dist: ", strafeWheel.getDistance());
        //telemetry.addData("Strafe Wheel POC: ", strafeWheel.getPOC());
    }

    private double getLeftDist(){
        return leftWheel.getDistance()* (23 / 37.678);
    }

    private double getRightDist(){
        return rightWheel.getDistance() * (23 / 37.678);
    }

    private double getForwardDist(){
        return rightWheel.getDistance() * (23 / 37.678);
    }
}
