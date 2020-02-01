package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odometry.Dead_Wheel;
import org.firstinspires.ftc.teamcode.Odometry.MA3_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.SRX_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.SRX_Odometry;
import org.firstinspires.ftc.teamcode.Odometry.SRX_Three_Wheel_Localizer;
import org.firstinspires.ftc.teamcode.Odometry.ThreeWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.Pure_Pursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.Universal.Math.Pose;
import org.opencv.core.Point;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;
import org.firstinspires.ftc.teamcode.Pure_Pursuit.CurvePoint;

import java.util.ArrayList;

@Autonomous(name = "Pure Pursuit Tuner", group = "Odometry")
public class PID_tuner extends OpMode {
    ThreeWheelTrackingLocalizer odos;
    SRX_Three_Wheel_Localizer localizer;
    ElapsedTime time;
    Mecanum_Drive drive;
    Intake intake;
    Flipper flip;
    ArrayList<CurvePoint> drivePoints = new ArrayList<>();
    ExpansionHubEx hub2;

    public void init(){
        localizer = new SRX_Three_Wheel_Localizer(new SRX_Encoder("intake_left", hardwareMap), new SRX_Encoder("intake_right", hardwareMap), new SRX_Encoder("lift_2", hardwareMap), hardwareMap, telemetry);
        odos = new ThreeWheelTrackingLocalizer(hardwareMap);
        time = new ElapsedTime();
        drive = new Mecanum_Drive(hardwareMap, telemetry);
        newState(State.turn);
        intake = new Intake(hardwareMap);
        flip = new Flipper(hardwareMap, telemetry);
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        drivePoints.add(new CurvePoint(0.0, 0.0, 1.0, 0.5, 10));
        drivePoints.add(new CurvePoint(24.0, 0.0, 1.0, 0.5, 10));
        drivePoints.add(new CurvePoint(24.0,24.0, 0.3, 0.5, 10));
        //drivePoints.add(new CurvePoint(-94.0,-60.0, 0.3, 0.5, 32));
    }

    private enum State{
        turn,
        Diagonal1,
        Diagonal2,
        Diagonal3,
        Diagonal4,
        Strafe,
        Forward,
        Strafe2,
        Back
    }

    State mRobotState = State.turn;

    @Override
    public void start(){
        time.startTime();
    }

    public void newState(State state){
        localizer.resetfirst();
        time.reset();
        mRobotState = state;
    }

    public void loop(){
        RevBulkData data = hub2.getBulkInputData();
        odos.dataUpdate(data);
        odos.update();

        drive.followPath(drivePoints, 0.0, odos.getEstimatedPose());
        drive.write();
    }
}
