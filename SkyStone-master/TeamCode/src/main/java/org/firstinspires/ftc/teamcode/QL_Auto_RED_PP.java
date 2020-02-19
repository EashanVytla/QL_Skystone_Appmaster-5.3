package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odometry.ThreeWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.Pure_Pursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.Universal.Math.Vector2D;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.ArrayList;

@Autonomous(name = "QL_SkyStone_Red", group = "Competition")
@Disabled
public class QL_Auto_RED_PP extends OpMode {
    ElapsedTime mStateTime = new ElapsedTime();
    Mecanum_Drive drive;
    ThreeWheelTrackingLocalizer odos;
    ExpansionHubEx hub2;

    public enum State{
        INITIALIZE,
        DRIVE_TO_BLOCK,
        DRIVE_TO_FOUNDATION
    }
    State mRobotState = State.INITIALIZE;

    @Override
    public void init() {
        RevExtensions2.init();
        drive = new Mecanum_Drive(hardwareMap, telemetry);
        odos = new ThreeWheelTrackingLocalizer(hardwareMap);
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        mStateTime.startTime();
    }

    @Override
    public void start(){
        mStateTime.reset();
    }

    public void loop(){
        RevBulkData data = hub2.getBulkInputData();
        odos.dataUpdate(data);
        odos.update();

        ArrayList<CurvePoint> drivePoints = new ArrayList<>();

        switch (mRobotState){
            case INITIALIZE:
                newState(State.DRIVE_TO_BLOCK);
                break;
            case DRIVE_TO_BLOCK:
                if(odos.getPoseEstimate().vec().distTo(new Vector2d(24, 10)) < 3){
                    newState(State.DRIVE_TO_FOUNDATION);
                }else{
                    drivePoints.add(new CurvePoint(0.0, 0.0, 1.0, 1.0, 32));
                    drivePoints.add(new CurvePoint(24.0, 10.0, 1.0, 1.0, 32));
                    drive.followPath(drivePoints, 0.0, odos.getPoseEstimate());
                    telemetry.addData("Dist to: ", odos.getPoseEstimate().vec().distTo(new Vector2d(24, 24)));
                }
                break;
            case DRIVE_TO_FOUNDATION:
                if(odos.getPoseEstimate().vec().distTo(new Vector2d(-94, 60)) < 2){
                    newState(State.DRIVE_TO_FOUNDATION);
                }else{
                    drivePoints.add(new CurvePoint(24.0, 10.0, 1.0, 1.0, 32));
                    drivePoints.add(new CurvePoint(24.0,0.0, 1.0, 1.0, 32));
                    drivePoints.add(new CurvePoint(-60.0,-94.0, 1.0, 1.0, 32));
                    drive.followPath(drivePoints, Math.PI/2, odos.getPoseEstimate());
                }
                break;
        }

        drive.write();
    }

    public void newState(State state){
        mStateTime.reset();
        mRobotState = state;
    }
}
