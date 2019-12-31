package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odometry.Dead_Wheel;
import org.firstinspires.ftc.teamcode.Odometry.MA3_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.SRX_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.SRX_Odometry;
import org.firstinspires.ftc.teamcode.Odometry.SRX_Three_Wheel_Localizer;
import org.firstinspires.ftc.teamcode.Odometry.ThreeWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.Universal.Math.Pose;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

@Autonomous(name = "PID_tuner", group = "Odometry")
public class PID_tuner extends OpMode {
    ThreeTrackingWheelLocalizer odos;
    SRX_Three_Wheel_Localizer localizer;
    ElapsedTime time;
    Mecanum_Drive drive;

    public void init(){
        localizer = new SRX_Three_Wheel_Localizer(new SRX_Encoder("intake_left", hardwareMap), new SRX_Encoder("intake_right", hardwareMap), new SRX_Encoder("lift_2", hardwareMap), hardwareMap, telemetry);
        odos = new ThreeWheelTrackingLocalizer(this.hardwareMap);
        time = new ElapsedTime();
        drive = new Mecanum_Drive(hardwareMap, telemetry);
        newState(State.turn);
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
        mRobotState = state;
        time.reset();
    }

    public void loop(){
        if(mRobotState == State.turn){
            if(time.time() >= 6.0){
                drive.setPower(0.0,0.0,0.0);
                newState(State.Diagonal1);
            }else {
                localizer.GoTo(new Pose2d(0.0, 0.0, Math.PI / 2));
            }
        }else if(mRobotState == State.Diagonal1){
            if(time.time() >= 6.0){
                drive.setPower(0.0,0.0,0.0);
                newState(State.Diagonal2);
            }else{
                localizer.GoTo(new Pose2d(24.0, 24.0, Math.PI / 2));
            }
        }else if(mRobotState == State.Diagonal2){
            if(time.time() >= 6.0){
                drive.setPower(0.0,0.0,0.0);
                newState(State.Diagonal3);
            }else{
                localizer.GoTo(new Pose2d(-24.0, -24.0, Math.PI / 2));
            }
        }else if(mRobotState == State.Diagonal3){
            if(time.time() >= 6.0){
                drive.setPower(0.0,0.0,0.0);
                newState(State.Diagonal4);
            }else{
                localizer.GoTo(new Pose2d(-24.0, 24.0, Math.PI / 2));
            }
        }else if(mRobotState == State.Diagonal4){
            if(time.time() >= 6.0){
                drive.setPower(0.0,0.0,0.0);
                newState(State.Strafe);
            }else{
                localizer.GoTo(new Pose2d(24.0, -24.0, Math.PI / 2));
            }
        }else if(mRobotState == State.Strafe){
            if(time.time() >= 6.0){
                drive.setPower(0.0,0.0,0.0);
                newState(State.Forward);
            }else{
                localizer.GoTo(new Pose2d(-24.0, 24.0, Math.PI / 2));
            }
        }else if(mRobotState == State.Forward){
            if(time.time() >= 6.0){
                drive.setPower(0.0,0.0,0.0);
            }else{
                localizer.GoTo(new Pose2d(24.0, 24.0, Math.PI / 2));
            }
        }


        telemetry.addData("POSE",odos.getPoseEstimate());
        localizer.OutputRaw();

        telemetry.addData("Angle:", localizer.getTrackWidthAngle());
        odos.update();
    }
}
