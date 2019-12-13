package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

@Autonomous(name = "Red Foundation", group = "Auto")
@Disabled
public class QL_Auto_Red_Foundation extends OpMode {
    Dead_Wheel leftWheel;
    Dead_Wheel rightWheel;
    Dead_Wheel strafeWheel;

    Mecanum_Drive drive;

    Flipper flip;

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;

    private long prev_time = System.currentTimeMillis();

    private enum State{
        STATE_INIT,
        STATE_DRIVE_TO_FOUNDATION,
        STATE_STRAFE,
        STATE_GRAB_FOUNDATION,
        STATE_PULL_FORWARD,
        STATE_TURN,
        STATE_PUSH_BACK,
        STATE_RELEASE,
        STATE_STRAFE_TO_EDGE,
        STATE_PARK,
        STATE_STOP
    }

    private State mRobotState = State.STATE_INIT;
    private ElapsedTime mStateTime = new ElapsedTime();

    private double memo = 0.0;

    public void newState(State s){
        mRobotState = s;
        mStateTime.reset();
    }

    public void init(){
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        leftWheel = new Dead_Wheel(new MA3_Encoder("a3", hardwareMap, 0.495));
        rightWheel = new Dead_Wheel(new MA3_Encoder("a4", hardwareMap, 1.365));
        strafeWheel = new Dead_Wheel(new MA3_Encoder("a1", hardwareMap, 2.464));
        drive = new Mecanum_Drive(hardwareMap, telemetry);
        flip = new Flipper(hardwareMap, telemetry);

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
    }

    public void start(){
        mStateTime.startTime();
    }

    public void loop(){
        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();

        flip.read();

        leftWheel.update(data);
        rightWheel.update(data2);
        strafeWheel.update(data);
        //drive.read(data);

        telemetry.addData("Refresh Rate: ", System.currentTimeMillis() - prev_time);
        prev_time = System.currentTimeMillis();

        switch (mRobotState){
            case STATE_INIT:
                //do init stuffs
                memo = getForwardDist();
                newState(State.STATE_DRIVE_TO_FOUNDATION);
                break;
            case STATE_DRIVE_TO_FOUNDATION:
                if (getForwardDist() - memo < -25){
                    memo = getStrafeDist();
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_STRAFE);
                }
                else{
                    drive.setPower(0.3, 0.0, 0.0);
                }
                telemetry.addData("Forward Dist: ", getForwardDist() - memo);
                break;
            case STATE_STRAFE:
                if (mStateTime.time() >= 0.5) {
                    if (getStrafeDist() - memo < -8) {
                        drive.setPower(0.0, 0.0, 0.0);
                        memo = getForwardDist();
                        newState(State.STATE_GRAB_FOUNDATION);
                    } else {
                        drive.setPower(0.0, -0.3, 0.0);
                    }
                    telemetry.addData("Strafe Dist: ", getStrafeDist() - memo);
                }
                break;
            case STATE_GRAB_FOUNDATION:
                //grab foundation logic
                flip.grabPlatform();
                memo = getForwardDist();
                if (mStateTime.time() > 2.0) {
                    newState(State.STATE_PULL_FORWARD);
                }
                break;
            case STATE_PULL_FORWARD:
                if (getForwardDist() - memo > 28){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_RELEASE);
                }
                else{
                    drive.setPower(-0.5, 0.0, 0.0);
                }
                telemetry.addData("Forward Dist: ", getForwardDist() - memo);
                break;
            case STATE_TURN:
                drive.read(data);
                if (drive.getOrientation().firstAngle < -Math.PI / 2){
                    drive.setPower(0.0, 0.0, 0.0);
                }
                else{
                    drive.setPower(0.0, 0.0, -0.3);
                }
                break;
            case STATE_PUSH_BACK:
                if (getForwardDist() - memo < -6){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_RELEASE);
                }
                else{
                    drive.setPower(0.3, 0.0, 0.0);
                }
                telemetry.addData("Forward Dist: ", getForwardDist() - memo);
                break;
            case STATE_RELEASE:
                //release foundation
                flip.resetPlatform();
                memo = getStrafeDist();
                if (mStateTime.time() >= 0.5){
                    newState(State.STATE_STRAFE_TO_EDGE);
                }
                break;
            case STATE_STRAFE_TO_EDGE:
                if (getStrafeDist() - memo > 30){
                    drive.setPower(0.0, 0.0, 0.0);
                    memo = getForwardDist();
                    newState(State.STATE_TURN);
                }
                else{
                    drive.setPower(0.0, 0.3, 0.0);
                }
                telemetry.addData("Strafe Dist: ", getStrafeDist() - memo);
                break;
            case STATE_PARK:
                if (getForwardDist() - memo > 36){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_STOP);
                }
                else{
                    drive.setPower(-0.3, 0.0, 0.0);
                }
                telemetry.addData("Forward Dist: ", getForwardDist() - memo);
                break;
            case STATE_STOP:
                telemetry.addData("Left Pos: ", leftWheel.getDistance());
                telemetry.addData("Right Pos: ", rightWheel.getDistance());
                telemetry.addData("Horizontal Pos: ", getStrafeDist() - memo);
                telemetry.addData("Tee Hee", "you know I had to do that");
                break;
        }
        flip.write();
        drive.write();
    }

    private double getForwardDist(){
        return leftWheel.getDistance() * (23 / 37.678);
    }

    private double getStrafeDist(){
        return strafeWheel.getDistance() * 7.0 / 17.536;
    }
}
