package org.firstinspires.ftc.teamcode;

import android.service.carrier.CarrierService;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

public class QL_Auto_Skystone_PIDTURN extends OpMode {
    Dead_Wheel leftWheel;
    Dead_Wheel rightWheel;
    Dead_Wheel strafeWheel;

    Mecanum_Drive drive;
    Double angle;
    Double strafe;
    Double drivein;

    Flipper flip;
    Intake intake;
    Integer SkyStonePos;

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;
    ElapsedTime mStateTime = new ElapsedTime();
    Double memo = 0.0;

    static final Double turnCone = 5.0;

    private long prev_time = System.currentTimeMillis();

    private enum State{
        STATE_DETECT,
        STATE_FORWARD,
        STATE_STRAFE,
        STATE_TURN,
        STATE_INTAKE,
        STATE_BACK,
        STATE_TURN2,
        STATE_CROSS,
        STATE_TURN3,
        STATE_GRAB_FOUNDATION,
        STATE_PULL_FORWARD,
        STATE_TURN4,
        STATE_PUSH_FOUNDATION,
        STATE_RELEASE,
        STATE_DRIVE_TO_FOUNDATION,
        STATE_REALLIGN,
        STATE_RETURN,
        BACK_FROM_FOUNDATION,
        STATE_STOP
    }

    State mRobotState = State.STATE_DETECT;

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
        flip.initialize();

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

        flip = new Flipper(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
    }

    public void start(){
        flip.start();
    }

    public void loop() {
        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();

        leftWheel.update(data);
        rightWheel.update(data2);
        strafeWheel.update(data);

        //drive.read(data);

        telemetry.addData("Refresh Rate: ", System.currentTimeMillis() - prev_time);
        prev_time = System.currentTimeMillis();

        switch(mRobotState) {
            case STATE_DETECT:
                SkyStonePos = 2;
                if(SkyStonePos == 0){
                    angle = 0.5;
                    strafe = -12.0;
                    drivein = 12.0;
                }else if(SkyStonePos == 1){
                    angle = 0.0;
                    strafe = -18.0;
                    drivein = 15.0;
                }else if(SkyStonePos == 2){
                    angle = 0.0;
                    strafe = -10.0;
                    drivein = 17.0;
                }
                memo = getForwardDist();
                newState(State.STATE_FORWARD);
                break;
            case STATE_FORWARD:
                if (getForwardDist() - memo >= 16 * (37.5 / 43)) {
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_STRAFE);
                } else {
                    drive.setPower(-0.3, 0.0, 0.0);
                }
                break;
            case STATE_STRAFE:
                if (getStrafeDist() <= strafe * (17 / 28.5)) {
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_TURN);
                } else {
                    drive.setPower(0.0, -0.3, 0.0);
                }
                break;
            case STATE_TURN:
                if (getangle() >= angle) {
                    drive.setPower(0.0, 0.0, 0.0);
                    memo = getForwardDist();
                    newState(State.STATE_INTAKE);
                } else {
                    drive.setPower(0.0, 0.0, 0.2);
                }
                drive.read(data);
                break;
            case STATE_INTAKE:
                intake.setPower(0.3);
                if (getForwardDist() - memo >= drivein * (37.5 / 43)) {
                    drive.setPower(0.0, 0.0, 0.0);
                    memo = getForwardDist();
                    newState(State.STATE_BACK);
                } else if(mStateTime.time() >= 3.0){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_BACK);
                } else {
                    drive.setPower(-0.2, 0.0, 0.0);
                }
                break;
            case STATE_BACK:
                if (getForwardDist() - memo <= -(drivein - 4) * (37.5 / 43)) {
                    drive.setPower(0.0, 0.0, 0.0);
                    memo = getForwardDist();
                    intake.setPower(0.0);
                    newState(State.STATE_TURN2);
                } else {
                    drive.setPower(0.2, 0.0, 0.0);
                }
                break;
            case STATE_TURN2:
                if (Math.abs(getangle() - (Math.PI / 2)) < Math.toRadians(turnCone)) {
                    drive.setPower(0,0,0);
                    memo = getForwardDist();
                    newState(State.STATE_CROSS);
                } else {
                    drive.targetTurn(Math.PI/2);
                    mStateTime.reset();
                }
                drive.read(data);
                break;
            case STATE_CROSS:
                if (getForwardDist() - memo <= -80 * (37.5 / 43)) {
                    drive.setPower(0,0,0);
                    newState(State.STATE_TURN3);
                } else {
                    drive.setPower(0.4, 0.0, 0.0);
                    mStateTime.reset();
                }
                break;
            case STATE_TURN3:
                if(mStateTime.time() >= 0.5){
                    if (getangle() >= Math.PI) {
                        intake.setPower(0.0);
                        if(mStateTime.time() >= 0.55){
                            memo = getForwardDist();
                            newState(State.STATE_DRIVE_TO_FOUNDATION);
                        }
                    } else {
                        drive.targetTurn(Math.PI);
                        mStateTime.reset();
                    }
                    drive.read(data);
                }
                break;
            case STATE_DRIVE_TO_FOUNDATION:
                if (mStateTime.time() >= 0.5) {
                    if (getForwardDist() - memo < -8){
                        memo = getForwardDist();
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.BACK_FROM_FOUNDATION);
                    }
                    else{
                        drive.setPower(0.3, 0.0, 0.0);
                    }
                    telemetry.addData("Forward Dist: ", getForwardDist() - memo);
                }
                break;
            //case DUMP_BLock
            case BACK_FROM_FOUNDATION:
                if (mStateTime.time() >= 0.5) {
                    if (getForwardDist() - memo > 10) {
                        drive.setPower(0.0, 0.0, 0.0);
                        if (mStateTime.time() >= 0.5) {
                            newState(State.STATE_TURN4);
                        }
                    } else {
                        drive.setPower(-0.3, 0.0, 0.0);
                    }
                }
                telemetry.addData("Forward Dist: ", getForwardDist() - memo);
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
                if (getForwardDist() - memo > 14){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_TURN4);
                }
                else{
                    drive.setPower(-0.5, 0.0, 0.0);
                }
                telemetry.addData("Forward Dist: ", getForwardDist() - memo);
                break;
            case STATE_TURN4:
                if(mStateTime.time() >= 0.5){
                    drive.read(data);
                    if (getangle() <= Math.PI/2){
                        if(mStateTime.time() >= 0.55){
                            newState(State.STATE_STOP);
                        }
                        //newState(State.STATE_RETURN);
                    }
                    else{
                        drive.targetTurn(Math.PI/2);
                        mStateTime.reset();
                    }
                }
                break;
            case STATE_RELEASE:
                //release foundation
                flip.resetPlatform();
                memo = getForwardDist();
                newState(State.STATE_PUSH_FOUNDATION);
                break;
            case STATE_PUSH_FOUNDATION:
                if (getForwardDist() - memo <= -12 * (37.5 / 43)) {
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_REALLIGN);
                } else {
                    drive.setPower(0.4, 0.0, 0.0);
                }
                break;
            case STATE_REALLIGN:
                drive.read(data);
                if (getangle() >= Math.PI/2){
                    if(mStateTime.time() >= 0.55){
                        memo = getForwardDist();
                        newState(State.STATE_RETURN);
                    }
                }
                else{
                    drive.targetTurn(Math.PI/2);
                    mStateTime.reset();
                }
                break;
            case STATE_RETURN:
                if (getForwardDist() - memo >= 69 * (37.5 / 43)) {
                    drive.setPower(0.0, 0.0, 0.0);
                } else {
                    drive.setPower(-0.5, 0.0, 0.0);
                }
                break;
            case STATE_STOP:
                drive.setPower(0.0,0.0,0.0);
        }
        drive.write();
        intake.write();
        flip.write();
        telemetry.addData("Strafe wheel dist: ", getStrafeDist());
        telemetry.addData("Forward: ", getForwardDist());
        telemetry.addData("forward - memo: ", getForwardDist() - memo);
        telemetry.addData("time: ", mStateTime.time());
        telemetry.addData("Angle", drive.angleWrap(drive.getExternalHeading()));
        telemetry.addData("State: ", mRobotState);
        telemetry.addData("Error", drive.getHeadingerror());
    }

    private double getForwardDist(){
        return leftWheel.getDistance() * (23 / 37.678);
    }

    private double getStrafeDist(){
        return strafeWheel.getDistance() * 7.0 / 17.536;
    }

    private double getangle(){return drive.angleWrap(drive.getExternalHeading());}
}
