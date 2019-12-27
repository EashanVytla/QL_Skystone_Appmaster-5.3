package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import static org.firstinspires.ftc.teamcode.VisionContstants.IMAGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.VisionContstants.IMAGE_WIDTH;

@Autonomous(name = "QL_Skystone_Blue", group = "Competition")
public class QL_2SkyStone_SERVO extends OpMode {
    Dead_Wheel leftWheel;
    Dead_Wheel rightWheel;
    Dead_Wheel strafeWheel;
    int SkystonePos;

    public OpenCvCamera webcam;
    //MultipleTelemetry telemetry;
    SkystoneDetectorPipeline pipeline;

    Mecanum_Drive drive;
    Intake intake;

    double memo = 0.0;
    double memoL = 0.0;
    double memoR = 0.0;
    Flipper flip;
    PID pid;

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;

    ElapsedTime mStateTime = new ElapsedTime();

    private enum State{
        STATE_SCAN,
        STATE_STRAFE,
        STATE_DRIVE_TO_BLOCK,
        STATE_PICKUP,
        STATE_STRAFEOUT,
        STATE_REALLIGN2,
        STATE_CROSS,
        STATE_DEPOSIT,
        STATE_RETURN,
        STATE_REALLIGN3,
        STATE_STRAFEIN,
        STATE_REALLIGN,
        STATE_PICKUP2,
        STATE_STRAFEOUT2,
        STATE_CROSS2,
        STATE_DEPOSIT2,
        STATE_FOUNDATION1,
        STATE_FOUNDATION2,
        STATE_FOUNDATION3,
        STATE_PULL_FOUNDATION,
        STATE_TURN_FOUNDATION,
        STATE_PLACE_FOUNDATION,
        STATE_PARK
    }
    State mRobotState = State.STATE_STRAFE;

    public void init(){
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        leftWheel = new Dead_Wheel(new MA3_Encoder("a3", hardwareMap, 0.495));
        rightWheel = new Dead_Wheel(new MA3_Encoder("a4", hardwareMap, 1.365));
        strafeWheel = new Dead_Wheel(new MA3_Encoder("a1", hardwareMap, 2.464));
        drive = new Mecanum_Drive(hardwareMap, telemetry);
        flip = new Flipper(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        pid = new PID(hardwareMap, telemetry);

        rightWheel.getEncoder().reverse();
        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();
        leftWheel.getEncoder().calibrate(data);
        rightWheel.getEncoder().calibrate(data2);
        strafeWheel.getEncoder().calibrate(data);
        leftWheel.setBehavior(1.5385, -0.319237); //1.5144 0.0361262
        rightWheel.setBehavior(1.5385, -0.319237); //1.5204 -0.00305571
        strafeWheel.setBehavior(1.53642, 0.0); //1.50608 -0.221642

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDetectorPipeline();
        webcam.openCameraDevice();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);

        flip.initialize();
    }

    @Override
    public void init_loop(){
        SkystonePos = pipeline.getSkyPos();
        if(SkystonePos == 0){
            SkystonePos = 1;
        }else if(SkystonePos == 1){
            SkystonePos = 0;
        }
        telemetry.addData("Skystone Pos: ", Math.abs(SkystonePos));
    }

    public void start(){
        mStateTime.startTime();
        newState(State.STATE_STRAFE);
    }

    int crossDist = 0;
    int ReturnTarget = 0;
    int drive_to_block = 0;

    public void loop(){
        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();
        SkystonePos = 2;
        flip.read();

        leftWheel.update(data);
        rightWheel.update(data2);
        strafeWheel.update(data);

        if(SkystonePos == 0){
            crossDist = 35;
        }else if(SkystonePos == 1){
            crossDist = 35;
            //Target Dist for Return is equal to ---
        }else if(SkystonePos == 2){
            crossDist = 45;
            drive_to_block = 10;
            ReturnTarget = 10;
            //Target Dist for Return is equal to ---
        }

        switch (mRobotState) {
            case STATE_STRAFE:
                if (Math.abs(getStrafeDist()) >= 29) {
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_REALLIGN);
                } else {
                    drive.strafe(29, Math.abs(getStrafeDist()), -0.3, telemetry);
                }
                break;
            case STATE_DRIVE_TO_BLOCK:
                if (mStateTime.time() >= 0.5) {
                    if (Math.abs(getForwardDist() - memo) > drive_to_block && mStateTime.time() >= 2.0) {
                        drive.setPower(0.0, 0.0, 0.0);
                        memo = getForwardDist();
                        newState(State.STATE_PICKUP);
                    } else if (mStateTime.time() >= 2.5) {
                        drive.setPower(0.0, 0.0, 0.0);
                        memo = getForwardDist();
                        newState(State.STATE_PICKUP);
                    } else {
                        drive.straight(drive_to_block, Math.abs(getForwardDist()) - memo, telemetry); //DON"T DO MATH.ABS TRUST ME!!!
                    }
                }
                break;
            case STATE_PICKUP:
                if (mStateTime.time() >= 2.0) {
                    memo = Math.abs(getStrafeDist());
                    memoL = Math.abs(getLeftDist());
                    memoR = Math.abs(getRightDist());
                    newState(State.STATE_STRAFEOUT);
                }
                break;
            case STATE_STRAFEOUT:
                if (Math.abs(getStrafeDist() - memo) >= 5) {
                    drive.setPower(0.0, 0.0, 0.0);
                    memo = getForwardDist();
                    newState(State.STATE_REALLIGN2);
                } else {
                    drive.strafe(5, Math.abs(getStrafeDist()) - memo, 0.3, telemetry);
                }
                break;
            case STATE_REALLIGN2:
                /*
                if(Math.abs(getLeftDist()) - memoL <= 0.1 && Math.abs(getRightDist()) - memoR <= 0.1){
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_CROSS);
                }else{
                    drive.setPowerSide(Range.clip(getLeftDist() + memoL, -0.15, 0.15), Range.clip(getRightDist() - memoR, -0.15, 0.15));
                }

                 */
                drive.read(data);
                if (getAngle() >= 0.0 && mStateTime.time() >= 1.5) {
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_CROSS);
                } else if (mStateTime.time() >= 2.0) {
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_CROSS);
                } else{
                    drive.targetTurn(0.0);
                }   
                break;
            case STATE_CROSS:
                if(mStateTime.time() >= 0.5){
                    if(Math.abs(getForwardDist() - memo) > crossDist && mStateTime.time() >= 3.0){
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_DEPOSIT);
                    }else if(mStateTime.time() >= 3.5){
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_DEPOSIT);
                    }else{
                        drive.straight(crossDist, Math.abs(getForwardDist()) - memo, telemetry); //DON"T DO MATH.ABS TRUST ME!!!
                    }
                }
                break;
            case STATE_DEPOSIT:
                if(mStateTime.time() >= 1.0){
                    memo = getForwardDist();
                    newState(State.STATE_RETURN);
                }
                break;
            case STATE_RETURN:
                if((-getForwardDist()) <= ReturnTarget && mStateTime.time() >= 3.0){
                    drive.setPower(0.0, 0.0, 0.0);
                    memo = Math.abs(getStrafeDist());
                    newState(State.STATE_REALLIGN3);
                }else if(mStateTime.time() >= 3.5){
                    drive.setPower(0.0, 0.0, 0.0);
                    memo = Math.abs(getStrafeDist());
                    newState(State.STATE_REALLIGN3);
                }else{
                    drive.straight(ReturnTarget, -getForwardDist(), telemetry); //DON"T DO MATH.ABS TRUST ME!!!
                }
                break;
            case STATE_STRAFEIN:
                if(mStateTime.time() >= 0.5){
                    if(Math.abs(getStrafeDist()) >= 29){
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_PICKUP2);
                    }else{
                        drive.strafe(29, Math.abs(getStrafeDist()), -0.3, telemetry);
                    }
                }
                break;
            case STATE_REALLIGN:
                if(mStateTime.time() >= 1.0){
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_DRIVE_TO_BLOCK);
                }else{
                    drive.setPowerSide(Range.clip(getLeftDist(), -0.15, 0.15), Range.clip(getRightDist(), -0.15, 0.15));
                }
                break;
            case STATE_REALLIGN3:
                if(mStateTime.time() >= 69.0){
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_STRAFEIN);
                }else{
                    drive.setPowerSide(Range.clip(getLeftDist(), -0.15, 0.15), Range.clip(getRightDist(), -0.15, 0.15));
                }
                break;
            case STATE_PICKUP2:
                if(mStateTime.time() >= 1.0){
                    memo = Math.abs(getStrafeDist());
                    newState(State.STATE_STRAFEOUT2);
                }
                break;
            case STATE_STRAFEOUT2:
                if(Math.abs(getStrafeDist() - memo) >= 5){
                    drive.setPower(0.0, 0.0, 0.0);
                    memo = Math.abs(getForwardDist());
                    newState(State.STATE_CROSS2);
                }else{
                    drive.strafe(5, Math.abs(getStrafeDist()), 0.3, telemetry);
                }
                break;
            case STATE_CROSS2:
                if (mStateTime.time() >= 0.5) {
                    if(Math.abs(getForwardDist() - memo) > (crossDist + 10) && mStateTime.time() >= 3.0){
                        drive.setPower(0.0, 0.0, 0.0);
                    }else if(mStateTime.time() >= 3.5){
                        drive.setPower(0.0, 0.0, 0.0);
                    }else{
                        drive.straight(crossDist + 10, Math.abs(getForwardDist()) - memo, telemetry); //DON"T DO MATH.ABS TRUST ME!!!
                    }
                }
                break;
        }
        telemetry.addData("Left Dist: ", getLeftDist());
        telemetry.addData("Right Dist", getRightDist());
        telemetry.addData("Forward Dist: ", Math.abs(getForwardDist()) - memo);
        telemetry.addData("Strafe Dist: ", getStrafeDist());
        telemetry.addData("mStateTime: ", mStateTime.time());
        telemetry.addData("State: ", mRobotState);

        flip.write();
        drive.write();
        intake.write();
    }

    private void newState(State state){
        mRobotState = state;
        mStateTime.reset();
    }

    private double getForwardDist(){
        return (rightWheel.getDistance());
    }

    private double getLeftDist(){
        return (leftWheel.getDistance());
    }

    private double getRightDist(){
        return (rightWheel.getDistance());
    }

    private double getStrafeDist(){
        //return (strafeWheel.getDistance() * 7.0 / 17.536) * 1.611111;
        return (strafeWheel.getDistance());
    }

    private double getAngle(){return drive.angleWrap(drive.getExternalHeading());}
}
