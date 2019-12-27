package org.firstinspires.ftc.teamcode;

import android.service.carrier.CarrierService;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import static org.firstinspires.ftc.teamcode.VisionContstants.IMAGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.VisionContstants.IMAGE_WIDTH;

@Autonomous(name = "QL_Auto_Skystone_Blue", group = "Competition")
@Disabled
public class QL_Auto_Skystone_Blue extends OpMode {
    Dead_Wheel leftWheel;
    Dead_Wheel rightWheel;
    Dead_Wheel strafeWheel;

    Mecanum_Drive drive;
    double angle;
    double strafe;
    double drivein;

    Flipper flip;
    Intake intake;
    int SkyStonePos = 2;

    boolean flag = false;

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;
    ElapsedTime mStateTime = new ElapsedTime();
    double memo = 0.0;
    double crossDist = 0.0;

    double power = 0.0;

    static final double turnCone = 10.0;

    private long prev_time = System.currentTimeMillis();

    private ElapsedTime time = new ElapsedTime();

    public OpenCvCamera webcam;
    //MultipleTelemetry telemetry;
    SkystoneDetectorPipeline pipeline;

    private enum State{
        STATE_DETECT,
        STATE_FORWARD,
        STATE_STRAFE,
        STATE_TURN,
        STATE_INTAKE,
        STATE_BACK,
        STATE_TURN2,
        STATE_CROSS,
        STATE_FLIP,
        STATE_TURN3,
        STATE_GRAB_FOUNDATION,
        STATE_PULL_FORWARD,
        STATE_TURN4,
        STATE_PUSH_FOUNDATION,
        STATE_RELEASE,
        STATE_DRIVE_TO_FOUNDATION,
        STATE_DEPOSIT,
        STATE_REPOSITION,
        STATE_RETURN,
        BACK_FROM_FOUNDATION,
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

        //dashboard = FtcDashboard.getInstance();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDetectorPipeline();
        webcam.openCameraDevice();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);

        flip.clampcap();
        time.startTime();
    }

    @Override
    public void init_loop(){
        SkyStonePos = pipeline.getSkyPos();
        if(SkyStonePos == 0){
            SkyStonePos = 1;
        }else if(SkyStonePos == 1){
            SkyStonePos = 0;
        }
        telemetry.addData("Skystone Pos: ", Math.abs(SkyStonePos));
    }

    public void start(){
        intake.kickout();
        flip.start();
        webcam.stopStreaming();
    }

    public void loop() {
        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();

        leftWheel.update(data);
        rightWheel.update(data2);
        strafeWheel.update(data);

        telemetry.addData("Refresh Rate: ", System.currentTimeMillis() - prev_time);
        prev_time = System.currentTimeMillis();

        switch(mRobotState) {
            case STATE_DETECT:
                if(SkyStonePos == 0){
                    angle = 0.0;
                    strafe = -1.0;
                    drivein = 14.0;
                    crossDist = -50.0;
                }else if(SkyStonePos == 1){
                    angle = 0.0;
                    strafe = 2.0;
                    drivein = 17.0;
                    crossDist = -47.0;
                    flag = true;
                }else if(SkyStonePos == 2){
                    angle = 0.0;
                    strafe = -10.0;
                    drivein = 17.0;
                    crossDist = -54.0;
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
                flip.clamp();
                intake.close();
                intake.setPower(0.0);
                break;
            case STATE_STRAFE:
                if(flag){
                    if (mStateTime.time() >= 0.5) {
                        if (getStrafeDist() >= strafe * (17 / 28.5)) {
                            drive.setPower(0.0, 0.0, 0.0);
                            flag = false;
                            newState(State.STATE_TURN);
                        } else {
                            drive.setPower(0.0, -0.35, 0.0);
                        }
                    }
                }else{
                    if (mStateTime.time() >= 0.5) {
                        if (getStrafeDist() <= strafe * (17 / 28.5)) {
                            drive.setPower(0.0, 0.0, 0.0);
                            flag = false;
                            newState(State.STATE_TURN);
                        } else {
                            drive.setPower(0.0, 0.35, 0.0);
                        }
                    }
                }

                break;
            case STATE_TURN:
                if (drive.getExternalHeading() >= -angle) {
                    drive.setPower(0.0, 0.0, 0.0);
                    memo = getForwardDist();
                    newState(State.STATE_INTAKE);
                } else {
                    drive.setPower(0.0, 0.0, -0.2);
                }
                drive.read(data);
                break;
            case STATE_INTAKE:
                intake.setPosition(0.7);
                intake.setPower(0.3);
                if (mStateTime.time() >= 0.5) {
                    if (getForwardDist() - memo >= drivein * (37.5 / 43)) {
                        drive.setPower(0.0, 0.0, 0.0);
                        memo = getForwardDist();
                        newState(State.STATE_BACK);
                    } else if (mStateTime.time() >= 3.0) {
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_BACK);
                    } else {
                        drive.setPower(-0.2, 0.0, 0.0);
                    }
                }
                break;
            case STATE_BACK:
                if (mStateTime.time() >= 0.5) {
                    intake.close();
                    if (getForwardDist() - memo <= -(drivein - 5) * (37.5 / 43)) {
                        drive.setPower(0.0, 0.0, 0.0);
                        memo = getForwardDist();
                        intake.setPower(0.0);
                        newState(State.STATE_TURN2);
                    } else {
                        drive.setPower(0.2, 0.0, 0.0);
                    }
                }
                break;
            case STATE_TURN2:
                if (Math.abs(getangle() - (Math.PI / 2 + + Math.toRadians(1.5))) < Math.toRadians(turnCone)) { //+ Math.toRadians(5.5)))
                    if (time.time() >= 0.7) {
                        drive.setPower(0.0, 0.0, 0.0);
                        memo = getForwardDist();
                        newState(State.STATE_CROSS);
                    }
                }else if(mStateTime.time() >= 2.0){
                    memo = getForwardDist();
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_CROSS);
                } else {
                    //drive.setPower(0.0, 0.0, 0.2);
                    time.reset();
                    drive.targetTurn((3 * Math.PI)/2); //+ Math.toRadians(5.5)
                }
                drive.read(data);
                break;
            case STATE_CROSS:
                if(mStateTime.time() >= 0.5){
                    if (getForwardDist() - memo <= crossDist * (37.5 / 43) * (162.25 / 100)) {   //-93
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_TURN3);
                    } else {
                        drive.setPower(0.4, 0.0, 0.0);
                    }
                }
                break;
            case STATE_TURN3:
                if(mStateTime.time() >= 0.5){
                    if (getangle() <= Math.PI) {
                        intake.setPower(0.0);
                        drive.setPower(0.0, 0.0, 0.0);
                        memo = getForwardDist();
                        newState(State.STATE_DRIVE_TO_FOUNDATION);
                    } else {
                        drive.setPower(0.0, 0.0, -0.2);
                    }
                    drive.read(data);
                }
                break;
            case STATE_DRIVE_TO_FOUNDATION:
                intake.setPower(0.2);
                if (mStateTime.time() >= 0.5) {
                    if (getForwardDist() - memo < -11){
                        memo = getForwardDist();
                        drive.setPower(0.0, 0.0, 0.0);
                        intake.setPower(0.0);
                        newState(State.STATE_FLIP);
                    }
                    else{
                        drive.setPower(0.3, 0.0, 0.0);
                    }
                    telemetry.addData("Forward Dist: ", getForwardDist() - memo);
                }
                break;
            //case DUMP_BLock
            case STATE_FLIP:
                intake.open();
                flip.operate(4);
                telemetry.addData("Flip State: ", flip.getFlipState().toString());
                telemetry.addData("Flip Target Position: ", flip.getFlipper().getPosition());
                telemetry.addData("Clamp Target Position: ", flip.getClamp().getPosition());
                if(mStateTime.time() >= 3.0){
                    newState(State.STATE_GRAB_FOUNDATION);
                }
                break;
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
                flip.grabPlatform();
                intake.close();
                flip.operate(0);
                if(mStateTime.time() >= 1.0){
                    flip.operate(1);
                }
                memo = getForwardDist();
                if (mStateTime.time() > 2.0) {
                    newState(State.STATE_PULL_FORWARD);
                }
                break;
            case STATE_PULL_FORWARD:
                if (getForwardDist() - memo > 16){
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
                    if (getangle() >= (3 * Math.PI) / 2){
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_RELEASE);
                    }
                    else{
                        drive.setPower(0.0, 0.0, 0.7);
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
                if (getForwardDist() - memo <= -14 * (37.5 / 43)) {
                    drive.setPower(0.0, 0.0, 0.0);
                    memo = getForwardDist();
                    newState(State.STATE_RETURN);
                }
                else if (mStateTime.time() >= 2.0){
                    drive.setPower(0.0, 0.0, 0.0);
                    memo = getForwardDist();
                    newState(State.STATE_RETURN);
                }
                else {
                    drive.setPower(0.4, 0.0, 0.0);
                }
                break;
            case STATE_REPOSITION:
                drive.read(data);
                if (getStrafeDist() - memo >= 1.0){
                    drive.setPower(0.0, 0.0, 0.0);
                    memo = getForwardDist();
                    newState(State.STATE_RETURN);
                }
                else{
                    drive.setPower(0.0, 0.4, 0.0);
                }
                break;
            case STATE_RETURN:
                if (getForwardDist() - memo >= 25.5 * (37.5 / 43)) {
                    drive.setPower(0.0, 0.0, 0.0);
                } else {
                    drive.setPower(-0.5, 0.0, 0.0);
                }
                break;
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
    }

    public void stop(){
        webcam.closeCameraDevice();
    }

    private double getForwardDist(){
        return rightWheel.getDistance() * (23 / 37.678);
    }

    private double getStrafeDist(){
        return strafeWheel.getDistance();
    }  //7.0 / 17.536

    private double getangle(){return drive.angleWrap(drive.getExternalHeading());}
}