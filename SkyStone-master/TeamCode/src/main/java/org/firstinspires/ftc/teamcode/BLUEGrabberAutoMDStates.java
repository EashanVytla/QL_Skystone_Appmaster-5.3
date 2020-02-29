package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.SRX_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.SRX_Three_Wheel_Localizer;
import org.firstinspires.ftc.teamcode.Odometry.ThreeWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.Pure_Pursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.Universal.Math.Pose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.VisionContstants.IMAGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.VisionContstants.IMAGE_WIDTH;

@Autonomous(name = "Ql_Auto_GRABBER_BLUE")
public class BLUEGrabberAutoMDStates extends OpMode {
    int SkystonePos;

    public OpenCvCamera webcam;
    SkystoneDetectorPipeline pipeline;

    ThreeWheelTrackingLocalizer odos;

    Mecanum_Drive drive;
    Intake intake;
    Grabber grab;

    FlipperV2 flip;
    int NUM_STONES = 3;
    int i = 0;

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;

    ElapsedTime mStateTime = new ElapsedTime();
    ElapsedTime correctionTime = new ElapsedTime();

    private enum State{
        STATE_DRIVE_TO_BLOCK,
        STATE_DRIVE_TO_BAR,
        STATE_DRIVE_TO_BAR_BACK,
        STATE_DRIVE_TO_FOUNDATION,
        STATE_GRAB_FOUNDATION,
        STATE_DRIVE_INTO_FOUNDATION,
        STATE_PULL_FORWARD,
        STATE_TURN,
        STATE_STRAFE,
        STATE_PARK,
        STATE_EXIT_PLATFORM,
        STATE_STOP
    }
    State mRobotState = State.STATE_DRIVE_TO_BLOCK;

    public void init(){
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        drive = new Mecanum_Drive(hardwareMap, telemetry);
        flip = new FlipperV2(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        grab = new Grabber(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDetectorPipeline();
        webcam.openCameraDevice();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        odos = new ThreeWheelTrackingLocalizer(this.hardwareMap);

        odos.setPos(odos.getPoseEstimate());
        odos.reset();
        odos.poseSet(new Pose2d(0, 0, -Math.PI / 2));
        correctionTime = new ElapsedTime();
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
        flip.resetPlatform();
    }

    public void start(){
        mStateTime.startTime();
        webcam.stopStreaming();
        webcam.closeCameraDevice();
        odos.reset();
        correctionTime.startTime();
        newState(State.STATE_DRIVE_TO_BLOCK);
    }

    ArrayList<Pose2d> drive_to_block = new ArrayList<>();
    ArrayList<Pose2d> drive_to_bar = new ArrayList<>();
    ArrayList<Pose2d> drive_to_bar_back = new ArrayList<>();
    ArrayList<Pose2d> drive_to_foundation = new ArrayList<>();
    Pose2d park = new Pose2d(36.026, -18.797, Math.PI - Math.toRadians(15));
    Pose2d exit_foundation = new Pose2d(85.304, -33.027, Math.toRadians(80));
    Pose2d drive_into_foundation = new Pose2d(85.304, -43.543, Math.PI/2);
    Pose2d pull_forward = new Pose2d(exit_foundation.getX() - 10,-8, Math.toRadians(115));
    double heading = 0.0;
    double dt = 0.0;
    double prevtime = 0.0;

    public void loop() {
        dt = System.currentTimeMillis() - prevtime;
        prevtime = System.currentTimeMillis();

        RevBulkData data = hub2.getBulkInputData();
        odos.dataUpdate(data);
        odos.update();
        Pose2d currentPos = new Pose2d(odos.getEstimatedPose().vec(), odos.getAbsoluteAngle());
        drive_to_foundation.add(new Pose2d(92.333, -31.835, Math.PI));
        drive_to_foundation.add(new Pose2d(92.333, -32.835, Math.PI));
        drive_to_foundation.add(new Pose2d(84.333, -35.835, Math.PI));
        drive_to_foundation.add(new Pose2d(80.333, -35.835, Math.PI - Math.toRadians(5.0)));

        drive_to_bar.add(new Pose2d(36.026, -18.797, Math.PI)); //First
        drive_to_bar.add(new Pose2d(36.026, -18.797, Math.PI)); //Second
        drive_to_bar.add(new Pose2d(36.026, -18.797, Math.PI)); //Third
        drive_to_bar.add(new Pose2d(36.026, -12.797, Math.PI)); //Fourth

        drive_to_bar_back.add(new Pose2d(36.026, -18.797, Math.PI)); //First

        if (SkystonePos == 0) {
            drive_to_block.add(new Pose2d(-24.159, -26.077, Math.PI));//first stone
            drive_to_block.add(new Pose2d(-4.159, -27.077, Math.PI));//second stone
            drive_to_block.add(new Pose2d(12.159, -25.077, Math.PI));//third stone
            drive_to_block.add(new Pose2d(4.159, -14.077, Math.PI));//fourth stone
        } else if (SkystonePos == 1) {
            drive_to_block.add(new Pose2d(-24.159, -26.077, Math.PI));//first stone
            drive_to_block.add(new Pose2d(-0.159, -26.077, Math.PI));//second stone
            drive_to_block.add(new Pose2d(16.159, -26.077, Math.PI));//third stone
            drive_to_block.add(new Pose2d(8.159, -26.077, Math.PI));//fourth stone
        } else if (SkystonePos == 2) {
            drive_to_block.add(new Pose2d(-24.159, -26.077, Math.PI));//first stone
            drive_to_block.add(new Pose2d(-0.159, -26.077, Math.PI));//second stone
            drive_to_block.add(new Pose2d(16.159, -26.077, Math.PI));//third stone
            drive_to_block.add(new Pose2d(8.159, -26.077, Math.PI));//fourth stone
        }

        switch (mRobotState) {
            case STATE_DRIVE_TO_BLOCK:
                double kickout = 0.0;
                if(i == 0){
                    kickout = 3.0;
                }else if(i == 1){
                    kickout = 4.5;
                }else{
                    kickout = 4.5;
                }
                if (currentPos.vec().distTo(drive_to_block.get(i).vec()) <= (i == 1 ? 2.5 : 1.5) && Math.abs(drive_to_block.get(i).getHeading() - currentPos.getHeading()) <= Math.toRadians(kickout)) {
                    if(correctionTime.time() <= 0.25){
                        newState(State.STATE_DRIVE_TO_BAR);
                    }else{
                        drive.goToPoint(currentPos, drive_to_block.get(i), 1.0, 1.0, 1.0);
                    }
                    //newState(State.STATE_STOP);
                } else {
                    if(i != 0 && currentPos.vec().distTo(drive_to_block.get(i).vec()) <= 17.0){
                        grab.partial();
                        grab.unclamp();
                    }else if(i == 0){
                        grab.partial();
                        grab.unclamp();
                    }

                    correctionTime.reset();
                    drive.goToPoint(currentPos, drive_to_block.get(i), 1.0, 1.0, 1.0);
                    telemetry.addData("Target Pos:", drive_to_block.get(i).toString());
                }
                break;
            case STATE_DRIVE_TO_BAR:
                if(mStateTime.time() >= (i != 0 ? 0.9 : 0.75)){
                    if (currentPos.vec().distTo(drive_to_bar.get(i).vec()) <= 2.0) {
                        newState(State.STATE_DRIVE_TO_FOUNDATION);
                    } else {
                        drive.goToPoint(currentPos, drive_to_bar.get(i), 1.0, 1.0, 1.0);
                    }
                }else{
                    if(i != 0){
                        grab.goDown();
                        if (mStateTime.time() >= 0.65){
                            grab.liftCrossing();
                        }
                        else if (mStateTime.time() >= 0.4){
                            grab.clamp();
                        }else if (mStateTime.time() >= 0.15){
                            drive.setPower(0.0,-0.3,0.0);
                        }
                    }else{
                        drive.goToPoint(currentPos, drive_to_block.get(i), 1.0, 1.0, 1.0);
                        grab.goDown();
                        if (mStateTime.time() >= 0.5){
                            grab.liftCrossing();
                        }
                        else if(mStateTime.time() >= 0.25){
                            grab.clamp();
                        }
                    }
                }

                break;
            case STATE_DRIVE_TO_FOUNDATION:
                if (currentPos.vec().distTo(drive_to_foundation.get(i).vec()) <= 2.0) {
                    if(correctionTime.time() >= 0.25){
                        i++;
                        if(i == NUM_STONES){
                            newState(State.STATE_EXIT_PLATFORM);
                            //newState(State.STATE_STOP);
                        }else{
                            newState(State.STATE_DRIVE_TO_BAR_BACK);
                            //newState(State.STATE_STOP);
                        }
                    }else{
                        drive.goToPoint(currentPos, drive_to_foundation.get(i), 1.0, 1.0, 1.0);
                    }

                }
                /*else if (currentPos.vec().distTo(drive_to_foundation.get(i).vec()) <= 6.0){
                    grab.unclamp();
                    drive.setPower(0.0,0.0,0.0);
                }*/else {
                    correctionTime.reset();
                    drive.goToPoint(currentPos, drive_to_foundation.get(i), 1.0, 1.0, 1.0);
                }
                break;
            case STATE_DRIVE_TO_BAR_BACK:
                if(mStateTime.time() >= 0.45){
                    if (currentPos.vec().distTo(drive_to_bar_back.get(i - 1).vec()) <= 2.0) {
                        newState(State.STATE_DRIVE_TO_BLOCK);
                        //newState(State.STATE_STOP);
                    } else {
                        drive.goToPoint(currentPos, drive_to_bar_back.get(i), 1.0, 1.0, 1.0);
                    }
                    if(currentPos.vec().distTo(drive_to_foundation.get(i - 1).vec()) >= 15){
                        grab.clamp();
                    }
                }else{
                    drive.setPower(0.0, (i == 3 ? -0.4 : -0.3), 0.0);
                    if(mStateTime.time() >= 0.15){
                        grab.unclamp();
                    }
                }
                break;
            case STATE_STRAFE:
                if(currentPos.getY() <= -30){
                    newState(State.STATE_PARK);
                }else{
                    drive.setPower(0.0, -1.0, 0.0);
                }
                break;
            case STATE_PARK:
                if(park.vec().distTo(currentPos.vec()) <= 1.0){
                    newState(State.STATE_STOP);
                }else{
                    drive.goToPoint(currentPos, park, 1.0, 1.0, 1.0);
                }
                break;
            case STATE_EXIT_PLATFORM:
                if(mStateTime.time() >= 0.45){
                    if(currentPos.vec().distTo(exit_foundation.vec()) <= 3.0 && Math.abs(currentPos.getHeading()) <= Math.PI/2) {
                        newState(State.STATE_DRIVE_INTO_FOUNDATION);
                    }else{
                        drive.goToPoint(currentPos, exit_foundation, 1.0, 1.0, 1.0);
                        flip.startKnocker();
                    }
                }else{
                    drive.setPower(0.0, -0.3, 0.0);
                    if(mStateTime.time() >= 0.15){
                        grab.unclamp();
                    }
                }
                break;
            case STATE_DRIVE_INTO_FOUNDATION:
                if(currentPos.vec().distTo(drive_into_foundation.vec()) <= 3.0 || mStateTime.time() >= 1.0) {
                    newState(State.STATE_GRAB_FOUNDATION);
                }
                else{
                    drive.goToPoint(currentPos, drive_into_foundation, 0.3, 0.3, 1.0);
                }
                break;
            case STATE_GRAB_FOUNDATION:
                if(mStateTime.time() >= 0.5){
                    newState(State.STATE_PULL_FORWARD);
                }else{
                    flip.grabPlatform();
                }
                break;
            case STATE_PULL_FORWARD:
                if(currentPos.vec().distTo(pull_forward.vec()) <= 4.0) {
                    newState(State.STATE_TURN);
                }else{
                    intake.setPower(-0.2);
                    drive.goToPoint(currentPos, pull_forward, 1.0, 1.0, 1.0);
                    mStateTime.reset();
                }
                break;
            case STATE_TURN:
                if(currentPos.getHeading() >= Math.toRadians(155.0)) {
                    flip.resetPlatform();
                    newState(State.STATE_PARK);
                    //newState(State.STATE_STOP);
                }else{
                    drive.setPower(0.0,0.0,1.0);
                }
                break;
            case STATE_STOP:
                drive.setPower(0.0,0.0,0.0);
                telemetry.addData("POS: ", currentPos.toString());
                drive.getOdos().outputtRaw(telemetry);
                break;
        }
        telemetry.addData("Pos: ", currentPos.toString());
        if (dt != 0.0){
            telemetry.addData("Refresh Rate: ", 1000 / dt);
        }

        grab.write();
        drive.write();
        telemetry.addData("State: ", mRobotState);
    }

    public boolean isPressG(boolean value, boolean previous){
        return value && !previous;
    }

    private void newState(State state){
        mStateTime.reset();
        correctionTime.reset();
        drive.resetIntegral();
        mRobotState = state;
    }
}
