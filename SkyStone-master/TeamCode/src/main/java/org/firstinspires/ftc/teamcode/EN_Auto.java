/*
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.Dead_Wheel;
import org.firstinspires.ftc.teamcode.Odometry.MA3_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.SRX_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.SRX_Three_Wheel_Localizer;
import org.firstinspires.ftc.teamcode.Odometry.ThreeWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.Universal.Math.Pose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import static org.firstinspires.ftc.teamcode.VisionContstants.IMAGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.VisionContstants.IMAGE_WIDTH;

@Autonomous(name = "COOL_KID_AUTO_PROTO", group = "Competition")
public class EN_Auto extends OpMode {
    int SkystonePos;

    public OpenCvCamera webcam;
    SkystoneDetectorPipeline pipeline;

    ThreeWheelTrackingLocalizer odos;
    SRX_Three_Wheel_Localizer localizer;
    Vertical_Elevator slides;

    Mecanum_Drive drive;
    Intake intake;

    double memo = 0.0;
    double savedDepositY = 0.0;
    double savedDepositX = 0.0;
    double storedY = 0.0;
    FlipperV2 flip;

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;

    ElapsedTime mStateTime = new ElapsedTime();
    ElapsedTime delay = new ElapsedTime();
    ElapsedTime motionProfile = new ElapsedTime();

    private enum State{
        STATE_DRIVE_TO_BLOCK,
        STATE_INTAKE,
        STATE_EXIT_POOL,
        STATE_CROSS,
        STATE_DRIVE_TO_FOUNDATION,
        STATE_GRAB_FOUNDATION,
        STATE_TURN,
        STATE_RELEASE,
        STATE_RETURN,
        STATE_INTAKE2,
        STATE_EXITPOOL2,
        STATE_CROSS2,
        STATE_RETURN2,
        STATE_INTAKE3,
        STATE_EXITPOOL3,
        STATE_CROSS3,
        STATE_RETURN3,
        STATE_INTAKE4,
        STATE_EXITPOOL4,
        STATE_CROSS4,
        STATE_PARK,
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDetectorPipeline();
        webcam.openCameraDevice();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        slides = new Vertical_Elevator(hardwareMap, telemetry);
        localizer = new SRX_Three_Wheel_Localizer(new SRX_Encoder("intake_left", hardwareMap), new SRX_Encoder("intake_right", hardwareMap), new SRX_Encoder("lift_2", hardwareMap), hardwareMap, telemetry);
        odos = new ThreeWheelTrackingLocalizer(this.hardwareMap);

        odos.setPoseEstimate(new Pose2d(0, 0, Math.PI / 2));
        localizer.getOdos().setPoseEstimate(new Pose2d(0, 0, Math.PI / 2));
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
        delay.startTime();
        newState(State.STATE_DRIVE_TO_BLOCK);
    }

    Pose2d cross_target = new Pose2d(-27.172, -91.387, -Math.PI/2);
    Pose2d return_target;
    Pose2d intake1;
    Pose2d intake2;
    Pose2d drive_to_block_target;
    Pose2d drive_to_block_target2;
    Pose2d drive_platform_target;
    Pose2d exit_pool;
    Pose2d return_target2;
    Pose2d exit_pool2;
    Pose2d saved_deposit = new Pose2d(0.0,0.0,0.0);
    Pose2d intake3;
    Pose2d return_target3;
    Pose2d intake4;
    Pose2d exit_pool4;
    Pose2d cross_target3 = new Pose2d(-21.173, -79, 0.0);
    double heading = 0.0;

    public double getHeading(){
        Pose2d currentPos = odos.getPoseEstimate();
        odos.update();

        if(currentPos.getHeading() <= Math.PI){
            heading = currentPos.getHeading();
        }else{
            heading = -((2 * Math.PI ) - currentPos.getHeading());
        }
        return heading;
    }

    public void loop(){
        RevBulkData data = hub2.getBulkInputData();
        flip.read();
        odos.dataUpdate(data);
        odos.update();
        localizer.update(data);
        Pose2d currentPos = odos.getPoseEstimate();

        if(SkystonePos == 0){
            drive_to_block_target = new Pose2d(-25.992, 18.104, Math.toRadians(50));
            return_target = new Pose2d(-26, -14.0, Math.toRadians(36.0));
            exit_pool = new Pose2d(-22, -10.5, 0.0);
            intake1 = new Pose2d(-37.337, 16.276, Math.toRadians(50));
            intake2 = new Pose2d(-45, -1.58, Math.toRadians(36.0));
        }else if(SkystonePos == 1){
            drive_to_block_target = new Pose2d(-24.992, 4.0, Math.PI/4);
            return_target = new Pose2d(-26, -17.0, Math.PI/4 + Math.toRadians(10.0));
            exit_pool = new Pose2d(-22, -10.5, 0.0);
            intake1 = new Pose2d(-37.337, 10, Math.PI/4);
            intake2 = new Pose2d(-45, -14, Math.PI/4);
        }else if(SkystonePos == 2){
            drive_to_block_target = new Pose2d(-24.797, -0.147, Math.PI/4);

            return_target = new Pose2d(-20.569, -25.559, Math.PI/4);
            return_target2 = new Pose2d(-18.569, -13.558, Math.PI/4);
            return_target3 = new Pose2d(-19.275, -6.529, Math.toRadians(Math.PI/4));

            exit_pool = new Pose2d(-20.020, 8.496, Math.PI/4);
            exit_pool2 = new Pose2d(-20.020, -18.496, Math.PI/4);
            exit_pool4 = new Pose2d(-18.822, -9.205, Math.PI/4);

            intake1 = new Pose2d(-34.435, 9.8, Math.PI/4);
            intake2 = new Pose2d(-36.689, -9.893, Math.PI/4);
            intake3 = new Pose2d(-33.047, -2.421, Math.PI/4);
            intake4 = new Pose2d(-36.036, 5.451, Math.PI/4);
        }

        switch (mRobotState) {
            case STATE_DRIVE_TO_BLOCK:
                if(Math.abs(drive_to_block_target.getX() + currentPos.getY()) <= 2.0 && Math.abs(drive_to_block_target.getHeading() + currentPos.getHeading()) <= 2.0) {
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_INTAKE);
                }else{
                    flip.start();
                    intake.setPower(0.0);
                    localizer.GoTo(drive_to_block_target, 1.0, 1.0, 1.0);
                    mStateTime.reset();
                    intake.kickout();
                    intake.open();
                }
                if (delay.time() >= 1.0){
                    intake.close();
                }
                flip.read();
                break;
            case STATE_INTAKE:
                if(Math.abs(intake1.getX() + currentPos.getY()) <= 1.5 && delay.time() >= 2.0 || flip.IntakeFeedback()){
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_EXIT_POOL);
                }else{
                    intake.close();
                    localizer.GoTo(intake1, 0.2, 0.2, 0.2);
                    if(flip.IntakeFeedback()){
                        intake.setPower(0.0);
                    }else{
                        intake.setPower(0.3);
                    }
                }
                break;
            case STATE_HANDSHAKE:
                if(delay.time() >= 0.5){
                    if(delay.time() >= 3.0){
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_EXIT_POOL);
                    }else{
                        localizer.GoTo(intake1, 0.15,0.15,0.15);
                        intake.open();
                        intake.setPower(0.15);
                    }
                }
                break;
            case STATE_EXIT_POOL:
                if (mStateTime.time() >= 0.3) {
                    if (Math.abs(exit_pool.getY() - currentPos.getX()) <= 4.0) {
                        newState(State.STATE_CROSS);
                    } else if (delay.time() >= 2.5) {
                        newState(State.STATE_CROSS);
                    } else {
                        intake.setPower(0.0);
                        localizer.GoTo(exit_pool, 1.0, 1.0, 1.0);
                    }
                }
                intake.open();
                flip.operate(4);
                break;
            case STATE_CROSS:
                if(Math.abs(currentPos.getX()) >= Math.abs(cross_target.getY()) - 4) {
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_DRIVE_TO_FOUNDATION);
                }else{
                    intake.setPower(0.15);

                    if(currentPos.getX() <= -55.551){
                        localizer.GoTo(cross_target, 1.0, 1.0, 1.0);
                    }else{
                        localizer.GoTo(new Pose2d(cross_target.getX(), cross_target.getY(), 0.0), 1.0, 1.0, 1.0);
                    }

                    flip.operate(4);
                    if(delay.time() >= 3.0){
                        intake.close();
                        flip.clamp();
                    }else{
                        intake.open();
                    }
                }
                telemetry.addData("Is Grabbed? ", flip.isGrabbed());

                break;
            case STATE_DRIVE_TO_FOUNDATION:
                if(mStateTime.time() >= 0.5) {
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_GRAB_FOUNDATION);
                }else{
                    intake.open();
                    localizer.GoTo(new Pose2d(-32.885, -89.7, -Math.PI/2), 1.0, 1.0, 1.0);
                }
                break;
            case STATE_GRAB_FOUNDATION:
                if(mStateTime.time() >= 0.75){
                    newState(State.STATE_PULL_FORWARD);
                }else{
                    flip.grabPlatform();
                    intake.close();
                }
                break;
            case STATE_PULL_FORWARD:
                if(Math.abs(-17 + currentPos.getY()) <= 4.0) {
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_TURN);
                }else if(mStateTime.time() >= 3.0){
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_TURN);
                }else{
                    intake.setPower(-0.2);
                    localizer.GoTo(new Pose2d(-17, -90, -Math.PI/2), 1.0, 1.0, 1.0);
                    mStateTime.reset();
                }
                break;
            case STATE_TURN:
                if(getHeading() >= Math.toRadians(-10.0) || mStateTime.time() >= 3.0) {
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_RETURN);
                }else{
                    localizer.GoTo(new Pose2d(-8.783, -70.605, Math.PI / 4), 1.0, 1.0, 1.0);
                    if (Math.abs(getHeading()) < Math.toRadians(35)){
                        flip.operate(1);
                    }
                    else{
                        flip.operate(0);
                    }
                }
                break;
            case STATE_DEPOSIT:
                if(mStateTime.time() >= 1.2){
                    newState(State.STATE_RETURN);
                }else{
                    flip.resetPlatform();
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                }
                break;
            case STATE_PUSH_BACK:
                if (mStateTime.time() >= 1.0){
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    savedDepositY = currentPos.getX();
                    newState(State.STATE_RETURN);
                }
                else{
                    localizer.getDrive().setPower(0.5, 0.0, 0.0);
                    localizer.getDrive().write();
                }
                break;
            case STATE_STRAFE:
                    if(Math.abs(-23 + currentPos.getY()) <= 2.5) {
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_PUSH_BACK);
                    }else{
                        if(delay.time() >= 2.0){
                        }
                        localizer.GoTo(new Pose2d(-23, storedY + 3, 0.0), 1.0, 1.0, 1.0);
                        mStateTime.reset();
                    }
                break;
            case STATE_RETURN:
                if(Math.abs(return_target.getY() - currentPos.getX()) <= 5.0 && Math.abs(return_target.getHeading() - currentPos.getHeading()) <= 3.0) {
                    drive.setPower(0.0,0.0,0.0)
                    flip.start();
                    newState(State.STATE_INTAKE2);
                }else{
                    flip.resetPlatform();
                    if(currentPos.getX() >= (return_target.getY() - 5)){
                        localizer.GoTo(return_target, 1.0,1.0,1.0);
                    }else{
                        localizer.GoTo(new Pose2d(return_target.getX(), return_target.getY(), 0.0), 1.0,1.0,1.0);
                    }
                    if(currentPos.getX() >= return_target.getY()/3){
                        intake.close();
                        intake.setPower(0.3);
                    }else{
                        intake.setPower(-1.0);
                    }
                    mStateTime.reset();
                }
                break;
            case STATE_INTAKE2:
                flip.read();
                if(Math.abs(intake2.getX() + currentPos.getY()) <= 3.0 || flip.IntakeFeedback()){
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_EXITPOOL2);
                }
                else{
                    localizer.GoTo(intake2, 0.2, 0.2, 0.2);
                    mStateTime.reset();
                    intake.close();
                    if(flip.IntakeFeedback()){
                        intake.setPower(0.0);
                    }else{
                        intake.setPower(0.3);
                    }
                }
                break;
            case STATE_HANDSHAKE2:
                if(mStateTime.time() >= 1.5){
                    newState(State.STATE_EXITPOOL2);
                }else{
                    intake.open();
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                }
                break;
            case STATE_EXITPOOL2:
                if (delay.time() >= 0.3) {
                    if (Math.abs(exit_pool2.getY() - currentPos.getX()) <= 4.0) {
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_CROSS2);
                    } else {
                        intake.setPower(0.0);
                        localizer.GoTo(exit_pool2, 0.8, 0.8, 0.8);
                        mStateTime.reset();
                    }
                }
                intake.open();
                flip.operate(4);
                break;
            case STATE_CROSS2:
                if(Math.abs(-76.972 - currentPos.getX()) <= 8.0 || delay.time() >= 3.0) {
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_RETURN2);
                }else{
                    intake.setPower(0.15);
                    saved_deposit = new Pose2d(-22.075, -76.972, 0.0);
                    if (Math.abs(-76.972 - currentPos.getX()) < 25){
                        flip.operate(0);
                        flip.clamp();
                    }
                    else if (Math.abs(-76.972 - currentPos.getX()) < 45){
                        flip.flipDown();
                        flip.clamp();
                    }
                    if(delay.time() >= 3.0){
                        intake.close();
                    }else{
                        intake.open();
                    }
                    localizer.GoTo(saved_deposit, 1.0,1.0,1.0);
                }
                break;
            case STATE_DEPOSIT2:
                if(mStateTime.time() >= 0.5){
                    newState(State.STATE_RETURN2);

                }else{
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                }
                break;
            case STATE_RETURN2:
                if (delay.time() >= 1.0) {
                    if (Math.abs(return_target2.getY() - currentPos.getX()) <= 5.0 && Math.abs(return_target2.getHeading() - currentPos.getHeading()) <= 3.0) {
                        localizer.getDrive().setPower(0.0, 0.0, 0.0);
                        localizer.getDrive().write();
                        newState(State.STATE_INTAKE3);
                    } else {
                        if (currentPos.getX() >= (return_target2.getY() - 8)) {
                            localizer.GoTo(return_target, 1.0, 1.0, 1.0);
                        } else {
                            localizer.GoTo(new Pose2d(return_target2.getX(), return_target2.getY(), 0.0), 1.0, 1.0, 1.0);
                        }
                        if (currentPos.getX() >= return_target2.getY() / 3) {
                            intake.close();
                            intake.setPower(0.3);
                        } else {
                            intake.setPower(-1.0);
                        }
                        mStateTime.reset();
                    }
                    flip.start();
                }
                else{
                    if (delay.time() >= 0.5) {
                        flip.operate(1);
                    }
                    drive.setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                }
                break;
            case STATE_INTAKE3:
                flip.read();
                if(Math.abs(intake3.getX() + currentPos.getY()) <= 3.0 || flip.IntakeFeedback()){
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_EXITPOOL3);
                }
                else if (delay.time() >= 3.0){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_EXITPOOL3);
                }
                else{
                    localizer.GoTo(intake3, 0.2, 0.2, 0.2);
                    mStateTime.reset();
                    intake.close();
                    if(flip.IntakeFeedback()){
                        intake.setPower(0.0);
                    }else{
                        intake.setPower(0.3);
                    }
                }
                break;
            case STATE_EXITPOOL3:
                if (delay.time() >= 0.3) {
                    if (Math.abs(return_target2.getY() - currentPos.getX()) <= 4.0) {
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_CROSS3);
                    } else if (delay.time() >= 2.5) {
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_CROSS3);
                    } else {
                        intake.setPower(0.0);
                        localizer.GoTo(new Pose2d(return_target2.getX(), return_target2.getY(), 0.0), 0.8, 0.8, 0.8);
                        mStateTime.reset();
                    }
                }
                intake.open();
                flip.operate(4);
                break;
            case STATE_CROSS3:
                if(Math.abs(-76.972 - currentPos.getX()) <= 8.0 || delay.time() >= 3.0) {
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_RETURN3);
                }else{
                    intake.setPower(0.15);
                    saved_deposit = new Pose2d(-22.075, -76.972, 0.0);

                    if (Math.abs(-76.972 - currentPos.getX()) < 25){
                        flip.operate(0);
                        flip.clamp();
                    }
                    else if (Math.abs(-76.972 - currentPos.getX()) < 45){
                        flip.flipDown();
                        flip.clamp();
                    }

                    flip.operate(4);
                    if(delay.time() >= 3.0){
                        intake.close();
                    }else{
                        intake.open();
                    }
                    localizer.GoTo(saved_deposit, 1.0,1.0,1.0);
                }

                break;
            case STATE_RETURN3:
                if (delay.time() >= 1.0) {
                    if (Math.abs(return_target3.getY() - currentPos.getX()) <= 8.0 && Math.abs(return_target3.getHeading() - currentPos.getHeading()) <= 8.0) {
                        localizer.getDrive().setPower(0.0, 0.0, 0.0);
                        localizer.getDrive().write();
                        newState(State.STATE_INTAKE4);
                    } else {
                        if (currentPos.getX() >= (return_target3.getY() - 8)) {
                            localizer.GoTo(return_target, 1.0, 1.0, 1.0);
                        } else {
                            localizer.GoTo(new Pose2d(return_target3.getX(), return_target3.getY(), 0.0), 1.0, 1.0, 1.0);
                        }
                        if (currentPos.getX() >= return_target3.getY() / 3) {
                            intake.close();
                            intake.setPower(0.3);
                        } else {
                            intake.setPower(-1.0);
                        }
                        mStateTime.reset();
                    }
                    flip.start();
                }
                else{
                    if (delay.time() >= 0.5) {
                        flip.operate(1);
                    }
                    drive.setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                }
                break;
            case STATE_INTAKE4:
                flip.read();
                if(Math.abs(intake4.getX() + currentPos.getY()) <= 3.0 || flip.IntakeFeedback()){
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_EXITPOOL4);
                }
                else{
                    localizer.GoTo(intake4, 0.2, 0.2, 0.2);
                    mStateTime.reset();
                    intake.close();
                    if(flip.IntakeFeedback()){
                        intake.setPower(0.0);
                    }else{
                        intake.setPower(0.3);
                    }
                }
                break;
            case STATE_CROSS4:
                if(Math.abs(-76.972 - currentPos.getX()) <= 8.0 || delay.time() >= 3.0) {
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_PARK);
                }else{
                    intake.setPower(0.15);
                    saved_deposit = new Pose2d(-22.075, -76.972, 0.0);
                    if (Math.abs(-76.972 - currentPos.getX()) < 25){
                        flip.operate(0);
                        flip.clamp();
                    }
                    else if (Math.abs(-76.972 - currentPos.getX()) < 45){
                        flip.flipDown();
                        flip.clamp();
                    }
                    if(delay.time() >= 3.0){
                        intake.close();
                    }else{
                        intake.open();
                    }
                    localizer.GoTo(saved_deposit, 1.0,1.0,1.0);
                }

                break;
            case STATE_EXITPOOL4:
                if (delay.time() >= 0.3) {
                    if (Math.abs(exit_pool4.getY() - currentPos.getX()) <= 4.0) {
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_CROSS4);
                    }else {
                        intake.setPower(0.0);
                        localizer.GoTo(exit_pool4, 0.8, 0.8, 0.8);
                        mStateTime.reset();
                    }
                }
                break;
            case STATE_PARK:
                if (delay.time() >= 1.0) {
                    if (Math.abs(-20 - currentPos.getX()) <= 3.0) {
                        localizer.getDrive().setPower(0.0, 0.0, 0.0);
                        localizer.getDrive().write();
                        newState(State.STATE_STOP);
                        telemetry.addData("Tee Hee :)", "Deal with it this is my senior year");
                    }else {
                        if (delay.time() >= 2.0) {
                            intake.setPower(0.0);
                        } else {
                            intake.setPower(-1.0);
                        }
                        intake.close();
                        intake.setPower(0.0);
                        localizer.GoTo(new Pose2d(-18.922, -35.976, 0.0), 1.0, 1.0, 1.0);
                        mStateTime.reset();
                    }
                }
                else{
                    if (delay.time() >= 0.5){
                        flip.operate(1);
                    }
                }
                break;
            case STATE_STOP:
                localizer.getDrive().setPower(0.0,0.0,0.0);
                localizer.getDrive().write();
                drive.setPower(0.0, 0.0, 0.0);

                telemetry.addData("Tee Hee :)", "Deal with it this is my senior year");
                break;
        }

        slides.write();
        if (mRobotState != State.STATE_PUSH_BACK) {
            drive.write();
        }
        flip.write();
        intake.write();
    }

    private void newState(State state){
        mStateTime.reset();
        delay.reset();
        localizer.reset();
        mRobotState = state;
    }
}*/
