package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

public class QL_Skystone_Auto_FOWARD extends OpMode {
    int SkystonePos;

    public OpenCvCamera webcam;
    //MultipleTelemetry telemetry;
    SkystoneDetectorPipeline pipeline;

    ThreeTrackingWheelLocalizer odos;
    SRX_Three_Wheel_Localizer localizer;
    Vertical_Elevator slides;

    Mecanum_Drive drive;
    Intake intake;

    Servo bservo;

    double memo = 0.0;
    double memoL = 0.0;
    double memoR = 0.0;
    Flipper flip;

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;

    ElapsedTime mStateTime = new ElapsedTime();
    ElapsedTime delay = new ElapsedTime();
    ElapsedTime motionProfile = new ElapsedTime();

    private enum State{
        STATE_SETUP,
        STATE_DRIVE_TO_BLOCK,
        STATE_INTAKE,
        STATE_EXIT_POOL,
        STATE_CROSS,
        STATE_DRIVE_TO_FOUNDATION,
        STATE_GRAB_FOUNDATION,
        STATE_PULL_FORWARD,
        STATE_TURN,
        STATE_PUSH_BACK,
        STATE_RELEASE,
        STATE_STRAFE,
        STATE_RETURN,
        STATE_INTAKE2,
        STATE_CROSS2,
        STATE_DEPOSIT,
        STATE_DEPOSIT2,
        STATE_RETURN2,
        STATE_PARK,
        STATE_HANDSHAKE,
        STATE_HANDSHAKE2
    }
    State mRobotState = State.STATE_SETUP;

    public void init(){
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        drive = new Mecanum_Drive(hardwareMap, telemetry);
        flip = new Flipper(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        bservo = hardwareMap.get(Servo.class, "Bmover");

        bservo.setPosition(0.0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDetectorPipeline();
        webcam.openCameraDevice();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        bservo = hardwareMap.get(Servo.class, "Bmover");

        slides = new Vertical_Elevator(hardwareMap, telemetry);
        localizer = new SRX_Three_Wheel_Localizer(new SRX_Encoder("intake_left", hardwareMap), new SRX_Encoder("intake_right", hardwareMap), new SRX_Encoder("lift_2", hardwareMap), hardwareMap, telemetry);
        odos = new ThreeWheelTrackingLocalizer(this.hardwareMap);
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
        delay.startTime();
        newState(State.STATE_SETUP);
    }

    Pose2d cross_target;
    Pose2d return_target;
    Pose2d intake1;
    Pose2d intake2;
    Pose2d drive_to_block_target;
    Pose2d drive_to_block_target2;
    Pose2d drive_platform_target;
    Pose2d exit_pool;
    Pose2d return_target2;
    Pose2d saved_deposit = new Pose2d(0.0,0.0,0.0);
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
        SkystonePos = 2;
        flip.read();
        odos.update();
        Pose2d currentPos = odos.getPoseEstimate();

        if(SkystonePos == 0){
            cross_target = new Pose2d();
        }else if(SkystonePos == 1){
            cross_target = new Pose2d();
            //Target Dist for Return is equal to ---
        }else if(SkystonePos == 2){
            cross_target = new Pose2d(-90, 25, Math.toRadians(179.0));
            drive_to_block_target = new Pose2d(-7, 28, -Math.PI/4 - Math.toRadians(5.0));
            return_target = new Pose2d(-27, 23.0, -Math.PI/4 - Math.toRadians(10.0));
            exit_pool = new Pose2d(-10.5, 22, -Math.PI/2);
            //drive_to_block_target2 = new Pose2d(-25, -2.0, Math.PI/4);
            intake1 = new Pose2d(10, 35, -Math.PI/4 - Math.toRadians(5.0));
            intake2 = new Pose2d(-18, 35, -Math.PI/4);
            //return_target2 = new Pose2d(-25, -13.0, Math.PI/4 + Math.toRadians(10.0));
        }

        switch (mRobotState) {
            case STATE_SETUP:
                if(mStateTime.time() >= 1.5){
                    newState(State.STATE_DRIVE_TO_BLOCK);
                }else{
                    intake.kickout();
                    intake.open();
                    flip.start();
                }
                break;
            case STATE_DRIVE_TO_BLOCK:
                if(Math.abs(drive_to_block_target.getY() - currentPos.getX()) <= 3.0) {
                    if(mStateTime.time() >= 0.5){
                        drive.setPower(0.0,0.0,0.0);
                        memo = localizer.getForwardDist();
                        newState(State.STATE_INTAKE);
                    }else{
                        localizer.GoTo(drive_to_block_target, 0.6, 0.6, 0.6);
                    }
                }else{
                    intake.setPower(0.0);
                    intake.close();
                    localizer.GoTo(drive_to_block_target, 0.6, 0.6, 0.6);
                    mStateTime.reset();
                }
                break;
            case STATE_INTAKE:
                if(Math.abs(intake1.getY() - currentPos.getX()) <= 3.0){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_EXIT_POOL);
                }else{
                    //drive.setPower(-0.15, 0.0, 0.0);
                    intake.close();
                    localizer.GoTo(intake1, 0.15, 0.15, 0.15);
                    intake.setPower(0.3);
                    /*
                    if(mStateTime.time() <= 1.5){
                        intake.setPower(0.3);
                    }else if(mStateTime.time() > 1.5 && mStateTime.time() <= 1.6){
                        intake.setPower(-0.2);
                    }else if(mStateTime.time() > 1.6){
                        intake.setPower(0.3);
                    }
                     */
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
                        flip.unclamp();
                        flip.operate(4);
                    }
                }
                break;
            case STATE_EXIT_POOL:
                if(Math.abs(exit_pool.getY() - currentPos.getX()) <= 2.0) {
                    if(mStateTime.time() >= 0.5){
                        drive.setPower(0.0,0.0,0.0);
                        intake.setPower(0.0);
                        newState(State.STATE_CROSS);
                    }else{
                        localizer.GoTo(exit_pool, 0.5, 0.5, 0.5);
                    }
                }else if(delay.time() >= 5.0){
                    drive.setPower(0.0,0.0,0.0);
                    intake.setPower(0.0);
                    newState(State.STATE_CROSS);
                }else{
                    flip.clamp();
                    localizer.GoTo(exit_pool, 0.5, 0.5, 0.5);
                    mStateTime.reset();
                }
                break;
            case STATE_CROSS:
                if(currentPos.getY() <= -85) {
                    if(mStateTime.time() >= 0.5){
                        intake.close();
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_DRIVE_TO_FOUNDATION);
                    }else{
                        localizer.GoTo(cross_target, 0.5, 0.5, 0.5);
                    }
                }else if(mStateTime.time() >= 2.0){
                    intake.close();
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_DRIVE_TO_FOUNDATION);
                }else{
                        /*
                        if(delay.time() >= 1.0){
                            intake.close();
                        }else{
                            intake.open();
                        }
                         */
                    intake.setPower(0.15);
                    if(currentPos.getY() <= -45){
                        localizer.GoTo(cross_target, 0.5, 0.5, 0.5);
                    }else{
                        localizer.GoTo(new Pose2d(cross_target.getX(), cross_target.getY(), 0.0), 0.5, 0.5, 0.5);
                    }

                    mStateTime.reset();

                    if(currentPos.getY() <= -50) {
                        intake.open();
                        flip.unclamp();
                        flip.operate(4);
                    }
                }
                telemetry.addData("Is Grabbed? ", flip.isGrabbed());
                break;
            case STATE_DRIVE_TO_FOUNDATION:
                if(Math.abs(31 - currentPos.getX()) <= 2.0) {
                    if(mStateTime.time() >= 1.0){
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_PULL_FORWARD);
                    }else{
                        flip.grabPlatform();
                        localizer.GoTo(new Pose2d(-90, 31, -Math.PI), 0.15, 0.15, 0.15);
                        intake.setPower(0.0);
                    }
                }else{
                    localizer.GoTo(new Pose2d(-90, 31, -Math.PI), 0.3, 0.3, 0.3);
                    mStateTime.reset();
                }
                break;
            case STATE_PULL_FORWARD:
                if(Math.abs(9 - currentPos.getX()) <= 2.0) {
                    if(mStateTime.time() >= 0.5){
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_TURN);
                    }else{
                        localizer.GoTo(new Pose2d(-90, -9, -Math.PI), 1.0, 1.0, 1.0);
                    }
                }else if(mStateTime.time() >= 3.0){
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_TURN);
                }else{
                    flip.operate(2);
                    localizer.GoTo(new Pose2d(-90, 9, -Math.PI), 1.0, 1.0, 1.0);
                    mStateTime.reset();
                }
                break;
            case STATE_TURN:
                if(Math.abs(getHeading()) <= 1.0) {
                    if(mStateTime.time() >= 0.5){
                        flip.resetPlatform();
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_DEPOSIT);
                    }else{
                        //drive.targetTurnPlatform(0.0, odos.getPoseEstimate().getHeading());
                        localizer.GoTo(new Pose2d(-85, 9, -Math.PI/4), 0.5, 0.5, 1.0);
                    }
                }else{
                    //drive.targetTurnPlatform(0.0, odos.getPoseEstimate().getHeading());
                    localizer.GoTo(new Pose2d(-85, 9, -Math.PI/4), 0.5, 0.5, 1.0);
                    mStateTime.reset();
                }
                break;
            case STATE_DEPOSIT:
                if(mStateTime.time() >= 1.2){
                    if(mStateTime.time() >= 2.0){
                        saved_deposit = new Pose2d(-((currentPos.getX() * 1.3030303) + 0.75), currentPos.getY() - 2, 0.0);
                        newState(State.STATE_RETURN);
                    }else{
                        flip.operate(1);
                    }
                }else{
                    flip.operate(0);
                    localizer.GoTo(new Pose2d(-85, 9, -Math.PI/2), 0.5, 0.5, 1.0);
                }
                break;
            case STATE_STRAFE:
                if(delay.time() >= 0.5){
                    if(Math.abs(26 - currentPos.getX()) <= 2.5) {
                        flip.start();
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_RETURN);
                    }else{
                        if(delay.time() >= 2.0){
                            flip.start();
                        }
                        localizer.GoTo(new Pose2d(-85, 26, -Math.PI/2), 0.5, 0.5, 0.5);
                        mStateTime.reset();
                    }
                }
                break;
            case STATE_RETURN:
                if(Math.abs(return_target.getX() + currentPos.getY()) <= 3.0) {
                    if(mStateTime.time() >= 0.5){
                        drive.setPower(0.0,0.0,0.0);
                        memo = localizer.getForwardDist();
                        newState(State.STATE_INTAKE2);
                    }else{
                        localizer.GoTo(return_target, 0.5, 0.5, 0.8);
                    }
                }else if(delay.time() >= 3.0){
                    drive.setPower(0.0,0.0,0.0);
                    memo = localizer.getForwardDist();
                    newState(State.STATE_INTAKE2);
                }else{
                    flip.operate(2);
                    flip.clamp();
                    if(currentPos.getY() >= (return_target.getX() - 2)){
                        localizer.GoTo(return_target, 0.5,0.5,0.8);
                    }else{
                        localizer.GoTo(return_target, 0.5,0.5,0.001);
                    }
                    if(currentPos.getY() >= return_target.getX()/3){
                        intake.close();
                        intake.setPower(0.3);
                    }
                    mStateTime.reset();
                }
                break;
            case STATE_INTAKE2:
                if(Math.abs(intake2.getY() - currentPos.getX()) <= 3.0){
                    if(mStateTime.time() >= 0.5){
                        drive.setPower(0.0, 0.0, 0.0);
                        intake.setPower(0.15);
                        newState(State.STATE_HANDSHAKE2);
                    }else{
                        localizer.GoTo(intake2, 0.15, 0.15, 0.15);
                        intake.close();
                        if(mStateTime.time() <= 1.5){
                            intake.setPower(0.3);
                        }else if(mStateTime.time() > 1.5 && mStateTime.time() <= 1.7){
                            intake.setPower(-0.2);
                        }else if(mStateTime.time() > 1.7){
                            intake.setPower(0.3);
                        }
                    }
                }else{
                    //drive.setPower(-0.15, 0.0, 0.0);
                    localizer.GoTo(intake2, 0.15, 0.15, 0.15);
                    mStateTime.reset();
                    intake.close();
                    if(mStateTime.time() <= 1.0){
                        intake.setPower(0.3);
                    }else if(mStateTime.time() > 1.0 && mStateTime.time() <= 2.0){
                        intake.setPower(-0.3);
                    }else if(mStateTime.time() > 2.0){
                        intake.setPower(0.3);
                    }
                }
                break;
            case STATE_HANDSHAKE2:
                if(mStateTime.time() >= 1.0){
                    newState(State.STATE_CROSS2);
                }else{
                    intake.open();
                    flip.operate(4);
                }
                break;
            case STATE_CROSS2:
                if(Math.abs(-80 + currentPos.getY()) <= 4.0) {
                    if(mStateTime.time() >= 1.0){
                        flip.flipDown();
                        intake.open();
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_DEPOSIT2);
                    }else{
                        //localizer.GoTo(new Pose2d(exit_pool.getX(), -80, 0.0), 0.5, 0.5, 0.8);
                        localizer.GoTo(saved_deposit, 0.7, 0.7, 0.9);
                    }
                }else if(mStateTime.time() >= 2.0){
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_DEPOSIT2);
                }else{
                    flip.clamp();
                    localizer.GoTo(saved_deposit, 0.7,0.7,0.9);
                    mStateTime.reset();
                }
                break;
            case STATE_DEPOSIT2:
                if(mStateTime.time() >= 1.2){
                    if(mStateTime.time() >= 2.0){
                        newState(State.STATE_PARK);
                    }else{
                        flip.operate(1);
                    }
                }else{
                    flip.operate(0);
                    //slides.PIDController(1);
                    localizer.GoTo(saved_deposit, 0.7, 0.7, 1.0);
                }
                break;
            case STATE_PARK:
                if(Math.abs(-30 + currentPos.getY()) <= 3.0) {
                    if(mStateTime.time() >= 0.5){
                        drive.setPower(0.0,0.0,0.0);
                    }else{
                        localizer.GoTo(new Pose2d(-30, 25, -Math.PI/2), 0.5, 0.5, 0.8);
                    }
                }else if(delay.time() >= 3.0){
                    drive.setPower(0.0,0.0,0.0);
                }else{
                    flip.operate(2);
                    flip.clamp();
                    intake.close();
                    intake.setPower(0.0);
                    localizer.GoTo(new Pose2d(-30, 25, 0.0), 0.5,0.5,0.8);
                    mStateTime.reset();
                }
                break;
        }
        telemetry.addData("POS: ", currentPos.toString());
        telemetry.addData("State: ", mRobotState);
        telemetry.addData("time: ", mStateTime.time());
        telemetry.addData("Power: ", drive.getOverallPower());
        telemetry.addData("Saved Pos: ", saved_deposit.toString());

        slides.write();
        drive.write();
        flip.write();
        intake.write();
    }

    private void newState(State state){
        mStateTime.reset();
        delay.reset();
        localizer.reset();
        mRobotState = state;
    }
}
