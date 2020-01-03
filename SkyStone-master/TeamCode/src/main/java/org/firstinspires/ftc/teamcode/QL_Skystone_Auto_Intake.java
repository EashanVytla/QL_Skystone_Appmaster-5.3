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

@Autonomous(name = "QL_SkyStone_Blue", group = "Competition")
public class QL_Skystone_Auto_Intake extends OpMode {
    int SkystonePos;

    public OpenCvCamera webcam;
    //MultipleTelemetry telemetry;
    SkystoneDetectorPipeline pipeline;

    ThreeTrackingWheelLocalizer odos;
    SRX_Three_Wheel_Localizer localizer;

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
            cross_target = new Pose2d(-25, -90, -Math.PI/2);
            drive_to_block_target = new Pose2d(-25, -2.0, Math.PI/4 + Math.toRadians(5.0));
            return_target = new Pose2d(-25, -30.0, Math.PI/4 - Math.toRadians(10.0));
            exit_pool = new Pose2d(-22, -10.5, 0.0);
            drive_to_block_target2 = new Pose2d(-25, -2.0, Math.PI/4);
            intake1 = new Pose2d(-35, 10, Math.PI/4 + Math.toRadians(5.0));
            intake2 = new Pose2d(-35, -18, Math.PI/4);
            return_target2 = new Pose2d(-25, -13.0, Math.PI/4 - Math.toRadians(10.0));
        }

        switch (mRobotState) {
            case STATE_SETUP:
                if(mStateTime.time() >= 1.0){
                    newState(State.STATE_DRIVE_TO_BLOCK);
                }else{
                    intake.kickout();
                    flip.start();
                }
                break;
            case STATE_DRIVE_TO_BLOCK:
                if(Math.abs(drive_to_block_target.getX() + currentPos.getY()) <= 2.0) {
                    if(mStateTime.time() >= 0.5){
                        drive.setPower(0.0,0.0,0.0);
                        memo = localizer.getForwardDist();
                        newState(State.STATE_INTAKE);
                    }else{
                        localizer.GoTo(drive_to_block_target, 0.5, 0.5, 0.5);
                    }
                }else{
                    intake.setPower(0.0);
                    intake.close();
                    localizer.GoTo(drive_to_block_target, 0.5, 0.5, 0.5);
                    mStateTime.reset();
                }
                break;
            case STATE_INTAKE:
                if(Math.abs(intake1.getY() - currentPos.getX()) <= 3.0){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_HANDSHAKE);
                }else{
                    //drive.setPower(-0.15, 0.0, 0.0);
                    intake.close();
                    localizer.GoTo(intake1, 0.15, 0.15, 0.15);
                    if(mStateTime.time() <= 1.5){
                        intake.setPower(0.3);
                    }else if(mStateTime.time() > 1.5 && mStateTime.time() <= 1.7){
                        intake.setPower(-0.2);
                    }else if(mStateTime.time() > 1.7){
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
                        intake.open();
                        localizer.GoTo(exit_pool, 0.5, 0.5, 0.5);
                        mStateTime.reset();
                    }
                break;
            case STATE_CROSS:
                    if(currentPos.getX() <= -85) {
                        if(mStateTime.time() >= 0.5){
                            intake.close();
                            drive.setPower(0.0,0.0,0.0);
                            newState(State.STATE_DRIVE_TO_FOUNDATION);
                        }else{
                            localizer.GoTo(cross_target, 0.5, 0.5, 0.3);
                        }
                    }else if(mStateTime.time() >= 2.0){
                        intake.close();
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_DRIVE_TO_FOUNDATION);
                    }else{
                        flip.operate(2);
                        if(delay.time() >= 1.0){
                            intake.close();
                        }else{
                            intake.open();
                        }
                        intake.setPower(0.15);
                        if(currentPos.getX() <= -63){
                            localizer.GoTo(cross_target, 0.5, 0.5, 0.5);
                        }else{
                            localizer.GoTo(cross_target, 0.5, 0.5, 0.001);
                        }
                        mStateTime.reset();
                    }
                    telemetry.addData("Is Grabbed? ", flip.isGrabbed());
                    break;
            case STATE_DRIVE_TO_FOUNDATION:
                    if(Math.abs(-31 + currentPos.getY()) <= 2.0) {
                        if(mStateTime.time() >= 1.0){
                            drive.setPower(0.0,0.0,0.0);
                            newState(State.STATE_PULL_FORWARD);
                        }else{
                            flip.grabPlatform();
                            localizer.GoTo(new Pose2d(-31, -90, -Math.PI/2), 0.15, 0.15, 0.15);
                            intake.setPower(0.0);
                        }
                    }else{
                        localizer.GoTo(new Pose2d(-31, -90, -Math.PI/2), 0.3, 0.3, 0.3);
                        mStateTime.reset();
                    }
                break;
            case STATE_PULL_FORWARD:
                    if(Math.abs(-9 + currentPos.getY()) <= 2.0) {
                        if(mStateTime.time() >= 0.5){
                            drive.setPower(0.0,0.0,0.0);
                            newState(State.STATE_TURN);
                        }else{
                            localizer.GoTo(new Pose2d(-9, -90, -Math.PI/2), 1.0, 1.0, 1.0);
                        }
                    }else if(mStateTime.time() >= 3.0){
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_TURN);
                    }else{
                        flip.operate(2);
                        localizer.GoTo(new Pose2d(-9, -90, -Math.PI/2), 1.0, 1.0, 1.0);
                        mStateTime.reset();
                    }
                break;
            case STATE_TURN:
                    if(Math.abs(getHeading()) <= 1.0) {
                        if(mStateTime.time() >= 0.5){
                            flip.resetPlatform();
                            drive.setPower(0.0,0.0,0.0);
                            flip.operate(1);
                            newState(State.STATE_STRAFE);
                        }else{
                            flip.operate(0);
                            //drive.targetTurnPlatform(0.0, odos.getPoseEstimate().getHeading());
                            localizer.GoTo(new Pose2d(-9, -85, Math.PI/4 + Math.toRadians(10.0)), 0.5, 0.5, 0.5);
                        }
                    }else{
                        if(delay.time() >= 5.0){
                            flip.operate(0);
                        }
                        //drive.targetTurnPlatform(0.0, odos.getPoseEstimate().getHeading());
                        localizer.GoTo(new Pose2d(-9, -85, Math.PI/4 + Math.toRadians(15.0)), 0.5, 0.5, 0.5);
                        mStateTime.reset();
                    }
                break;
            case STATE_STRAFE:
                if(delay.time() >= 0.5){
                    if(Math.abs(-26 + currentPos.getY()) <= 2.5) {
                            flip.start();
                            drive.setPower(0.0,0.0,0.0);
                            newState(State.STATE_RETURN);
                    }else{
                        if(delay.time() >= 2.0){
                            flip.start();
                        }
                        localizer.GoTo(new Pose2d(-26, -85, 0.0), 0.5, 0.5, 0.5);
                        mStateTime.reset();
                    }
                }
                break;
            case STATE_RETURN:
                if(Math.abs(return_target.getY() - currentPos.getX()) <= 2.0) {
                    if(mStateTime.time() >= 0.5){
                        drive.setPower(0.0,0.0,0.0);
                        memo = localizer.getForwardDist();
                        newState(State.STATE_INTAKE2);
                    }else{
                        localizer.GoTo(return_target, 0.5, 0.5, 0.8);
                    }
                }else if(mStateTime.time() >= 2.0){
                    drive.setPower(0.0,0.0,0.0);
                    memo = localizer.getForwardDist();
                    newState(State.STATE_INTAKE2);
                }else{
                    flip.clamp();
                    if(currentPos.getX() >= (return_target.getY() - 10)){
                        localizer.GoTo(return_target, 0.5,0.5,0.8);
                    }else{
                        localizer.GoTo(return_target, 0.5,0.5,0.001);
                    }
                    if(currentPos.getX() >= return_target.getY()/3){
                        intake.close();
                        intake.setPower(0.3);
                    }
                    mStateTime.reset();
                }
                break;
            case STATE_INTAKE2:
                if(Math.abs(intake2.getX() + currentPos.getY()) <= 3.0){
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
                if(Math.abs(-80 - currentPos.getX()) <= 4.0) {
                    if(mStateTime.time() >= 0.5){
                        flip.flipDown();
                        intake.open();
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_DEPOSIT);
                    }else{
                        localizer.GoTo(new Pose2d(exit_pool.getX(), -80, 0.0), 0.5, 0.5, 0.8);
                    }
                }else if(mStateTime.time() >= 2.0){
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_DEPOSIT);
                }else{
                    flip.clamp();
                    localizer.GoTo(new Pose2d(exit_pool.getX(), -80, 0.0), 0.5,0.5,0.5);
                    mStateTime.reset();
                }
                break;
            case STATE_DEPOSIT:
                if(mStateTime.time() >= 0.5){
                    if(mStateTime.time() >= 1.0){
                        newState(State.STATE_PARK);
                    }else{
                        flip.unclamp();
                    }
                }else{
                    flip.operate(0);
                }
                break;
            case STATE_PARK:
                if(Math.abs(-45 - currentPos.getX()) <= 2.0) {
                    if(mStateTime.time() >= 0.5){
                        intake.setPower(0.0);
                        drive.setPower(0.0,0.0,0.0);
                    }else{
                        localizer.GoTo(new Pose2d(exit_pool.getX(), -45, 0.0), 0.5, 0.5, 0.8);
                    }
                }else if(mStateTime.time() >= 2.0){
                    drive.setPower(0.0,0.0,0.0);
                }else{
                    flip.clamp();
                    localizer.GoTo(new Pose2d(exit_pool.getX(), -45, 0.0), 0.5,0.5,0.8);
                    mStateTime.reset();
                }
                break;
        }
        telemetry.addData("POS: ", currentPos.toString());
        telemetry.addData("State: ", mRobotState);
        telemetry.addData("time: ", mStateTime.time());
        telemetry.addData("Power: ", drive.getOverallPower());

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
