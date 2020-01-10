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

@Autonomous(name = "QL_SkyStone_Blue", group = "Competition")
public class BLUE_QL_Skystone_Auto_Intake extends OpMode {
    int SkystonePos;

    public OpenCvCamera webcam;
    //MultipleTelemetry telemetry;
    SkystoneDetectorPipeline pipeline;

    ThreeWheelTrackingLocalizer odos;
    SRX_Three_Wheel_Localizer localizer;
    Vertical_Elevator slides;

    Mecanum_Drive drive;
    Intake intake;

    Servo bservo;

    double memo = 0.0;
    double savedDepositY = 0.0;
    double savedDepositX = 0.0;
    double storedY = 0.0;
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
        STATE_HANDSHAKE2,
        STATE_EXITPOOL2
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
        RevBulkData data = hub2.getBulkInputData();
        flip.read();
        odos.dataUpdate(data);
        odos.update();
        localizer.update(data);
        Pose2d currentPos = odos.getPoseEstimate();

        if(SkystonePos == 0){
            cross_target = new Pose2d(-21, -100, -Math.PI/2);
            //drive_to_block_target = new Pose2d(-24, 12.0, Math.toRadians(55.983)); //Math.PI/4 + Math.toRadians(10)
            drive_to_block_target = new Pose2d(-25.992, 18.104, Math.toRadians(50));
            return_target = new Pose2d(-26, -14.0, Math.toRadians(36.0));
            exit_pool = new Pose2d(-22, -10.5, 0.0);
            //drive_to_block_target2 = new Pose2d(-21, -2.0, Math.PI/4);
            //intake1 = new Pose2d(-44, 15.104, Math.toRadians(55.983));
            //intake2 = new Pose2d(-53, -13, Math.PI/4);
            intake1 = new Pose2d(-37.337, 16.276, Math.toRadians(50));
            intake2 = new Pose2d(-45, -1.58, Math.toRadians(36.0));
            //return_target2 = new Pose2d(-25, -13.0, Math.PI/4 + Math.toRadians(10.0));
        }else if(SkystonePos == 1){
            cross_target = new Pose2d(-21, -100, -Math.PI/2);
            drive_to_block_target = new Pose2d(-24.992, 4.0, Math.PI/4);
            return_target = new Pose2d(-26, -17.0, Math.PI/4 + Math.toRadians(10.0));
            exit_pool = new Pose2d(-22, -10.5, 0.0);
            //drive_to_block_target2 = new Pose2d(-25, -2.0, Math.PI/4);
            intake1 = new Pose2d(-37.337, 10, Math.PI/4);
            intake2 = new Pose2d(-45, -14, Math.PI/4);
            //return_target2 = new Pose2d(-25, -13.0, Math.PI/4 + Math.toRadians(10.0));
        }else if(SkystonePos == 2){
            cross_target = new Pose2d(-21, -100, -Math.PI/2);
            drive_to_block_target = new Pose2d(-24.992, -6, Math.PI/4);
            return_target = new Pose2d(-24, -26.0, Math.PI/4 + Math.toRadians(10.0));
            exit_pool = new Pose2d(-22, -10.5, 0.0);
            //drive_to_block_target2 = new Pose2d(-25, -2.0, Math.PI/4);
            intake1 = new Pose2d(-37.337, 2, Math.PI/4);
            intake2 = new Pose2d(-38, -3, Math.PI/4);
        }

        switch (mRobotState) {
            case STATE_SETUP:
                if(mStateTime.time() >= 1.0){
                    newState(State.STATE_DRIVE_TO_BLOCK);
                }else{
                    intake.kickout();
                    intake.open();
                }
                break;
            case STATE_DRIVE_TO_BLOCK:
                if(Math.abs(drive_to_block_target.getX() + currentPos.getY()) <= 2.0 && Math.abs(drive_to_block_target.getHeading() + currentPos.getHeading()) <= 2.0) {
                    if(mStateTime.time() >= 0.5){
                        drive.setPower(0.0,0.0,0.0);
                        memo = localizer.getForwardDist();
                        newState(State.STATE_INTAKE);
                    }else{
                        localizer.GoTo(drive_to_block_target, 0.6, 0.6, 0.8);
                    }
                }else{
                    flip.start();
                    intake.setPower(0.0);
                    //intake.close();
                    localizer.GoTo(drive_to_block_target, 0.6, 0.6, 0.8);
                    mStateTime.reset();
                }
                if (delay.time() >= 1.0){
                    intake.close();
                }
                break;
            case STATE_INTAKE:
                if(Math.abs(intake1.getX() + currentPos.getY()) <= 1.5 && delay.time() >= 2.0 || flip.IntakeFeedback()){
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_EXIT_POOL);
                }else{
                    //drive.setPower(-0.15, 0.0, 0.0);
                    intake.close();
                    localizer.GoTo(intake1, 0.2, 0.2, 0.2);
                    //localizer.getDrive().setPower(-0.2, 0.0, 0.0);
                    //localizer.getDrive().write();
                    if(flip.IntakeFeedback()){
                        intake.setPower(0.0);
                    }else{
                        /*
                        if (mStateTime.time() <= 1.4) {
                            intake.setPower(0.3);
                        } else if (mStateTime.time() > 1.4 && mStateTime.time() <= 1.5) {
                            intake.setPower(-0.3);
                        } else if (mStateTime.time() > 1.6) {
                            intake.setPower(0.3);
                        }
                         */
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
                if(Math.abs(exit_pool.getY() - currentPos.getX()) <= 4.0) {
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_CROSS);
                }else if(delay.time() >= 2.5){
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_CROSS);
                }else{
                    flip.clamp();
                    intake.close();
                    intake.setPower(0.0);
                    localizer.GoTo(exit_pool, 0.8, 0.8, 0.8);
                    mStateTime.reset();
                }
                break;
            case STATE_CROSS:
                if(currentPos.getX() <= -85) {
                    if(mStateTime.time() >= 0.5){
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_DRIVE_TO_FOUNDATION);
                    }else{
                        localizer.GoTo(cross_target, 0.15, 0.15, 0.15);
                    }
                }else if(mStateTime.time() >= 2.0){
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

                    /*
                    if(mStateTime.time() <= 1.0){
                        intake.setPower(0.15);
                    }else if(mStateTime.time() > 1.0 && mStateTime.time() <= 1.2){
                        intake.setPower(-0.15);
                    }else if(mStateTime.time() > 1.2){
                        intake.setPower(0.15);
                    }

                     */
                    intake.setPower(0.15);

                    if(currentPos.getX() <= -35){
                        localizer.GoTo(cross_target, 0.7, 0.7, 0.8);
                    }else{
                        localizer.GoTo(new Pose2d(cross_target.getX(), cross_target.getY(), 0.0), 0.7, 0.7, 0.7);
                    }

                    mStateTime.reset();

                    if(currentPos.getX() <= -55) {
                        intake.open();
                        flip.unclamp();
                        flip.operate(4);
                    }
                }
                telemetry.addData("Is Grabbed? ", flip.isGrabbed());
                break;
            case STATE_DRIVE_TO_FOUNDATION:
                /*
                if(Math.abs(-31 + currentPos.getY()) <= 2.0) {
                    if(mStateTime.time() >= 1.0){
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_PULL_FORWARD);
                    }else{
                        flip.grabPlatform();
                        localizer.GoTo(new Pose2d(-32, -90, -Math.PI/2), 0.15, 0.15, 0.15);
                        intake.setPower(0.0);
                    }
                }else{
                    localizer.GoTo(new Pose2d(-32, -90, -Math.PI/2), 0.15, 0.15, 0.15);
                    mStateTime.reset();
                }
                 */
                if(mStateTime.time() >= 0.3) {
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_GRAB_FOUNDATION);
                }else{
                    intake.open();
                    flip.flipDown();
                    flip.clamp();
                    localizer.GoTo(new Pose2d(-41, -90, -Math.PI/2), 0.3, 0.3, 0.3);
                }
                break;
            case STATE_GRAB_FOUNDATION:
                if(mStateTime.time() >= 0.75){
                    newState(State.STATE_PULL_FORWARD);
                }else{
                    flip.grabPlatform();
                    intake.close();
                    //flip.operate(0);
                }
                break;
            case STATE_PULL_FORWARD:
                if(Math.abs(-17 + currentPos.getY()) <= 4.0) {
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_TURN);
                    /*
                    if(mStateTime.time() >= 0.5){
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_TURN);
                    }else{
                        localizer.GoTo(new Pose2d(-17, -90, -Math.PI/2), 1.0, 1.0, 1.0);
                    }

                     */
                }else if(mStateTime.time() >= 3.0){
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_TURN);
                }else{
                    //flip.operate(1);
                    intake.setPower(-0.2);
                    localizer.GoTo(new Pose2d(-17, -90, -Math.PI/2), 1.0, 1.0, 1.0);
                    mStateTime.reset();
                }
                break;
            case STATE_TURN:
                if(getHeading() >= Math.toRadians(-10.0) || mStateTime.time() >= 3.0) {
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    //saved_deposit = new Pose2d(-((currentPos.getY() * 1.3030303) + 0.75), currentPos.getX() - 2, 0.0);
                    //storedY = currentPos.getY();
                    newState(State.STATE_DEPOSIT);
                }else{
                    //localizer.targetTurnPlatform(0.0, currentPos.getHeading());
                    //drive.setPower(-0.5, 0.0, 1.0);
                    localizer.GoTo(new Pose2d(-2, -62, Math.PI / 4), 1.0, 1.0, 1.0);
                    flip.operate(2);
                    //localizer.GoTo(new Pose2d(-9, -85, Math.PI/4), 0.5, 0.5, 1.0);
                }
                break;
            case STATE_DEPOSIT:
                if(mStateTime.time() >= 1.2){
                    if(mStateTime.time() >= 2.0){
                        //flip.unclamp();
                        storedY = currentPos.getX();
                        savedDepositX = currentPos.getY();
                        newState(State.STATE_RETURN);
                    }else{
                        flip.operate(1);
                    }
                }else{
                    flip.operate(0);
                    flip.resetPlatform();
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    //localizer.GoTo(new Pose2d(-9, -90, 0.0), 0.9, 0.9, 0.9);
                }
                break;
            case STATE_PUSH_BACK:
                if (mStateTime.time() >= 0.75){
                    flip.operate(1);
                }

                if (mStateTime.time() >= 1.0){
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    savedDepositY = currentPos.getX();
                    newState(State.STATE_RETURN);
                }
                else{
                    //localizer.GoTo(new Pose2d(26, 90, 0.0), 1.0, 1.0, 1.0);
                    localizer.getDrive().setPower(0.5, 0.0, 0.0);
                    localizer.getDrive().write();
                    flip.operate(2);
                }
                break;
            case STATE_STRAFE:
                if(delay.time() >= 0.5){
                    if(Math.abs(-23 + currentPos.getY()) <= 2.5) {
                        drive.setPower(0.0,0.0,0.0);
                        newState(State.STATE_PUSH_BACK);
                    }else{
                        if(delay.time() >= 2.0){
                            flip.start();
                        }
                        localizer.GoTo(new Pose2d(-23, storedY + 3, 0.0), 0.6, 0.6, 0.6);
                        mStateTime.reset();
                    }
                }
                break;
            case STATE_RETURN:
                if(Math.abs(return_target.getY() - currentPos.getX()) <= 5.0 && Math.abs(return_target.getHeading() - currentPos.getHeading()) <= 3.0) {
                    if(mStateTime.time() >= 0.5){
                        localizer.getDrive().setPower(0.0,0.0,0.0);
                        localizer.getDrive().write();
                        memo = localizer.getForwardDist();
                        newState(State.STATE_INTAKE2);
                    }else{
                        localizer.GoTo(return_target, 0.6, 0.6, 0.8);
                    }
                }else if(delay.time() >= 3.0){
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    memo = localizer.getForwardDist();
                    newState(State.STATE_INTAKE2);
                }else{
                    flip.resetPlatform();
                    if(currentPos.getX() >= (return_target.getY() - 2)){
                        localizer.GoTo(return_target, 0.6,0.6,0.8);
                    }else{
                        localizer.GoTo(new Pose2d(return_target.getX(), return_target.getY(), 0.0), 0.6,0.6,0.6);
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
                if(Math.abs(intake2.getX() + currentPos.getY()) <= 3.0 || flip.IntakeFeedback()){
                    if(mStateTime.time() >= 0.5){
                        localizer.getDrive().setPower(0.0, 0.0, 0.0);
                        localizer.getDrive().write();
                        newState(State.STATE_HANDSHAKE2);
                    }else{
                        localizer.GoTo(intake2, 0.2, 0.2, 0.2);
                        intake.close();
                        if(flip.IntakeFeedback()){
                            intake.setPower(0.0);
                        }else{
                            intake.setPower(0.3);
                        }
                    }
                }
                else if (delay.time() >= 3.0){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_HANDSHAKE2);
                }
                else{
                    //drive.setPower(-0.15, 0.0, 0.0);
                    if(delay.time() >= 0.75){
                        flip.operate(2);
                        flip.clamp();
                    }
                    localizer.GoTo(intake2, 0.2, 0.2, 0.2);
                    mStateTime.reset();
                    intake.close();
                    if(flip.IntakeFeedback()){
                        intake.setPower(0.0);
                    }else{
                        intake.setPower(0.3);
                        /*
                        if (mStateTime.time() <= 1.2) {
                            intake.setPower(0.3);
                        } else if (mStateTime.time() > 1.2 && mStateTime.time() <= 1.4) {
                            intake.setPower(-0.3);
                        } else if (mStateTime.time() > 1.4) {
                            intake.setPower(0.3);
                        }

                         */
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
                    flip.operate(4);
                }
                break;
            case STATE_EXITPOOL2:
                if(Math.abs(exit_pool.getY() - currentPos.getX()) <= 0.0 || delay.time() >= 1.75) {
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_CROSS2);
                }else if(delay.time() >= 2.5){
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_CROSS2);
                }else{
                    flip.clamp();
                    intake.setPower(0.0);
                    localizer.GoTo(exit_pool, 0.8, 0.8, 0.8);
                    mStateTime.reset();
                }
                break;
            case STATE_CROSS2:
                if(Math.abs(-90 - currentPos.getX()) <= 8.0 || delay.time() >= 3.0) {
                        flip.flipDown();
                        localizer.getDrive().setPower(0.0,0.0,0.0);
                        localizer.getDrive().write();
                        newState(State.STATE_DEPOSIT2);
                }else{
                    intake.setPower(0.15);
                    if(delay.time() >= 0.5){
                        intake.close();
                    }
                    saved_deposit = new Pose2d(-18, -90, 0.0);
                    flip.clamp();
                    flip.flipDown();
                    if(currentPos.getY() >= 56){
                        flip.operate(0);
                    }
                    localizer.GoTo(saved_deposit, .8,.8,.8);
                    mStateTime.reset();
                }
                break;
            case STATE_DEPOSIT2:
                if(mStateTime.time() >= 1.2){
                    if(mStateTime.time() >= 1.7){
                        newState(State.STATE_PARK);
                    }else{
                        flip.operate(1);
                    }
                }else{
                    //slides.PIDController(1);
                    flip.operate(0);
                }
                break;
            case STATE_PARK:
                if(Math.abs(-20 - currentPos.getX()) <= 3.0) {
                    if(mStateTime.time() >= 0.5){
                        localizer.getDrive().setPower(0.0,0.0,0.0);
                        localizer.getDrive().write();
                    }else{
                        localizer.GoTo(new Pose2d(-20, -40, 0.0), 0.6, 0.5, 0.6);
                    }
                }else if(delay.time() >= 3.0){
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                }else{
                    //slides.setTargetPosBasic(20,-1.0);
                    if(delay.time() >= 2.0){
                        intake.setPower(0.0);
                    }else{
                        intake.setPower(-1.0);
                    }
                    flip.operate(2);
                    flip.clamp();
                    intake.close();
                    intake.setPower(0.0);
                    localizer.GoTo(new Pose2d(-20, -40, 0.0), 0.6,0.5,0.6);
                    mStateTime.reset();
                }
                break;
        }
        telemetry.addData("POS: ", currentPos.toString());
        telemetry.addData("State: ", mRobotState);
        telemetry.addData("time: ", mStateTime.time());
        telemetry.addData("Saved Pos: ", saved_deposit.toString());
        telemetry.addData("Saved Pos Strafe: ", storedY);
        telemetry.addData("Stored Dep PosX: ", savedDepositX);
        telemetry.addData("Stored Dep PosY: ", savedDepositY);

        slides.read(data);
        slides.write();
        if (mRobotState != State.STATE_PUSH_BACK) {
            drive.write();
        }
        flip.write();
        intake.write();
        flip.read();
    }

    private void newState(State state){
        mStateTime.reset();
        delay.reset();
        localizer.reset();
        mRobotState = state;
    }
}