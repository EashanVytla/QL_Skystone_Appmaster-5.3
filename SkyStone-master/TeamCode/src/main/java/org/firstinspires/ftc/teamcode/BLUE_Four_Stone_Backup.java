package org.firstinspires.ftc.teamcode;

import android.security.keystore.SecureKeyImportUnavailableException;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name = "BACKUP-Blue_4Stone_Auto-BACKUP", group = "Competition")
//@Disabled
public class BLUE_Four_Stone_Backup extends OpMode {
    int SkystonePos;

    public OpenCvCamera webcam;
    //MultipleTelemetry telemetry;
    SkystoneDetectorPipeline pipeline;

    ThreeWheelTrackingLocalizer odos;
    SRX_Three_Wheel_Localizer localizer;
    Vertical_Elevator slides;

    Mecanum_Drive drive;
    Intake intake;

    double returnDelayTime = 0.0;
    double memo = 0.0;
    double storedY = 0.0;
    FlipperV2 flip;

    boolean parkRequested = false;

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
        STATE_EXITPOOL2,
        STATE_STOP,
        STATE_INTAKE3,
        STATE_CROSS3,
        STATE_RETURN3,
        STATE_INTAKE4,
        STATE_EXITPOOL3,
        STATE_EXITPOOL4,
        STATE_CROSS4,
        STATE_PARKFAILSAFE
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

        odos.poseSet(new Pose2d(0, 0, Math.PI / 2));
        localizer.getOdos().poseSet(new Pose2d(0, 0, Math.PI / 2));
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
        webcam.stopStreaming();
        webcam.closeCameraDevice();
        newState(State.STATE_DRIVE_TO_BLOCK);
    }

    Pose2d cross_target = new Pose2d(-24.172, -91.387, -Math.PI/2);
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
    Pose2d exit_pool2_real;
    Pose2d exit_pool3_real;
    Pose2d exit_pool3;
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
        slides.read(data);
        odos.dataUpdate(data);
        odos.update();
        localizer.update(data);
        //Pose2d currentPos = odos.getPoseEstimate();
        Pose2d currentPos = new Pose2d(odos.getPoseEstimate().vec(), odos.getAbsoluteAngle());

        if(SkystonePos == 0){
            drive_to_block_target = new Pose2d(-24.068, 18.899, Math.toRadians(45));//toRadians(28.105)

            return_target = new Pose2d(-25.569, -17.559, Math.PI/4);
            return_target2 = new Pose2d(-23.569, -30.559, Math.PI/4);
            return_target3 = new Pose2d(-39.469, 3.063, Math.PI/4);
            //22.626
            exit_pool = new Pose2d(-20.020, 16.496, Math.PI/4);
            exit_pool2 = new Pose2d(-27.020, -10.496, Math.PI/4);
            exit_pool3 = new Pose2d(-28.020, -15.496, Math.PI/4);
            exit_pool4 = new Pose2d(-28.806, 0.429,Math.PI/4);

            intake1 = new Pose2d(-40.843, 23.816, Math.toRadians(45));
            intake2 = new Pose2d(-38.689, -8.293, Math.PI/4);
            intake3 = new Pose2d(-46.689, -6.893, Math.PI/4); //Send to state Stop and find the positions
            intake4 = new Pose2d(-56.702, 13.165, Math.PI/4); //Send to state Stop and find the positions
        }else if(SkystonePos == 1){
            drive_to_block_target = new Pose2d(-24.000, 6, Math.PI/4);

            return_target = new Pose2d(-20.569, -25.559, Math.PI/4);
            return_target2 = new Pose2d(-20.5, -31.906, Math.PI/4);
            //return_target2 = new Pose2d(-24.0, -27.880, Math.toRadians(35));//45
            return_target3 = new Pose2d(-43, 9.379, 0.0);
            //22.626
            exit_pool = new Pose2d(-20.020, 8.496, Math.PI/4);
            exit_pool2 = new Pose2d(-23.020, -18.496, Math.PI/4);
            exit_pool3 = new Pose2d(-23.075, -15.546,Math.PI/4);
            //exit_pool3 = new Pose2d(-23.075, -25.546,Math.toRadians(35));
            exit_pool4 = new Pose2d(-23.023, -0.22,0.0);

            intake1 = new Pose2d(-40.435, 11.8, Math.PI/4);
            intake2 = new Pose2d(-36.689, -6.893, Math.PI/4);
            intake3 = new Pose2d(-60, 3.870, Math.PI/4); //Send to state Stop and find the positions
            //intake3 = new Pose2d(-58.5, 17.870, Math.toRadians(35));
            intake4 = new Pose2d(-43, 20, 0.0); //Send to state Stop and find the positions
        }else if(SkystonePos == 2){
            drive_to_block_target = new Pose2d(-25.797, -3, Math.PI/4);

            return_target = new Pose2d(-23.569, -30.559, Math.PI/4);
            return_target2 = new Pose2d(-44, -2, 0.0);
            return_target3 = new Pose2d(-40, 9, 0.0);
            //22.626
            exit_pool = new Pose2d(-20.020, 8.496, Math.PI/4);
            //exit_pool2_real= new Pose2d(-15.020, -10.496, Math.PI/4);
            //exit_pool2 = new Pose2d( -5.0,-15, Math.PI/4);    //This is +5
            //exit_pool3_real = new Pose2d(-15.000, -2.5,0.0);
            //exit_pool3 = new Pose2d( -5.0,-7, 0.0); //This is +5
            exit_pool2 = new Pose2d(-22.020, -15.496, Math.PI/4);
            exit_pool3 = new Pose2d(-20.000, -7.5,0.0);
            exit_pool4 = new Pose2d(-18.822, -3.5,0.0);

            intake1 = new Pose2d(-34.435, 7.8, Math.PI/4);
            intake2 = new Pose2d(-36.689, -16.893, Math.PI/4);
            intake3 = new Pose2d(-44, 15, 0.0);
            intake4 = new Pose2d(-40, 26, 0.0);
        }

        switch (mRobotState) {
            case STATE_DRIVE_TO_BLOCK:
                if(Math.abs(drive_to_block_target.getX() + currentPos.getY()) <= 2.0 && Math.abs(drive_to_block_target.getHeading() + currentPos.getHeading()) <= 2.0 && (Math.abs(drive_to_block_target.getX()) - Math.abs(currentPos.getY())) <= 2.0) {
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_INTAKE);
                }else{
                    flip.start();
                    intake.setPower(0.0);
                    if(SkystonePos == 0){
                        localizer.GoTo(drive_to_block_target, 1.0, 1.0, 0.8);
                    }else{
                        localizer.GoTo(drive_to_block_target, 1.0, 1.0, 1.0);
                    }
                    mStateTime.reset();
                    intake.kickoutBLUE();
                    //intake.open();
                }
                if (delay.time() >= 1.0){
                    intake.close();
                }
                flip.read();
                break;
            case STATE_INTAKE:
                if(Math.abs(intake1.getX() + currentPos.getY()) <= 1.5 || flip.IntakeFeedback()){
                    if(SkystonePos == 1){
                        intake.setPower(0.0);
                    }
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_HANDSHAKE);
                }else{
                    intake.close();
                    if(SkystonePos == 0){
                        localizer.GoTo(intake1, 0.3, 0.3, 0.3);
                    }else{
                        localizer.GoTo(intake1, 0.2, 0.2, 0.2);
                    }
                    if(flip.IntakeFeedback()){
                        intake.setPower(0.0);
                    }else{
                        if(SkystonePos == 0){
                            intake.setPower(0.5);
                        }else{
                            intake.setPower(0.3);
                        }
                    }
                }
                break;
            case STATE_HANDSHAKE:
                if(delay.time() >= 0.5){
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_EXIT_POOL);
                }else{
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    intake.open();
                    intake.setPower(0.0);
                    if(delay.time() >= 0.3){
                        flip.unclamp();
                        flip.flipflipper();
                    }
                }
                break;
            case STATE_EXIT_POOL:
                if(SkystonePos == 0){
                    if (Math.abs(exit_pool.getX() + currentPos.getY()) <= 6.0) {
                        newState(State.STATE_CROSS);
                    } else if (delay.time() >= 3.0) {
                        newState(State.STATE_CROSS);
                    } else {
                        intake.setPower(0.0);
                        localizer.GoTo(exit_pool, 1.0, 1.0, 1.0);
                    }
                    if(currentPos.getX() <= -5){
                        flip.clamp();
                    }
                }else{
                    if (Math.abs(exit_pool.getY() - currentPos.getX()) <= 4.0) {
                        newState(State.STATE_CROSS);
                    } else if (delay.time() >= 3.0) {
                        newState(State.STATE_CROSS);
                    } else {
                        intake.setPower(0.0);
                        localizer.GoTo(exit_pool, 1.0, 1.0, 1.0);
                    }
                    if(currentPos.getX() <= -5){
                        flip.clamp();
                    }
                }

                break;
            case STATE_CROSS:
                if(Math.abs(currentPos.getX()) >= Math.abs(cross_target.getY()) - 8) {
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    drive.setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    flip.clamp();
                    newState(State.STATE_DRIVE_TO_FOUNDATION);
                    //newState(State.STATE_STOP);
                }else{
                    intake.setPower(0.15);
                    flip.startKnocker();
                    if(currentPos.getX() <= -30){
                        localizer.GoTo(cross_target, 1.0, 1.0, 1.0);
                        flip.flipDown();
                    }else{
                        localizer.GoTo(new Pose2d(cross_target.getX(), cross_target.getY(), 0.0), 1.0, 1.0, 1.0);
                    }

                    if(currentPos.getX() <= -55.551) {
                        intake.open();
                    }

                }
                if(currentPos.getX() <= -5){
                    flip.clamp();
                }
                telemetry.addData("Is Grabbed? ", flip.isGrabbed());
                break;
            case STATE_DRIVE_TO_FOUNDATION:
                if(mStateTime.time() >= 0.5) {
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_GRAB_FOUNDATION);
                }else{
                    flip.clamp();
                    //flip.flipDown();
                    intake.open();
                    flip.flipDown();
                    //flip.clamp();
                    localizer.GoTo(new Pose2d(-34.885, cross_target.getY(), -Math.PI/2), 1.0, 1.0, 1.0);
                }
                break;
            case STATE_GRAB_FOUNDATION:
                if(mStateTime.time() >= 0.25){
                    newState(State.STATE_PULL_FORWARD);
                }else{
                    flip.grabPlatform();
                    intake.close();
                    flip.partialDeposit();
                    //flip.operate(0);
                }
                break;
            case STATE_PULL_FORWARD:
                if(Math.abs(-12 + currentPos.getY()) <= 4.0) {
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_TURN);
                }else if(mStateTime.time() >= 3.0){
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_TURN);
                }else{
                    intake.setPower(-0.2);
                    localizer.GoTo(new Pose2d(-12, -90, -Math.PI/2), 1.0, 1.0, 1.0);
                    mStateTime.reset();
                }
                break;
                /*
            case STATE_TURN:
                if(getHeading() >= Math.toRadians(-10.0) || mStateTime.time() >= 3.0) {
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_RETURN);
                }else{
                    localizer.GoTo(new Pose2d(-8.783, -70.605, Math.PI / 4), 1.0, 1.0, 1.0);
                    //localizer.GoTo(new Pose2d(-8.783, -70.605, 0.0), 1.0, 1.0, 1.0);
                    //flip.operate(2);
                    if (Math.abs(getHeading()) < Math.toRadians(25)){
                        flip.operate(1);
                    }
                }
                break;
                 */
            case STATE_TURN:
                if(getHeading() >= Math.toRadians(-10.0)) {
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_RETURN);
                }else{
                    //localizer.GoTo(new Pose2d(-8.783, -70.605, (Math.PI / 2) + Math.toRadians()), 1.0, 1.0, 1.0);
                    localizer.getDrive().setPower(0.0,0.0,1.0);
                    localizer.getDrive().write();
                    //localizer.GoTo(new Pose2d(-8.783, -70.605, 0.0), 1.0, 1.0, 1.0);
                    //flip.operate(2);
                    if (Math.abs(getHeading()) < Math.toRadians(25)){
                        flip.operate(1);
                    }
                }
                break;
            case STATE_RETURN:
                if(Math.abs(return_target.getY() - currentPos.getX()) <= 5.0 && Math.abs(return_target.getHeading() - currentPos.getHeading()) <= 3.0) {
                    //localizer.getDrive().setPower(0.0,0.0,0.0);
                    //localizer.getDrive().write();
                    memo = localizer.getForwardDist();
                    newState(State.STATE_INTAKE2);
                }else{
                    intake.close();
                    flip.resetPlatform();
                    if((currentPos.getY() >= 16.569)){
                        if (currentPos.getX() >= (return_target.getY() - 5)) {
                            localizer.GoTo(return_target, 1.0, 1.0, 1.0);
                        }else {
                            localizer.GoTo(new Pose2d(return_target.getX(), return_target.getY(), 0.0), 1.0, 1.0, 1.0);
                        }
                    }else{
                        localizer.GoTo(new Pose2d(-40, -70, 0.0), 1.0, 1.0, 1.0);
                    }

                    if(currentPos.getX() >= return_target.getY()/3){
                        //intake.close();
                        intake.setPower(0.3);
                    }else{
                        intake.setPower(-1.0);
                    }
                    mStateTime.reset();
                }
                break;
            case STATE_INTAKE2:
                flip.read();
                if(Math.abs(intake2.getX() + currentPos.getY()) <= 3.0 &&  Math.abs(intake2.getY() - currentPos.getX()) <= 3.0|| flip.IntakeFeedback()){
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_EXITPOOL2);
                } else{
                    //drive.setPower(-0.15, 0.0, 0.0);
                    if(delay.time() >= 0.75){
                        //flip.operate(2);
                        //flip.clamp();
                    }
                    localizer.GoTo(intake2, 0.2, 0.2, 0.2);
                    mStateTime.reset();
                    intake.close();
                    if(delay.time() >= 0.7){
                        flip.start();
                    }
                    if(flip.IntakeFeedback()){
                        intake.setPower(0.0);
                    }else{
                        intake.setPower(0.3);
                    }
                }
                break;
            case STATE_EXITPOOL2:
                if (delay.time() >= 0.3) {
                    if (Math.abs(exit_pool2.getY() - currentPos.getX()) <= 4.0 && Math.abs(Math.abs(exit_pool2.getX()) - Math.abs(currentPos.getY())) <= 4.0) {
                        newState(State.STATE_CROSS2);
                    } else if (delay.time() >= 2.5) {
                        newState(State.STATE_CROSS2);
                    } else {
                        //flip.clamp();
                        intake.setPower(0.0);
                        localizer.GoTo(exit_pool2, 0.8, 0.8, 0.8);
                        mStateTime.reset();
                    }
                }
                intake.open();
                flip.operate(4);
                break;
            case STATE_CROSS2:
                if(SkystonePos == 0){
                    saved_deposit = new Pose2d(-23.075, -83.492, 0.0);
                }else if(SkystonePos == 2){
                    saved_deposit = new Pose2d(-23.075, -81.972, 0.0);
                }else if(SkystonePos == 1){
                    saved_deposit = new Pose2d(-23.075, -81.972, 0.0);
                }
                if(Math.abs(saved_deposit.getY() - currentPos.getX()) <= 8.0 && Math.abs(saved_deposit.getX() + currentPos.getY()) <= 2.0 || delay.time() >= 3.0) {
                    //flip.flipDown();
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    drive.setPower(0.0,0.0,0.0);
                    slides.setPower(0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_RETURN2);
                    //newState(State.STATE_PARK);
                }else{
                    intake.setPower(0.15);
                    if(delay.time() >= 3.0){
                        intake.close();
                        flip.clamp();
                    }else{
                        intake.open();
                        if (Math.abs(-78.972 - currentPos.getX()) > 40) {
                            flip.operate(4);
                        }
                    }
                    if (Math.abs(-78.972 - currentPos.getX()) <= 35){
                        flip.operate(0);
                    }
                    if(Math.abs(currentPos.getX()) >= 40){
                        slides.PIDController(1);
                        slides.setStack_count(1);
                    }
                    else if (Math.abs(-78.972 - currentPos.getX()) <= 45){
                        flip.flipDown();
                    }
                    else if (Math.abs(-78.972 - currentPos.getX()) <= 50){
                        flip.clamp();
                    }
                    if(delay.time() >= 3.0){
                        intake.close();
                    }else{
                        intake.open();
                    }
                    localizer.GoTo(saved_deposit, 1.0,1.0,1.0);
                }
                //flip.clamp();
                //flip.getClamp().getServo().setPosition(0.8);
                break;
            case STATE_RETURN2:
                if(SkystonePos == 1){
                    returnDelayTime = 0.5;
                }else{
                    returnDelayTime = 0.75;
                }
                if (delay.time() >= returnDelayTime) {
                    if (Math.abs(return_target2.getY() - currentPos.getX()) <= 4.0 && Math.abs(return_target2.getX() + currentPos.getY()) <= 2.0) {
                        localizer.getDrive().setPower(0.0, 0.0, 0.0);
                        localizer.getDrive().write();
                        slides.setPower(0.0);
                        newState(State.STATE_INTAKE3);
                    } else {
                        if(Math.abs(currentPos.getX()) < 70){
                            slides.dropSlides(-0.75);
                        }
                        intake.close();
                        flip.resetPlatform();
                        if (currentPos.getX() >= -20) {
                            localizer.GoTo(return_target2, 1.0, 1.0, 1.0);
                        } else {
                            if(SkystonePos == 1 || SkystonePos == 0){
                                localizer.GoTo(new Pose2d(return_target2.getX(), return_target2.getY(), 0.0), 1.0, 1.0, 1.0);
                            }else{
                                localizer.GoTo(new Pose2d(-19.5, return_target2.getY(), 0.0), 1.0, 1.0, 1.0);
                            }

                        }
                        if (currentPos.getX() >= return_target2.getY() / 3) {
                            //intake.close();
                            intake.setPower(0.3);
                        } else {
                            intake.setPower(-1.0);
                        }
                        mStateTime.reset();
                        flip.operate(1);
                    }

                }
                else{
                    if(SkystonePos == 1){
                        if(delay.time() >= 0.25){
                            flip.operate(1);
                        }
                    }else{
                        if (delay.time() >= 0.5) {
                            flip.operate(1);
                        }
                    }
                    slides.setPower(0.0);
                    localizer.GoTo(saved_deposit, 1.0,1.0,1.0);
                }
                break;
            case STATE_INTAKE3:
                flip.read();
                if(SkystonePos == 1){
                    if(currentPos.getX() >= -20){
                        flip.start();
                    }
                }else{
                    if(delay.time() >= 0.5){
                        flip.start();
                    }
                }

                if(Math.abs(intake3.getX() + currentPos.getY()) <= 3.0 &&  Math.abs(intake3.getY() - currentPos.getX()) <= 3.0|| flip.IntakeFeedback()){
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    if(delay.time() >= 0.5){
                        newState(State.STATE_EXITPOOL3);
                    }
                }
                else{
                    if(SkystonePos == 1){
                        localizer.GoTo(intake3, 0.3, 0.3, 0.3);
                    }else{
                        localizer.GoTo(intake3, 0.3, 0.3, 0.3);
                    }
                    mStateTime.reset();
                    intake.close();

                    if(flip.IntakeFeedback()){
                        intake.setPower(0.0);
                    }else{
                        if(SkystonePos == 1){
                            intake.setPower(0.3);
                        }else{
                            intake.setPower(0.5);
                        }
                    }
                }
                break;
            case STATE_EXITPOOL3:
                if (delay.time() >= 0.3) {
                    if (Math.abs(Math.abs(exit_pool3.getY()) - Math.abs(currentPos.getX())) <= 4.0 && Math.abs(Math.abs(exit_pool3.getX()) - Math.abs(currentPos.getY())) <= 4.0) {
                        newState(State.STATE_CROSS3);
                    } else {
                        intake.setPower(0.0);
                        localizer.GoTo(exit_pool3, 1.0, 1.0, 1.0);
                        mStateTime.reset();
                    }
                }
                intake.setPower(0.0);
                intake.open();
                flip.operate(4);
                break;
            case STATE_CROSS3:
                if(SkystonePos == 0){
                    saved_deposit = new Pose2d(-24.428, -83.072, Math.toRadians(5));
                }else if(SkystonePos == 1){
                    saved_deposit = new Pose2d(-22.428, -79.072, Math.toRadians(5));
                }else if(SkystonePos == 2){
                    saved_deposit = new Pose2d(-22.428, -79.072, Math.toRadians(5));
                }
                if(Math.abs(saved_deposit.getY() - currentPos.getX()) <= 8.0 && Math.abs(saved_deposit.getX() + currentPos.getY()) <= 2.0 || delay.time() >= 5.0) {
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    drive.setPower(0.0, 0.0, 0.0);
                    slides.setPower(0.0);
                    newState(State.STATE_RETURN3);
                }else{
                    if(delay.time() >= 3.0){
                        intake.close();
                        flip.clamp();
                    }else{
                        intake.open();
                        intake.setPower(0.0);
                        if (Math.abs(-77.972 - currentPos.getX()) > 40) {
                            flip.operate(4);
                        }
                    }
                    if (Math.abs(-77.972 - currentPos.getX()) <= 30){
                        flip.operate(0);
                    }
                    if(Math.abs(currentPos.getX()) >= 40){
                        slides.PIDController(1);
                        slides.setStack_count(1);
                    }
                    else if (Math.abs(currentPos.getX()) >= 35){//Joemama
                        flip.flipDown();
                    }
                    else if (Math.abs(-77.972 - currentPos.getX()) <= 35){
                        flip.clamp();
                    }
                    if(delay.time() >= 3.0){
                        intake.close();
                        flip.clamp();
                    }else{
                        intake.open();
                    }
                    localizer.GoTo(saved_deposit, 1.0,1.0,1.0);
                }
                break;
            case STATE_RETURN3:
                if (delay.time() >= returnDelayTime) {
                    if (Math.abs(return_target3.getY() - currentPos.getX()) <= 4.0 && Math.abs(return_target3.getX() + currentPos.getY()) <= 2.0) {
                        localizer.getDrive().setPower(0.0, 0.0, 0.0);
                        localizer.getDrive().write();
                        newState(State.STATE_INTAKE4);
                    } else {
                        intake.close();
                        flip.resetPlatform();
                        if(Math.abs(currentPos.getX()) < 70){
                            slides.dropSlides(-0.75);
                        }
                        if (currentPos.getX() >= -20) {
                            localizer.GoTo(return_target3, 1.0, 1.0, 1.0);
                        } else {
                            if (SkystonePos == 1){
                                localizer.GoTo(new Pose2d(-17.5, return_target3.getY(), 0.0), 1.0, 1.0, 1.0);
                            }
                            else if(SkystonePos == 0){
                                localizer.GoTo(new Pose2d(-22, return_target3.getY(), 0.0), 1.0, 1.0, 1.0);
                            }
                            else{
                                localizer.GoTo(new Pose2d(-17.5, return_target3.getY(), 0.0), 1.0, 1.0, 1.0);
                            }

                        }
                        if (currentPos.getX() >= return_target3.getY() / 3) {
                            //intake.close();
                            intake.setPower(0.3);
                        } else {
                            intake.setPower(-1.0);
                        }
                        if(delay.time() >= 2.0){
                            flip.start();
                        }
                        mStateTime.reset();
                    }
                }
                else{
                    if(SkystonePos == 1){
                        if(delay.time() >= 0.25){
                            flip.operate(1);
                        }
                    }else{
                        if (delay.time() >= 0.25) {
                            flip.operate(1);
                        }
                        else{
                            flip.clamp();
                        }
                    }

                    slides.setStack_check(1);
                    slides.setPower(0.0);
                    localizer.GoTo(saved_deposit, 1.0,1.0,1.0);
                }
                break;
            case STATE_INTAKE4:
                flip.read();
                if(Math.abs(intake4.getX() + currentPos.getY()) <= 3.0 &&  Math.abs(intake4.getY() - currentPos.getX()) <= 3.0){
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    parkRequested = true;
                    newState(State.STATE_EXITPOOL4);
                }else if(flip.IntakeFeedback()){
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_EXITPOOL4);
                }
                else if (delay.time() >= 3.0){
                    drive.setPower(0.0, 0.0, 0.0);
                    parkRequested = true;
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
                    if(delay.time() >= 0.5){
                        flip.start();
                    }
                }
                break;
            case STATE_CROSS4:
                if(SkystonePos == 0){
                    saved_deposit = new Pose2d(-21.912, -82.249, Math.toRadians(10));
                }else if(SkystonePos == 1){
                    saved_deposit = new Pose2d(-20.912, -79.249, Math.toRadians(10));
                }else if(SkystonePos == 2){
                    saved_deposit = new Pose2d(-19.912, -79.249, Math.toRadians(10));
                }
                if(Math.abs(saved_deposit.getY() - currentPos.getX()) <= 8.0 && Math.abs(saved_deposit.getX() + currentPos.getY()) <= 2.0 || delay.time() >= 5.0) {
                    localizer.getDrive().setPower(0.0,0.0,0.0);
                    drive.setPower(0.0,0.0,0.0);
                    localizer.getDrive().write();
                    drive.setPower(0.0,0.0,0.0);
                    slides.setPower(0.0);
                    newState(State.STATE_PARK);
                }else{
                    intake.setPower(0.15);
                    if(delay.time() >= 3.0){
                        intake.close();
                        flip.clamp();
                    }else{
                        intake.open();
                        if (Math.abs(-78.972 - currentPos.getX()) > 40) {
                            flip.operate(4);
                        }
                    }
                    if(Math.abs(currentPos.getX()) >= 40){
                        slides.PIDController(1);
                        slides.setStack_count(1);
                    }
                    if (Math.abs(-80.972 - currentPos.getX()) <= 30){
                        flip.operate(0);
                    }
                    else if (Math.abs(-80.972 - currentPos.getX()) <= 40){
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
                        if(!parkRequested){
                            newState(State.STATE_CROSS4);
                        }else{
                            flip.start();
                            intake.setPower(0.0);
                            newState(State.STATE_PARKFAILSAFE);
                        }
                    } else if (delay.time() >= 2.5) {
                        drive.setPower(0.0, 0.0, 0.0);
                        if(!parkRequested){
                            newState(State.STATE_CROSS4);
                        }else{
                            flip.start();
                            flip.clamp();
                            intake.setPower(0.0);
                            newState(State.STATE_PARKFAILSAFE);
                        }
                    } else {
                        intake.setPower(0.0);
                        localizer.GoTo(exit_pool4, 0.8, 0.8, 0.8);
                        mStateTime.reset();
                    }
                }
                intake.open();
                if(!parkRequested){
                    flip.operate(4);
                }

                break;
            case STATE_PARK:
                //if (SkystonePos == 1) {
                //  returnDelayTime = 0.5;
                //}
                if (delay.time() >= returnDelayTime) {
                    if(Math.abs(currentPos.getX()) < 55){
                        slides.dropSlides(-0.75);
                    }
                    if (Math.abs(currentPos.getX()) < 65){
                        flip.start();
                    }
                    if (Math.abs(-20 - currentPos.getX()) <= 3.0) {
                        localizer.getDrive().setPower(0.0, 0.0, 0.0);
                        localizer.getDrive().write();
                        newState(State.STATE_STOP);
                        telemetry.addData("Tee Hee :)", "Deal with it this is my senior year");
                    } else if (delay.time() >= 3.0) {
                        localizer.getDrive().setPower(0.0, 0.0, 0.0);
                        localizer.getDrive().write();
                        newState(State.STATE_STOP);
                    } else {
                        if (delay.time() >= 2.0) {
                            intake.setPower(0.0);
                        } else {
                            intake.setPower(-1.0);
                        }
                        intake.close();
                        intake.setPower(0.0);
                        localizer.GoTo(new Pose2d(-16.922, -40.976, 0.0), 1.0, 1.0, 1.0);
                        mStateTime.reset();
                    }
                }
                else{
                    if(SkystonePos == 1){
                        if(delay.time() >= 0.25){
                            flip.operate(1);
                        }
                    }else{
                        if (delay.time() >= 0.25){
                            flip.operate(1);
                        }
                    }

                    slides.setStack_count(1);
                    slides.setPower(0.0);
                    localizer.GoTo(saved_deposit, 1.0,1.0,1.0);
                }
                break;
            case STATE_PARKFAILSAFE:
                if (Math.abs(currentPos.getX()) < 65){
                    flip.start();
                }
                if (Math.abs(-16 - currentPos.getX()) <= 3.0) {
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_STOP);
                    telemetry.addData("Tee Hee :)", "Deal with it this is my senior year");
                } else if (delay.time() >= 3.0) {
                    localizer.getDrive().setPower(0.0, 0.0, 0.0);
                    localizer.getDrive().write();
                    newState(State.STATE_STOP);
                } else {
                    if (delay.time() >= 2.0) {
                        intake.setPower(0.0);
                    } else {
                        intake.setPower(-1.0);
                    }
                    intake.close();
                    intake.setPower(0.0);
                    localizer.GoTo(new Pose2d(-16.922, -40.976, 0.0), 1.0, 1.0, 1.0);
                    mStateTime.reset();
                }
                break;
            case STATE_STOP:
                localizer.getDrive().setPower(0.0,0.0,0.0);
                localizer.getDrive().write();
                drive.setPower(0.0, 0.0, 0.0);

                telemetry.addData("Tee Hee :)", "Deal with it this is my senior year");
                telemetry.addData("Tee Hee :)", "I Kicked you out of All your Discords");
                break;
        }
        telemetry.addData("POS: ", currentPos.toString());
        telemetry.addData("State: ", mRobotState);
        telemetry.addData("time: ", mStateTime.time());
        telemetry.addData("Saved Pos: ", saved_deposit.toString());
        telemetry.addData("Saved Pos Strafe: ", storedY);
        telemetry.addData("cross_target", cross_target.getY());

        //slides.write();
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
}