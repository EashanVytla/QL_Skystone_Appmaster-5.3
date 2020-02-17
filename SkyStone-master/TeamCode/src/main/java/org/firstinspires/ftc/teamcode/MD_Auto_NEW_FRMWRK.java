package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.ThreeWheelTrackingLocalizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import static org.firstinspires.ftc.teamcode.VisionContstants.IMAGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.VisionContstants.IMAGE_WIDTH;

@Autonomous(name = "Vector_Gang_IS_DUMB", group = "Competition")
@Disabled
public class MD_Auto_NEW_FRMWRK extends OpMode {
    int SkystonePos;

    public OpenCvCamera webcam;
    //MultipleTelemetry telemetry;
    SkystoneDetectorPipeline pipeline;

    ThreeWheelTrackingLocalizer odos;
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
        STATE_CROSS4
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
        odos = new ThreeWheelTrackingLocalizer(this.hardwareMap);

        //odos.poseSet(new Pose2d(0, 0, Math.PI / 2));
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

    Pose2d cross_target = new Pose2d(27.172, 91.387, Math.PI);
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
    Pose2d exit_pool3;
    Pose2d cross_target3 = new Pose2d(23.075, 80.972, -Math.PI/2);
    double heading = 0.0;

    public double getHeading(){
        if(odos.getAbsoluteAngle() <= Math.PI){
            heading = odos.getAbsoluteAngle();
        }else{
            heading = -((2 * Math.PI ) - odos.getAbsoluteAngle());
        }
        return heading;
    }

    public void loop(){
        RevBulkData data = hub2.getBulkInputData();
        flip.read();
        slides.read(data);
        odos.dataUpdate(data);
        odos.update();
        //Pose2d currentPos = currentPos;
        Pose2d currentPos = new Pose2d(odos.getPoseEstimate().vec(), odos.getAbsoluteAngle());

        if(SkystonePos == 0){
            drive_to_block_target = new Pose2d(26.068, -13.899, -Math.toRadians(56.895));//toRadians(28.105)

            return_target = new Pose2d(25.569, 10.559, -Math.PI/4);
            return_target2 = new Pose2d(23.569, 30.559, -Math.PI/4);
            return_target3 = new Pose2d(39.469, 3.063, -Math.PI/4);
            //22.626
            exit_pool = new Pose2d(20.020, -16.496, -Math.PI/4);
            exit_pool2 = new Pose2d(27.020, 10.496, -Math.PI/4);
            exit_pool3 = new Pose2d(28.020, 15.496, -Math.PI/4);
            exit_pool4 = new Pose2d(28.806, -0.429,-Math.PI/4);

            intake1 = new Pose2d(34.843, -21.816, -Math.toRadians(56.895));
            intake2 = new Pose2d(38.689, -0.893, -Math.PI/4);
            intake3 = new Pose2d(36.689, 16.893, -Math.PI/4); //Send to state Stop and find the positions
            intake4 = new Pose2d(56.702, -13.165, -Math.PI/4); //Send to state Stop and find the positions
        }else if(SkystonePos == 1){
            drive_to_block_target = new Pose2d(25.797, -5, -Math.PI/4);

            return_target = new Pose2d(20.569, 22.559, -Math.PI/4);
            return_target2 = new Pose2d(44, 12.587, -Math.PI/2);
            return_target3 = new Pose2d(44, -8.379, -Math.PI/2);
            //22.626
            exit_pool = new Pose2d(20.020, -8.496, -Math.PI/4);
            exit_pool2 = new Pose2d(20.020, 18.496, -Math.PI/4);
            exit_pool3 = new Pose2d(22.075, 10.546,-Math.PI/2);
            exit_pool4 = new Pose2d(23.023, 0.22,-Math.PI/2);

            intake1 = new Pose2d(34.435, -15.8, -Math.PI/4);
            intake2 = new Pose2d(36.689, 3.893, -Math.PI/4);
            intake3 = new Pose2d(44, -1.912, -Math.PI/2); //Send to state Stop and find the positions
            intake4 = new Pose2d(44, -26, -Math.PI/2); //Send to state Stop and find the positions
        }else if(SkystonePos == 2){
            drive_to_block_target = new Pose2d(25.797, 3, -Math.PI/4);

            return_target = new Pose2d(23.569, 30.559, -Math.PI/4);
            return_target2 = new Pose2d(44, 2, -Math.PI/2);
            return_target3 = new Pose2d(43, -9, -Math.PI/2);
            //22.626
            exit_pool = new Pose2d(20.020, -8.496, -Math.PI/4);
            exit_pool2 = new Pose2d(20.020, 15.496, -Math.PI/4);
            exit_pool3 = new Pose2d(18.822, 3.5,-Math.PI/2);
            exit_pool4 = new Pose2d(18.822, 3.5,-Math.PI/2);

            intake1 = new Pose2d(34.435, -7.8, -Math.PI/4);
            intake2 = new Pose2d(36.689, 16.893, -Math.PI/4);
            intake3 = new Pose2d(44, -15, -Math.PI/2);
            intake4 = new Pose2d(43, -26, -Math.PI/2);
        }

        switch (mRobotState) {
            case STATE_DRIVE_TO_BLOCK:
                if(drive_to_block_target.vec().distTo(currentPos.vec()) <= 3.0) {
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_INTAKE);
                }else{
                    flip.start();
                    intake.setPower(0.0);
                    if(SkystonePos == 0){
                        drive.goToPoint(currentPos, drive_to_block_target, 1.0, 1.0, 0.8);
                    }else{
                        drive.goToPoint(currentPos, drive_to_block_target, 1.0, 1.0, 1.0);
                    }
                    telemetry.addData("REAL CurrentPos: ", currentPos.toString());
                    telemetry.addData("REAL Error: ", drive_to_block_target.vec().distTo(currentPos.vec()));
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
                if(intake1.vec().distTo(currentPos.vec()) <= 3 || flip.IntakeFeedback()){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_HANDSHAKE);
                }else{
                    intake.close();
                    drive.goToPoint(currentPos, intake1, 0.2, 0.2, 0.2);
                    if(flip.IntakeFeedback()){
                        intake.setPower(0.0);
                    }else{
                        intake.setPower(0.3);
                    }
                }
                break;
            case STATE_HANDSHAKE:
                if(delay.time() >= 0.5){
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_EXIT_POOL);
                }else{
                    intake.open();
                    intake.setPower(0.0);
                    if(delay.time() >= 0.3){
                        flip.unclamp();
                        flip.flipflipper();
                    }
                }
                break;
            case STATE_EXIT_POOL:
                if (exit_pool.vec().distTo(currentPos.vec()) <= 4.0) {
                    newState(State.STATE_CROSS);
                } else if (delay.time() >= 3.0) {
                    newState(State.STATE_CROSS);
                } else {
                    intake.setPower(0.0);
                    drive.goToPoint(currentPos, exit_pool, 1.0, 1.0, 1.0);
                }
                telemetry.addData("REAL CurrentPos: ", currentPos.toString());
                telemetry.addData("REAL Error: ", exit_pool.vec().distTo(currentPos.vec()));
                if(currentPos.getY() <= -5){
                    flip.clamp();
                }
                break;
            case STATE_CROSS:
                if(cross_target.vec().distTo(currentPos.vec()) <= 8) {
                    drive.setPower(0.0,0.0,0.0);
                    flip.clamp();
                    newState(State.STATE_DRIVE_TO_FOUNDATION);
                }else{
                    intake.setPower(0.15);
                    flip.startKnocker();
                    if(currentPos.getY() >= 30){
                        drive.goToPoint(currentPos, cross_target, 1.0, 1.0, 1.0);
                        flip.flipDown();
                    }else{
                        drive.goToPoint(currentPos, new Pose2d(cross_target.getX(), cross_target.getY(), -Math.PI/2), 1.0, 1.0, 1.0);
                    }

                    telemetry.addData("REAL CurrentPos: ", currentPos.toString());
                    telemetry.addData("REAL Error: ", cross_target.vec().distTo(currentPos.vec()));

                    if(currentPos.getY() >= 55.551) {
                        intake.open();
                    }
                }
                if(currentPos.getY() >= 5){
                    flip.clamp();
                }
                telemetry.addData("Is Grabbed? ", flip.isGrabbed());
                break;
            case STATE_DRIVE_TO_FOUNDATION:
                if(mStateTime.time() >= 0.5){
                    newState(State.STATE_GRAB_FOUNDATION);
                }else{
                    flip.clamp();
                    intake.open();
                    flip.flipDown();
                    telemetry.addData("REAL CurrentPos: ", currentPos.toString());
                    telemetry.addData("REAL Error: ", new Pose2d(34.885, 92.7, Math.PI).vec().distTo(currentPos.vec()));
                    drive.goToPoint(currentPos, new Pose2d(34.885, 92.7, Math.PI), 1.0, 1.0, 1.0);
                }
                break;
            case STATE_GRAB_FOUNDATION:
                if(mStateTime.time() >= 0.25){
                    newState(State.STATE_PULL_FORWARD);
                }else{
                    flip.grabPlatform();
                    intake.close();
                    flip.partialDeposit();
                }
                break;
            case STATE_PULL_FORWARD:
                if(new Pose2d(17, 90, Math.PI).vec().distTo(currentPos.vec()) <= 3.0) {
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_TURN);
                }else if(mStateTime.time() >= 3.0){
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_TURN);
                }else{
                    intake.setPower(-0.2);
                    telemetry.addData("REAL CurrentPos: ", currentPos.toString());
                    telemetry.addData("REAL Error: ", new Pose2d(17, 90, Math.PI).vec().distTo(currentPos.vec()));
                    drive.goToPoint(currentPos, new Pose2d(17, 90, Math.PI), 1.0, 1.0, 1.0);
                    mStateTime.reset();
                }
                break;
            case STATE_TURN:
                if(getHeading() >= Math.toRadians(-10.0) || mStateTime.time() >= 3.0) {
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_RETURN);
                }else{
                    drive.goToPoint(currentPos, new Pose2d(8.783, 70.605, -Math.PI/4), 1.0, 1.0, 1.0);
                    telemetry.addData("REAL CurrentPos: ", currentPos.toString());
                    telemetry.addData("REAL Error: ",  new Pose2d(8.783, 70.605, -Math.PI/4).vec().distTo(currentPos.vec()));
                    if (Math.abs(getHeading()) < Math.toRadians(25)){
                        flip.operate(1);
                    }
                }
                break;
            case STATE_RETURN:
                if(return_target.vec().distTo(currentPos.vec()) <= 3.0) {
                    newState(State.STATE_INTAKE2);
                }else{
                    flip.resetPlatform();
                    if (currentPos.getY() >= (return_target.getY() - 5)) {
                        drive.goToPoint(currentPos, return_target, 1.0, 1.0, 1.0);
                    } else {
                        drive.goToPoint(currentPos, new Pose2d(return_target.getX(), return_target.getY(), Math.PI), 1.0, 1.0, 1.0);
                    }
                    telemetry.addData("REAL CurrentPos: ", currentPos.toString());
                    telemetry.addData("REAL Error: ",  return_target.vec().distTo(currentPos.vec()));
                    if(currentPos.getY() >= return_target.getY()/3){
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
                if(intake2.vec().distTo(currentPos.vec()) <= 3.0|| flip.IntakeFeedback()){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_EXITPOOL2);
                }
                else if (delay.time() >= 3.0){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_EXITPOOL2);
                }
                else{
                    drive.goToPoint(currentPos, intake2, 0.2, 0.2, 0.2);
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
            case STATE_HANDSHAKE2:
                if(mStateTime.time() >= 1.5){
                    newState(State.STATE_EXITPOOL2);
                }else{
                    intake.open();
                    drive.setPower(0.0, 0.0, 0.0);
                }
                break;
            case STATE_EXITPOOL2:
                if (delay.time() >= 0.3) {
                    if (exit_pool2.vec().distTo(currentPos.vec()) <= 4.0) {
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_CROSS2);
                    } else if (delay.time() >= 2.5) {
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_CROSS2);
                    } else {
                        intake.setPower(0.0);
                        drive.goToPoint(currentPos, exit_pool2, 0.8, 0.8, 0.8);
                        mStateTime.reset();
                    }
                }
                intake.open();
                flip.operate(4);
                break;
            case STATE_CROSS2:
                if(cross_target3.vec().distTo(currentPos.vec()) <= 4.0 || delay.time() >= 4.0) {
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_RETURN2);
                }else{
                    intake.setPower(0.15);
                    if(delay.time() >= 3.0){
                        intake.close();
                        flip.clamp();
                    }else{
                        intake.open();
                        if (Math.abs(cross_target3.getY() - currentPos.getY()) > 40) {
                            flip.operate(4);
                        }
                    }
                    if (Math.abs(cross_target3.getY() - currentPos.getY()) <= 30){
                        flip.operate(0);
                    }
                    else if (Math.abs(cross_target3.getY() - currentPos.getY()) <= 35){
                        flip.flipDown();
                    }
                    else if (Math.abs(cross_target3.getY() - currentPos.getY()) <= 50){
                        flip.clamp();
                    }
                    if(delay.time() >= 3.0){
                        intake.close();
                    }else{
                        intake.open();
                    }
                    drive.goToPoint(currentPos, saved_deposit, 1.0,1.0,1.0);
                }
                break;
            case STATE_RETURN2:
                if (delay.time() >= 1.0) {
                    if (return_target2.vec().distTo(currentPos.vec()) <= 2.0) {
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_INTAKE3);
                    } else {
                        flip.resetPlatform();
                        if (currentPos.getY() >= 20) {
                            drive.goToPoint(currentPos, return_target2, 1.0, 1.0, 1.0);
                        } else {
                            if(SkystonePos == 1 || SkystonePos == 0){
                                drive.goToPoint(currentPos, new Pose2d(return_target2.getX(), return_target2.getY(), Math.PI), 1.0, 1.0, 1.0);
                            }else{
                                drive.goToPoint(currentPos, new Pose2d(17.5, return_target2.getY(), Math.PI), 1.0, 1.0, 1.0);
                            }

                        }
                        if (currentPos.getY() >= return_target2.getY() / 3) {
                            intake.close();
                            intake.setPower(0.3);
                        } else {
                            intake.setPower(-1.0);
                        }
                        mStateTime.reset();
                    }
                }
                else{
                    if (delay.time() >= 0.5) {
                        flip.operate(1);
                    }

                    drive.setPower(0.0, 0.0, 0.0);
                    drive.setPower(0.0, 0.0, 0.0);
                    
                }
                break;
            case STATE_INTAKE3:
                flip.read();
                if(delay.time() >= 0.5){
                    flip.start();
                }
                if(intake3.vec().distTo(currentPos.vec()) <= 1.0 || flip.IntakeFeedback()){
                    drive.setPower(0.0, 0.0, 0.0);
                    
                    newState(State.STATE_EXITPOOL3);
                }
                else if (delay.time() >= 3.0){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_EXITPOOL3);
                }
                else{
                    drive.goToPoint(currentPos, intake3, 0.3, 0.3, 0.3);
                    mStateTime.reset();
                    intake.close();
                    if(delay.time() >= 0.5){
                        flip.start();
                    }
                    if(flip.IntakeFeedback()){
                        intake.setPower(0.0);
                    }else{
                        intake.setPower(0.5);
                    }
                }
                break;
            case STATE_EXITPOOL3:
                if (delay.time() >= 0.3) {
                    if (exit_pool3.vec().distTo(currentPos.vec()) <= 4.0) {
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_CROSS3);
                    } else if (delay.time() >= 3.0) {
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_CROSS3);
                    } else {
                        intake.setPower(0.0);
                        drive.goToPoint(currentPos, exit_pool3, 0.8, 0.8, 0.8);
                        mStateTime.reset();
                    }
                }
                intake.setPower(0.0);
                intake.open();
                flip.operate(4);
                break;
            case STATE_CROSS3:
                if(cross_target3.vec().distTo(currentPos.vec()) <= 3.0 || delay.time() >= 5.0) {
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_RETURN3);
                }else{
                    if(delay.time() >= 3.0){
                        intake.close();
                        flip.clamp();
                    }else{
                        intake.open();
                        intake.setPower(0.0);
                        if (Math.abs(cross_target3.getY() - currentPos.getY()) > 40) {
                            flip.operate(4);
                        }
                    }
                    if (Math.abs(cross_target3.getY() - currentPos.getY()) <= 30){
                        flip.operate(0);
                    }
                    if(Math.abs(currentPos.getY()) >= 45){
                        slides.PIDController(1);
                    }
                    else if (Math.abs(currentPos.getY()) >= 40){//Joemama
                        flip.flipDown();
                    }
                    else if (Math.abs(cross_target3.getY() - currentPos.getY()) <= 35){
                        flip.clamp();
                    }
                    if(delay.time() >= 3.0){
                        intake.close();
                        flip.clamp();
                    }else{
                        intake.open();
                    }
                    drive.goToPoint(currentPos, saved_deposit, 1.0,1.0,1.0);
                }
                break;
            case STATE_RETURN3:
                if (delay.time() >= 1.0) {//used to be 1.0
                    if (return_target3.vec().distTo(currentPos.vec()) <= 3.0) {
                        drive.setPower(0.0, 0.0, 0.0);
                        
                        newState(State.STATE_INTAKE4);
                    } else {
                        flip.resetPlatform();
                        if(Math.abs(currentPos.getY()) < 70){
                            slides.dropSlides(-0.75);
                        }
                        if (currentPos.getY() >= -20) {
                            drive.goToPoint(currentPos, return_target3, 1.0, 1.0, 1.0);
                        } else {
                            if (SkystonePos == 1){
                                drive.goToPoint(currentPos, new Pose2d(return_target3.getX(), return_target3.getY(), Math.PI), 1.0, 1.0, 1.0);
                            }
                            else if(SkystonePos == 0){
                                drive.goToPoint(currentPos, new Pose2d(25, return_target3.getY(), Math.PI), 1.0, 1.0, 1.0);
                            }
                            else{
                                drive.goToPoint(currentPos, new Pose2d(17.5, return_target3.getY(), Math.PI), 1.0, 1.0, 1.0);
                            }

                        }
                        if (currentPos.getY() >= return_target3.getY() / 3) {
                            intake.close();
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
                    if (delay.time() >= 0.5) {
                        flip.operate(1);
                    }
                    else{
                        flip.clamp();
                    }
                    drive.setPower(0.0, 0.0, 0.0);
                }
                break;
            case STATE_INTAKE4:
                flip.read();
                if(intake4.vec().distTo(currentPos.vec()) <= 3.0|| flip.IntakeFeedback()){
                    newState(State.STATE_EXITPOOL4);
                }
                else if (delay.time() >= 3.0){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_EXITPOOL4);
                }
                else{
                    //drive.setPower(-0.15, 0.0, 0.0);
                    drive.goToPoint(currentPos, intake4, 0.2, 0.2, 0.2);
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
                if(cross_target3.vec().distTo(currentPos.vec()) <= 3.0 || delay.time() >= 5.0) {
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_PARK);
                }else{
                    intake.setPower(0.15);
                    if(delay.time() >= 3.0){
                        intake.close();
                        flip.clamp();
                    }else{
                        intake.open();
                        if (Math.abs(cross_target3.getY() - currentPos.getY()) > 40) {
                            flip.operate(4);
                        }
                    }
                    if(Math.abs(currentPos.getY()) >= 40){
                        slides.PIDController(2);
                    }
                    if (Math.abs(cross_target3.getY() - currentPos.getY()) <= 30){
                        flip.operate(0);
                    }
                    else if (Math.abs(cross_target3.getY() - currentPos.getY()) <= 40){
                        flip.clamp();
                    }
                    if(delay.time() >= 3.0){
                        intake.close();
                    }else{
                        intake.open();
                    }
                    drive.goToPoint(currentPos, saved_deposit, 1.0,1.0,1.0);
                }
                break;
            case STATE_EXITPOOL4:
                if (delay.time() >= 0.3) {
                    if (exit_pool4.vec().distTo(currentPos.vec()) <= 3.0) {
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_CROSS4);
                    } else if (delay.time() >= 2.5) {
                        drive.setPower(0.0, 0.0, 0.0);
                        newState(State.STATE_CROSS4);
                    } else {
                        intake.setPower(0.0);
                        drive.goToPoint(currentPos, exit_pool4, 0.8, 0.8, 0.8);
                        mStateTime.reset();
                    }
                }
                intake.open();
                flip.operate(4);
                break;
            case STATE_PARK:
                if (delay.time() >= 1.0) {
                    if(Math.abs(currentPos.getY()) < 55){
                        slides.dropSlides(-0.75);
                    }
                    if (Math.abs(currentPos.getY()) < 65){
                        flip.start();
                    }
                    if (Math.abs(-20 - currentPos.getY()) <= 3.0) {
                        drive.setPower(0.0, 0.0, 0.0);
                        
                        newState(State.STATE_STOP);
                        telemetry.addData("Tee Hee :)", "Deal with it this is my senior year");
                    } else if (delay.time() >= 3.0) {
                        drive.setPower(0.0, 0.0, 0.0);
                        
                        newState(State.STATE_STOP);
                    } else {
                        if (delay.time() >= 2.0) {
                            intake.setPower(0.0);
                        } else {
                            intake.setPower(-1.0);
                        }
                        intake.close();
                        intake.setPower(0.0);
                        drive.goToPoint(currentPos, new Pose2d(16.922, 35.976, Math.PI), 1.0, 1.0, 1.0);
                        mStateTime.reset();
                    }
                }
                else{
                    if (delay.time() >= 0.5){
                        flip.operate(1);
                    }
                    drive.setPower(0.0,0.0,0.0);
                }
                break;
            case STATE_STOP:
                drive.setPower(0.0, 0.0, 0.0);

                telemetry.addData("Tee Hee :)", "Deal with it this is my senior year");
                break;
        }
        telemetry.addData("POS: ", currentPos.toString());
        telemetry.addData("State: ", mRobotState);
        telemetry.addData("time: ", mStateTime.time());

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
        mRobotState = state;
    }
}
