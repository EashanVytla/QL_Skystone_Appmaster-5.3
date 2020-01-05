
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@Autonomous(name = "QL_Skystone_Blue", group = "Competition")
@Disabled
public class QL_2SkyStone_SERVO extends OpMode {
    Dead_Wheel leftWheel;
    Dead_Wheel rightWheel;
    Dead_Wheel strafeWheel;
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
    PID pid;

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;

    //Reallign Coeffients
    double kpRL = 0.21;
    double kpPV = 0.15;

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
        STATE_PULL_FORWARD,
        STATE_PUSH_BACK,
        STATE_TURN,
        STATE_REALLIGN4,
        STATE_PIVOT,
        STATE_DRIVE_TO_FOUNDATION,
        STATE_PARK,
        STATE_RELEASE
    }
    State mRobotState = State.STATE_DRIVE_TO_BLOCK;

    public void init(){
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        leftWheel = new Dead_Wheel(new MA3_Encoder("a3", hardwareMap, 0.0));
        rightWheel = new Dead_Wheel(new MA3_Encoder("a4", hardwareMap, 0.0));
        strafeWheel = new Dead_Wheel(new MA3_Encoder("a1", hardwareMap, 0.0));
        drive = new Mecanum_Drive(hardwareMap, telemetry);
        flip = new Flipper(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        pid = new PID(hardwareMap, telemetry);
        bservo = hardwareMap.get(Servo.class, "Bmover");


        rightWheel.getEncoder().reverse();
        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();
        leftWheel.getEncoder().calibrate(data);
        rightWheel.getEncoder().calibrate(data2);
        strafeWheel.getEncoder().calibrate(data);
        bservo.setPosition(0.0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDetectorPipeline();
        webcam.openCameraDevice();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        bservo = hardwareMap.get(Servo.class, "Bmover");
        flip.initialize();

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
        newState(State.STATE_DRIVE_TO_BLOCK);
    }

    Pose2d cross_target;
    Pose2d return_target;
    Pose2d drive_to_block_target;
    Pose2d drive_platform_target;

    public void loop(){
        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();
        SkystonePos = 2;
        flip.read();
        odos.update();
        Pose2d currentPos = odos.getPoseEstimate();

        leftWheel.update(data);
        rightWheel.update(data2);
        strafeWheel.update(data);

        if(SkystonePos == 0){
            cross_target = new Pose2d();
        }else if(SkystonePos == 1){
            cross_target = new Pose2d();
            //Target Dist for Return is equal to ---
        }else if(SkystonePos == 2){
            cross_target = new Pose2d(-18, -47);
            drive_to_block_target = new Pose2d(-31, 7.5, 0.0);
            return_target = new Pose2d();
            drive_platform_target = new Pose2d();
            //Target Dist for Return is equal to ---
        }

        switch (mRobotState) {
            case STATE_DRIVE_TO_BLOCK:
                if(Math.abs(drive_to_block_target.getX() - currentPos.getY()) <= 1.0){
                    if(mStateTime.time() >= 1.0){
                        drive.setPower(0.0, 0.0, 0.0);
                        memo = Math.abs(getStrafeDist());
                        newState(State.STATE_PICKUP);
                    }else{
                        localizer.GoTo(drive_to_block_target,0.5,0.5,0.5);
                    }
                }else{
                    localizer.GoTo(drive_to_block_target,0.5,0.5,0.5);
                    mStateTime.reset();
                }
                break;
            case STATE_PICKUP:
                if (mStateTime.time() >= 2.0) {
                    memo = Math.abs(getStrafeDist());
                    newState(State.STATE_STRAFEOUT);
                }else{
                    bservo.setPosition(0.6);
                }
                break;
            case STATE_STRAFEOUT:
                if (Math.abs(-21 - currentPos.getX()) <= 1.0 && Math.abs(11 - currentPos.getY()) <= 1.0 && mStateTime.time() >= 3.0) {
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_CROSS);
                } else if(mStateTime.time() >= 4.0){
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_CROSS);
                }else{
                    localizer.GoTo(new Pose2d(-21, 11, 0.0),0.5,0.5,0.5);
                }
                break;
            case STATE_CROSS:
                if (Math.abs(cross_target.getX() - currentPos.getX()) <= 1.0 && Math.abs(cross_target.getX() - currentPos.getY()) <= 1.0) {
                    if(mStateTime.time() >= 1.0){
                        drive.setPower(0.0, 0.0, 0.0);
                        memo = getForwardDist();
                        newState(State.STATE_RETURN);
                    }else{
                        localizer.GoTo(cross_target,0.5,0.5,0.5);
                    }
                } else {
                    localizer.GoTo(cross_target,0.5,0.5,0.5);
                    mStateTime.reset();
                }
                break;
            case STATE_RETURN:
                if (Math.abs(0 - currentPos.getX()) <= 1.0 && Math.abs(0 - currentPos.getY()) <= 1.0 && mStateTime.time() >= 3.0) {
                    drive.setPower(0.0, 0.0, 0.0);
                    memo = getForwardDist();
                    //newState(State.STATE_REALLIGN2);
                } else {
                    localizer.GoTo(new Pose2d(0.0, 0.0, 0.0),0.5,0.5,0.5);
                }
                break;
            case STATE_REALLIGN2:
                if(Math.abs(Math.abs(getLeftDist()) - memoL) <= 0.1 && Math.abs(Math.abs(getRightDist()) - memoR) <= 0.1){
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_CROSS);
                }else if(mStateTime.time() >= 3.0) {
                    drive.setPower(0.0,0.0,0.0);
                    newState(State.STATE_CROSS);
                }else{
                    drive.setPowerSide(Range.clip((getLeftDist() + drive_to_block_target.getX()) * kpRL, -0.4, 0.4), Range.clip((getRightDist() + drive_to_block_target.getY()) * kpRL, -0.4, 0.4));
                }
                break;
        }
        telemetry.addData("POS: ", odos.getPoseEstimate().toString());
        telemetry.addData("mStateTime: ", mStateTime.time());
        telemetry.addData("State: ", mRobotState);

        flip.write();
        drive.write();
        intake.write();
    }

    private void newState(State state){
        localizer.resetfirst();
        mStateTime.reset();
        mRobotState = state;
    }

    private double getForwardDist(){
        return (rightWheel.getDistance());//* 100/162
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