package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.SRX_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.SRX_Three_Wheel_Localizer;
import org.firstinspires.ftc.teamcode.Odometry.ThreeWheelTrackingLocalizer;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevExtensions2;

public class RED_Foundation_Auto_VA_States extends OpMode {
    SkystoneDetectorPipeline pipeline;

    ThreeWheelTrackingLocalizer odos;
    //SRX_Three_Wheel_Localizer localizer;

    Mecanum_Drive drive;
    FlipperV2 flip;
    Intake intake;

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;

    ElapsedTime mStateTime = new ElapsedTime();

    public void init() {
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        drive = new Mecanum_Drive(hardwareMap, telemetry);
        flip = new FlipperV2(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);

        odos = new ThreeWheelTrackingLocalizer(this.hardwareMap);

        odos.poseSet(new Pose2d(0, 0, -Math.PI / 2));
        //localizer.getOdos().poseSet(new Pose2d(0, 0, -Math.PI / 2));
        //localizer.inverse();
        mStateTime.startTime();
        odos.inverse();
    }


    public enum State{
        STATE_DELAY,
        STATE_DRIVE_TO_FOUNDATION,
        STATE_GRAB_FOUNDATION,
        STATE_PULL_FORWARD,
        STATE_TURN,
        STATE_PUSH_BACK,
        STATE_RELEASE,
        STATE_STRAFE,
        STATE_PARK
    }

    State mRobotState = State.STATE_DELAY;

    public void loop() {
        Pose2d currentPos = new Pose2d(odos.getEstimatedPose().vec(), (2 * Math.PI) - odos.getAbsoluteAngle());

        Pose2d drive_to_block_target = new Pose2d(-38, -38, Math.PI);
        Pose2d pull_forward_target = new Pose2d(-38, -38, Math.PI);
        Pose2d park_target = new Pose2d(-38, -38, Math.PI);

        switch (mRobotState) {
            case STATE_DELAY:
                if (mStateTime.time() >= 21) {
                    newState(State.STATE_DRIVE_TO_FOUNDATION);
                }
                break;
            case STATE_DRIVE_TO_FOUNDATION:
                if (currentPos.vec().distTo(drive_to_block_target.vec()) <= 1.0) {
                    newState(State.STATE_GRAB_FOUNDATION);
                } else {
                    drive.goToPoint(currentPos, drive_to_block_target, 1.0, 1.0, 1.0);
                }
                flip.startKnocker();
            case STATE_GRAB_FOUNDATION:
                if (mStateTime.time() >= 0.5) {
                    newState(State.STATE_PULL_FORWARD);
                } else {
                    flip.grabPlatform();
                }
                break;
            case STATE_PULL_FORWARD:
                if (currentPos.vec().distTo(pull_forward_target.vec()) <= 1.0) {
                    newState(State.STATE_TURN);
                } else {
                    drive.goToPoint(currentPos, pull_forward_target, 1.0, 1.0, 1.0);
                }
                break;
            case STATE_TURN:
                if (currentPos.getHeading() >= Math.toRadians(-5.0)) {
                    drive.setPower(0.0, 0.0, 0.0);
                    newState(State.STATE_PARK);
                } else {
                    drive.setPower(0.0, 0.0, -1.0);
                }
                break;
            case STATE_PARK:
                if (currentPos.vec().distTo(park_target.vec()) <= 1.0) {
                    drive.setPower(0.0, 0.0, 0.0);
                } else {
                    drive.goToPoint(currentPos, park_target, 1.0, 1.0, 1.0);
                }
                break;
        }
        drive.write();
    }

    public void newState(State state){
        mRobotState = state;
        mStateTime.reset();
    }
}
