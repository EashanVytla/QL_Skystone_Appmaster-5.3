package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Odometry.Dead_Wheel;
import org.firstinspires.ftc.teamcode.Odometry.MA3_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.SRX_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.SRX_Three_Wheel_Localizer;
import org.firstinspires.ftc.teamcode.Odometry.ThreeWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.Universal.Math.Vector2;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

@TeleOp(name = "Teleop", group = "Competition")
public class Teleop extends OpMode{
    private Mecanum_Drive drive;
    private Vertical_Elevator elevator;
    private Intake intake;
    //private GrabberV2 grabber;
    private FlipperV2 flipper;

    private long prev_time = System.currentTimeMillis();

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;
    private boolean mode;

    private boolean previous = false;

    Dead_Wheel leftWheel;
    Dead_Wheel rightWheel;
    Dead_Wheel strafeWheel;

    Double prevStrafe = 0.0;
    Tape_Extention tape;
    SRX_Three_Wheel_Localizer localizer;
    ThreeWheelTrackingLocalizer odos;

    boolean previous6 = false;
    boolean previous7 = false;
    boolean previous8 = false;
    boolean previous9 = false;
    boolean previous10 = false;

    //ZONE 1 BOUNDS
    final double ZONE1_RIGHTBOUNDS = 0.0;
    final double ZONE1_LEFTBOUNDS = 0.0;
    final double ZONE1_UPPERBOUNDS = 0.0;

    //ZONE0 BOUNDS
    final double ZONE0_RIGHTBOUNDS = 0.0;
    final double ZONE0_LEFTBOUNDS = 0.0;
    final double ZONE0_UPPERBOUNDS = 0.0;

    //ZONE2 BOUNDS
    final double ZONE2_RIGHTBOUNDS = 0.0;
    final double ZONE2_LEFTBOUNDS = 0.0;
    final double ZONE2_UPPERBOUNDS = 0.0;

    Pose2d stored_pos = new Pose2d(0.0,0.0,0.0);
    Pose2d pos = new Pose2d(0.0,0.0,0.0);

    public void init(){
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        drive = new Mecanum_Drive(hardwareMap, telemetry);
        elevator = new Vertical_Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        //grabber = new GrabberV2(hardwareMap);
        flipper = new FlipperV2(hardwareMap, telemetry);
        tape = new Tape_Extention(hardwareMap);

        //grabber.initialize();
        //flipper.initialize();

        intake.initIntake();

        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();

        odos = new ThreeWheelTrackingLocalizer(hardwareMap);
        localizer = new SRX_Three_Wheel_Localizer(new SRX_Encoder("intake_left", hardwareMap), new SRX_Encoder("intake_right", hardwareMap), new SRX_Encoder("lift_2", hardwareMap), hardwareMap, telemetry);
    }

    public enum State{
        IDLE,
        AUTOALLIGN,
        CAPSTONE
    }

    State automationSt = State.IDLE;

    @Override public void start(){
        flipper.start();
        intake.start();
    }

    public boolean isPress(boolean value){
        boolean val = value && !previous;
        previous = value;
        return val;
    }

    public boolean isPress2(boolean value, boolean prev){
        boolean val = value && !prev;
        prev = value;
        return val;
    }

    int zone = 1;
    double heading = 0.0;

    public void loop() {
        long dt = System.currentTimeMillis() - prev_time;
        if (dt != 0.0){
            //telemetry.addData("Refresh Rate: ", 1000 / dt);
        }
        //telemetry.addData("Dt: ", dt);
        prev_time = System.currentTimeMillis();
        RevBulkData data = hub2.getBulkInputData();

        if (data != null) {
            elevator.read(data);
            odos.dataUpdate(data);
            odos.update();
        }

        Pose2d currentPos = new Pose2d(odos.getEstimatedPose().getX(), odos.getEstimatedPose().getY(), heading);

        if (currentPos.getHeading() <= Math.PI) {
            heading = currentPos.getHeading();
        } else {
            heading = -((2 * Math.PI) - currentPos.getHeading());
        }

        if (gamepad1 != null && gamepad2 != null) {
            //todo: Tune these:
            elevator.operate(gamepad2, gamepad1);
            if (odos.getEstimatedPose().getY() >= ZONE1_LEFTBOUNDS && odos.getEstimatedPose().getY() <= ZONE1_RIGHTBOUNDS && odos.getEstimatedPose().getX() > ZONE1_UPPERBOUNDS) {
                zone = 1;
            } else if (odos.getEstimatedPose().getY() >= ZONE0_LEFTBOUNDS && odos.getEstimatedPose().getY() <= ZONE0_RIGHTBOUNDS && odos.getEstimatedPose().getX() > ZONE0_UPPERBOUNDS) {
                zone = 0;
            } else if (odos.getEstimatedPose().getY() >= ZONE2_LEFTBOUNDS && odos.getEstimatedPose().getY() <= ZONE2_RIGHTBOUNDS && odos.getEstimatedPose().getX() > ZONE2_UPPERBOUNDS) {
                zone = 2;
            }
            localizer.updateodos();

            if (isPress2(gamepad1.y, previous6)){
                odos.poseSet(new Pose2d(0.0,0.0,0.0));
            }

            /*
            if(isPress2(gamepad1.right_bumper, previous10)){
                odos.poseSet(new Pose2d(0.0,0.0,0.0));
            }
            previous10 = gamepad1.right_bumper;
             */

            if (isPress2(gamepad1.right_bumper, previous7)){
                if(Mecanum_Drive.Companion.getCapstone()){
                    odos.poseSet(new Pose2d(0.0,9.0,0.0));
                    Mecanum_Drive.Companion.setAutomateLock2(true);
                }
            }

            //telemetry.addData("Zone: ", zone);

            drive.setCurrentPos(new Pose2d(currentPos.getX(), currentPos.getY(), heading));
            drive.drive(gamepad1, gamepad2);
            drive.write();
        } else {
            drive.setPower(0.0, 0.0, 0.0);
            drive.write();
        }
        telemetry.addData("Level:", elevator.getStack_count());
        telemetry.addData("Feeder: ", Vertical_Elevator.Companion.getDepositCheck());

        //telemetry.addData("current pos: ", drive.getCurrentPos().toString());
        telemetry.addData("Position: ", drive.getOdos().getEstimatedPose());
        //telemetry.addData("Stored Position: ", drive.getPosition().toString());
        //telemetry.addData("pos: ", pos.toString());
        //telemetry.addData("State: ", automationSt);

        previous6 = gamepad1.y;
        previous7 = gamepad1.left_bumper;
        previous8 = gamepad1.dpad_left;

        intake.operate(gamepad1, gamepad2);
        //grabber.operate(gamepad2);
        flipper.operate(gamepad1, gamepad2);
        tape.operate(gamepad2);
        //telemetry.addData("DRIVETRAIN MODE", (mode ? "Field Centric" : "Robot Centric"));
        telemetry.addData("DRIVETRAIN MODE", (drive.getMode() ? "Slow Mode" : "Regular Speed"));
        //telemetry.addData("1mAngle: ", drive.getExternalHeading());

        //Vector2 v = new Vector2(gamepad1.left_stick_x, gamepad1.left_stick_y);
        //v.rotate(-drive.getExternalHeading());

        telemetry.addData("Slide Pos: ", elevator.getLiftHeight());
        telemetry.addData("Slide Error: ", elevator.getError());

    }
}
