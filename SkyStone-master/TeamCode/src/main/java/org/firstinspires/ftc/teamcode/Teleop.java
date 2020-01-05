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
    private Flipper flipper;

    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;
    private boolean mode;

    private boolean previous = false;

    private long prev_time = System.currentTimeMillis();

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
        flipper = new Flipper(hardwareMap, telemetry);
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

    public void loop(){
        RevBulkData data = hub2.getBulkInputData();
        intake.close();
        intake.write();
        if (data != null){
            elevator.read(data);
        }

        Pose2d currentPos = odos.getPoseEstimate();
        odos.update();

        if (gamepad1 != null && gamepad2 != null) {
            //todo: Tune these:
            if(odos.getPoseEstimate().getY() >= ZONE1_LEFTBOUNDS && odos.getPoseEstimate().getY() <= ZONE1_RIGHTBOUNDS && odos.getPoseEstimate().getX() > ZONE1_UPPERBOUNDS){
                zone = 1;
            }else if(odos.getPoseEstimate().getY() >= ZONE0_LEFTBOUNDS && odos.getPoseEstimate().getY() <= ZONE0_RIGHTBOUNDS && odos.getPoseEstimate().getX() > ZONE0_UPPERBOUNDS){
                zone = 0;
            }else if(odos.getPoseEstimate().getY() >= ZONE2_LEFTBOUNDS && odos.getPoseEstimate().getY() <= ZONE2_RIGHTBOUNDS && odos.getPoseEstimate().getX() > ZONE2_UPPERBOUNDS){
                zone = 2;
            }
            localizer.updateodos();

            if(currentPos.getHeading() <= Math.PI){
                heading = currentPos.getHeading();
            }else{
                heading = -((2 * Math.PI ) - currentPos.getHeading());
            }

            telemetry.addData("Zone: ", zone);

            if(automationSt == State.IDLE){
                drive.drive(gamepad1, gamepad2);
                if(isPress2(gamepad1.y, previous6)){
                    stored_pos = new Pose2d(currentPos.getY(), currentPos.getX(), heading);
                }else if(isPress2(gamepad1.left_bumper, previous7)){
                    pos = new Pose2d(stored_pos.getX(), stored_pos.getY(), stored_pos.getHeading());
                    //pos = new Pose2d(0.0, 20.0, 0.0);
                    automationSt = State.AUTOALLIGN;
                }else if(isPress2(gamepad1.dpad_left, previous8)){
                    pos = new Pose2d(stored_pos.getX() + 8, stored_pos.getY(), stored_pos.getHeading());
                    automationSt = State.CAPSTONE;
                }else{
                    automationSt = State.IDLE;
                }
            }else if(automationSt == State.AUTOALLIGN){
                if(gamepad1.left_stick_y >= 0.7){
                    automationSt = State.IDLE;
                }else{
                    localizer.GoToTOP(pos, 0.5, 0.5, 0.5);
                }
            }
            drive.write();
        }
        else{
            drive.setPower(0.0, 0.0, 0.0);
            drive.write();
        }

        telemetry.addData("current pos: ", currentPos.toString());
        telemetry.addData("Stored Position: ", stored_pos.toString());
        telemetry.addData("pos: ", pos.toString());
        telemetry.addData("State: ", automationSt);
        previous6 = gamepad1.y;
        previous7 = gamepad1.left_bumper;
        previous8 = gamepad1.dpad_left;

        elevator.operate(gamepad2);
        intake.operate(gamepad1, gamepad2);
        //grabber.operate(gamepad2);
        flipper.operate(gamepad1, gamepad2);
        tape.operate(gamepad2);
        //telemetry.addData("DRIVETRAIN MODE", (mode ? "Field Centric" : "Robot Centric"));
        telemetry.addData("DRIVETRAIN MODE", (drive.getMode() ? "Slow Mode" : "Regular Speed"));
        telemetry.addData("Stored Pos: ", drive.getStored_pos().toString());
        //telemetry.addData("1mAngle: ", drive.getExternalHeading());

        Vector2 v = new Vector2(gamepad1.left_stick_x, gamepad1.left_stick_y);
        v.rotate(-drive.getExternalHeading());

        //telemetry.addData("Slide Error: ", elevator.getError());

        telemetry.addData("Case: ", flipper.getRcase());

        //telemetry.addData("Boundry Condition", elevator.getBoundaryConditions());

        //telemetry.addData("Drive Vector: ", v.toString());

        //telemetry.addData("Intake Left Motor Power", intake.getMotors()[0].getMotor().getPower());
        //telemetry.addData("Intake Right Motor Power", intake.getMotors()[1].getMotor().getPower());

        //telemetry.addData("Intake Left prev power", intake.getMotors()[0].getPrev_power());
        //telemetry.addData("Intake Right prev power", intake.getMotors()[1].getPrev_power());

        //telemetry.addData("Up Left Power: ", drive.getMotors().get(0).getPrev_power());
        //telemetry.addData("Back Left Power: ", drive.getMotors().get(1).getPrev_power());
        //telemetry.addData("Back Right Power: ", drive.getMotors().get(2).getPrev_power());
        //telemetry.addData("Up Right Power: ", drive.getMotors().get(3).getPrev_power());

        //telemetry.addData("Refresh Rate: ", (System.currentTimeMillis() - prev_time));
        //prev_time = System.currentTimeMillis();
        //telemetry.addData("Write Frequency: 1 /", drive.getRefreshRate());

        //telemetry.addData("Slide Motor 1 Pos: ", elevator.getMotors()[0].getCurrentPosition());
        //telemetry.addData("Slide Motor 2 Pos: ", elevator.getMotors()[1].getCurrentPosition());


        telemetry.addData("Slide Power: ", gamepad2.right_stick_y);
        //flipper.ShowPos();
    }

    private double getForwardDist(){
        return leftWheel.getDistance() * (23 / 37.678);
    }

    private double getStrafeDist(){
        return strafeWheel.getDistance() * 7.0 / 17.536;
    }

    private double getAngle(){return drive.angleWrap(drive.getExternalHeading());}
}
