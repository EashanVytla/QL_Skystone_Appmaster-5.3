package org.firstinspires.ftc.teamcode;

import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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

    public void init(){
        RevExtensions2.init();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        drive = new Mecanum_Drive(hardwareMap);
        elevator = new Vertical_Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        //grabber = new GrabberV2(hardwareMap);
        flipper = new Flipper(hardwareMap, telemetry);
        //tape = new Tape_Extention(hardwareMap);

        //grabber.initialize();
        //flipper.initialize();

        intake.initIntake();

        RevBulkData data = hub.getBulkInputData();
        RevBulkData data2 = hub2.getBulkInputData();

        leftWheel = new Dead_Wheel(new MA3_Encoder("a3", hardwareMap, 0.495));
        rightWheel = new Dead_Wheel(new MA3_Encoder("a4", hardwareMap, 1.365));
        strafeWheel = new Dead_Wheel(new MA3_Encoder("a1", hardwareMap, 2.464));

        rightWheel.getEncoder().reverse();
        strafeWheel.getEncoder().reverse();
        leftWheel.getEncoder().calibrate(data);
        rightWheel.getEncoder().calibrate(data2);
        strafeWheel.getEncoder().calibrate(data);
        leftWheel.setBehavior(1.5385 * 2 * 0.797, -0.319237); //1.5144 0.0361262
        rightWheel.setBehavior(1.5385 * 2 * 0.797, -0.319237); //1.5204 -0.00305571
        strafeWheel.setBehavior(1.53642 * 2 * 0.797, 0.0); //1.50608 -0.221642
    }

    @Override public void start(){
        flipper.start();
        intake.start();
    }

    public boolean isPress(boolean value){
        boolean val = value && !previous;
        previous = value;
        return val;
    }

    public void loop(){
        drive.read(hub.getBulkInputData());
        elevator.read(hub2.getBulkInputData());
        intake.read(hub2.getBulkInputData());


        drive.drive(gamepad1, gamepad2);

        elevator.operate(gamepad2);
        intake.operate(gamepad1, gamepad2);
        //grabber.operate(gamepad2);
        flipper.operate(gamepad1, gamepad2);
        //tape.operate(gamepad1);
        //telemetry.addData("DRIVETRAIN MODE", (mode ? "Field Centric" : "Robot Centric"));
        telemetry.addData("DRIVETRAIN MODE", (drive.getMode() ? "Slow Mode" : "Regular Speed"));
        telemetry.addData("IMU", drive.angleWrap(drive.getExternalHeading()));

        telemetry.addData("Angle: ", drive.getExternalHeading());

        Vector2 v = new Vector2(gamepad1.left_stick_x, gamepad1.left_stick_y);
        v.rotate(-drive.getExternalHeading());

        telemetry.addData("Slide Error: ", elevator.getError());

        telemetry.addData("Case: ", flipper.getRcase());
        telemetry.addData("Is Dropped: ", elevator.isDropped());

        telemetry.addData("Boundry Condition", elevator.getBoundaryConditions());

        telemetry.addData("Drive Vector: ", v.toString());

        telemetry.addData("Intake Left Motor Power", intake.getMotors()[0].getMotor().getPower());
        telemetry.addData("Intake Right Motor Power", intake.getMotors()[1].getMotor().getPower());

        telemetry.addData("Intake Left prev power", intake.getMotors()[0].getPrev_power());
        telemetry.addData("Intake Right prev power", intake.getMotors()[1].getPrev_power());

        telemetry.addData("Up Left Power: ", drive.getMotors().get(0).getPrev_power());
        telemetry.addData("Back Left Power: ", drive.getMotors().get(1).getPrev_power());
        telemetry.addData("Back Right Power: ", drive.getMotors().get(2).getPrev_power());
        telemetry.addData("Up Right Power: ", drive.getMotors().get(3).getPrev_power());

        telemetry.addData("Refresh Rate: ", (System.currentTimeMillis() - prev_time));
        prev_time = System.currentTimeMillis();
        telemetry.addData("Write Frequency: 1 /", drive.getRefreshRate());
        telemetry.addData("Up Left: ", drive.getMotors().get(0).getCurrentPosition());
        telemetry.addData("Back Left: ", drive.getMotors().get(1).getCurrentPosition());
        telemetry.addData("Back Right: ", drive.getMotors().get(2).getCurrentPosition());
        telemetry.addData("Up Right: ", drive.getMotors().get(3).getCurrentPosition());

        telemetry.addData("Slide Motor 1 Pos: ", elevator.getMotors()[0].getCurrentPosition());
        telemetry.addData("Slide Motor 2 Pos: ", elevator.getMotors()[1].getCurrentPosition());
        telemetry.addData("mode", drive.getSlow_mode());

        telemetry.addData("Slide Power: ", gamepad2.right_stick_y);
        flipper.ShowPos();
    }

    private double getForwardDist(){
        return leftWheel.getDistance() * (23 / 37.678);
    }

    private double getStrafeDist(){
        return strafeWheel.getDistance() * 7.0 / 17.536;
    }

    private double getAngle(){return drive.angleWrap(drive.getExternalHeading());}
}
