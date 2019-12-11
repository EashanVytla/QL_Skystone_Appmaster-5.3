package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Universal.Math.Vector2;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevExtensions2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mecanum Optimization Test", group = "REV Optimized")
public class mecanum_test extends OpMode {
    private Mecanum_Drive drive;
    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;

    private long prev_time = System.currentTimeMillis();

    public void init(){
        RevExtensions2.init();

        drive = new Mecanum_Drive(hardwareMap);
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
    }

    public void loop(){
        drive.read(hub.getBulkInputData());
        drive.drive(gamepad1, gamepad2);

        telemetry.addData("Angle: ", drive.getExternalHeading());

        Vector2 v = new Vector2(gamepad1.left_stick_x, gamepad1.left_stick_y);
        v.rotate(-drive.getExternalHeading());
        telemetry.addData("Drive Vector: ", v.toString());

        telemetry.addData("Up Left Power: ", drive.getMotors().get(0).getPrev_power());
        telemetry.addData("Back Left Power: ", drive.getMotors().get(1).getPrev_power());
        telemetry.addData("Back Right Power: ", drive.getMotors().get(2).getPrev_power());
        telemetry.addData("Up Right Power: ", drive.getMotors().get(3).getPrev_power());

        telemetry.addData("Refresh Rate: ", 1000 / (System.currentTimeMillis() - prev_time));
        prev_time = System.currentTimeMillis();
        telemetry.addData("Write Frequency: 1 /", drive.getRefreshRate());
        telemetry.addData("Up Left: ", drive.getMotors().get(0).getCurrentPosition());
        telemetry.addData("Back Left: ", drive.getMotors().get(1).getCurrentPosition());
        telemetry.addData("Back Right: ", drive.getMotors().get(2).getCurrentPosition());
        telemetry.addData("Up Right: ", drive.getMotors().get(3).getCurrentPosition());
    }
}
