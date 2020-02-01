package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevExtensions2;

@TeleOp(name = "Slide Test", group = "Slides")
@Disabled
public class slide_test extends OpMode {
    private Vertical_Elevator elevator;

    private ExpansionHubEx hub2;

    public void init(){
        RevExtensions2.init();

        elevator = new Vertical_Elevator(hardwareMap, telemetry);
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
    }

    public void loop(){
        elevator.read(hub2.getBulkInputData());
        elevator.operate(gamepad2, gamepad1);

        telemetry.addData("Motor 1 Pos: ", elevator.getMotors()[0].getCurrentPosition());
        telemetry.addData("Motor 2 Pos: ", elevator.getMotors()[1].getCurrentPosition());

        telemetry.addData("Motor 1 Target: ", elevator.getMotors()[0].getMotor().getTargetPosition());
        telemetry.addData("Motor 2 Target: ", elevator.getMotors()[1].getMotor().getTargetPosition());
    }
}
