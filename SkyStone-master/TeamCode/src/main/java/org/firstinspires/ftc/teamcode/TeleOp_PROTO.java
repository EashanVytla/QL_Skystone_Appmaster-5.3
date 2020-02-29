package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Tele-Op Proto")
@Disabled
public class TeleOp_PROTO extends OpMode {
    Flipper flip;
    Vertical_Elevator slides;

    @Override
    public void init() {
        flip = new Flipper(
                hardwareMap,
                telemetry
        );
        slides = new Vertical_Elevator(
                hardwareMap,
                telemetry
        );
    }

    @Override
    public void loop() {
        flip.operate(gamepad1, gamepad2);
        slides.operate(gamepad1, gamepad2);
    }
}
