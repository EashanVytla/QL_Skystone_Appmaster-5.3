package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry

class Tape_Extention(hardwareMap: HardwareMap){
    var CServo : CRServo

    init {
        CServo = hardwareMap.get(CRServo::class.java, "tape")
    }

    fun operate(g : Gamepad){
        CServo.power = 0.5 * (g.left_trigger - g.right_trigger)

    }
}