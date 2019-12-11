package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime

class Tape_Extention(hardwareMap: HardwareMap){
    var CServo : Caching_CR_Servo
    var time = ElapsedTime()
    var prevtime = 0.0

    init {
        CServo = Caching_CR_Servo(hardwareMap, "Tape_Servo")
        time.startTime()
    }

    fun operate(g : Gamepad){
        if(g.x){
            CServo.setPower(2.0)
            if(time.time() - prevtime >= 1.0){
                CServo.setPower(0.0)
            }
        }else{
            prevtime = time.time()
        }
    }
}