package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry

class ReAllign(map: HardwareMap, t: Telemetry){
    var aligner : Caching_Servo
    var time = ElapsedTime()

    init{
        aligner = Caching_Servo(map, "Aligner")
    }

    fun initialize(){
        aligner.setPosition(0.0)
    }

    fun operate(g:Gamepad){
        if(g.x){
            aligner.setPosition(0.4)
        } else if(g.y){
            aligner.setPosition(0.0)
        }
    }

}