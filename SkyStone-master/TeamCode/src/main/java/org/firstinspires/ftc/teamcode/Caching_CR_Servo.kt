package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs

class Caching_CR_Servo(hardwareMap : HardwareMap, name : String) {
    var servo = hardwareMap.get(CRServo::class.java, name)
    var query = 0.0
    var prev_power = 0.0

    val EPSILON = 0.001

    fun setPower(power : Double){
        if (abs(prev_power - power) > EPSILON){
            query = power
        }
    }

    fun write(){
        servo.power = query
        prev_power = query
    }
}