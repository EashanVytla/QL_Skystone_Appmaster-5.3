package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.openftc.revextensions2.RevBulkData
import kotlin.math.abs

class Caching_Servo(hardwareMap: HardwareMap, name : String){
    var servo = hardwareMap.get(Servo::class.java, name)
    var prev_pos = -1.0
    var query = -1.0

    val EPSILON = 0.001

    var prev_write = 0
    var current_write = 0

    fun setPosition(pos : Double){
        if (abs(pos - prev_pos) > EPSILON){
            query = pos
        }
        else{
            query = -1.0
        }
    }

    fun getPosition() : Double{
        return prev_pos
    }

    fun write(){
        if (query != -1.0) {
            servo.position = query
            prev_pos = query
        }
    }

    fun write(rate : Double){
        current_write++
        if (prev_write.toDouble() / current_write.toDouble() < rate){
            write()
            prev_write = current_write
        }
    }
}