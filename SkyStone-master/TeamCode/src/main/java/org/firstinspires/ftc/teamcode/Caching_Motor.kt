package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.openftc.revextensions2.RevBulkData
import kotlin.math.abs

class Caching_Motor(hardwareMap : HardwareMap, name : String) {
    val motor : DcMotor = hardwareMap.get(DcMotor::class.java, name)
    var prev_power = 0.0

    var query = -2.0

    var pos = 0

    val EPSILON = 0.001

    var prev_write = 0
    var current_write = 0

    fun setPower(power : Double){
        if (abs(prev_power - power) > EPSILON){
            query = power
        }
        else{
            query = -2.0
        }
    }

    fun write(){
        if (query != -2.0) {
            motor.power = query
            prev_power = query
        }
    }

    fun write(rate : Double) {
        current_write++
        if (prev_write.toDouble() / current_write.toDouble() < rate) {
            write()
            prev_write = current_write
        }
    }

    fun getCurrentPosition() : Int{
        return pos
    }

    fun read(data : RevBulkData){
        pos = data.getMotorCurrentPosition(motor)
    }
}