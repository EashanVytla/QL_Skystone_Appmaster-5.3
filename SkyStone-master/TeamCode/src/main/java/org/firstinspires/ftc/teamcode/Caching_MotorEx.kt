package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.*
import org.openftc.revextensions2.RevBulkData
import kotlin.math.abs

class Caching_MotorEx(hardwareMap : HardwareMap, name : String) {
    var motor : DcMotorEx = hardwareMap.dcMotor.get(name) as DcMotorEx

    var prev_power = 0.0
    var pos = 0

    fun write(){
        motor.power = prev_power
    }

    fun setPower(power : Double){
        if (abs(prev_power - power) < 0.001){
            prev_power = power
        }
    }

    fun read(data : RevBulkData){
        pos = data.getMotorCurrentPosition(motor)
    }

    fun getCurrentPosition() : Int{
        return pos
    }

    fun setPIDFCoefficients(mode : DcMotor.RunMode, coeffs : PIDFCoefficients){
        motor.setPIDFCoefficients(mode, coeffs)
    }

    fun getPIDFCoefficients(mode : DcMotor.RunMode) : PIDFCoefficients{
        return motor.getPIDFCoefficients(mode)
    }
}