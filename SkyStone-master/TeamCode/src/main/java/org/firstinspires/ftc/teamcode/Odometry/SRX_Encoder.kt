package org.firstinspires.ftc.teamcode.Odometry

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.openftc.revextensions2.RevBulkData

class SRX_Encoder(name : String, hardwareMap: HardwareMap){
    var encoder : DcMotor
    var distance = 0.0
    var reverse = false

    init {
        encoder = hardwareMap.get(DcMotor::class.java, name)
        encoder.mode = DcMotor.RunMode.RUN_USING_ENCODER
        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    fun reset(){
        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    fun reverse(){
        reverse = true
    }

    fun update(data : RevBulkData){
        distance = encoder.currentPosition.toDouble() / (4096/(2.552 * Math.PI))
    }

    fun getDist() : Double{
        if(reverse == true){
            distance = -distance
        }
        return distance
    }
}