package org.firstinspires.ftc.teamcode.Odometry

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import org.openftc.revextensions2.RevBulkData

class MA3_Encoder(name : String, hardwareMap: HardwareMap, offset : Double) {
    var e = 0.0
    var sign = 1.0
    var pos = 0.0
    val encoder = hardwareMap.get(AnalogInput::class.java, name)
    val offset = offset
    var prev_velo = 0.0

    val MAX_ACCEL = 60

    var previous = 0.0
    var base = 0.0


    fun reverse(){
        sign *= -1.0
    }

    fun calibrate(data : RevBulkData){
        //base = (data.getAnalogInputValue(encoder).toDouble() * (2 * Math.PI / 520) - offset + (2 * Math.PI)) % (2 * Math.PI)
        base = (data.getAnalogInputValue(encoder).toDouble() * (2 * Math.PI / 368.474576) - offset + (2 * Math.PI)) % (2 * Math.PI)
    }

    fun getRawDist(data: RevBulkData) : Double{
        return data.getAnalogInputValue(encoder).toDouble()
    }

    fun getDist() : Double{
        return e * sign
    }

    fun update(data : RevBulkData) {
        //pos = (data.getAnalogInputValue(encoder).toDouble() * (2 * Math.PI / 520) - offset - base + (2 * Math.PI)) % (2 * Math.PI)
        pos = (data.getAnalogInputValue(encoder).toDouble() * (2 * Math.PI / 368.474576) - offset - base + (2 * Math.PI)) % (2 * Math.PI)
        //pos = (data.getAnalogInputValue(encoder).toDouble())// * (2 * Math.PI / 3.3))

        when {
            pos - previous < -Math.toRadians(200.0) -> e += ((2 * Math.PI) + (pos - previous))
            pos - previous > Math.toRadians(200.0) -> e -= ((2 * Math.PI) + (previous - pos))
            else -> e += (pos - previous)
        }

        previous = pos
        pos = (data.getAnalogInputValue(encoder).toDouble() * (2 * Math.PI / 368.474576) - offset + (2 * Math.PI)) % (2 * Math.PI)
        //pos = (data.getAnalogInputValue(encoder).toDouble() * (2 * Math.PI / 520) - offset - base + (2 * Math.PI)) % (2 * Math.PI)
    }
}