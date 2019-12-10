package org.firstinspires.ftc.teamcode

import org.openftc.revextensions2.RevBulkData
import kotlin.math.abs

class Dead_Wheel(encoder: MA3_Encoder) {
    var encoder = encoder
    var dist = 0.0
    var POC = 0.0

    var m = 1.0
    var b = 0.0

    fun update(data : RevBulkData){
        encoder.update(data)
        dist = encoder.getDist() * 1.88976 / 6
        POC = ((angleWrap(encoder.pos) % (Math.PI / 2)) - (Math.PI / 4)) / abs((angleWrap(encoder.pos) % (Math.PI / 2)) - (Math.PI / 4))
    }

    fun angleWrap(angle : Double) : Double{
        return (angle + (2 * Math.PI)) % (2 * Math.PI)
    }

    fun setBehavior(m : Double, b : Double){
        this.m = m
        this.b = b
    }

    fun getDistance() : Double{
        return (m * dist) + b
    }
}