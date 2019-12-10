package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.openftc.revextensions2.RevBulkData

class Grabber(hardwareMap: HardwareMap) {
    var grabbers : Array<Caching_Servo>

    var write_index = 0

    init{
        grabbers = arrayOf(Caching_Servo(hardwareMap, "grabber_1"), Caching_Servo(hardwareMap, "grabber_2"))
    }

    fun write(){
        grabbers[write_index].write()
        write_index = (write_index + 1) % 2
    }

    fun clamp(pos: Double){
        grabbers[0].setPosition(pos)
        grabbers[1].setPosition(1-pos)
    }

    fun unclamp(){
        grabbers[0].setPosition(0.5)
        grabbers[1].setPosition(0.5)
    }

    fun initialize(){
        grabbers[0].setPosition(0.0)
        grabbers[1].setPosition(1.0)
        write()
        write()
    }

    //Soon this will go into automation and there will be no need for operation

    fun operate(clamp_pos: Double, g:Gamepad){
        var clickL = false
        var clickR = false

        if(g.right_bumper) {
            clickR = true
        }else if (g.left_bumper) {
            clickL = true
        }

        if(clickR){
            clamp(clamp_pos)
        }else if (clickL){
            unclamp()
        }

        write()
    }
}