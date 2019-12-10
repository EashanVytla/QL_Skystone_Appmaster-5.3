package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap

class GrabberV2(h: HardwareMap){
    var grabbers : Array<Caching_Servo>

    var write_index = 0
    internal var imu: BNO055IMU? = null

    init{
        grabbers = arrayOf(Caching_Servo(h, "grabber_1"), Caching_Servo(h, "grabber_2"))
    }

    fun write(){
        grabbers[write_index].write()
        write_index = (write_index + 1) % 2
    }

    fun grab(grabPos: Double, clampPos: Double){
        grabbers[0].setPosition(grabPos)
        grabbers[1].setPosition(clampPos)
    }

    fun unclamp(){
        grabbers[0].setPosition(0.0)
        grabbers[1].setPosition(0.0)
    }

    fun initialize(){
        grabbers[0].setPosition(0.0)
        grabbers[1].setPosition(0.0)
        write()
        write()
    }

    //Soon this will go into automation and there will be no need for operation

    fun operate(g: Gamepad){
        var clickL = false
        var clickR = false

        if(g.right_bumper) {
            clickR = true
        }else if (g.left_bumper) {
            clickL = true
        }

        if(clickR){
            grab(0.9, 0.1)
        }else if (clickL){
            unclamp()
        }

        write()
    }

}
