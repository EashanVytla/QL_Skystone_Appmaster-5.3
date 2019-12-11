package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.openftc.revextensions2.RevBulkData
import java.util.*

class Intake(hardwareMap: HardwareMap) {
    var motors : Array<Caching_Motor>

    val UPDATE_RATE = 1.0
    var write_index = 0
    val open : Array<Caching_Servo>
    var time = ElapsedTime()

    init{
        motors = arrayOf(Caching_Motor(hardwareMap, "intake_left"), Caching_Motor(hardwareMap, "intake_right"))
        open = arrayOf(Caching_Servo(hardwareMap, "intake_left_jaw"), Caching_Servo(hardwareMap, "intake_right_jaw"))
        time.startTime()
    }

    fun start(){
        close()
    }

    enum class clamp{
        OPEN,
        CLOSE
    }

    var clampst = clamp.CLOSE
    fun read(data : RevBulkData){
        motors.map {
            it.read(data)
        }
    }

    fun write(){
        motors[write_index].write()
        open[write_index].write()
        write_index = (write_index + 1) % 2
    }

    fun setPower(power : Double){
        motors[0].setPower(power)
        motors[1].setPower(-power)
    }

    fun setPosition(position : Double){
        open[0].setPosition(position)
        open[1].setPosition(1-position)
    }

    fun kickout(){
        open()
        setPower(0.5)
        write()
        write()
    }

    fun initIntake(){
        open[0].setPosition(0.5)
        open[1].setPosition(0.5)
    }

    fun open(){
        open[0].setPosition(0.0)
        open[1].setPosition(1.0)
    }

    fun close(){
        open[0].setPosition(0.75) //0.7
        open[1].setPosition(0.25) //0.2
    }

    fun newState(clampState: clamp){
        clampst = clampState
        time.reset()
    }

    fun operate(g1 : Gamepad, g2: Gamepad){
        setPower((0.35 * g1.right_trigger) - (0.2 * g1.left_trigger))

        if(g2.left_bumper|| g2.left_trigger > 0.5 || g2.right_trigger > 0.5){
            newState(clamp.OPEN)
        }

        if(g1.left_bumper){
            open()
        }else{
            close()
        }

        if(clampst == clamp.OPEN){
            if (time.time() >= 5.0){
                close()
            }
            else{
                open()
            }
        }

        write()
    }
}