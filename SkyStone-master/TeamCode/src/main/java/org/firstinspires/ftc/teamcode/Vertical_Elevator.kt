package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Universal.Motion.MotionProfile
import org.openftc.revextensions2.RevBulkData
import kotlin.math.abs

class Vertical_Elevator(map : HardwareMap, t : Telemetry){
    var motors : Array<Caching_Motor>

    val UPDATE_RATE = 1
    var write_index = 0
    var dclickt = ElapsedTime()

    var target = 0
    var first = true
    var x = 1

    var telemetry = t
    var isDropped = false
    var touch = map.get(DigitalChannel::class.java, "touch")
    val DROPDOWNPOS = 130

    var zero = 0.0

    var time = ElapsedTime()
    var clicks = 0

    var stack_count = 0

    var TargetPos = arrayOf(0, 593/2, 1071/2, 1459/2, 1890/2, 1216, 1405, 1646, 1880, 2095, 2330, 2567)

    var fine_tune = 1.0
    var error = 0.0

    var lastError = 0.0

    var previous = false;
    var previous2 = false;
    var lastTime = System.currentTimeMillis()

    var holdTime = ElapsedTime()
    var mStateTime = ElapsedTime()
    var mLeaveTime = ElapsedTime()
    var flip : FlipperV2
    var savedPos = 0.0

    var gff = 0.25//0.1926

    var increaseQueried = false


    companion object{
        const val speed = 1.0
        const val kp = 0.005 * speed
        const val kd = 0.001 * speed
        const val FF = 0.000
        const val k = 0.000183908
        var depositCheck = false
    }

    fun getDepositCheck() : Boolean{
        return depositCheck
    }

    enum class slideState{
        STATE_RAISE_INCREMENT,
        STATE_RAISE,
        STATE_DROP,
        STATE_IDLE,
        STATE_LEAVE_STACK,
        STATE_AUTOMATION,
        STATE_CAPSTONE
    }

    enum class leaveState{
        STATE_DROP,
        STATE_RAISE,
        STATE_RETURN,
        STATE_IDLE
    }

    var mLeaveState = leaveState.STATE_IDLE

    enum class slideBoundary{
        STATE_LOW,
        STATE_OPTIMAL,
        STATE_HIGH,
        STATE_UNKNOWN
    }

    var mSlideState = slideState.STATE_IDLE

    init{
        motors = arrayOf(Caching_Motor(map, "lift_1"), Caching_Motor(map, "lift_2"))
        touch.mode = DigitalChannel.Mode.INPUT
        //motors[1].motor.direction = DcMotorSimple.Direction.REVERSE
        motors.map{
            it.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        dclickt.startTime()
        holdTime.startTime()
        mStateTime.startTime()
        mLeaveTime.startTime()
        flip = FlipperV2(map, telemetry)
    }

    fun PIDController(TargetLevel : Int){
        error = TargetPos[TargetLevel] - getLiftHeight()

        var prop_gain = kp * error
        var deriv_gain = kd * ((error - lastError) / (System.currentTimeMillis() - lastTime))
        var ff_gain = 0.0
        if (TargetLevel > 8) {
            ff_gain = FF * TargetPos[TargetLevel]
        }
        lastTime = System.currentTimeMillis()
        var power = prop_gain + deriv_gain + ff_gain

        //if(stack_count >= 6){
            //power += 0.04 * (stack_count - 5)
        //}
        if (abs(power) > 0.001) {
            setPower(power)
        }
        else{
            setPower(0.0)
        }
        lastError = error
        telemetry.addData("Control Effort", power)
    }

    fun PIDControllerPos(Position : Double){
        error = Position - getLiftHeight()

        var prop_gain = kp * error
        var deriv_gain = kd * ((error - lastError) / (System.currentTimeMillis() - lastTime))
        var ff_gain = 0.0
        if (Position > TargetPos[8]) {
            ff_gain = FF * Position
        }
        lastTime = System.currentTimeMillis()
        var power = prop_gain + deriv_gain + ff_gain

        //if(stack_count >= 6){
        //power += 0.04 * (stack_count - 5)
        //}
        if (abs(power) > 0.001) {
            setPower(power)
        }
        else{
            setPower(0.0)
        }
        lastError = error
        telemetry.addData("Control Effort", power)
    }

    fun read(data : RevBulkData){
        motors[0].read(data)
        isDropped = !data.getDigitalInputState(touch)
    }

    fun write(){
        motors[write_index].write()
        write_index = (write_index + 1) % 2
    }

    fun setPower(power : Double){
        motors.map{
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        //Intantiating the power to the default power
        var newPower = power
        //Only applying the gravity feedforward if the current lift height is greater than the elevator's lowest position

        if (getLiftHeight() > 2299 / 2){
            newPower += k * (getLiftHeight() - 2299 / 2) //Subtracts the halfway point so that the x value in our spring force calculation starts at the half-way point
        }
        //First Checks to see if the power are positive and slides are going up because we don't want spring feedforward to be activated if the slides are going down
        if (power >= 0.0){
            //Checks if the lift height is greater than the threshold value of half way of the maximum of the elevator
            if (getLiftHeight() >= DROPDOWNPOS * 2){
                newPower += gff
            }
        }
        else{
            if (getLiftHeight() > 2299 / 2){
                newPower *= 0.5
            }
            if (getLiftHeight() < DROPDOWNPOS){
                newPower *= 1.5
            }
        }
        /*else{
            //If the slides are going down just give the driver or pid controller full control over the elevator
            if (getLiftHeight() < DROPDOWNPOS * 2){
                newPower /= Math.abs(newPower)
                newPower = Range.clip(newPower, -0.1, 0.1)
            }
        }*/
        motors[0].setPower(Range.clip(-newPower, -1.0, 1.0))
        motors[1].setPower(Range.clip(newPower, -1.0, 1.0))

        telemetry.addData("Speed Set", power)
        telemetry.addData("Left: ", motors[0].motor.power)
        telemetry.addData("Right: ", motors[1].motor.power)
    }

    fun getLiftHeight() : Double{
        return -motors[0].getCurrentPosition().toDouble()
    }

    fun getBoundaryConditions() : slideBoundary{
        if (getLiftHeight() >= 50){
            return slideBoundary.STATE_OPTIMAL
        }
        else if (getLiftHeight() < 100){
            return slideBoundary.STATE_LOW
        }
        else{
            return slideBoundary.STATE_UNKNOWN
        }
    }

    fun newState(s : slideState){
        mSlideState = s
        mStateTime.reset()
    }

    fun increment_stack_count(){
        stack_count = (stack_count + 1) % 12
    }

    fun decrement_stack_count(){
        stack_count = Math.abs(stack_count - 1)
    }

    fun setTargetPosition(target : Int){
        motors[0].motor.targetPosition = target
        motors[1].motor.targetPosition = target//motors[0].getCurrentPosition() //test to see if pos1 = -pos2, recommended test is to output encoder positions from both motors and analyze

        motors.map{
            it.motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            it.motor.power = 0.3
        }
    }

    fun setTargetPosBasic(target: Int, power: Double){
        if(stack_count > 1){
            //it.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            if(getLiftHeight() < (target)){
                setPower(power)
            }else if(getLiftHeight() >= (target)) {
                setPower(0.2)
            }
        }else if(stack_count <= 1){
            //it.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            if(getLiftHeight() < target){
                setPower(power)
            }else if(getLiftHeight() >= target) {
                setPower(0.2)
            }
        }
        if(power <= 0.0){
            telemetry.addData("State:", "Dropping")
            if(getLiftHeight() >= DROPDOWNPOS){
                setPower(power)
            }else {
                setPower(0.0)
            }
        }
    }

    fun dropSlides(power : Double){
        telemetry.addData("State:", "Dropping")
        if(stack_count < 3){
            if(getLiftHeight() >= DROPDOWNPOS - 20){
                setPower(power)
            }else {
                setPower(0.0)
            }
        }else{
            if(getLiftHeight() >= DROPDOWNPOS){
                setPower(power)
            }else {
                setPower(0.0)
            }
        }
    }

    fun newLeaveState(leaveState: leaveState){
        mLeaveState = leaveState
        mLeaveTime.reset()
    }

    fun isPress2(value : Boolean, previous : Boolean) : Boolean{
        return value && !previous
    }

    fun operate(g2 : Gamepad, g1 : Gamepad){
        motors.map {
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        if (getLiftHeight() < DROPDOWNPOS) {
            gff = 0.0
        }
        else{
            gff = 0.1
        }
        if(isPress2(g2.y, previous)){
            depositCheck = !depositCheck
        }
        previous = g2.y

        /*if(isPress2(g2.a, previous2)){
            newState(slideState.STATE_CAPSTONE)
        }
        previous2 = g2.a*/

        if (g2.x){
            newState(slideState.STATE_RAISE)
        }else if(g1.right_bumper){
            if(depositCheck){
                newState(slideState.STATE_LEAVE_STACK)
            }
            increaseQueried = true
            //newState(slideState.STATE_LEAVE_STACK)
        }else if (g2.b){
            newState(slideState.STATE_DROP)
        }else if(g2.dpad_up && dclickt.time() >= 0.3){
            increment_stack_count()
            newState(slideState.STATE_RAISE_INCREMENT)
            dclickt.reset()
        }else if(g2.dpad_down && dclickt.time() >= 0.3){
            decrement_stack_count()
            newState(slideState.STATE_RAISE_INCREMENT)
            dclickt.reset()
        }/*else if(g.dpad_left){
            if(stack_count >= 2){
                PIDController(stack_count - 2)
                if (abs(getLiftHeight() - TargetPos[stack_count - 2]) > 20){
                    holdTime.reset()
                }else if (mStateTime.time() > 2.0){
                    newState(slideState.STATE_IDLE)
                } else{
                    if (holdTime.time() >= 0.5) {
                        newState(slideState.STATE_IDLE)
                    }
                }

            }

        }
        */


        if(mSlideState == slideState.STATE_RAISE_INCREMENT){
            newState(slideState.STATE_IDLE)
        }else if(mSlideState == slideState.STATE_LEAVE_STACK){
            if(stack_count <= 10 && !Mecanum_Drive.capstone && true){
                fine_tune = 1.0
                if(mLeaveState == leaveState.STATE_IDLE){
                    savedPos = getLiftHeight()
                    newLeaveState(leaveState.STATE_DROP)
                }else if(mLeaveState == leaveState.STATE_DROP){
                    if(mLeaveTime.time() >= 0.5){
                        newLeaveState(leaveState.STATE_RAISE)
                    }else{
                        flip.unclamp()
                        flip.write()
                    }
                }else if(mLeaveState == leaveState.STATE_RAISE){
                    if(abs((savedPos + 570 / 2) - getLiftHeight()) >= 15.0 && mLeaveTime.time() < 1.0){
                        PIDControllerPos(savedPos + 570)
                    }else{
                        setPower(0.0)
                        newLeaveState(leaveState.STATE_RETURN)
                    }
                }else if(mLeaveState == leaveState.STATE_RETURN){
                    if(mLeaveTime.time() >= 1.0/*0.45*/){
                        newState(slideState.STATE_DROP)
                    }else{
                        setPower(0.0)
                        flip.operate(2)
                    }
                    if (increaseQueried) {
                        increment_stack_count()
                        increaseQueried = false
                    }
                }
            }else {
                if (mStateTime.time() >= 1.0) {
                    newState(slideState.STATE_IDLE)
                } else {
                    if (first) {
                        first = false
                        Mecanum_Drive.capTime.reset()
                    }
                    flip.operate(5)
                }
                flip.write()
            }
        }else if (mSlideState == slideState.STATE_RAISE) {
                PIDController(stack_count)
            if(stack_count <= 4){
                if (abs(TargetPos[stack_count] - getLiftHeight()) < 102.6 / 2) {
                    if(stack_count != 2 && stack_count != 3 && stack_count != 4){
                        flip.operate(0)
                    }
                }
                if (Math.abs(g2.right_stick_y) >= 0.15) {
                    newState(slideState.STATE_IDLE)
                }
            }else {
                if (abs(TargetPos[stack_count] - getLiftHeight()) < 102.6 * 17 / 2) {
                    flip.operate(0)
                }
                if (Math.abs(g2.right_stick_y) >= 0.15) {
                    newState(slideState.STATE_IDLE)
                }
            }
        }
        else if (mSlideState == slideState.STATE_DROP){
            fine_tune = 1.0
            dropSlides(-0.75)
            if (stack_count > 4) {
                flip.operate(2)
            }
            if(getLiftHeight() <= DROPDOWNPOS){
                setPower(0.0)
                newLeaveState(leaveState.STATE_IDLE)
                newState(slideState.STATE_IDLE)
            }
            if (increaseQueried) {
                increment_stack_count()
                increaseQueried = false
            }
        }
        else if (mSlideState == slideState.STATE_IDLE){
            /*
            if(g2.right_stick_y < 0){
                setPower(-0.35 * g2.right_stick_y)
            }else{
                if (getLiftHeight() >= DROPDOWNPOS) {
                    setPower(-0.05 * g2.right_stick_y)
                }
                else{
                    setPower(-0.1 * g2.right_stick_y);
                }
            }

             */
            if(g2.right_stick_y < 0){
                setPower(-0.35 * g2.right_stick_y) //UP
            }else{
                setPower(-0.1 * g2.right_stick_y) //DOWN
            }
        }else if (mSlideState == slideState.STATE_CAPSTONE){
            if(stack_count > 1) {
                PIDControllerPos(TargetPos[stack_count - 2] + (Math.abs(TargetPos[stack_count - 1] - TargetPos[stack_count - 2]) / 2).toDouble())
            }
            if (Math.abs(g2.right_stick_y) >= 0.15) {
                newState(slideState.STATE_IDLE)
            }
        }
        /*if (mSlideState != slideState.STATE_RAISE) {
            write()
        }*/
        telemetry.addData("Joystick Power: ", g2.right_stick_y)
        telemetry.addData("Level:", stack_count)
        telemetry.addData("Feeder: ", depositCheck)
    }
}