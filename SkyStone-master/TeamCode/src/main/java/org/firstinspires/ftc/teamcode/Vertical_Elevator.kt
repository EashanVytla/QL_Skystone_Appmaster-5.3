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
    var DynamicDropPos = 0

    var telemetry = t
    var isDropped = false
    var touch = map.get(DigitalChannel::class.java, "touch")
    val DROPDOWNPOS = 175

    var zero = 0.0

    var time = ElapsedTime()
    var clicks = 0

    var TargetPos = arrayOf(0, 593/2, 1071/2, 1484/2, 1940/2, 1251, 1430, 1661, 1890, 2105, 2340, 2567)

    var fine_tune = 1.0
    var error = 0.0

    var lastError = 0.0
    val DOWN_AUTOMATION_THRESHOLD = 5

    var previous = false;
    var previous2 = false;
    var lastTime = System.currentTimeMillis()

    var holdTime = ElapsedTime()
    var mStateTime = ElapsedTime()
    var mLeaveTime = ElapsedTime()
    var flip : FlipperV2
    var savedPos = 0.0

    val GFF_VAL = 0.115
    var gff = GFF_VAL//0.25//0.1926

    val speed = 1.0

    val kp = 0.005 * speed
    val kd = 0.001 * speed

    val k = 0.000153908
    var stack_check = 0
    var stack_count = 1

    companion object{
        var depositCheck = false
        var increaseQueried = false
        var reset = false
        var stack_countStatic = 1
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
        /*
        if (TargetLevel > 8) {
            ff_gain = FF * TargetPos[TargetLevel]
        }
         */
        lastTime = System.currentTimeMillis()
        var power = prop_gain + deriv_gain + ff_gain

        if (abs(power) > 0.001) {
            setPower(power)
        }
        else{
            setPower(0.0)
        }
        lastError = error
        //telemetry.addData("Control Effort", power)
    }

    fun PIDControllerPos(Position : Double){
        error = Position - getLiftHeight()

        var prop_gain = kp * error
        var deriv_gain = kd * ((error - lastError) / (System.currentTimeMillis() - lastTime))
        var ff_gain = 0.0
        if (Position > TargetPos[8]) {
            ff_gain = gff * Position
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
        //telemetry.addData("Control Effort", power)
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
        var new_power = power
        if (power >= 0.0){
            if (getLiftHeight() > TargetPos[5] + 50){
                new_power += k * (getLiftHeight() - TargetPos[5])
            }
        }
        if(mSlideState == slideState.STATE_IDLE || mSlideState == slideState.STATE_LEAVE_STACK){
            if(getLiftHeight() >= DROPDOWNPOS * 0.15){
                if(getLiftHeight() >= DROPDOWNPOS && getLiftHeight() < TargetPos[4]){
                    new_power += gff
                }else if(getLiftHeight() <= 80){
                    new_power = power
                }else{
                    if(power == 0.0){
                        new_power += gff
                    }else{
                        new_power += (gff + 0.1)
                    }
                }
            }
        }

        motors[0].setPower(Range.clip(-new_power, -1.0, 1.0))
        motors[1].setPower(Range.clip(new_power, -1.0, 1.0))

        write()
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
        if (stack_count <= 1){
            DynamicDropPos = 88
        } else if(stack_count == 2){
            DynamicDropPos = 94
        }else if(stack_count == 3){
            DynamicDropPos = 87
        }else{
            DynamicDropPos = 150
        }

        if(getLiftHeight() >= DynamicDropPos){
            setPower(power)
        }else {
            setPower(0.0)
            newLeaveState(leaveState.STATE_IDLE)
            newState(slideState.STATE_IDLE)
        }
    }

    fun newLeaveState(leaveState: leaveState){
        mLeaveState = leaveState
        mLeaveTime.reset()
    }

    fun isPress2(value : Boolean, previous : Boolean) : Boolean{
        return value && !previous
    }

    var b = false
    var previous3 = false

    var previous4 = false
    var check = false

    fun operate(g2 : Gamepad, g1 : Gamepad){
        stack_countStatic = stack_count
        motors.map {
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        if (getLiftHeight() < DROPDOWNPOS) {
            gff = 0.0
        }
        else{
            gff = GFF_VAL
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
        }else if(isPress2(g1.right_bumper, previous4)){
            b = false
            newState(slideState.STATE_LEAVE_STACK)
            increaseQueried = true
        }else if (isPress2(g2.b, previous3)){
            b = true
            if(stack_count < DOWN_AUTOMATION_THRESHOLD + 2){
                reset = !reset
            }
            newState(slideState.STATE_DROP)
        }else if(g2.dpad_up && dclickt.time() >= 0.3){
            increment_stack_count()
            newState(slideState.STATE_RAISE_INCREMENT)
            dclickt.reset()
        }else if(g2.dpad_down && dclickt.time() >= 0.3){
            decrement_stack_count()
            newState(slideState.STATE_RAISE_INCREMENT)
            dclickt.reset()
        }


        previous4 = g1.right_bumper

        if(mSlideState == slideState.STATE_RAISE_INCREMENT){
            newState(slideState.STATE_IDLE)
        }else if(mSlideState == slideState.STATE_LEAVE_STACK){
            if(stack_count <= 10 && depositCheck){
                fine_tune = 1.0
                if(mLeaveState == leaveState.STATE_IDLE){
                    savedPos = getLiftHeight()
                    newLeaveState(leaveState.STATE_DROP)
                }else if(mLeaveState == leaveState.STATE_DROP){
                    if(mLeaveTime.time() >= 0.5){
                        newLeaveState(leaveState.STATE_RAISE)
                    }else{
                        setPower(0.0)
                        flip.unclamp()
                        flip.write()
                    }
                }else if(mLeaveState == leaveState.STATE_RAISE){
                    if(abs((savedPos + 520 / 2) - getLiftHeight()) >= 15.0 && mLeaveTime.time() < 1.0){
                        PIDControllerPos(savedPos + 520)
                    }else{
                        //stack_count += 2
                        savedPos += 2
                        setPower(0.0)
                        newLeaveState(leaveState.STATE_RETURN)
                    }
                }else if(mLeaveState == leaveState.STATE_RETURN){
                    if(mLeaveTime.time() >= 1.0/*0.45*/){
                        newState(slideState.STATE_DROP)
                    }else{
                        setPower(0.0)
                        flip.operate(2)
                        flip.write()
                    }
                    if (increaseQueried && stack_count < 11) {
                        increment_stack_count()
                        increaseQueried = false
                    }
                }
            }else {
                stack_check = stack_count
                if (mStateTime.time() >= 1.0) {
                    newState(slideState.STATE_IDLE)
                } else {
                    if (first) {
                        first = false
                        Mecanum_Drive.capTime.reset()
                    }
                    flip.operate(1)
                }
                if (increaseQueried && stack_count < 11) {
                    increment_stack_count()
                    increaseQueried = false
                }
            }
        }else if (mSlideState == slideState.STATE_RAISE) {
            PIDController(stack_count)
            if(stack_count <= 4){
                flip.operate(0)
                /*
                if (abs(TargetPos[stack_count] - getLiftHeight()) < 102.6 / 2) {
                    flip.operate(0)
                }

                 */
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
            dropSlides(-0.5)

            if(b){
                if (stack_count > DOWN_AUTOMATION_THRESHOLD + 1 || !reset) { //EX: 4 is 5
                    flip.operate(2)
                }
            }else{
                flip.operate(2)
            }

            if(reset){
                if (increaseQueried && stack_count < 11) {
                    increment_stack_count()
                    increaseQueried = false
                }
            }
        }
        else if (mSlideState == slideState.STATE_IDLE){
            if(g2.right_stick_y < 0){
                setPower(-0.35 * g2.right_stick_y) //UP
            }else{
                setPower(-0.15 * g2.right_stick_y) //DOWN
            }
        }else if (mSlideState == slideState.STATE_CAPSTONE){
            if(stack_count > 1) {
                PIDControllerPos(TargetPos[stack_count - 2] + (Math.abs(TargetPos[stack_count - 1] - TargetPos[stack_count - 2]) / 2).toDouble())
            }
            if (Math.abs(g2.right_stick_y) >= 0.15) {
                newState(slideState.STATE_IDLE)
            }
        }
        previous3 = g2.b
        /*if (mSlideState != slideState.STATE_RAISE) {
            write()
        }*/
        //telemetry.addData("Joystick Power: ", g2.right_stick_y)
        //flip.write()
    }
}