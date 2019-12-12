package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.Universal.Math.Pose
import org.firstinspires.ftc.teamcode.Universal.Math.Vector2
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.RevBulkData
import java.util.*
import kotlin.collections.ArrayList
import kotlin.math.*

class Mecanum_Drive(hardwareMap : HardwareMap){
    var motors = ArrayList<Caching_Motor>()

    var strafeWheel = Dead_Wheel(MA3_Encoder( "a1", hardwareMap, 2.464))

    var prev_pos = ArrayList<Double>()
    var prev_heading = 0.0

    val imu : LynxEmbeddedIMU
    val hub : ExpansionHubEx

    var currentWriteIndex = 0

    val TRACK_WIDTH = 15.75
    val DRIVE_WIDTH = 13.625

    var pos = Pose()

    val EPSILON = 0.001

    var currHeading = 0.0
    var headingReadCount = 0
    var headingAccessCount = 0
    val headingUpdateFrequency = 0.1

    var angle: Double = 0.toDouble()
    var prevheading = 0.0
    var slowmode2 = false

    var integralError = 0.0
    var prev_time = System.currentTimeMillis()
    var dt = 0.0

    var time = ElapsedTime()

    var scale = 1.0

    var orientation : Orientation
    var fine_tune = 1.0

    var previous = false
    var previous2 = false
    var previous3 = false
    var slow_mode = false

    var mode = false

    var headingerror: Double = 0.toDouble()

    var headingLock = false

    var automateLock = false

    enum class State{
        STATE_STRAFE,
        STATE_IDLE
    }

    var state = State.STATE_IDLE

    var mStateTime = ElapsedTime()

    var previous4 = false

    companion object{
        var refresh_rate = 0.5  //ngl this is kinda scary but you do what u gotta do to get 300 hz

        const val kpA = 0.39//0.35625
        const val kiA = 0.008//0.005
        const val kdA = 0.2
        const val kp = 1.0
    }

    init{
        motors.add(Caching_Motor(hardwareMap, "up_left"))
        motors.add(Caching_Motor(hardwareMap, "back_left"))
        motors.add(Caching_Motor(hardwareMap, "back_right"))
        motors.add(Caching_Motor(hardwareMap, "up_right"))

        strafeWheel.encoder.reverse()
        strafeWheel.setBehavior(1.53642 * 2.0 * 0.797, 0.0) //1.50608 -0.221642

        motors.forEach {
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        motors[0].motor.direction = DcMotorSimple.Direction.REVERSE
        motors[1].motor.direction = DcMotorSimple.Direction.REVERSE

        hub = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 2")
        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.standardModule, 0)
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)
        orientation = imu.angularOrientation
        time.startTime()

        val data = hub.bulkInputData
        strafeWheel.encoder.calibrate(data)
    }

    fun newState(s : State){
        mStateTime.reset()
        state = s
    }

    private fun tweakRefreshRate(gamepad : Gamepad){
        if (gamepad.dpad_up){
            raiseRefreshRate()
        }
        else if (gamepad.dpad_down){
            dropRefreshRate()
        }
    }

    private fun raiseRefreshRate(){
        var num = (1 / refresh_rate).toInt()
        num++
        refresh_rate = 1.div(num.toDouble())
    }

    private fun dropRefreshRate(){
        var num = (1 / refresh_rate).toInt()
        if (num > 1) {
            num--
            refresh_rate = 1.div(num.toDouble())
        }
    }

    fun getRefreshRate() : Double{
        return refresh_rate
    }

    fun getPower() : Array<Double>{
        return arrayOf(motors[0].prev_power, motors[1].prev_power, motors[2].prev_power, motors[3].prev_power)
    }

    fun pivotTo(heading : Double, fPower : Double) : Boolean{
        val error = heading - getExternalHeading()
        val power = kpA * error * 0.5
        if (abs(error) > Math.toRadians(10.0)) {
            setPower(fPower, 0.0, power)
            return true
        }
        else{
            setPower(0.0, 0.0, 0.0)
            return false
        }
    }

    fun targetTurn(targetAngle : Double){
        dt = (System.currentTimeMillis() - prev_time).toDouble()
        prev_time = System.currentTimeMillis()
        angle = angleWrap(getExternalHeading())
        headingerror = targetAngle - angle

        if (abs(headingerror) > Math.toRadians(180.0)){
            if (headingerror > 0) {
                headingerror = -((Math.PI * 2) - abs(headingerror))
            }
            else{
                headingerror = ((Math.PI * 2) - abs(headingerror))
            }
        }

        val prop = headingerror * kpA
        val integral = integralError * kiA
        val deriv = (headingerror - prevheading) * kdA / dt

        val power = prop + integral + deriv
        if (Math.abs(power) < 0.3) {
            integralError += headingerror
        }
        prevheading = headingerror

        setPower(0.0, 0.0, Range.clip(power, -1.0, 1.0))
        //write()
    }

    fun targetTurnPlatform(targetAngle : Double){
        dt = (System.currentTimeMillis() - prev_time).toDouble()
        prev_time = System.currentTimeMillis()
        angle = angleWrap(getExternalHeading())
        headingerror = targetAngle - angle

        if (abs(headingerror) > Math.toRadians(180.0)){
            if (headingerror > 0) {
                headingerror = -((Math.PI * 2) - abs(headingerror))
            }
            else{
                headingerror = ((Math.PI * 2) - abs(headingerror))
            }
        }

        val prop = headingerror * kp
        val power = prop
        if (Math.abs(power) < 0.3) {
            integralError += headingerror
        }
        prevheading = headingerror

        setPower(0.0, 0.0, Range.clip(power, -1.0, 1.0))
    }

    fun targetTurn(drive : Vector2d, targetAngle : Double){
        dt = (System.currentTimeMillis() - prev_time).toDouble()
        prev_time = System.currentTimeMillis()
        angle = angleWrap(getExternalHeading())
        headingerror = targetAngle - angle

        val prop = headingerror * kpA
        val integral = integralError * kiA
        val deriv = (headingerror - prevheading) * kdA / dt

        val power = prop + integral + deriv
        if (Math.abs(power) < 0.3) {
            integralError += headingerror
        }
        prevheading = headingerror

        setPower(drive.y, drive.x, Range.clip(power, -1.0, 1.0))
    }

    fun isPress2(value : Boolean, previous : Boolean) : Boolean{
        return value && !previous
    }

    fun drive(gamepad : Gamepad, gamepad2: Gamepad){
        slowmode2 = gamepad.right_trigger > 0.0

        if(isPress2(gamepad2.x, previous2)){
            slow_mode = true
        }else if(isPress2(gamepad2.b, previous3)){
            slow_mode = false
        }

        /*if (isPress2(gamepad2.dpad_left, previous4) && !Flipper.capped){
            automateLock = !automateLock
            if (automateLock){
                newState(State.STATE_STRAFE)
            }
            else{
                newState(State.STATE_IDLE)
            }
        }*/

        previous2 = gamepad2.x
        previous3 = gamepad2.b
        previous4 = gamepad2.dpad_left

        if (slow_mode || slowmode2){
            fine_tune = 0.5
        }
        else{
            fine_tune = 0.9
        }

        if (!automateLock) {
            setPower(fine_tune * gamepad.left_stick_y.toDouble(), fine_tune * gamepad.left_stick_x.toDouble(), -0.5 * gamepad.right_stick_x.toDouble())
        }
        else{
            capstoneStrafe()
            //targetTurn(Vector2d(fine_tune * gamepad.left_stick_y.toDouble(), fine_tune * gamepad.left_stick_x), Math.PI / 2)
        }
        write()
    }

    fun capstoneStrafe(){
        if (state == State.STATE_STRAFE){
            automateLock = true
            if (mStateTime.time() >= 1.5){
                automateLock = false
                newState(State.STATE_IDLE)
            }
            else if (getStrafeDist() > 3.0){
                automateLock = false
                newState(State.STATE_IDLE)
            }
            else{
                setPower(0.0, 0.3, 0.0)
            }
        }
    }

    private fun getStrafeDist(): Double {
        return strafeWheel.getDistance() * 7.0 / 17.536
    }


    fun toggleHeadingLock(){
        headingLock = !headingLock
        integralError = 0.0
    }

    fun isPress(test : Boolean) : Boolean{
        return test && !previous
    }

    fun angleWrap(angle : Double) : Double{
        return (angle + (2 * Math.PI)) % (2 * Math.PI)
    }

    fun f_drive(gamepad1 : Gamepad){
        //tweakRefreshRate(gamepad1)
        if (isPress(gamepad1.right_bumper)){
            slow_mode = !slow_mode
        }
        previous = gamepad1.right_bumper

        if (slow_mode){
            fine_tune = 0.5
        }
        else{
            fine_tune = 1.0
        }

        val r = hypot(gamepad1.left_stick_y, gamepad1.left_stick_x)
        val theta = atan2(/*-*/gamepad1.left_stick_y, gamepad1.left_stick_x).toDouble()
        val v = Vector2(r * cos(theta), r * sin(theta))
        v.rotate(angleWrap(getExternalHeading()))
        setPower(v, -0.5 * gamepad1.right_stick_x)
        write()
    }

    fun read(data : RevBulkData) {
        motors.forEach {
            it.read(data)
        }
        headingReadCount++
        if (headingAccessCount.toDouble() / headingReadCount.toDouble() < headingUpdateFrequency) {
            headingAccessCount++
            currHeading = imu.angularOrientation.firstAngle.toDouble()
            orientation = imu.angularOrientation
        }
        if (!automateLock){
            strafeWheel.encoder.calibrate(data)
        }
        else{
            strafeWheel.update(data)
        }
        headingReadCount++
    }

    fun setPower(y : Double, x : Double, rightX : Double){

        var FrontLeftVal = y - x + rightX
        var FrontRightVal = y + x - rightX
        var BackLeftVal = y + x + rightX
        var BackRightVal = y - x - rightX

        //Move range to between 0 and +1, if not already
        val wheelPowers = doubleArrayOf(FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal)
        Arrays.sort(wheelPowers)
        if (wheelPowers[3] > 1) {
            FrontLeftVal /= wheelPowers[3]
            FrontRightVal /= wheelPowers[3]
            BackLeftVal /= wheelPowers[3]
            BackRightVal /= wheelPowers[3]
        }
        motors[0].setPower(FrontLeftVal)
        motors[1].setPower(BackLeftVal)
        motors[2].setPower(BackRightVal)
        motors[3].setPower(FrontRightVal)
    }

    fun setPower(v : Vector2, rightX : Double){
        setPower(v.y, v.x, rightX)
    }

    fun write(){
        motors[currentWriteIndex].write()
        currentWriteIndex = (currentWriteIndex + 1) % 4
    }

    fun getExternalHeading() : Double{
        return orientation.firstAngle.toDouble()
    }

    fun getEstimatedPose(){
        val wheelVelocities = ArrayList<Double>()
        motors.forEachIndexed{index, motor ->
            wheelVelocities.add(motor.getCurrentPosition() - prev_pos[index])
            prev_pos[index] = motor.getCurrentPosition().toDouble()
        }
        val k = (TRACK_WIDTH + DRIVE_WIDTH) / 2.0
        val (frontLeft, rearLeft, rearRight, frontRight) = wheelVelocities

        val heading = getExternalHeading()
        val offset = Pose(wheelVelocities.sum(), rearLeft + frontRight - frontLeft - rearRight, heading - prev_heading)
        prev_heading = heading
        //relativeOdometryUpdate(offset)
    }

    /*fun relativeOdometryUpdate(robotPoseDelta : Pose){
        val dtheta = robotPoseDelta.angle

        val (sineTerm, cosTerm) = if (abs(dtheta) < EPSILON) {
            1.0 - dtheta * dtheta / 6.0 to dtheta / 2.0
        } else {
            sin(dtheta) / dtheta to (1 - cos(dtheta)) / dtheta
        }

        val fieldPositionDelta = Vector2(sineTerm * robotPoseDelta.x - cosTerm * robotPoseDelta.y,
                cosTerm * robotPoseDelta.x + sineTerm * robotPoseDelta.y)
        fieldPositionDelta.rotate(pos.angle)

        val fieldPoseDelta = Pose(fieldPositionDelta.x, fieldPositionDelta.y, dtheta)

        pos.add(fieldPoseDelta)
    }*/
}