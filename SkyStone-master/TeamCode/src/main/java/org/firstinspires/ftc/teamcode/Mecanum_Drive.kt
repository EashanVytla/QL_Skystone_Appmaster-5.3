package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.Odometry.Dead_Wheel
import org.firstinspires.ftc.teamcode.Odometry.SRX_Encoder
import org.firstinspires.ftc.teamcode.Odometry.SRX_Three_Wheel_Localizer
import org.firstinspires.ftc.teamcode.Odometry.ThreeWheelTrackingLocalizer
import org.firstinspires.ftc.teamcode.Universal.Math.Pose
import org.firstinspires.ftc.teamcode.Universal.Math.Vector2
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.RevBulkData
import java.util.*
import kotlin.collections.ArrayList
import kotlin.math.*

class Mecanum_Drive(hardwareMap : HardwareMap, telemetry: Telemetry){
    var motors = ArrayList<Caching_Motor>()
    var theta = 0.0

    var prev_pos = ArrayList<Double>()
    var prev_heading = 0.0

    val imu : LynxEmbeddedIMU
    val hub : ExpansionHubEx
    val hub2 : ExpansionHubEx

    var currentWriteIndex = 0

    val TRACK_WIDTH = 15.75
    val DRIVE_WIDTH = 13.625

    var pos = Pose()

    val EPSILON = 0.001

    var currHeading = 0.0
    var headingReadCount = 0
    var headingAccessCount = 0
    val headingUpdateFrequency = 0.1

    var rangle : Double = 0.0
    //var angle: Double = 0.toDouble()
    var prevheading = 0.0
    var slowmode2 = false

    var integralError = 0.0
    var prev_time = System.currentTimeMillis()
    var dt = 0.0

    var time = ElapsedTime()

    var scale = 1.0
    var slow_mode3 = false

    var orientation : Orientation
    var fine_tune = 1.0

    var previous = false
    var previous2 = false
    var previous3 = false
    var previous4 = false
    var previous5 = false
    var slow_mode = false

    //var pid: PID

    var mode = false

    var headingerror = 0.0
    //var headingerror: Double = 0.toDouble()

    var headingLock = false

    var automateLock = false
    var automateLock2 = false
    var flipper : Flipper

    //Rotational straight line coefficients
    val kpr = 0.9
    val kir = 0.0
    val kdr = 0.0

    //Rotational STRAFE coefficients
    val kprs = 0.6

    //Straight Line Coefficients
    val kpstr = 0.055 //.0625 //0.055 //0.02 //0.1
    val kistr = 0.0 //0.002 //0.0
    val kdstr = 5.6 //5.0 //4.0 //0.2

    var preverror = 0.0
    var preverrorstr = 0.0
    var error = 0.0
    var drifterror = 0.0
    var integralEstraight = 0.0
    var integralEdrift = 0.0
    var targetangle = 0.0
    var telemetry = telemetry

    var p = Pose2d(0.0, 0.0, 0.0)
    var prevpos = DoubleArray(4)
    var robotHeading = 0.0

    enum class aastate{
        STATE_IDOL,
        STATE_MOVING,
    }

    var AutoAllignSt = aastate.STATE_IDOL

    enum class State{
        STATE_STRAFE,
        STATE_IDLE
    }

    var state = State.STATE_IDLE

    var mStateTime = ElapsedTime()
    var angleOffset = 0.0

    companion object{
        var refresh_rate = 0.5  //ngl this is kinda scary but you do what u gotta do to get 300 hz

        const val kpA = 0.39//0.35625
        const val kiA = 0.008//0.005
        const val kdA = 0.2
        const val kp = 1.4
    }

    var data : RevBulkData
    var data2 : RevBulkData


    init{
        motors.add(Caching_Motor(hardwareMap, "up_left"))
        motors.add(Caching_Motor(hardwareMap, "back_left"))
        motors.add(Caching_Motor(hardwareMap, "back_right"))
        motors.add(Caching_Motor(hardwareMap, "up_right"))

        //pid = PID(hardwareMap, telemetry)



        hub = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 2")
        hub2 = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 1")

        data = hub.bulkInputData
        data2 = hub2.bulkInputData

        /*
        leftWheel.setBehavior(1.5385 * 2 * 0.797, -0.319237) //1.5144 0.0361262
        rightWheel.setBehavior(1.5385 * 2 * 0.797, -0.319237) //1.5204 -0.00305571
        strafeWheel.setBehavior(1.53642 * 2 * 0.797, 0.0) //1.50608 -0.221642

         */

        /*
        leftWheel.setBehavior(1.5385, -0.319237) //1.5144 0.0361262

        rightWheel.setBehavior(1.5385, -0.319237) //1.5204 -0.00305571

        strafeWheel.setBehavior(1.53642, 0.0) //1.50608 -0.221642

         */


        motors.forEach {
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        motors[0].motor.direction = DcMotorSimple.Direction.REVERSE
        motors[1].motor.direction = DcMotorSimple.Direction.REVERSE

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.standardModule, 0)
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)
        orientation = imu.angularOrientation
        time.startTime()

        flipper = Flipper(hardwareMap, telemetry)
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

    fun getOverallPower() : Double{
        return (motors[0].motor.power + motors[1].motor.power + motors[2].motor.power + motors[3].motor.power)/4
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

    fun scalePower(speed: Double) : Double{
        //return 0.5 * Math.pow(2 * (speed - 0.5), 3.0) + 0.5
        /*
        if(speed != 0.0){
            return Math.pow(speed, 3.0)/Math.abs(speed)
        }else {
            return 0.0
        }
         */
        return speed
    }

    /*
    fun autoAllign(state : AutoAllignSt, gamepad: Gamepad){
        if(state == AutoAllignSt.STATE_IDLE){
            setPower(fine_tune * scalePower(gamepad.left_stick_y.toDouble()), fine_tune * scalePower(gamepad.left_stick_x.toDouble()), -0.5 * gamepad.right_stick_x.toDouble())
        }else if(state == AutoAllignSt.STATE_STRAFE){
            setPower(0.0, gamepad.left_stick_y.toDouble(), 0.0)
        }else if(state == AutoAllignSt.STATE_TURN){
            if(time.time() >= 2.0){
                targetTurn((Math.PI/2 - angleOffset))
            }
        }
    }

     */

    fun isPress2(value : Boolean, previous : Boolean) : Boolean{
        return value && !previous
    }

    var stored_pos = Pose2d(0.0,0.0,0.0)

    fun drive(gamepad : Gamepad, gamepad2: Gamepad){
        slowmode2 = gamepad.right_trigger > 0.0

        if(isPress2(gamepad2.x, previous2)){
            slow_mode = !slow_mode
        }else if(isPress2(gamepad.a, previous5)){
            slow_mode3 = !slow_mode3
        }
        /*
        if (isPress2(gamepad2.dpad_left, previous4) && !Flipper.capped){
            automateLock = !automateLock
            if (automateLock){
                newState(State.STATE_STRAFE)
            }
            else{
                newState(State.STATE_IDLE)
            }
        }

         */

        /*
        if (isPress2(gamepad2.x, previous2)){
            automateLock2 = !automateLock2
            autoAllign(AutoAllignSt.STATE_TURN, gamepad)
            if (automateLock2){
                autoAllign(AutoAllignSt.STATE_STRAFE, gamepad)
            }
            else{
                autoAllign(AutoAllignSt.STATE_IDLE, gamepad)
            }
        }

         */

        previous2 = gamepad2.x
        previous3 = gamepad2.b
        previous4 = gamepad2.dpad_left
        previous5 = gamepad.a

        if (slow_mode || slowmode2){
            fine_tune = 0.5
        }else if(slow_mode3){
            fine_tune = 0.3
        } else{
            fine_tune = 0.9
        }

        if (!automateLock) {
            setPower(fine_tune * scalePower(gamepad.left_stick_y.toDouble()), fine_tune * scalePower(gamepad.left_stick_x.toDouble()), -0.5 * gamepad.right_stick_x.toDouble())
        }
        else{
            //capstoneStrafe()
            //targetTurn(Vector2d(fine_tune * gamepad.left_stick_y.toDouble(), fine_tune * gamepad.left_stick_x), Math.PI / 2)
        }
        write()
    }





    /*
    fun capstoneStrafe(){
        if (state == State.STATE_STRAFE){
            automateLock = true
            if (mStateTime.time() >= 1.5){
                automateLock = false
                newState(State.STATE_IDLE)
            }
            else if (localizer.getForwardDist() > 3.0){ //todo: change this to strafe val
                automateLock = false
                newState(State.STATE_IDLE)
            }
            else{
                setPower(0.0, 0.3, 0.0)
            }
        }
    }

     */

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

    fun centricSetPower(y : Double, x : Double, rot : Double, heading: Double){
        val r = hypot(y, x)
        val theta = atan2(/*-*/y, x).toDouble()
        val v = Vector2(r * cos(theta), r * sin(theta))
        v.rotate(heading)
        setPower(v, rot)
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

    fun getAngle() : Double{
        return angleWrap(getExternalHeading())
    }

    fun setPowerSide(left : Double, right : Double){
        motors[0].setPower(left)
        motors[1].setPower(left)
        motors[2].setPower(right)
        motors[3].setPower(right)
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
    }
    */

    fun targetTurn(targetAngle : Double){
        dt = (System.currentTimeMillis() - prev_time).toDouble()
        prev_time = System.currentTimeMillis()
        rangle = angleWrap(getExternalHeading())
        headingerror = targetAngle - rangle

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
        val deriv = ((headingerror - prevheading) * kdA / dt)

        val power = prop + integral + deriv
        if (Math.abs(power) < 0.3) {
            integralError += headingerror
        }
        prevheading = headingerror

        setPower(0.0, 0.0, Range.clip(power, -1.0, 1.0))
        //write()
    }


    fun targetTurnPlatform(targetAngle : Double, heading : Double){
        dt = (System.currentTimeMillis() - prev_time).toDouble()
        prev_time = System.currentTimeMillis()
        headingerror = targetAngle - heading

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

    fun targetTurnPlatform(targetAngle : Double){
        dt = (System.currentTimeMillis() - prev_time).toDouble()
        prev_time = System.currentTimeMillis()
        rangle = angleWrap(getExternalHeading())
        headingerror = targetAngle - rangle

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
        rangle = angleWrap(getExternalHeading())
        headingerror = targetAngle - rangle

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
}