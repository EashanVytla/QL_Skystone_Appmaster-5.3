package org.firstinspires.ftc.teamcode.Odometry

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Odometry.SRX_Encoder
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.RevBulkData

class SRX_Odometry(/*Left_Wheel : SRX_Encoder,*/ Right_Wheel : SRX_Encoder, /*Strafe_Wheel : SRX_Encoder*/ telemetry: Telemetry, hardwaremap : HardwareMap){
    var t : Telemetry
    //var left : SRX_Encoder
    var right : SRX_Encoder
    var hub : ExpansionHubEx
    //var strafe : SRX_Encoder

    init {
        t = telemetry
        //left = Left_Wheel
        right = Right_Wheel
        //strafe = Strafe_Wheel
        hub = hardwaremap.get(ExpansionHubEx::class.java,"Expansion Hub 1")
    }

    fun update(data: RevBulkData){
        right.update(data)
    }

    fun OutputRaw(){
        var data = hub.bulkInputData
        update(data)
        t.addData("Right", right.getDist())
    }
}