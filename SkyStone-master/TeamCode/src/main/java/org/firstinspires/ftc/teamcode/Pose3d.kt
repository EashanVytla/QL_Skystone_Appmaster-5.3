package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d

class Pose3d(pos : Pose2d, z : Double) {
    var pos : Pose2d = pos
    var z : Double = z

    constructor(x : Double, y : Double, theta : Double, input_z : Double) : this(Pose2d(x, y, theta), input_z){
        pos = Pose2d(x, y, theta)
        z = input_z
    }
}