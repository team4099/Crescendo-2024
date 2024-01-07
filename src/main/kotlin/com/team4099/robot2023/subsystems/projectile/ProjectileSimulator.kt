package com.team4099.robot2023.subsystems.projectile

import com.team4099.robot2023.util.Velocity2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import kotlin.math.cos
import kotlin.math.sin
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.*
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.pow

class ProjectileSimulator(
    val length: Length = (1.feet + 2.inches),
    val width: Length = (1.feet + 2.inches),
    val height: Length = 2.inches,
    var startingPose: Pose3d,
    var launchVelocity: LinearVelocity,
    var robotVelocity: Velocity2d
) {

    var totalTime = 0.0.seconds
    var curPose = startingPose
    var xVel = launchVelocity * cos(startingPose.rotation.y) + robotVelocity.x
    var zVel = launchVelocity * sin(startingPose.rotation.y)
    var yVel = robotVelocity.y

    fun changeLaunchAngle(angle: Angle){
        startingPose = Pose3d(startingPose.translation, Rotation3d(startingPose.rotation.x, angle.inRadians, startingPose.rotation.z))
    }

    fun update(dt: Time){
        val dx = dt * xVel
        val dy = dt * yVel
        val dz = dt * zVel
        //val dz = (dt.inSeconds * (zVel.inMetersPerSecond + totalTime.inSeconds * -9.8)).meters
        val z = (zVel * totalTime).value + 0.5 * (-9.8) * totalTime.inSeconds.pow(2)

        // assume rotation doesn't really get affected by projectile motion
        Logger.recordOutput("Shooter/dz", dz.inMeters)
        Logger.recordOutput("Shooter/xVel", xVel.inMetersPerSecond)
        Logger.recordOutput("Shooter/zVel", zVel.inMetersPerSecond)
        curPose = curPose.transformBy(Transform3d(Translation3d(dx, dy, dz).translation3d, Rotation3d()))

        Logger.recordOutput("Shooter/projectile", doubleArrayOf(curPose.x,curPose.y,curPose.z,curPose.rotation.x,curPose.rotation.y,curPose.rotation.z))
        totalTime += dt

//        Logger.recordOutput("Shooter/projectileCorners", curPose.transformBy(Transform3d(Translation3d(width/2, length/2, height/2).translation3d, Rotation3d())),
//            curPose.transformBy(Transform3d(Translation3d(width/2, length/2, -height/2).translation3d, Rotation3d())),
//            curPose.transformBy(Transform3d(Translation3d(-width/2, length/2, height/2).translation3d, Rotation3d())),
//            curPose.transformBy(Transform3d(Translation3d(-width/2, length/2, -height/2).translation3d, Rotation3d())),
//            curPose.transformBy(Transform3d(Translation3d(width/2, -length/2, height/2).translation3d, Rotation3d())),
//            curPose.transformBy(Transform3d(Translation3d(width/2, -length/2, -height/2).translation3d, Rotation3d())),
//            curPose.transformBy(Transform3d(Translation3d(-width/2, -length/2, height/2).translation3d, Rotation3d())),
//            curPose.transformBy(Transform3d(Translation3d(-width/2, -length/2, -height/2).translation3d, Rotation3d())))
    }

}