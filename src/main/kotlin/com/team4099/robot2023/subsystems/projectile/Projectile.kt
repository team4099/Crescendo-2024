package com.team4099.robot2023.subsystems.projectile

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ShooterConstants
import com.team4099.robot2023.util.Velocity2d
import com.team4099.robot2023.util.toPose3d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond

class Projectile: SubsystemBase() {

    var poseSupplier = { Pose2d() }
    var fieldVelSupplier = { Velocity2d() }

    val launchAngle = LoggedTunableValue("Projectile/launchAngleDegrees", 30.0.degrees, Pair({it.inDegrees}, {it.degrees}))
    val launchVelocity = LoggedTunableValue("Projectile/launchVelocity", 5.0.meters.perSecond, Pair({it.inMetersPerSecond}, {it.meters.perSecond}))

    var angle = 0.0.degrees
    var vel = 0.0.meters.perSecond

    fun setPoseSupplier(dtPoseSupplier: () -> Pose2d, dtFieldVelSupplier: () -> Velocity2d){
        poseSupplier = dtPoseSupplier
        fieldVelSupplier = dtFieldVelSupplier
    }

    var projectile = ProjectileSimulator(startingPose = poseSupplier().toPose3d().transformBy(ShooterConstants.SHOOTER_TRANSFORMATION).pose3d, launchVelocity = 0.0.meters.perSecond, robotVelocity = fieldVelSupplier())
    var requestToShoot = false

    override fun periodic() {
        if (launchAngle.hasChanged() || launchVelocity.hasChanged()){
            angle = launchAngle.get()
            vel = launchVelocity.get()
        }

        if (requestToShoot){
            projectile.update(Constants.Universal.LOOP_PERIOD_TIME)
        } else {
            projectile = ProjectileSimulator(startingPose = poseSupplier().toPose3d().transformBy(ShooterConstants.SHOOTER_TRANSFORMATION).pose3d, launchVelocity = vel, robotVelocity = fieldVelSupplier())
            projectile.changeLaunchAngle(angle)
        }

        if (projectile.curPose.z <= 0.0){
            requestToShoot = false
        }
        Logger.recordOutput("Shooter/requestToShoot", requestToShoot)
    }

    fun shoot(): Command{
        return runOnce{
            requestToShoot = true
        }
    }
}