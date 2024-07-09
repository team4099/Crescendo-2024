package com.team4099.robot2023.util

import com.choreo.lib.ChoreoTrajectory
import com.team4099.lib.trajectory.CustomHolonomicDriveController
import com.team4099.lib.trajectory.CustomTrajectoryGenerator
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.Trajectory
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.kinematics.ChassisAccels
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.perSecond

class CustomTrajectory(
    val poseSupplier: () -> Pose2d,
    val trajectory: TrajectoryTypes,
    val trajectoryGenerator: CustomTrajectoryGenerator,
    val swerveDriveController: CustomHolonomicDriveController
) {
    fun sample(time: Time) : Request.DrivetrainRequest.ClosedLoop {
        return when (trajectory) {
            is Trajectory -> {
                val desiredState = trajectory.sample(time.inSeconds)
                val desiredRotation = trajectoryGenerator.holonomicRotationSequence.sample(time.inSeconds)
                val nextDriveState = swerveDriveController.calculate(
                    poseSupplier().pose2d,
                    desiredState,
                    desiredRotation
                )

                val chassisSpeeds = ChassisSpeeds(nextDriveState.vxMetersPerSecond, nextDriveState.vyMetersPerSecond, nextDriveState.omegaRadiansPerSecond)

                val xAccel =
                    desiredState.accelerationMetersPerSecondSq.meters.perSecond.perSecond *
                            desiredState.curvatureRadPerMeter.radians.cos
                val yAccel =
                    desiredState.accelerationMetersPerSecondSq.meters.perSecond.perSecond *
                            desiredState.curvatureRadPerMeter.radians.sin
                val chassisAccels = ChassisAccels(xAccel, yAccel, 0.0.radians.perSecond.perSecond).chassisAccelsWPILIB

                Request.DrivetrainRequest.ClosedLoop(chassisSpeeds, chassisAccels)
            }
            is ChoreoTrajectory -> {
                val trajectoryState = trajectory.sample(time.inSeconds)
                Request.DrivetrainRequest.ClosedLoop(trajectoryState.chassisSpeeds)
            }
            else -> {
                println("Unexpected trajectory type ${trajectory::javaClass}")
                Request.DrivetrainRequest.ClosedLoop(ChassisSpeeds(0.0, 0.0, 0.0))
            }
        }
    }
}