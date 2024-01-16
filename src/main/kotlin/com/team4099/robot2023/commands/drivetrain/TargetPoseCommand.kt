package com.team4099.robot2023.commands.drivetrain

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreePerSecond
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI
import kotlin.math.tan

class TargetPoseCommand(val drivetrain: Drivetrain, val targetPose: Pose2d) : Command() {
    private val thetaPID: ProfiledPIDController<Radian, Velocity<Radian>>

    val thetakP =
        LoggedTunableValue(
            "Pathfollow/thetakP",
            Pair({ it.inDegreesPerSecondPerDegree }, { it.degrees.perSecond.perDegree })
        )
    val thetakI =
        LoggedTunableValue(
            "Pathfollow/thetakI",
            Pair(
                { it.inDegreesPerSecondPerDegreeSeconds }, { it.degrees.perSecond.perDegreeSeconds }
            )
        )
    val thetakD =
        LoggedTunableValue(
            "Pathfollow/thetakD",
            Pair(
                { it.inDegreesPerSecondPerDegreePerSecond },
                { it.degrees.perSecond.perDegreePerSecond }
            )
        )

    val thetaMaxVel =
        LoggedTunableValue("Pathfollow/thetaMaxVel", DrivetrainConstants.PID.MAX_AUTO_ANGULAR_VEL)
    val thetaMaxAccel =
        LoggedTunableValue("Pathfollow/thetaMaxAccel", DrivetrainConstants.PID.MAX_AUTO_ANGULAR_ACCEL)
    var desiredAngle: Angle = 0.0.degrees

    init {
        addRequirements(drivetrain)

        Logger.recordOutput("Odometry/targetedPose", doubleArrayOf(targetPose.x.inMeters, targetPose.y.inMeters, targetPose.rotation.inRadians))

        if (RobotBase.isReal()) {
            thetakP.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KP)
            thetakI.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KI)
            thetakD.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KD)
        } else {
            thetakP.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP)
            thetakI.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI)
            thetakD.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)
        }

        thetaPID =
            ProfiledPIDController(
                thetakP.get(),
                thetakI.get(),
                thetakD.get(),
                TrapezoidProfile.Constraints(thetaMaxVel.get(), thetaMaxAccel.get())
            )
        thetaPID.enableContinuousInput(-PI.radians, PI.radians)
    }

    override fun initialize() {
        thetaPID.reset(drivetrain.odometryPose.rotation)
    }

    override fun execute() {
        Logger.recordOutput("ActiveCommands/TargetPoseCommand", true)

        val currentPose = drivetrain.odometryPose
        desiredAngle =
            tan((currentPose.y - targetPose.y).inMeters / (currentPose.x - targetPose.x).inMeters)
                .radians

        // 30.meters, 27.meters is goal, 3 meters / 4
        // 17.5 meters 13.5 meters is goal


        Logger.recordOutput("AutoLevel/desiredAngleTargetPoseCmd", desiredAngle.inDegrees)
        val thetaFeedback = thetaPID.calculate(currentPose.rotation, desiredAngle)

        drivetrain.currentRequest =
            Request.DrivetrainRequest.OpenLoop(
                thetaFeedback,
                Pair(drivetrain.fieldVelocity.x, drivetrain.fieldVelocity.y),
                fieldOriented = true
            )

        Logger.recordOutput("AutoLevel/CurrentYawDegrees", drivetrain.odometryPose.rotation.inDegrees)
        Logger.recordOutput("AutoLevel/DesiredYawDegrees", desiredAngle.inDegrees)
        Logger.recordOutput("AutoLevel/thetaFeedbackDPS", thetaFeedback.inDegreesPerSecond)
    }

    override fun isFinished(): Boolean {
        return (drivetrain.odometryPose.rotation - desiredAngle).absoluteValue <
                DrivetrainConstants.PID.AUTO_THETA_ALLOWED_ERROR
    }

    override fun end(interrupted: Boolean) {
        drivetrain.currentRequest =
            Request.DrivetrainRequest.OpenLoop(
                0.0.radians.perSecond, Pair(drivetrain.fieldVelocity.x, drivetrain.fieldVelocity.y)
            )
    }
}
