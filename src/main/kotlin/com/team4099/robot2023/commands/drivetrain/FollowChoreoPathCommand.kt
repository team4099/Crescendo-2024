package com.team4099.robot2023.commands.drivetrain

import com.choreo.lib.ChoreoTrajectory
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.math.asPose2d
import com.team4099.lib.math.asTransform2d
import com.team4099.lib.trajectory.CustomHolonomicDriveController
import com.team4099.robot2023.auto.PathStore
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.hal.Clock
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.pplib.PathPlannerHolonomicDriveController
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreePerSecond
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.inMetersPerSecondPerMeter
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterSeconds
import org.team4099.lib.units.derived.inMetersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadian
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadianPerSecond
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadianSeconds
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.metersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perMeter
import org.team4099.lib.units.derived.perMeterSeconds
import org.team4099.lib.units.derived.perRadian
import org.team4099.lib.units.derived.perRadianPerSecond
import org.team4099.lib.units.derived.perRadianSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI
import kotlin.math.hypot

class FollowChoreoPathCommand(val drivetrain: Drivetrain, val path: ChoreoTrajectory) :
  Command() {
  private val translationToleranceAtEnd = 1.inches
  private val thetaToleranceAtEnd = 2.5.degrees

  private var trajCurTime = 0.0.seconds
  private var trajStartTime = 0.0.seconds

  private var pathFollowRequest = Request.DrivetrainRequest.ClosedLoop(
    ChassisSpeeds(0.0.meters.perSecond, 0.0.meters.perSecond, 0.0.radians.perSecond).chassisSpeedsWPILIB
  )
  private var lastSampledPose = Pose2d()
  private var changeStartTimeOnExecute = true

  private val atReference: Boolean
    get() =
      (
              (lastSampledPose.translation - drivetrain.odomTRobot.translation).magnitude.meters <=
                      translationToleranceAtEnd &&
                      (lastSampledPose.rotation - drivetrain.odomTRobot.rotation).absoluteValue <=
                      thetaToleranceAtEnd
              )

  val thetakP =
    LoggedTunableValue(
      "Pathfollow/thetakP",
      Pair({ it.inRadiansPerSecondPerRadian }, { it.radians.perSecond.perRadian })
    )
  val thetakI =
    LoggedTunableValue(
      "Pathfollow/thetakI",
      Pair(
        { it.inRadiansPerSecondPerRadianSeconds }, { it.radians.perSecond.perRadianSeconds }
      )
    )
  val thetakD =
    LoggedTunableValue(
      "Pathfollow/thetakD",
      Pair(
        { it.inRadiansPerSecondPerRadianPerSecond },
        { it.radians.perSecond.perRadianPerSecond }
      )
    )

  val thetaMaxVel =
    LoggedTunableValue("Pathfollow/thetaMaxVel", DrivetrainConstants.PID.MAX_AUTO_ANGULAR_VEL)
  val thetaMaxAccel =
    LoggedTunableValue("Pathfollow/thetaMaxAccel", DrivetrainConstants.PID.MAX_AUTO_ANGULAR_ACCEL)

  val poskP =
    LoggedTunableValue(
      "Pathfollow/posKP",
      DrivetrainConstants.PID.AUTO_POS_KP,
      Pair({ it.inMetersPerSecondPerMeter }, { it.meters.perSecond.perMeter })
    )
  val poskI =
    LoggedTunableValue(
      "Pathfollow/posKI",
      DrivetrainConstants.PID.AUTO_POS_KI,
      Pair({ it.inMetersPerSecondPerMeterSeconds }, { it.meters.perSecond.perMeterSeconds })
    )
  val poskD =
    LoggedTunableValue(
      "Pathfollow/posKD",
      DrivetrainConstants.PID.AUTO_POS_KD,
      Pair(
        { it.inMetersPerSecondPerMetersPerSecond }, { it.metersPerSecondPerMetersPerSecond }
      )
    )

  private val xPID: PIDController<Meter, Velocity<Meter>>
  private val yPID: PIDController<Meter, Velocity<Meter>>

  private val thetaPID: PIDController<Radian, Velocity<Radian>>

  init {
    addRequirements(drivetrain)

    if (RobotBase.isSimulation()) {
      thetakP.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP)
      thetakI.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI)
      thetakD.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)
    }

    xPID = PIDController(poskP.get(), poskI.get(), poskD.get())
    yPID = PIDController(poskP.get(), poskI.get(), poskD.get())
    thetaPID = PIDController(thetakP.get(), thetakI.get(), thetakD.get())

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    trajStartTime = Clock.fpgaTime + path.samples[0].timestamp.seconds

    thetaPID.reset()
    xPID.reset()
    yPID.reset()
  }

  override fun execute() {
    if (changeStartTimeOnExecute) {
      trajStartTime = Clock.fpgaTime + path.samples[0].timestamp.seconds
      changeStartTimeOnExecute = false
    }

    // Sampling the trajectory for a state that we're trying to target
    trajCurTime = Clock.fpgaTime - trajStartTime
    val stateFromTrajectory = path.sample(trajCurTime.inSeconds)

    lastSampledPose = Pose2d(stateFromTrajectory.pose)
    Logger.recordOutput("Pathfollow/pose", drivetrain.fieldTRobot.pose2d)
    Logger.recordOutput("Pathfollow/targetPose", lastSampledPose.pose2d)

    val xVelocity = stateFromTrajectory.velocityX.meters.perSecond
    val xFF = xPID.calculate(drivetrain.fieldTRobot.x, stateFromTrajectory.x.meters)

    val yVelocity = stateFromTrajectory.velocityY.meters.perSecond
    val yFF = yPID.calculate(drivetrain.fieldTRobot.y, stateFromTrajectory.y.meters)

    val thetaVelocity = stateFromTrajectory.angularVelocity.radians.perSecond
    val thetaFF = thetaPID.calculate(drivetrain.fieldTRobot.rotation, stateFromTrajectory.pose.rotation.degrees.degrees)

    // Set closed loop request
    pathFollowRequest.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xVelocity + xFF,
      yVelocity + yFF,
      thetaVelocity + thetaFF,
      drivetrain.fieldTRobot.rotation
    ).chassisSpeedsWPILIB
    drivetrain.currentRequest = pathFollowRequest

    // Update trajectory time
    trajCurTime = Clock.fpgaTime - trajStartTime
  }

  //    override fun isFinished(): Boolean {
  //        trajCurTime = Clock.fpgaTime - trajStartTime
  //        return endPathOnceAtReference &&
  //                (!keepTrapping || swerveDriveController.atReference()) &&
  //                trajCurTime > trajectoryGenerator.driveTrajectory.totalTimeSeconds.seconds
  //    }
  override fun isFinished(): Boolean {
    return atReference
  }

  override fun end(interrupted: Boolean) {
    if (interrupted) {
      // Stop where we are if interrupted
      drivetrain.currentRequest =
        Request.DrivetrainRequest.OpenLoop(
          0.0.radians.perSecond, Pair(0.0.meters.perSecond, 0.0.meters.perSecond)
        )
    } else {
      // Execute one last time to end up in the final state of the trajectory
      // Since we weren't interrupted, we know curTime > endTime
      execute()
      drivetrain.currentRequest =
        Request.DrivetrainRequest.OpenLoop(
          0.0.radians.perSecond, Pair(0.0.meters.perSecond, 0.0.meters.perSecond)
        )
    }
  }
}