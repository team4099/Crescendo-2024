package com.team4099.robot2023.commands.drivetrain

import com.pathplanner.lib.path.PathPlannerPath
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.hal.Clock
import org.team4099.lib.pplib.PathPlannerHolonomicDriveController
import org.team4099.lib.pplib.PathPlannerRotationPID
import org.team4099.lib.pplib.PathPlannerTranslationPID
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreePerSecond
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.inMetersPerSecondPerMeter
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterSeconds
import org.team4099.lib.units.derived.inMetersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.metersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perMeter
import org.team4099.lib.units.derived.perMeterSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond

class FollowPathPlannerPathCommand(val drivetrain: Drivetrain, val path: PathPlannerPath) :
  Command() {
  private val translationToleranceAtEnd = 1.inches
  private val thetaToleranceAtEnd = 2.5.degrees

  private var swerveDriveController: PathPlannerHolonomicDriveController

  private var trajCurTime = 0.0.seconds
  private var trajStartTime = 0.0.seconds

  private var pathFollowRequest = Request.DrivetrainRequest.ClosedLoop(ChassisSpeeds(0.0, 0.0, 0.0))
  private var lastSampledPose = Pose2d()
  private val atReference: Boolean
    get() =
      (
        (lastSampledPose.translation - drivetrain.odometryPose.translation).magnitude.meters <=
          translationToleranceAtEnd &&
          (lastSampledPose.rotation - drivetrain.odometryPose.rotation).absoluteValue <=
          thetaToleranceAtEnd
        )

  val thetakP =
    LoggedTunableValue(
      "Pathfollow/thetakP",
      DrivetrainConstants.PID.AUTO_THETA_PID_KP,
      Pair({ it.inDegreesPerSecondPerDegree }, { it.degrees.perSecond.perDegree })
    )
  val thetakI =
    LoggedTunableValue(
      "Pathfollow/thetakI",
      DrivetrainConstants.PID.AUTO_THETA_PID_KI,
      Pair(
        { it.inDegreesPerSecondPerDegreeSeconds }, { it.degrees.perSecond.perDegreeSeconds }
      )
    )
  val thetakD =
    LoggedTunableValue(
      "Pathfollow/thetakD",
      DrivetrainConstants.PID.AUTO_THETA_PID_KD,
      Pair(
        { it.inDegreesPerSecondPerDegreePerSecond },
        { it.degrees.perSecond.perDegreePerSecond }
      )
    )

  val thetaMaxVel =
    LoggedTunableValue("Pathfollow/thetaMaxVel", DrivetrainConstants.PID.MAX_AUTO_ANGULAR_VEL)
  val thetaMaxAccel =
    LoggedTunableValue("Pathfollow/thetaMaxAccel", DrivetrainConstants.PID.MAX_AUTO_ANGULAR_ACCEL)

  val poskP =
    LoggedTunableValue(
      "Pathfollow/poskP",
      DrivetrainConstants.PID.AUTO_POS_KP,
      Pair({ it.inMetersPerSecondPerMeter }, { it.meters.perSecond.perMeter })
    )
  val poskI =
    LoggedTunableValue(
      "Pathfollow/poskI",
      DrivetrainConstants.PID.AUTO_POS_KI,
      Pair({ it.inMetersPerSecondPerMeterSeconds }, { it.meters.perSecond.perMeterSeconds })
    )
  val poskD =
    LoggedTunableValue(
      "Pathfollow/poskD",
      DrivetrainConstants.PID.AUTO_POS_KD,
      Pair(
        { it.inMetersPerSecondPerMetersPerSecond }, { it.metersPerSecondPerMetersPerSecond }
      )
    )

  var translationPID: PathPlannerTranslationPID
  var rotationPID: PathPlannerRotationPID
  //    private val pathConstraints = PathPlannerHolonomicDriveController.Companion.PathConstraints(
  //        maxVelocity = DrivetrainConstants.MAX_AUTO_VEL,
  //        maxAcceleration = DrivetrainConstants.MAX_AUTO_ACCEL,
  //        maxAngularVelocity = thetaMaxVel.get(),
  //        maxAngularAcceleration = thetaMaxAccel.get()
  //    )

  init {
    addRequirements(drivetrain)

    translationPID = PathPlannerTranslationPID(poskP.get(), poskI.get(), poskD.get())
    rotationPID = PathPlannerRotationPID(thetakP.get(), thetakI.get(), thetakD.get())

    swerveDriveController =
      PathPlannerHolonomicDriveController(
        translationPID,
        rotationPID,
        DrivetrainConstants.MAX_AUTO_VEL,
        Translation2d(
          DrivetrainConstants.DRIVETRAIN_LENGTH / 2,
          DrivetrainConstants.DRIVETRAIN_WIDTH / 2
        )
          .magnitude
          .meters,
      )
  }

  override fun initialize() {
    trajStartTime = Clock.fpgaTime
  }

  override fun execute() {
    if (thetakP.hasChanged() || thetakI.hasChanged() || thetakD.hasChanged()) {
      rotationPID = PathPlannerRotationPID(thetakP.get(), thetakI.get(), thetakD.get())
      swerveDriveController =
        PathPlannerHolonomicDriveController(
          translationPID,
          rotationPID,
          DrivetrainConstants.MAX_AUTO_VEL,
          Translation2d(
            DrivetrainConstants.DRIVETRAIN_LENGTH / 2,
            DrivetrainConstants.DRIVETRAIN_WIDTH / 2
          )
            .magnitude
            .meters,
        )
    }

    if (poskP.hasChanged() || poskI.hasChanged() || poskD.hasChanged()) {
      translationPID = PathPlannerTranslationPID(poskP.get(), poskI.get(), poskD.get())
      swerveDriveController =
        PathPlannerHolonomicDriveController(
          translationPID,
          rotationPID,
          DrivetrainConstants.MAX_AUTO_VEL,
          Translation2d(
            DrivetrainConstants.DRIVETRAIN_LENGTH / 2,
            DrivetrainConstants.DRIVETRAIN_WIDTH / 2
          )
            .magnitude
            .meters,
        )
    }

    val currentSpeeds = drivetrain.targetedChassisSpeeds
    val poseRotation = drivetrain.odometryPose.rotation.inRotation2ds
    val generatedTrajectory = path.getTrajectory(currentSpeeds, poseRotation)

    // Sampling the trajectory for a state that we're trying to target
    val stateFromTrajectory = generatedTrajectory.sample(trajCurTime.inSeconds)

    // Retrieves the last sampled pose, so we can keep our `atReference` variable updated
    Logger.recordOutput(
      "Odometry/targetPose",
      doubleArrayOf(
        lastSampledPose.x.inMeters,
        lastSampledPose.y.inMeters,
        lastSampledPose.rotation.inRadians
      )
    )
    Logger.recordOutput(
      "Pathfollow/thetaSetpoint", stateFromTrajectory.targetHolonomicPose.rotation.degrees
    )
    Logger.recordOutput("Pathfollow/currentTheta", drivetrain.odometryPose.rotation.inDegrees)

    lastSampledPose = Pose2d(stateFromTrajectory.targetHolonomicPose)

    val targetedChassisSpeeds =
      swerveDriveController.calculateRobotRelativeSpeeds(
        drivetrain.odometryPose, stateFromTrajectory
      )

    // Set closed loop request
    pathFollowRequest.chassisSpeeds = targetedChassisSpeeds.chassisSpeedsWPILIB
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
