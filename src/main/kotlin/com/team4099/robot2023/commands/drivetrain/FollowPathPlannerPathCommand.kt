package com.team4099.robot2023.commands.drivetrain

import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.PathPlannerTrajectory
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.logging.toDoubleArray
import com.team4099.lib.math.asPose2d
import com.team4099.lib.math.asTransform2d
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.util.FrameType
import com.team4099.robot2023.util.inverse
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform2d
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

class FollowPathPlannerPathCommand(
  val drivetrain: Drivetrain,
  val path: PathPlannerPath,
  var stateFrame: FrameType = FrameType.ODOMETRY,
  var pathFrame: FrameType = FrameType.FIELD,
) : Command() {
  private val translationToleranceAtEnd = 1.inches
  private val thetaToleranceAtEnd = 2.5.degrees

  private var swerveDriveController: PathPlannerHolonomicDriveController

  private var trajCurTime = 0.0.seconds
  private var trajStartTime = 0.0.seconds

  private var pathFollowRequest = Request.DrivetrainRequest.ClosedLoop(ChassisSpeeds(0.0, 0.0, 0.0))
  private var lastSampledPose = Pose2d()

  private var drivePoseSupplier: () -> Pose2d
  private var odoTField: Transform2d = Transform2d(Translation2d(), 0.0.degrees)
  private var lastStableOdoTField: Transform2d = Transform2d(Translation2d(), 0.0.degrees)

  private val atReference: Boolean
    get() =
      (
        (lastSampledPose.translation - drivePoseSupplier().translation).magnitude.meters <=
          translationToleranceAtEnd &&
          (lastSampledPose.rotation - drivePoseSupplier().rotation).absoluteValue <=
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
  //  private val thetaPID: PIDController<Radian, Velocity<Radian>>
  //  private val posPID: PIDController<Meter, Velocity<Meter>>
  //    private val pathConstraints = PathPlannerHolonomicDriveController.Companion.PathConstraints(
  //        maxVelocity = DrivetrainConstants.MAX_AUTO_VEL,
  //        maxAcceleration = DrivetrainConstants.MAX_AUTO_ACCEL,
  //        maxAngularVelocity = thetaMaxVel.get(),
  //        maxAngularAcceleration = thetaMaxAccel.get()
  //    )

  init {
    addRequirements(drivetrain)

    when (stateFrame) {
      FrameType.ODOMETRY -> drivePoseSupplier = { drivetrain.odomTRobot }
      FrameType.FIELD -> {
        drivePoseSupplier = { drivetrain.fieldTRobot }
        // if we're already in field frame we do not want to shift by `odoTField` again
        pathFrame = FrameType.FIELD
      }
    }

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

  private lateinit var currentSpeeds: ChassisSpeeds
  private lateinit var poseRotation: Rotation2d
  private lateinit var generatedTrajectory: PathPlannerTrajectory

  private lateinit var pathTransform: Transform2d

  override fun initialize() {
    trajStartTime = Clock.fpgaTime
    odoTField = drivetrain.odomTField
    lastStableOdoTField = drivetrain.lastStableOdomTField

    currentSpeeds = drivetrain.targetedChassisSpeeds
    poseRotation = drivePoseSupplier().rotation.inRotation2ds
    generatedTrajectory = path.getTrajectory(currentSpeeds, poseRotation)
    pathTransform = Pose2d(path.previewStartingHolonomicPose).asTransform2d()
  }

  override fun execute() {
    if (thetakP.hasChanged() ||
      thetakI.hasChanged() ||
      thetakD.hasChanged() ||
      poskP.hasChanged() ||
      poskI.hasChanged() ||
      poskD.hasChanged()
    ) {
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

    val currentSpeeds = drivetrain.targetedChassisSpeeds
    val poseRotation = drivetrain.odomTRobot.rotation.inRotation2ds
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
    val robotPoseInSelectedFrame: Pose2d = drivePoseSupplier()
    if (pathFrame == stateFrame) {
      // odoTField x fieldTRobot
      lastSampledPose =
        (Pose2d(stateFromTrajectory.targetHolonomicPose) - pathTransform.asPose2d()).asPose2d()
    } else {
      when (pathFrame) {
        FrameType.ODOMETRY ->
          lastSampledPose =
            odoTField
              .asPose2d()
              .transformBy(Pose2d(stateFromTrajectory.targetHolonomicPose).asTransform2d())
        FrameType.FIELD ->
          lastSampledPose =
            odoTField
              .inverse()
              .asPose2d()
              .transformBy(Pose2d(stateFromTrajectory.targetHolonomicPose).asTransform2d())
      }
    }

    Logger.recordOutput("Pathfollow/thetaSetpointDegrees", lastSampledPose.rotation.inDegrees)
    Logger.recordOutput("Pathfollow/currentThetaDegrees", drivePoseSupplier().rotation.inDegrees)

    //    var targetedChassisSpeeds =
    //      swerveDriveController.calculateRobotRelativeSpeeds(
    //        (robotPoseInSelectedFrame + pathTransform), stateFromTrajectory
    //      )

    val pathFrameTRobotPose = (robotPoseInSelectedFrame + pathTransform)

    val targettedSpeeds =
      swerveDriveController.calculateRobotRelativeSpeeds(pathFrameTRobotPose, stateFromTrajectory)
    //    // Calculate feedforward velocities (field-relative).
    //    val xFF = (stateFromTrajectory.velocityMps *
    // stateFromTrajectory.heading.cos).meters.perSecond
    //    val yFF = (stateFromTrajectory.velocityMps *
    // stateFromTrajectory.heading.sin).meters.perSecond
    //
    //    // Calculate feedback velocities (based on position error).
    //    val xFeedback =
    //      posPID.calculate(pathFrameTRobotPose.x, stateFromTrajectory.positionMeters.x.meters)
    //    val yFeedback =
    //      posPID.calculate(pathFrameTRobotPose.y, stateFromTrajectory.positionMeters.y.meters)
    //    val thetaFeedback =
    //      thetaPID.calculate(
    //        pathFrameTRobotPose.rotation,
    //        stateFromTrajectory.targetHolonomicRotation.radians.radians
    //      )
    //
    //    // Return next output.
    //    val targetedChassisSpeeds =
    //      org.team4099.lib.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
    //        xFF + xFeedback, yFF + yFeedback, thetaFeedback, pathFrameTRobotPose.rotation
    //      )
    //
    //    Logger.recordOutput("Pathfollow/yFF+F", (yFF + yFeedback).inMetersPerSecond)
    //    Logger.recordOutput("Pathfollow/xFF+F", (xFF + xFeedback).inMetersPerSecond)
    //    Logger.recordOutput("Pathfollow/thetaFeedback", (thetaFeedback).inDegreesPerSecond)
    //    Logger.recordOutput(
    //      "Pathfollow/thetaFeedbackNorm", targetedChassisSpeeds.omega.inDegreesPerSecond
    //    )

    Logger.recordOutput(
      "Pathfollow/fieldTRobotVisualized",
      (robotPoseInSelectedFrame + pathTransform).toDoubleArray().toDoubleArray()
    )
    Logger.recordOutput(
      "Pathfollow/fieldTRobotTargetVisualized",
      Pose2d(stateFromTrajectory.targetHolonomicPose).toDoubleArray().toDoubleArray()
    )

    // Set closed loop request
    pathFollowRequest.chassisSpeeds = targettedSpeeds.chassisSpeedsWPILIB
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
    return atReference && trajCurTime.inSeconds > generatedTrajectory.totalTimeSeconds
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
