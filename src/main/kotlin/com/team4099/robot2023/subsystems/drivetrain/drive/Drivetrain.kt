package com.team4099.robot2023.subsystems.drivetrain.drive

import com.team4099.lib.hal.Clock
import com.team4099.lib.math.asPose2d
import com.team4099.lib.math.asTransform2d
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.util.Alert
import com.team4099.robot2023.util.CustomLogger
import com.team4099.robot2023.util.FieldFrameEstimator
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import org.team4099.lib.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.inMetersPerSecond

class Drivetrain(val gyroIO: GyroIO, swerveModuleIOs: DrivetrainIO) : SubsystemBase() {
  // TODO: Add default values
  private val gyroNotConnectedAlert = Alert("Failed to connect gyro", Alert.AlertType.ERROR)

  private val swerveModules = swerveModuleIOs.getSwerveModules()

  private val gyroYawOffset = 0.0.radians

  private val gyroInputs = GyroIO.GyroIOInputs()

  private val fieldFrameEstimator = FieldFrameEstimator(VecBuilder.fill(0.003, 0.003, 0.0001))

  private val angularVelocityTarget = 0.0.radians.perSecond

  private val targetedDriveVector = Pair(0.0.meters.perSecond, 0.0.meters.perSecond)

  private val isFieldOriented = true

  var isInAuto = false

  private val frontLeftLocation =
    Translation2d(DrivetrainConstants.DRIVETRAIN_LENGTH / 2, DrivetrainConstants.DRIVETRAIN_WIDTH / 2)

  private val frontRightLocation =
    Translation2d(DrivetrainConstants.DRIVETRAIN_LENGTH / 2, -DrivetrainConstants.DRIVETRAIN_WIDTH / 2)

  private val backLeftLocation =
    Translation2d(-DrivetrainConstants.DRIVETRAIN_LENGTH / 2, DrivetrainConstants.DRIVETRAIN_WIDTH)

  private val backRightLocation =
    Translation2d(-DrivetrainConstants.DRIVETRAIN_LENGTH / 2, -DrivetrainConstants.DRIVETRAIN_WIDTH/2)

  var targetedChassisSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds(
    0.0, 0.0, 0.0
  )

  var targetedChassisAccels = edu.wpi.first.math.kinematics.ChassisSpeeds(
    0.0, 0.0, 0.0
  )

  private val swerveDriveKinematics = SwerveDriveKinematics(
    frontLeftLocation.translation2d,
    frontRightLocation.translation2d,
    backLeftLocation.translation2d,
    backRightLocation.translation2d
  )

  private var swerveDriveOdometry: SwerveDriveOdometry = SwerveDriveOdometry (
    swerveDriveKinematics,
    gyroInputs.gyroYaw.inRotation2ds,
    swerveModules.map{it.modulePosition}.toTypedArray()
  )

  private var undriftedSwerveDriveOdometry: SwerveDriveOdometry = SwerveDriveOdometry (
    swerveDriveKinematics,
    gyroInputs.gyroYaw.inRotation2ds,
    swerveModules.map{it.modulePosition}.toTypedArray()
  )
  private var setpointStates = mutableListOf(SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState())

  val odomTRobot: Pose2d get() = Pose2d(swerveDriveOdometry.poseMeters)

  val fieldTRobot: Pose2d get() =
    (fieldFrameEstimator.getLatestOdometryTField().inverse() + odomTRobot.asTransform2d())
      .asPose2d()

  val odomTField: Transform2d get() = fieldFrameEstimator.getLatestOdometryTField()

  val odomTSpeaker: Transform2d get() = fieldFrameEstimator.getLatestOdometryTSpeaker()

  private val lastModulePositions = arrayOf(
    SwerveModuleState(),
    SwerveModuleState(),
    SwerveModuleState(), SwerveModuleState())

  private var rawGryoAngle = odomTRobot.rotation
  init {
    zeroSteer()
  }

  fun zeroSteer(isAuto: Boolean = false) {
    swerveModules.forEach { it.zeroSteer(isAuto) }
  }



  override fun periodic() {
    CustomLogger.processInputs("Drivetrain", gyroInputs)

    gyroIO.updateInputs(gyroInputs)

    val timeProfiledGeneratedAt = Clock.realTimestamp

    swerveModules.forEach {it.updateInputs()}

    gyroNotConnectedAlert.set(!gyroInputs.gyroConnected)

    swerveModules.forEach { it.periodic() }

    val measuredStates = arrayOfNulls<SwerveModuleState>(4)
    for (i in 0..3) {
      measuredStates[i] =
        SwerveModuleState(
          swerveModules[i].inputs.driveVelocity.inMetersPerSecond,
          swerveModules[i].inputs.steerPosition.inRotation2ds
        )
    }
    val chassisState: ChassisSpeeds = ChassisSpeeds(swerveDriveKinematics.toChassisSpeeds(*measuredStates))
    val fieldVelCalculated =
      Translation2d(
        chassisState.vx.inMetersPerSecond.meters, chassisState.vy.inMetersPerSecond.meters
      )
  }
}