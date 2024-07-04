package com.team4099.robot2023.subsystems.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.math.asPose2d
import com.team4099.lib.math.asTransform2d
import com.team4099.lib.vision.TimestampedTrigVisionUpdate
import com.team4099.lib.vision.TimestampedVisionUpdate
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.Drivetrain.TunableDriveStates.currentState
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.subsystems.drivetrain.swervemodule.SwerveModule
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.util.Alert
import com.team4099.robot2023.util.CustomLogger
import com.team4099.robot2023.util.FMSData
import com.team4099.robot2023.util.FieldFrameEstimator
import com.team4099.robot2023.util.Velocity2d
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import com.team4099.robot2023.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

class Drivetrain(private val gyroIO: GyroIO, val swerveModules: List<SwerveModule>) :
  SubsystemBase() {
  object TunableDriveStates {

    var currentState: DrivetrainState = DrivetrainState.UNINITIALIZED
  }
  private val gyroNotConnectedAlert =
    Alert(
      "Gyro is not connected, field relative driving will be significantly worse.",
      Alert.AlertType.ERROR
    )

  private val gyroInputs = GyroIO.GyroIOInputs()

  private var gyroYawOffset = 0.0.radians

  // TODO: tune these
  private var fieldFrameEstimator = FieldFrameEstimator(VecBuilder.fill(0.003, 0.003, 0.0001))

  private var angularVelocityTarget = 0.degrees.perSecond

  private var targetedDriveVector = Velocity2d.ZERO_VELOCITY_VECTOR

  private var isFieldOriented = true

  private var targetedChassisSpeeds =
    ChassisSpeeds(0.0.meters.perSecond, 0.0.meters.perSecond, 0.0.radians.perSecond)
      .chassisSpeedsWPILIB

  private var targetedChassisAccels =
    ChassisSpeeds(0.0.meters.perSecond, 0.0.meters.perSecond, 0.0.radians.perSecond)
      .chassisSpeedsWPILIB

  private var isInAutonomous = false

  var targetPose: Pose2d = Pose2d(0.0.meters, 0.0.meters, 0.0.radians)

  var fieldVelocity = Velocity2d.ZERO_VELOCITY_VECTOR
    private set

  var robotVelocity = Velocity2d.ZERO_VELOCITY_VECTOR
    private set

  private var omegaVelocity = 0.0.radians.perSecond

  private var characterizationInput = 0.0.volts

  private val testAngle =
    LoggedTunableValue("Drivetrain/testAngle", 0.degrees, Pair({ it.inDegrees }, { it.degrees }))

  private val swerveModuleID =
    LoggedTunableValue("Drivetrain/testModule", 0.degrees, Pair({ it.inDegrees }, { it.degrees }))

  var currentRequest: DrivetrainRequest = DrivetrainRequest.ZeroSensors()
    set(value) {
      when (value) {
        is DrivetrainRequest.OpenLoop -> {
          angularVelocityTarget = value.angularVelocity
          targetedDriveVector = value.driveVector
          isFieldOriented = value.fieldOriented
        }
        is DrivetrainRequest.ClosedLoop -> {
          targetedChassisSpeeds = value.chassisSpeeds
          targetedChassisAccels = value.chassisAccels
        }
        is DrivetrainRequest.ZeroSensors -> {
          isInAutonomous = value.isInAutonomous
        }
        is DrivetrainRequest.Characterize -> {
          characterizationInput = value.voltage
        }
        else -> {}
      }
      field = value
    }

  val moduleTranslations =
    listOf(
      DrivetrainConstants.FRONT_LEFT_LOCATION,
      DrivetrainConstants.FRONT_RIGHT_LOCATION,
      DrivetrainConstants.BACK_LEFT_LOCATION,
      DrivetrainConstants.BACK_RIGHT_LOCATION
    )

  private val swerveDriveKinematics =
    SwerveDriveKinematics(
      DrivetrainConstants.FRONT_LEFT_LOCATION.translation2d,
      DrivetrainConstants.FRONT_RIGHT_LOCATION.translation2d,
      DrivetrainConstants.BACK_LEFT_LOCATION.translation2d,
      DrivetrainConstants.BACK_RIGHT_LOCATION.translation2d
    )

  private var swerveDriveOdometry =
    SwerveDriveOdometry(
      swerveDriveKinematics,
      gyroInputs.gyroYaw.inRotation2ds,
      swerveModules.map { it.modulePosition }.toTypedArray()
    )

  private var setPointStates =
    mutableListOf(
      SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState()
    )

  val odomTRobot: Pose2d
    get() = Pose2d(swerveDriveOdometry.poseMeters)

  // NOTE(parth): This should be expected to be noisy. Avoid using this directly if possible.
  val fieldTRobot: Pose2d
    get() =
      (fieldFrameEstimator.getLatestOdometryTField().inverse() + odomTRobot.asTransform2d())
        .asPose2d()

  val odomTField: Transform2d
    get() = fieldFrameEstimator.getLatestOdometryTField()

  val odomTSpeaker: Transform2d
    get() = fieldFrameEstimator.getLatestOdometryTSpeaker()

  private var rawGyroAngle = odomTRobot.rotation

  private val lastModulePositions =
    arrayOf(
      SwerveModulePosition(),
      SwerveModulePosition(),
      SwerveModulePosition(),
      SwerveModulePosition()
    )

  init {
    // Wheel speeds
    zeroSteering()
  }

  override fun periodic() {
    val startTime = Clock.realTimestamp
    gyroIO.updateInputs(gyroInputs)

    // Read new gyro and wheel data from sensors

    swerveModules.forEach { it.updateInputs() }

    CustomLogger.processInputs("Gyro", gyroInputs)

    gyroNotConnectedAlert.set(!gyroInputs.gyroConnected) // send gyro alert if gyro disconnects

    swerveModules.forEach { it.periodic() }

    // Update field velocity
    // create a list to hold updated velos
    val measuredStates =
      mutableListOf(
        SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState()
      )
    // collect updated velos
    for (i in 0..3) {
      measuredStates[i] =
        SwerveModuleState(
          swerveModules[i].inputs.driveVelocity.inMetersPerSecond,
          swerveModules[i].inputs.steerPosition.inRotation2ds
        )
    }
    // convert them into wpi lib chassis speeds
    val chassisState: ChassisSpeeds =
      ChassisSpeeds(swerveDriveKinematics.toChassisSpeeds(*measuredStates.toTypedArray()))
    val fieldVelCalculated =
      Translation2d(
        chassisState.vx.inMetersPerSecond.meters, chassisState.vy.inMetersPerSecond.meters
      )
        .rotateBy(odomTRobot.rotation) // we don't use this but it's there if you want it ig

    robotVelocity = Velocity2d(chassisState.vx, chassisState.vy)
    fieldVelocity = Velocity2d(fieldVelCalculated.x.perSecond, fieldVelCalculated.y.perSecond)

    omegaVelocity = chassisState.omega
    if (!gyroInputs.gyroConnected) {
      gyroInputs.gyroYawRate = omegaVelocity
      rawGyroAngle += Constants.Universal.LOOP_PERIOD_TIME * gyroInputs.gyroYawRate
      gyroInputs.gyroYaw = rawGyroAngle + gyroYawOffset
    }

    // updating odometry every loop cycle
    updateOdometry()

    CustomLogger.recordOutput("FieldFrameEstimator/odomTSpeaker", odomTSpeaker.transform2d)

    CustomLogger.recordOutput(
      "Drivetrain/odomTRobotRotationInDegrees", odomTRobot.rotation.inDegrees
    )

    CustomLogger.recordOutput(
      "Drivetrain/xRobotVelocityMetersPerSecond", robotVelocity.x.inMetersPerSecond
    )
    CustomLogger.recordOutput(
      "Drivetrain/yRobotVelocityMetersPerSecond", robotVelocity.y.inMetersPerSecond
    )
    CustomLogger.recordOutput(
      "Drivetrain/xFieldVelocityMetersPerSecond", fieldVelocity.x.inMetersPerSecond
    )
    CustomLogger.recordOutput(
      "Drivetrain/yFieldVelocityMetersPerSecond", fieldVelocity.y.inMetersPerSecond
    )
    CustomLogger.recordOutput("Odometry/pose", odomTRobot.pose2d)
    CustomLogger.recordOutput("FieldFrameEstimator/robotPose", fieldTRobot.pose2d)

    CustomLogger.recordOutput("Drivetrain/ModuleStates", *measuredStates.toTypedArray())
    CustomLogger.recordOutput("Drivetrain/SetpointStates", *setPointStates.toTypedArray())

    CustomLogger.recordOutput("Odometry/pose", odomTRobot.pose2d)
    CustomLogger.recordOutput(
      "Odometry/pose3d",
      Pose3d(
        odomTRobot.x,
        odomTRobot.y,
        0.0.meters,
        Rotation3d(gyroInputs.gyroRoll, gyroInputs.gyroPitch, gyroInputs.gyroYaw)
      )
        .pose3d
    )

    CustomLogger.recordOutput("FieldFrameEstimator/odomTField", odomTField.transform2d)

    CustomLogger.recordOutput("Odometry/targetPose", targetPose.pose2d)

    // Log the current state
    CustomLogger.recordOutput("Drivetrain/currentState", currentState.toString())
    // Log the current state
    CustomLogger.recordOutput("Drivetrain/currentRequest", currentRequest.javaClass.toString())

    var nextState = currentState

    when (currentState) {
      DrivetrainState.UNINITIALIZED -> {
        // Transitions
        nextState = DrivetrainState.ZEROING_SENSORS
      }
      DrivetrainState.ZEROING_SENSORS -> {
        zeroSensors(isInAutonomous)
        // Transitions
        currentRequest = DrivetrainRequest.Idle()
        nextState = fromRequestToState(currentRequest)
      }
      DrivetrainState.IDLE -> {
        nextState = fromRequestToState(currentRequest)
      }
      DrivetrainState.OPEN_LOOP -> {
        // Outputs
        setOpenLoop(angularVelocityTarget, targetedDriveVector, isFieldOriented)
        CustomLogger.recordDebugOutput(
          "Drivetrain/TargetVelocityXInMPS", targetedDriveVector.x.inMetersPerSecond
        )
        CustomLogger.recordDebugOutput(
          "Drivetrain/TargetVelocityYInMPS", targetedDriveVector.y.inMetersPerSecond
        )
        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
      DrivetrainState.CLOSED_LOOP -> {
        // Outputs
        setClosedLoop(targetedChassisSpeeds, targetedChassisAccels)

        CustomLogger.recordOutput("Drivetrain/targetChassisSpeeds", targetedChassisSpeeds)
        CustomLogger.recordOutput("Drivetrain/targetChassisAccels", targetedChassisAccels)

        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
      DrivetrainState.CHARACTERIZE -> {
        swerveModules.forEach { it.runCharacterization(characterizationInput) }

        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
    }

    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/DrivetrainLoopTimeMS",
      (Clock.realTimestamp - startTime).inMilliseconds
    )

    currentState = nextState
  }

  private fun updateOdometry() {
    for (i in 0 until 4) {
      lastModulePositions[i] =
        SwerveModulePosition(
          swerveModules[i].inputs.drivePosition.inMeters,
          swerveModules[i].inputs.steerPosition.inRotation2ds
        )
    }

    swerveDriveOdometry.update(gyroInputs.gyroYaw.inRotation2ds, lastModulePositions)
    fieldFrameEstimator.addDriveData(Clock.fpgaTime, odomTRobot)
  }

  /**
   * @param angularVelocity Represents the angular velocity of the chassis
   * @param driveVector Pair of linear velocities: First is X vel, second is Y vel
   * @param fieldOriented Are the chassis speeds driving relative to field (aka use gyro or not)
   */
  fun setOpenLoop(
    angularVelocity: AngularVelocity,
    driveVector: Velocity2d,
    fieldOriented: Boolean = true
  ) {
    CustomLogger.recordDebugOutput("Drivetrain/isFieldOriented", fieldOriented)
    // flip the direction based on alliance color
    val driveVectorRespectiveToAlliance =
      if (FMSData.allianceColor == DriverStation.Alliance.Blue) driveVector
      else driveVector.unaryMinus()

    CustomLogger.recordDebugOutput(
      "Drivetrain/driveVectorRespectiveToAllianceXInMPS",
      driveVectorRespectiveToAlliance.x.inMetersPerSecond
    )
    CustomLogger.recordDebugOutput(
      "Drivetrain/driveVectorRespectiveToAllianceYInMPS",
      driveVectorRespectiveToAlliance.y.inMetersPerSecond
    )

    val swerveModuleStates: Array<SwerveModuleState>
    var desiredChassisSpeeds: ChassisSpeeds

    // calculated chassis speeds, apply field oriented transformation
    if (fieldOriented) {
      desiredChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
          driveVectorRespectiveToAlliance.x,
          driveVectorRespectiveToAlliance.y,
          angularVelocity,
          odomTRobot.rotation
        )
    } else {
      desiredChassisSpeeds =
        ChassisSpeeds(
          driveVectorRespectiveToAlliance.x,
          driveVectorRespectiveToAlliance.y,
          angularVelocity,
        )
    }

    if (DrivetrainConstants.MINIMIZE_SKEW) {
      desiredChassisSpeeds =
        ChassisSpeeds(
          edu.wpi.first.math.kinematics.ChassisSpeeds.discretize(
            desiredChassisSpeeds.vx.inMetersPerSecond,
            desiredChassisSpeeds.vy.inMetersPerSecond,
            desiredChassisSpeeds.omega.inRadiansPerSecond,
            Constants.Universal.LOOP_PERIOD_TIME.inSeconds
          )
        )
    }
    CustomLogger.recordDebugOutput(
      "Drivetrain/desiredChassisSpeedsVXInMPS", desiredChassisSpeeds.vx.inMetersPerSecond
    )
    CustomLogger.recordDebugOutput(
      "Drivetrain/desiredChassisSpeedsVYInMPS", desiredChassisSpeeds.vx.inMetersPerSecond
    )
    CustomLogger.recordDebugOutput(
      "Drivetrain/omegaDegreesPerSecond", desiredChassisSpeeds.omega.inDegreesPerSecond
    )

    // convert target chassis speeds to individual module setpoint states
    swerveModuleStates =
      swerveDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds.chassisSpeedsWPILIB)
    // makes sure the requested wheel speeds are actually possible if not it lowers them
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DrivetrainConstants.DRIVE_SETPOINT_MAX.inMetersPerSecond
    )
    setPointStates = swerveModuleStates.toMutableList()

    // set each module openloop based on corresponding states
    for (moduleIndex in 0 until DrivetrainConstants.WHEEL_COUNT) {
      swerveModules[moduleIndex].setOpenLoop(swerveModuleStates[moduleIndex])
    }
  }

  /**
   * Sets the drivetrain to the specified angular and X & Y velocities based on the current angular
   * and linear acceleration. Calculates both angular and linear velocities and acceleration and
   * calls setPositionClosedLoop for each SwerveModule object.
   *
   * @param angularVelocity The angular velocity of a specified drive
   * @param driveVector.first The linear velocity on the X axis
   * @param driveVector.second The linear velocity on the Y axis
   * @param angularAcceleration The angular acceleration of a specified drive
   * @param driveAcceleration.first The linear acceleration on the X axis
   * @param driveAcceleration.second The linear acceleration on the Y axis
   * @param fieldOriented Defines whether module states are calculated relative to field
   */
  fun setClosedLoop(
    chassisSpeeds: edu.wpi.first.math.kinematics.ChassisSpeeds,
    chassisAccels: edu.wpi.first.math.kinematics.ChassisSpeeds =
      edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0)
  ) {
    // Allows parameter chassisSpeeds to have its value reassigned
    var chassisSpeeds = chassisSpeeds

    // If theres skew consider slowing down
    if (DrivetrainConstants.MINIMIZE_SKEW) {
      chassisSpeeds =
        edu.wpi.first.math.kinematics.ChassisSpeeds.discretize(
          chassisSpeeds.vxMetersPerSecond,
          chassisSpeeds.vyMetersPerSecond,
          chassisSpeeds.omegaRadiansPerSecond,
          Constants.Universal.LOOP_PERIOD_TIME.inSeconds
        )
    }

    val velSwerveModuleStates: Array<SwerveModuleState> =
      swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds)
    val accelSwerveModuleStates: Array<SwerveModuleState> =
      swerveDriveKinematics.toSwerveModuleStates(chassisAccels)

    SwerveDriveKinematics.desaturateWheelSpeeds(
      velSwerveModuleStates, DrivetrainConstants.MAX_AUTO_VEL.inMetersPerSecond
    )

    setPointStates = velSwerveModuleStates.toMutableList()

    // Once we have all of our states obtained for both velocity and acceleration, apply these
    // states to each swerve module
    for (moduleIndex in 0 until DrivetrainConstants.WHEEL_COUNT) {
      swerveModules[moduleIndex].setClosedLoop(
        accelSwerveModuleStates[moduleIndex], velSwerveModuleStates[moduleIndex]
      )
    }
  }

  fun resetModuleZero() {
    swerveModules.forEach { it.resetModuleZero() }
  }

  /** Zeros all the sensors on the drivetrain. */
  fun zeroSensors(isInAutonomous: Boolean) {
    zeroGyroPitch()
    zeroGyroRoll()
    zeroSteering(isInAutonomous)

    if (!isInAutonomous) {
      zeroDrive()
    }
  }

  /** Resets the field frame estimator given some current pose of the robot. */
  fun resetFieldFrameEstimator(fieldTRobot: Pose2d) {
    fieldFrameEstimator.resetFieldFrameFilter(
      odomTRobot.asTransform2d() + fieldTRobot.asTransform2d().inverse()
    )
  }

  fun tempZeroGyroYaw(toAngle: Angle = 0.0.degrees) {
    swerveDriveOdometry.resetPosition(
      gyroInputs.gyroYaw.inRotation2ds,
      lastModulePositions,
      Pose2d(odomTRobot.x, odomTRobot.y, toAngle).pose2d
    )
  }

  /**
   * Sets the gyroOffset in such a way that when added to the gyro angle it gives back toAngle.
   *
   * @param toAngle Zeros the gyro to the value
   */
  fun zeroGyroYaw(toAngle: Angle = 0.degrees) {
    // TODO(parth): This feels incorrect -- I think the first arg should be the gyro angle and the
    if (RobotBase.isSimulation()) {
      // NOTE(parth): The gyro itself should never need to be reset in-match on a real robot, the
      // odometry can be updated directly
      gyroIO.zeroGyroYaw(toAngle)
    }

    // TODO(parth): Update the field frame estimator's transform here too, otherwise it will need to
    // re-converge
    swerveDriveOdometry.resetPosition(
      gyroInputs.gyroYaw.inRotation2ds,
      lastModulePositions,
      Pose2d(odomTRobot.x, odomTRobot.y, toAngle).pose2d
    )

    fieldFrameEstimator.resetFieldFrameFilter(
      Transform2d(odomTField.translation, gyroInputs.gyroYaw)
    )

    if (!(gyroInputs.gyroConnected)) {
      gyroYawOffset = toAngle - rawGyroAngle
    }
  }

  fun zeroGyroPitch(toAngle: Angle = 0.0.degrees) {
    gyroIO.zeroGyroPitch(toAngle)
  }

  fun zeroGyroRoll(toAngle: Angle = 0.0.degrees) {
    gyroIO.zeroGyroRoll(toAngle)
  }

  /** Zeros the steering motors for each swerve module. */
  fun zeroSteering(isInAutonomous: Boolean = false) {
    swerveModules.forEach { it.zeroSteer() }
  }

  /** Zeros the drive motors for each swerve module. */
  private fun zeroDrive() {
    swerveModules.forEach { it.zeroDrive() }
  }

  fun addVisionData(visionData: List<TimestampedVisionUpdate>) {
    fieldFrameEstimator.addVisionData(visionData)
  }

  fun addSpeakerVisionData(visionData: TimestampedTrigVisionUpdate) {
    fieldFrameEstimator.addSpeakerVisionData(visionData)
  }

  companion object {
    // Drivetrain multithreading
    var odometryLock: Lock = ReentrantLock()
    fun setOdometryLock(locked: Boolean) {
      if (locked) {
        odometryLock.lock()
      } else {
        odometryLock.unlock()
      }
    }

    // Drivetrain states for state machine.
    enum class DrivetrainState {
      UNINITIALIZED,
      IDLE,
      ZEROING_SENSORS,
      OPEN_LOOP,
      CLOSED_LOOP,
      CHARACTERIZE;

      inline fun equivalentToRequest(request: Request.DrivetrainRequest): Boolean {
        return (
          (request is DrivetrainRequest.ZeroSensors && this == ZEROING_SENSORS) ||
            (request is DrivetrainRequest.OpenLoop && this == OPEN_LOOP) ||
            (request is DrivetrainRequest.ClosedLoop && this == CLOSED_LOOP) ||
            (request is DrivetrainRequest.Idle && this == IDLE) ||
            (request is DrivetrainRequest.Characterize && this == CHARACTERIZE)
          )
      }
    }

    inline fun fromRequestToState(request: Request.DrivetrainRequest): DrivetrainState {
      return when (request) {
        is DrivetrainRequest.OpenLoop -> DrivetrainState.OPEN_LOOP
        is DrivetrainRequest.ClosedLoop -> DrivetrainState.CLOSED_LOOP
        is DrivetrainRequest.ZeroSensors -> DrivetrainState.ZEROING_SENSORS
        is DrivetrainRequest.Idle -> DrivetrainState.IDLE
        is DrivetrainRequest.Characterize -> DrivetrainState.CHARACTERIZE
      }
    }
  }
}
