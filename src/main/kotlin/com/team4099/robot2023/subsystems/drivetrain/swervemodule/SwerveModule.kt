package com.team4099.robot2023.subsystems.drivetrain.swervemodule

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.util.CustomLogger
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase.isReal
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerMeters
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecond
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecondPerSecond
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perMeterPerSecond
import org.team4099.lib.units.derived.perMeterPerSecondPerSecond
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.IEEErem
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.withSign

class SwerveModule(private val io: SwerveModuleIO) {

  val inputs = SwerveModuleIO.SwerveModuleIOInputs()

  var modulePosition = SwerveModulePosition()
    private set

  private var posDeltas = mutableListOf<SwerveModulePosition>()

  private var velocitySetpoint: LinearVelocity = 0.0.meters.perSecond

  private var accelerationSetpoint: LinearAcceleration = 0.0.meters.perSecond.perSecond

  private var steeringSetpoint: Angle = 0.0.degrees

  private var lastDrivePos = 0.0.meters

  private val steerkP =
    LoggedTunableValue(
      "Drivetrain/modulesteerkP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree })
    )
  private val steerkI =
    LoggedTunableValue(
      "Drivetrain/modulesteerkI",
      Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  private val steerkD =
    LoggedTunableValue(
      "Drivetrain/modulesteerkD",
      Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  private val drivekP =
    LoggedTunableValue(
      "Drivetrain/drivekP",
      Pair({ it.inVoltsPerMetersPerSecond }, { it.volts.perMeterPerSecond })
    )
  private val drivekI =
    LoggedTunableValue(
      "Drivetrain/drivekI",
      Pair({ it.inVoltsPerMeters }, { it.volts / (1.meters.perSecond * 1.seconds) })
    )
  private val drivekD =
    LoggedTunableValue(
      "Drivetrain/drivekD",
      Pair({ it.inVoltsPerMetersPerSecondPerSecond }, { it.volts.perMeterPerSecondPerSecond })
    )

  private val drivekV =
    LoggedTunableValue(
      "Drivetrain/drivekV",
      Pair({ it.inVoltsPerMetersPerSecond }, { it.volts.perMeterPerSecond })
    )

  private val drivekA =
    LoggedTunableValue(
      "Drivetrain/drivekA",
      Pair({ it.inVoltsPerMetersPerSecondPerSecond }, { it.volts.perMeterPerSecondPerSecond })
    )

  private val steerMaxAccel =
    LoggedTunableValue("Drivetrain/steerMaxAcceleration", DrivetrainConstants.STEERING_ACCEL_MAX)
  private val steerMaxVelo =
    LoggedTunableValue("Drivetrain/steerMaxVelocity", DrivetrainConstants.STEERING_VEL_MAX)

  init {
    if (isReal()) {
      steerkP.initDefault(DrivetrainConstants.PID.STEERING_KP)
      steerkI.initDefault(DrivetrainConstants.PID.STEERING_KI)
      steerkD.initDefault(DrivetrainConstants.PID.STEERING_KD)

      drivekP.initDefault(DrivetrainConstants.PID.DRIVE_KP)
      drivekI.initDefault(DrivetrainConstants.PID.DRIVE_KI)
      drivekD.initDefault(DrivetrainConstants.PID.DRIVE_KD)
    } else {
      steerkP.initDefault(DrivetrainConstants.PID.SIM_STEERING_KP)
      steerkI.initDefault(DrivetrainConstants.PID.SIM_STEERING_KI)
      steerkD.initDefault(DrivetrainConstants.PID.SIM_STEERING_KD)

      drivekP.initDefault(DrivetrainConstants.PID.SIM_DRIVE_KP)
      drivekI.initDefault(DrivetrainConstants.PID.SIM_DRIVE_KI)
      drivekD.initDefault(DrivetrainConstants.PID.SIM_DRIVE_KD)
    }
    drivekV.initDefault(DrivetrainConstants.PID.DRIVE_KV)
    drivekA.initDefault(DrivetrainConstants.PID.DRIVE_KA)
  }

  fun updateInputs() {
    io.updateInputs(inputs)
  }
  fun periodic() {
    posDeltas.add(
      SwerveModulePosition(
        (inputs.drivePosition - lastDrivePos).inMeters, inputs.steerPosition.inRotation2ds
      )
    )

    CustomLogger.processInputs(io.label, inputs)
    CustomLogger.recordDebugOutput("${io.label}/lastDrivePos", lastDrivePos.inMeters)

    CustomLogger.recordDebugOutput(
      "${io.label}/velocitySetpoint", velocitySetpoint.inMetersPerSecond
    )
    CustomLogger.recordDebugOutput(
      "${io.label}/accelerationSetpoint", accelerationSetpoint.inMetersPerSecondPerSecond
    )
    CustomLogger.recordDebugOutput("${io.label}/steeringSetpoint", steeringSetpoint.inDegrees)
    CustomLogger.recordDebugOutput("${io.label}/lastDrivePos", lastDrivePos.inMeters)

    lastDrivePos = inputs.drivePosition

    modulePosition.distanceMeters = inputs.drivePosition.inMeters
    modulePosition.angle = inputs.steerPosition.inRotation2ds

    if (steerkD.hasChanged() || steerkP.hasChanged() || steerkI.hasChanged()) {
      io.configureSteerPID(steerkP.get(), steerkI.get(), steerkD.get())
    }

    if (drivekD.hasChanged() ||
      drivekP.hasChanged() ||
      drivekI.hasChanged() ||
      drivekA.hasChanged() ||
      drivekV.hasChanged()
    ) {
      io.configureDrivePID(
        drivekP.get(), drivekI.get(), drivekD.get(), drivekV.get(), drivekA.get()
      )
    }
    if (steerMaxVelo.hasChanged() || steerMaxAccel.hasChanged()) {
      io.configSteerMotionMagic(steerMaxVelo.get(), steerMaxAccel.get())
    }
  }

  fun setOpenLoop(steering: Angle, speed: LinearVelocity, optimize: Boolean = true) {
    var steeringDifference =
      (steering - inputs.steerPosition).inRadians.IEEErem(2 * Math.PI).radians
    val shouldInvert = steeringDifference.absoluteValue > (Math.PI / 2).radians && optimize
    if (shouldInvert) {
      steeringDifference -= Math.PI.withSign(steeringDifference.inRadians).radians
    }
    val outputSpeed =
      if (shouldInvert) {
        speed * -1
      } else {
        speed
      }
    steeringSetpoint = inputs.steerPosition + steeringDifference
    io.setOpenLoop(steeringSetpoint, outputSpeed)
  }

  fun setPositionOpenLoop(desiredState: SwerveModuleState, optimize: Boolean = true) {
    if (optimize) {
      val optimizedState =
        SwerveModuleState.optimize(desiredState, inputs.steerPosition.inRotation2ds)
      io.setOpenLoop(
        optimizedState.angle.angle,
        optimizedState.speedMetersPerSecond.meters.perSecond *
          cos(abs((optimizedState.angle.angle - inputs.steerPosition).inRadians))
      )
    } else {
      io.setOpenLoop(
        desiredState.angle.angle,
        desiredState.speedMetersPerSecond.meters.perSecond *
          cos(abs((desiredState.angle.angle - inputs.steerPosition).inRadians))
      )
    }
  }
  fun setPositionClosedLoop(
    desiredAccelState: SwerveModuleState,
    desiredVeloState: SwerveModuleState,
    optimize: Boolean = true
  ) {
    if (optimize) {
      val optmizedVeloState =
        SwerveModuleState.optimize(desiredVeloState, inputs.steerPosition.inRotation2ds)
      val optmizedAccelState =
        SwerveModuleState.optimize(desiredAccelState, inputs.steerPosition.inRotation2ds)

      steeringSetpoint = optmizedVeloState.angle.angle
      velocitySetpoint = optmizedVeloState.speedMetersPerSecond.meters.perSecond
      accelerationSetpoint = optmizedAccelState.speedMetersPerSecond.meters.perSecond.perSecond

      io.setClosedLoop(steeringSetpoint, velocitySetpoint, accelerationSetpoint)
    } else {
      steeringSetpoint = desiredVeloState.angle.angle
      velocitySetpoint = desiredVeloState.speedMetersPerSecond.meters.perSecond
      accelerationSetpoint = desiredAccelState.speedMetersPerSecond.meters.perSecond.perSecond

      io.setClosedLoop(steeringSetpoint, velocitySetpoint, accelerationSetpoint)
    }
  }
  fun resetModuleZero() {
    io.resetModuleZero()
  }
  fun zeroSteer() {
    io.zeroSteering()
  }
  fun setDriveBrakeMode(brake: Boolean) {
    io.setDriveBrakeMode(brake)
  }
  fun setSteeringBrakeMode(brake: Boolean) {
    io.setSteeringBrakeMode(brake)
  }
  fun zeroDrive() {
    io.zeroDrive()
  }
  fun runCharacterization(input: ElectricalPotential) {
    io.runCharacterization(input)
  }
}
