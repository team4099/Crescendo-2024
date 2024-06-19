package com.team4099.robot2023.subsystems.drivetrain.swervemodule

import com.sun.org.apache.xalan.internal.lib.ExsltMath.abs
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.DrivetrainConstants
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase.isReal
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.angle
import org.team4099.lib.units.derived.degrees
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
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

class SwerveModule(val io: SwerveModuleIO) {

  val inputs = SwerveModuleIO.SwerveModuleIOInputs()

  var modulePosition = SwerveModulePosition()

  var posDeltas = mutableListOf<SwerveModulePosition>()

  private var velocitySetpoint: LinearVelocity = 0.0.meters.perSecond

  private var accelerationSetpoint: LinearAcceleration = 0.0.meters.perSecond.perSecond

  private var steeringSetpoint: Angle = 0.0.degrees

  var lastDrivePos = 0.0.meters

  var invert: Boolean = false

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
      "Drivetrain/driveka",
      Pair({ it.inVoltsPerMetersPerSecondPerSecond }, { it.volts.perMeterPerSecondPerSecond })
    )

  private val steerMaxAccel =
    LoggedTunableValue("Drivetrain/steerMaxAcceleration", DrivetrainConstants.STEERING_ACCEL_MAX)
  private val steerMaxVelo =
    LoggedTunableValue("Drivetrain/steerMaxVelocity", DrivetrainConstants.STEERING_VEL_MAX)

  fun init() {
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

    lastDrivePos = inputs.drivePosition

    modulePosition.distanceMeters = inputs.drivePosition.inMeters
    modulePosition.angle = inputs.steerPosition.inRotation2ds

    if (steerkD.hasChanged() || steerkP.hasChanged() || steerkI.hasChanged()) {
      io.configSteerPID(steerkP.get(), steerkI.get(), steerkD.get())
    }

    if (drivekD.hasChanged() ||
      drivekP.hasChanged() ||
      drivekI.hasChanged() ||
      drivekA.hasChanged() ||
      drivekV.hasChanged()
    ) {
      io.configDrivePID(drivekP.get(), drivekI.get(), drivekD.get(), drivekV.get(), drivekA.get())
    }
  }
  fun openLoop(desiredState: SwerveModuleState, optimize: Boolean) {
    if (optimize) {
      var optimizedState =
        SwerveModuleState.optimize(desiredState, inputs.steerPosition.inRotation2ds)
      io.setOpenLoop(
        optimizedState.angle.angle,
        optimizedState.speedMetersPerSecond.meters.perSecond *
          Math.cos(abs((optimizedState.angle.angle - inputs.steerPosition).inRadians))
      )
    } else {
      io.setOpenLoop(
        desiredState.angle.angle,
        desiredState.speedMetersPerSecond.meters.perSecond *
          Math.cos(abs((desiredState.angle.angle - inputs.steerPosition).inRadians))
      )
    }
  }
  fun closedLoop(
    desiredAccelState: SwerveModuleState,
    desiredVeloState: SwerveModuleState,
    optimize: Boolean
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
  fun zeroSteer(isInAuto: Boolean) {
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
}
