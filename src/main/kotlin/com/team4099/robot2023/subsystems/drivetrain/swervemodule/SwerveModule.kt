package com.team4099.robot2023.subsystems.drivetrain.swervemodule

import com.sun.org.apache.xalan.internal.lib.ExsltMath.abs
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.util.DebugLogger
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase.isReal
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.*
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.*

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
    Logger.processInputs(io.label, inputs)
    DebugLogger.recordDebugOutput("${io.label}/lastDrivePos", lastDrivePos.inMeters)

    DebugLogger.recordDebugOutput("${io.label}/velocitySetpoint", velocitySetpoint.inMetersPerSecond)
    DebugLogger.recordDebugOutput("${io.label}/accelerationSetpoint", accelerationSetpoint.inMetersPerSecondPerSecond)
    DebugLogger.recordDebugOutput("${io.label}/steeringSetpoint", steeringSetpoint.inDegrees)
    DebugLogger.recordDebugOutput("${io.label}/lastDrivePos", lastDrivePos.inMeters)

    Logger.recordOutput("${io.label}/driveAppliedVoltage", inputs.driveAppliedVoltage.inVolts)
    Logger.recordOutput("${io.label}/swerveAppliedVoltage", inputs.swerveAppliedVoltage.inVolts)

    Logger.recordOutput("${io.label}/drivePosition", inputs.drivePosition.inMeters)
    Logger.recordOutput("${io.label}/steerPosition", inputs.steerPosition.inRadians)
    Logger.recordOutput("${io.label}/driveVelocity", inputs.driveVelocity.inMetersPerSecond)
    Logger.recordOutput("${io.label}/steerVelocity", inputs.steerVelocity.inRadiansPerSecond)
    Logger.recordOutput("${io.label}/driveTemp", inputs.driveTemp.inCelsius)
    Logger.recordOutput("${io.label}/steerTemp", inputs.steerTemp.inCelsius)
    DebugLogger.recordDebugOutput("${io.label}/drift", inputs.drift.inMeters)

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
    }
  }
}
