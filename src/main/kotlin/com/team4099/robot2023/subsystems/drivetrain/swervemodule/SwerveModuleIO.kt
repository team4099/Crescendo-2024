package com.team4099.robot2023.subsystems.drivetrain.swervemodule

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.Value
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond

interface SwerveModuleIO {

  class SwerveModuleIOInputs : LoggableInputs {

    var driveAppliedVoltage = 0.0.volts
    var steerAppliedVoltage = 0.0.volts

    var statorCurrentDrive = 0.0.amps
    var supplyCurrentDrive = 0.0.amps
    var statorCurrentSteer = 0.0.amps
    var supplyCurrentSteer = 0.0.amps

    var drivePosition = 0.0.meters
    var steerPosition = 0.0.radians

    var driveVelocity = 0.0.meters.perSecond
    var steerVelocity = 0.0.radians.perSecond

    var steerTemp = 0.0.celsius
    var driveTemp = 0.0.celsius

    var steerOdometryPos = listOf<Angle>()
    var driveOdometryPos = listOf<Length>()

    var potentiometerOutputRaw = 0.0
    var potentiometerOutputRadians = 0.0.radians

    var drift = 0.0.meters

    override fun toLog(table: LogTable?) {
      table?.put("driveAppliedVoltage", driveAppliedVoltage.inVolts)
      table?.put("swerveAppliedVoltage", steerAppliedVoltage.inVolts)
      table?.put("statorCurrentDrive", statorCurrentDrive.inAmperes)
      table?.put("supplyCurrentDrive", supplyCurrentDrive.inAmperes)
      table?.put("statorCurrentSteer", statorCurrentSteer.inAmperes)
      table?.put("supplyCurrentSteer", supplyCurrentSteer.inAmperes)
      table?.put("drivePosition", drivePosition.inMeters)
      table?.put("steerPosition", steerPosition.inRadians)
      table?.put("steerTemp", steerTemp.inCelsius)
      table?.put("driveTemp", driveTemp.inCelsius)
      table?.put("driveVelocity", driveVelocity.inMetersPerSecond)
      table?.put("steerVelocity", steerVelocity.inRadiansPerSecond)
      table?.put("drift", drift.inMeters)
      table?.put("potentiometerOutputRaw", potentiometerOutputRaw)
      table?.put("potentiometerOutputRadians", potentiometerOutputRadians.inRadians)

      if (driveOdometryPos.size > 0) {
        table?.put("odometryDrivePositionsMeters", driveOdometryPos[0].inMeters)
      } else {
        table?.put("odometryDrivePositionsMeters", 0.0)
      }

      if (steerOdometryPos.size > 0) {
        table?.put("odometrySteerPositionsDegrees", steerOdometryPos[0].inDegrees)
      } else {
        table?.put("odometrySteerPositionsDegrees", 0.0)
      }
    }

    override fun fromLog(table: LogTable?) {
      table?.get("driveAppliedVoltage", driveAppliedVoltage.inVolts)?.let {
        driveAppliedVoltage = it.volts
      }
      table?.get("swerveAppliedVoltage", steerAppliedVoltage.inVolts)?.let {
        steerAppliedVoltage = it.volts
      }
      table?.get("statorCurrentDrive", statorCurrentDrive.inAmperes)?.let {
        statorCurrentDrive = it.amps
      }
      table?.get("supplyCurrentDrive", supplyCurrentDrive.inAmperes)?.let {
        supplyCurrentDrive = it.amps
      }
      table?.get("statorCurrentSteer", statorCurrentSteer.inAmperes)?.let {
        statorCurrentSteer = it.amps
      }
      table?.get("supplyCurrentSteer", supplyCurrentSteer.inAmperes)?.let {
        supplyCurrentSteer = it.amps
      }
      table?.get("steerTemp", steerTemp.inCelsius)?.let { steerTemp = it.celsius }
      table?.get("driveTemp", driveTemp.inCelsius)?.let { driveTemp = it.celsius }
      table?.get("drivePosition", drivePosition.inMeters)?.let { drivePosition = it.meters }
      table?.get("steerPosition", steerPosition.inRadians)?.let { steerPosition = it.radians }
      table?.get("driveVelocity", driveVelocity.inMetersPerSecond)?.let {
        driveVelocity = it.meters.perSecond
      }
      table?.get("steerVelocity", steerVelocity.inRadiansPerSecond)?.let {
        steerVelocity = it.radians.perSecond
      }
      table?.get("drift", drift.inMeters)?.let { drift = it.meters }
      table?.get("potentiometerOutputRaw", potentiometerOutputRaw)?.let {
        potentiometerOutputRaw = it
      }
      table?.get("potentiometerOutputRaw", potentiometerOutputRadians.inRadians)?.let {
        potentiometerOutputRadians = it.radians
      }
    }
  }
  val label: String

  fun updateInputs(inputs: SwerveModuleIOInputs) {}

  fun setOpenLoop(steering: Angle, speed: LinearVelocity) {}

  fun setClosedLoop(angle: Angle, speed: LinearVelocity, acceleration: LinearAcceleration) {}

  fun resetModuleZero() {}

  fun zeroSteering(isInAuto: Boolean = false) {}

  fun zeroDrive() {}

  fun setDriveBrakeMode(brake: Boolean) {}

  fun setSteeringBrakeMode(brake: Boolean) {}

  fun runCharacterization(input: ElectricalPotential) {}

  fun setSteeringSetpoint(angle: Angle) {}

  fun configureSteerPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {}

  fun configureDrivePID(
    kP: ProportionalGain<Velocity<Meter>, Volt>,
    kI: IntegralGain<Velocity<Meter>, Volt>,
    kD: DerivativeGain<Velocity<Meter>, Volt>,
    kV: Value<Fraction<Volt, Velocity<Meter>>>,
    kA: Value<Fraction<Volt, Velocity<Velocity<Meter>>>>
  ) {}

  fun configSteerMotionMagic(maxVel: AngularVelocity, maxAccel: AngularAcceleration) {}
}
