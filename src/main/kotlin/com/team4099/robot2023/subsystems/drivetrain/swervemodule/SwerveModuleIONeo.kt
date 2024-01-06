package com.team4099.robot2023.subsystems.drivetrain.swervemodule

import com.revrobotics.CANSparkMax
import com.revrobotics.SparkMaxAbsoluteEncoder
import com.revrobotics.SparkMaxPIDController
import com.team4099.robot2023.config.constants.DrivetrainConstants
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor
import org.team4099.lib.units.sparkMaxLinearMechanismSensor

class SwerveModuleIONeo(
  private val steeringMotor: CANSparkMax,
  private val driveMotor: CANSparkMax,
  private val zeroOffset: Angle,
  override val label: String
) : SwerveModuleIO {
  private val steeringSensor =
    sparkMaxAngularMechanismSensor(
      steeringMotor,
      DrivetrainConstants.NeoConstants.STEERING_SENSOR_GEAR_RATIO,
      DrivetrainConstants.STEERING_COMPENSATION_VOLTAGE
    )
  private val driveSensor =
    sparkMaxLinearMechanismSensor(
      driveMotor,
      DrivetrainConstants.NeoConstants.DRIVE_SENSOR_GEAR_RATIO,
      DrivetrainConstants.WHEEL_DIAMETER,
      DrivetrainConstants.DRIVE_COMPENSATION_VOLTAGE
    )
  private val throughBoreEncoder =
    steeringMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)

  private val potentiometerOutput: Angle
    get() {
      return throughBoreEncoder.position.rotations
    }

  private val steeringPIDController: SparkMaxPIDController = steeringMotor.pidController
  private val drivePIDController: SparkMaxPIDController = driveMotor.pidController

  init {
    steeringMotor.restoreFactoryDefaults()
    steeringMotor.clearFaults()

    steeringMotor.enableVoltageCompensation(
      DrivetrainConstants.STEERING_COMPENSATION_VOLTAGE.inVolts
    )
    steeringMotor.setSmartCurrentLimit(
      DrivetrainConstants.STEERING_STATOR_CURRENT_LIMIT.inAmperes.toInt()
    )

    steeringMotor.idleMode = CANSparkMax.IdleMode.kCoast

    steeringMotor.burnFlash()

    driveMotor.restoreFactoryDefaults()
    driveMotor.clearFaults()

    driveMotor.enableVoltageCompensation(DrivetrainConstants.DRIVE_COMPENSATION_VOLTAGE.inVolts)
    driveMotor.setSmartCurrentLimit(
      DrivetrainConstants.DRIVE_STATOR_CURRENT_LIMIT.inAmperes.toInt()
    )
    driveMotor.idleMode = CANSparkMax.IdleMode.kBrake

    driveMotor.burnFlash()

    // TODO: add motor checker stuff
  }

  override fun updateInputs(inputs: SwerveModuleIO.SwerveModuleIOInputs) {
    inputs.driveAppliedVoltage = driveMotor.busVoltage.volts * driveMotor.appliedOutput
    inputs.steeringAppliedVoltage = steeringMotor.busVoltage.volts * steeringMotor.appliedOutput

    inputs.driveStatorCurrent = driveMotor.outputCurrent.amps
    inputs.driveSupplyCurrent = inputs.driveStatorCurrent * driveMotor.appliedOutput
    inputs.steeringStatorCurrent = steeringMotor.outputCurrent.amps
    inputs.steeringSupplyCurrent = inputs.steeringStatorCurrent * steeringMotor.appliedOutput

    inputs.drivePosition = driveSensor.position
    inputs.steeringPosition = steeringSensor.position

    inputs.driveVelocity = driveSensor.velocity
    inputs.steeringVelocity = steeringSensor.velocity

    inputs.driveTemp = driveMotor.motorTemperature.celsius
    inputs.steeringTemp = steeringMotor.motorTemperature.celsius

    inputs.potentiometerOutputRadians = potentiometerOutput
  }

  override fun setSteeringSetpoint(angle: Angle) {
    steeringPIDController.setReference(
      steeringSensor.positionToRawUnits(angle), CANSparkMax.ControlType.kPosition, 0
    )
  }

  override fun setClosedLoop(
    steering: Angle,
    speed: LinearVelocity,
    acceleration: LinearAcceleration
  ) {
    val feedforward = DrivetrainConstants.PID.DRIVE_KS * speed.sign

    drivePIDController.setReference(
      driveSensor.velocityToRawUnits(speed),
      CANSparkMax.ControlType.kVelocity,
      0,
      feedforward.inVolts
    )

    setSteeringSetpoint(steering)
  }

  /**
   * Open Loop Control using PercentOutput control on a Falcon
   *
   * @param steering: Desired angle
   * @param speed: Desired speed
   */
  override fun setOpenLoop(steering: Angle, speed: LinearVelocity) {
    driveMotor.set(speed / DrivetrainConstants.NeoConstants.DRIVE_SETPOINT_MAX)
    setSteeringSetpoint(steering)
  }

  override fun resetModuleZero() {
    println("Absolute Potentiometer Value $label ($potentiometerOutput")
  }

  override fun zeroSteering() {
    steeringMotor.encoder.position =
      steeringSensor.positionToRawUnits(potentiometerOutput - zeroOffset)
  }

  override fun zeroDrive() {
    driveMotor.encoder.position = 0.0
  }

  override fun configureDrivePID(
    kP: ProportionalGain<Velocity<Meter>, Volt>,
    kI: IntegralGain<Velocity<Meter>, Volt>,
    kD: DerivativeGain<Velocity<Meter>, Volt>
  ) {
    drivePIDController.p = driveSensor.proportionalVelocityGainToRawUnits(kP)
    drivePIDController.i = driveSensor.integralVelocityGainToRawUnits(kI)
    drivePIDController.d = driveSensor.derivativeVelocityGainToRawUnits(kD)
  }

  override fun configureSteeringPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    steeringPIDController.p = steeringSensor.proportionalPositionGainToRawUnits(kP)
    steeringPIDController.i = steeringSensor.integralPositionGainToRawUnits(kI)
    steeringPIDController.d = steeringSensor.derivativePositionGainToRawUnits(kD)
  }

  override fun setDriveBrakeMode(brake: Boolean) {
    if (brake) {
      driveMotor.idleMode = CANSparkMax.IdleMode.kBrake
    } else {
      driveMotor.idleMode = CANSparkMax.IdleMode.kCoast
    }
  }

  override fun setSteeringBrakeMode(brake: Boolean) {
    if (brake) {
      steeringMotor.idleMode = CANSparkMax.IdleMode.kBrake
    } else {
      steeringMotor.idleMode = CANSparkMax.IdleMode.kCoast
    }
  }
}
