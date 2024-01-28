package com.team4099.robot2023.subsystems.flywheel

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.phoenix6.VelocityVoltage
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.Constants.Shooter.FLYWHEEL_LEFT_MOTOR_ID
import com.team4099.robot2023.config.constants.Constants.Shooter.FLYWHEEL_RIGHT_MOTOR_ID
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.falconspin.Falcon500
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.newtons
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object FlywheelIOTalon : FlywheelIO {

  private val flywheelLeftTalon: TalonFX = TalonFX(FLYWHEEL_LEFT_MOTOR_ID)

  private val flywheelRightTalon: TalonFX = TalonFX(FLYWHEEL_RIGHT_MOTOR_ID)

  private val flywheelLeftConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  private val flywheelRightConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  var leftVelocityRequest =
    VelocityVoltage(-1337.degrees.perSecond, slot = 0, feedforward = -1337.volts)
  var rightVelocityRequest =
    VelocityVoltage(-1337.degrees.perSecond, slot = 0, feedforward = -1337.volts)

  private val flywheelLeftSensor =
    ctreAngularMechanismSensor(
      flywheelLeftTalon,
      FlywheelConstants.LEFT_GEAR_RATIO,
      FlywheelConstants.VOLTAGE_COMPENSATION,
    )

  private val flywheelRightSensor =
    ctreAngularMechanismSensor(
      flywheelRightTalon,
      FlywheelConstants.RIGHT_GEAR_RATIO,
      FlywheelConstants.VOLTAGE_COMPENSATION,
    )

  var leftFlywheelStatorCurrentSignal: StatusSignal<Double>
  var leftFlywheelSupplyCurrentSignal: StatusSignal<Double>
  var leftFlywheelTempSignal: StatusSignal<Double>
  var leftFlywheelDutyCycle: StatusSignal<Double>

  var rightFlywheelStatorCurrentSignal: StatusSignal<Double>
  var rightFlywheelSupplyCurrentSignal: StatusSignal<Double>
  var rightFlywheelTempSignal: StatusSignal<Double>
  var rightFlywheelDutyCycle: StatusSignal<Double>
  var motorVoltage: StatusSignal<Double>
  var motorTorque: StatusSignal<Double>

  init {

    flywheelLeftTalon.configurator.apply(TalonFXConfiguration())
    flywheelRightTalon.configurator.apply(TalonFXConfiguration())

    flywheelLeftTalon.clearStickyFaults()
    flywheelRightTalon.clearStickyFaults()

    flywheelLeftConfiguration.Slot0.kP =
      flywheelLeftSensor.proportionalVelocityGainToRawUnits(FlywheelConstants.PID.LEFT_REAL_KP)
    flywheelLeftConfiguration.Slot0.kI =
      flywheelLeftSensor.integralVelocityGainToRawUnits(FlywheelConstants.PID.LEFT_REAL_KI)
    flywheelLeftConfiguration.Slot0.kD =
      flywheelLeftSensor.derivativeVelocityGainToRawUnits(FlywheelConstants.PID.LEFT_REAL_KD)

    flywheelRightConfiguration.Slot0.kP =
      flywheelRightSensor.proportionalVelocityGainToRawUnits(FlywheelConstants.PID.RIGHT_REAL_KP)
    flywheelRightConfiguration.Slot0.kI =
      flywheelRightSensor.integralVelocityGainToRawUnits(FlywheelConstants.PID.RIGHT_REAL_KI)
    flywheelRightConfiguration.Slot0.kD =
      flywheelRightSensor.derivativeVelocityGainToRawUnits(FlywheelConstants.PID.RIGHT_REAL_KD)
    //
    // flywheelSensor.velocityFeedforwardToRawUnits(FlywheelConstantsConstants.PID.flywheel_KFF)

    flywheelLeftConfiguration.CurrentLimits.SupplyCurrentLimit =
      FlywheelConstants.LEFT_FLYWHEEL_SUPPLY_CURRENT_LIMIT.inAmperes
    flywheelLeftConfiguration.CurrentLimits.SupplyCurrentThreshold =
      FlywheelConstants.LEFT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT.inAmperes
    flywheelLeftConfiguration.CurrentLimits.SupplyTimeThreshold =
      FlywheelConstants.LEFT_flywheel_TRIGGER_THRESHOLD_TIME.inSeconds
    flywheelLeftConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    flywheelLeftConfiguration.CurrentLimits.StatorCurrentLimit =
      FlywheelConstants.LEFT_FLYWHEEL_STATOR_CURRENT_LIMIT.inAmperes
    flywheelLeftConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

    flywheelRightConfiguration.CurrentLimits.SupplyCurrentLimit =
      FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT.inAmperes
    flywheelRightConfiguration.CurrentLimits.SupplyCurrentThreshold =
      FlywheelConstants.RIGHT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT.inAmperes
    flywheelRightConfiguration.CurrentLimits.SupplyTimeThreshold =
      FlywheelConstants.RIGHT_flywheel_TRIGGER_THRESHOLD_TIME.inSeconds
    flywheelRightConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    flywheelRightConfiguration.CurrentLimits.StatorCurrentLimit =
      FlywheelConstants.RIGHT_FLYWHEEL_STATOR_CURRENT_LIMIT.inAmperes
    flywheelRightConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

    flywheelLeftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    flywheelRightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake

    flywheelRightTalon.configurator.apply(flywheelRightConfiguration)

    flywheelLeftTalon.setControl(Follower(Constants.Shooter.FLYWHEEL_RIGHT_MOTOR_ID, true))

    rightFlywheelStatorCurrentSignal = flywheelRightTalon.statorCurrent
    rightFlywheelSupplyCurrentSignal = flywheelRightTalon.supplyCurrent
    rightFlywheelTempSignal = flywheelRightTalon.deviceTemp
    rightFlywheelDutyCycle = flywheelRightTalon.dutyCycle
    motorVoltage = flywheelRightTalon.motorVoltage
    motorTorque = flywheelRightTalon.torqueCurrent

    leftFlywheelStatorCurrentSignal = flywheelLeftTalon.statorCurrent
    leftFlywheelSupplyCurrentSignal = flywheelLeftTalon.supplyCurrent
    leftFlywheelTempSignal = flywheelLeftTalon.deviceTemp
    leftFlywheelDutyCycle = flywheelLeftTalon.dutyCycle

    MotorChecker.add(
      "Shooter",
      "Flywheel",
      MotorCollection(
        mutableListOf(Falcon500(flywheelRightTalon, "Flywheel Right Motor")),
        FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT,
        90.celsius,
        FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT - 30.amps,
        110.celsius
      )
    )
    MotorChecker.add(
      "Shooter",
      "Flywheel",
      MotorCollection(
        mutableListOf(Falcon500(flywheelLeftTalon, "Flywheel Left Motor")),
        FlywheelConstants.LEFT_FLYWHEEL_SUPPLY_CURRENT_LIMIT,
        90.celsius,
        FlywheelConstants.LEFT_FLYWHEEL_SUPPLY_CURRENT_LIMIT - 30.amps,
        110.celsius
      )
    )
    MotorChecker.add(
      "Shooter",
      "Flywheel",
      MotorCollection(
        mutableListOf(Falcon500(flywheelRightTalon, "Flywheel Right Motor")),
        FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT,
        90.celsius,
        FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT - 30.amps,
        110.celsius
      )
    )
  }

  override fun configPID(
    rightkP: ProportionalGain<Velocity<Radian>, Volt>,
    rightkI: IntegralGain<Velocity<Radian>, Volt>,
    rightkD: DerivativeGain<Velocity<Radian>, Volt>,
    leftkP: ProportionalGain<Velocity<Radian>, Volt>,
    leftkI: IntegralGain<Velocity<Radian>, Volt>,
    leftkD: DerivativeGain<Velocity<Radian>, Volt>
  ) {
    val PIDRightConfig = Slot0Configs()
    PIDRightConfig.kP = flywheelRightSensor.proportionalVelocityGainToRawUnits(rightkP)
    PIDRightConfig.kI = flywheelRightSensor.integralVelocityGainToRawUnits(rightkI)
    PIDRightConfig.kD = flywheelRightSensor.derivativeVelocityGainToRawUnits(rightkD)

    val PIDLeftConfig = Slot0Configs()
    PIDLeftConfig.kP = flywheelLeftSensor.proportionalVelocityGainToRawUnits(leftkP)
    PIDLeftConfig.kI = flywheelLeftSensor.integralVelocityGainToRawUnits(leftkI)
    PIDLeftConfig.kD = flywheelLeftSensor.derivativeVelocityGainToRawUnits(leftkD)

    flywheelLeftTalon.configurator.apply(PIDLeftConfig)
    flywheelRightTalon.configurator.apply(PIDRightConfig)
  }

  override fun setFlywheelVoltage(
    voltageRight: ElectricalPotential,
    voltageLeft: ElectricalPotential
  ) {
    flywheelRightTalon.setVoltage(voltageRight.inVolts)
    flywheelLeftTalon.setVoltage(voltageLeft.inVolts)
  }

  override fun setFlywheelVelocity(
    rightVelocity: AngularVelocity,
    leftVelocity: AngularVelocity,
    feedforwardLeft: ElectricalPotential,
    feedforwardRight: ElectricalPotential
  ) {
    FlywheelIOTalon.rightVelocityRequest.setFeedforward(feedforwardRight)
    FlywheelIOTalon.rightVelocityRequest.setVelocity(rightVelocity)
    FlywheelIOTalon.flywheelRightTalon.setControl(
      FlywheelIOTalon.rightVelocityRequest.velocityVoltagePhoenix6
    )
    FlywheelIOTalon.leftVelocityRequest.setFeedforward(feedforwardLeft)
    FlywheelIOTalon.leftVelocityRequest.setVelocity(leftVelocity)
    FlywheelIOTalon.flywheelRightTalon.setControl(
      FlywheelIOTalon.leftVelocityRequest.velocityVoltagePhoenix6
    )
  }
  override fun updateInputs(inputs: FlywheelIO.FlywheelIOInputs) {
    inputs.rightFlywheelVelocity = flywheelRightSensor.velocity
    inputs.rightFlywheelAppliedVoltage = motorVoltage.value.volts
    inputs.rightFlywheelStatorCurrent = rightFlywheelStatorCurrentSignal.value.amps
    inputs.rightFlywheelSupplyCurrent = rightFlywheelSupplyCurrentSignal.value.amps
    inputs.rightFlywheelTemperature = rightFlywheelTempSignal.value.celsius
    // TODO fix unit for torque
    inputs.rightFlywheelTorque = motorTorque.value.newtons
    inputs.rightFlywheelDutyCycle = rightFlywheelDutyCycle.value.volts

    inputs.leftFlywheelVelocity = flywheelLeftSensor.velocity
    inputs.leftFlywheelAppliedVoltage = leftFlywheelDutyCycle.value.volts
    inputs.leftFlywheelStatorCurrent = leftFlywheelStatorCurrentSignal.value.amps
    inputs.leftFlywheelSupplyCurrent = leftFlywheelSupplyCurrentSignal.value.amps
    inputs.leftFlywheelTemperature = leftFlywheelTempSignal.value.celsius
  }

  override fun setFlywheelBrakeMode(brake: Boolean) {
    val motorOutputConfig = MotorOutputConfigs()

    if (brake) {
      motorOutputConfig.NeutralMode = NeutralModeValue.Brake
    } else {
      motorOutputConfig.NeutralMode = NeutralModeValue.Coast
    }

    flywheelRightTalon.configurator.apply(motorOutputConfig)
  }
}
