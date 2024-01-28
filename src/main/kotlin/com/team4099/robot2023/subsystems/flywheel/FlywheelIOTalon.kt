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
import com.team4099.robot2023.config.constants.Constants.Shooter.FLYWHEEL_RIGHT_MOTOR_ID
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.falconspin.Falcon500
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import com.team4099.robot2023.subsystems.wrist.WristIOTalon
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
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object FlywheelIOTalon : FlywheelIO {

  private val flywheelRightTalon: TalonFX = TalonFX(FLYWHEEL_RIGHT_MOTOR_ID)

  private val flywheelRightConfiguration: TalonFXConfiguration = TalonFXConfiguration()

 var velocityRequest = VelocityVoltage(-1337.degrees.perSecond, slot = 0, feedforward = -1337.volts)

  private val flywheelRightSensor =
    ctreAngularMechanismSensor(
      flywheelRightTalon,
      FlywheelConstants.RIGHT_GEAR_RATIO,
      FlywheelConstants.VOLTAGE_COMPENSATION,
    )


  var rightFlywheelStatorCurrentSignal: StatusSignal<Double>
  var rightFlywheelSupplyCurrentSignal: StatusSignal<Double>
  var rightFlywheelTempSignal: StatusSignal<Double>
  var rightFlywheelDutyCycle: StatusSignal<Double>
  var motorVoltage : StatusSignal<Double>
  var motorTorque :StatusSignal<Double>

  init {

    flywheelRightConfiguration.Slot0.kP =
      flywheelRightSensor.proportionalVelocityGainToRawUnits(FlywheelConstants.PID.REAL_KP)
    flywheelRightConfiguration.Slot0.kI =
      flywheelRightSensor.integralVelocityGainToRawUnits(FlywheelConstants.PID.REAL_KI)
    flywheelRightConfiguration.Slot0.kD =
      flywheelRightSensor.derivativeVelocityGainToRawUnits(FlywheelConstants.PID.REAL_KD)
    //
    // flywheelSensor.velocityFeedforwardToRawUnits(FlywheelConstantsConstants.PID.flywheel_KFF)

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

    flywheelRightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake

    flywheelRightTalon.configurator.apply(flywheelRightConfiguration)

    flywheelRightTalon.setControl(Follower(Constants.Shooter.FLYWHEEL_RIGHT_MOTOR_ID, true))

    rightFlywheelStatorCurrentSignal = flywheelRightTalon.statorCurrent
    rightFlywheelSupplyCurrentSignal = flywheelRightTalon.supplyCurrent
    rightFlywheelTempSignal = flywheelRightTalon.deviceTemp
    rightFlywheelDutyCycle = flywheelRightTalon.dutyCycle
    motorVoltage = flywheelRightTalon.motorVoltage
    motorTorque =flywheelRightTalon.torqueCurrent



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
        mutableListOf(Falcon500(flywheelRightTalon, "Flywheel Right Motor")),
        FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT,
        90.celsius,
        FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT - 30.amps,
        110.celsius
      )
    )
  }

  override fun configPID(
    kP: ProportionalGain<Velocity<Radian>, Volt>,
    kI: IntegralGain<Velocity<Radian>, Volt>,
    kD: DerivativeGain<Velocity<Radian>, Volt>
  ) {
    val PIDConfig = Slot0Configs()
    PIDConfig.kP = flywheelRightSensor.proportionalVelocityGainToRawUnits(kP)
    PIDConfig.kI = flywheelRightSensor.integralVelocityGainToRawUnits(kI)
    PIDConfig.kD = flywheelRightSensor.derivativeVelocityGainToRawUnits(kD)


    flywheelRightTalon.configurator.apply(PIDConfig)
  }

  override fun setFlywheelVoltage(voltage: ElectricalPotential) {
    flywheelRightTalon.setVoltage(voltage.inVolts)
  }

  override fun setFlywheelVelocity(velocity: AngularVelocity, feedforward: ElectricalPotential) {
    FlywheelIOTalon.velocityRequest.setFeedforward(feedforward)
    FlywheelIOTalon.velocityRequest.setVelocity(velocity)
    FlywheelIOTalon.flywheelRightTalon.setControl(FlywheelIOTalon.velocityRequest.velocityVoltagePhoenix6)
  }
  override fun updateInputs(inputs: FlywheelIO.FlywheelIOInputs) {
    inputs.rightFlywheelVelocity = flywheelRightSensor.velocity
    inputs.rightFlywheelAppliedVoltage = motorVoltage.value.volts
    inputs.rightFlywheelStatorCurrent = rightFlywheelStatorCurrentSignal.value.amps
    inputs.rightFlywheelSupplyCurrent = rightFlywheelSupplyCurrentSignal.value.amps
    inputs.rightFlywheelTemperature = rightFlywheelTempSignal.value.celsius
    //TODO fix unit for torque
    inputs.rightFlywheelTorque = motorTorque.value
    inputs.rightFlywheelDutyCycle = rightFlywheelDutyCycle.value.volts


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
