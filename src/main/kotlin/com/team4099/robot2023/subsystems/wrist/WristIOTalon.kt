package com.team4099.robot2023.subsystems.wrist

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.team4099.lib.phoenix6.PositionVoltage
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.config.constants.WristConstants
import com.team4099.robot2023.subsystems.falconspin.Falcon500
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

object WristIOTalon : WristIO {

  private val wristTalon: TalonFX = TalonFX(Constants.WRIST.WRIST_MOTOR_ID)

  private val wristConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  private val absoluteEncoder: CANcoder = CANcoder(Constants.WRIST.CANCODER_ID)
  private val absoluteEncoderConfiguration: MagnetSensorConfigs = MagnetSensorConfigs()

  private val wristSensor =
    ctreAngularMechanismSensor(
      wristTalon,
      FlywheelConstants.RIGHT_GEAR_RATIO,
      FlywheelConstants.VOLTAGE_COMPENSATION,
    )

  var statorCurrentSignal: StatusSignal<Double>
  var supplyCurrentSignal: StatusSignal<Double>
  var tempSignal: StatusSignal<Double>
  var dutyCycle: StatusSignal<Double>
  init {
    wristTalon.configurator.apply(TalonFXConfiguration())

    wristTalon.clearStickyFaults()

    absoluteEncoderConfiguration.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf
    absoluteEncoderConfiguration.SensorDirection = SensorDirectionValue.CounterClockwise_Positive
    absoluteEncoderConfiguration.MagnetOffset =
      WristConstants.ABSOLUTE_ENCODER_OFFSET.inDegrees / 180

    absoluteEncoder.configurator.apply(absoluteEncoderConfiguration)

    wristConfiguration.Feedback.FeedbackRemoteSensorID = absoluteEncoder.deviceID
    wristConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder

    wristConfiguration.Feedback.SensorToMechanismRatio = WristConstants.WRIST_GEAR_RATIO
    wristConfiguration.Feedback.RotorToSensorRatio = WristConstants.WRIST_ENCODER_GEAR_RATIO

    wristConfiguration.Slot0.kP =
      wristSensor.proportionalVelocityGainToRawUnits(FlywheelConstants.PID.REAL_KP)
    wristConfiguration.Slot0.kI =
      wristSensor.integralVelocityGainToRawUnits(FlywheelConstants.PID.REAL_KI)
    wristConfiguration.Slot0.kD =
      wristSensor.derivativeVelocityGainToRawUnits(FlywheelConstants.PID.REAL_KD)

    wristConfiguration.CurrentLimits.SupplyCurrentLimit =
      FlywheelConstants.LEFT_FLYWHEEL_SUPPLY_CURRENT_LIMIT.inAmperes
    wristConfiguration.CurrentLimits.SupplyCurrentThreshold =
      FlywheelConstants.LEFT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT.inAmperes
    wristConfiguration.CurrentLimits.SupplyTimeThreshold =
      FlywheelConstants.LEFT_flywheel_TRIGGER_THRESHOLD_TIME.inSeconds
    wristConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    wristConfiguration.CurrentLimits.StatorCurrentLimit =
      FlywheelConstants.LEFT_FLYWHEEL_STATOR_CURRENT_LIMIT.inAmperes
    wristConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

    wristConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake

    wristTalon.configurator.apply(wristConfiguration)

    statorCurrentSignal = wristTalon.statorCurrent
    supplyCurrentSignal = wristTalon.supplyCurrent
    tempSignal = wristTalon.deviceTemp
    dutyCycle = wristTalon.dutyCycle

    MotorChecker.add(
      "Wrist",
      "Wrist",
      MotorCollection(
        mutableListOf(Falcon500(wristTalon, "Wrist Motor")),
        WristConstants.WRIST_STATOR_CURRENT_LIMIT,
        90.celsius,
        WristConstants.WRIST_STATOR_CURRENT_LIMIT - 30.amps,
        110.celsius
      )
    )
  }

  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    val PIDConfig = Slot0Configs()
    PIDConfig.kP = wristSensor.proportionalPositionGainToRawUnits(kP)
    PIDConfig.kI = wristSensor.integralPositionGainToRawUnits(kI)
    PIDConfig.kD = wristSensor.derivativePositionGainToRawUnits(kD)

    wristTalon.configurator.apply(PIDConfig)
  }

  override fun setWristVoltage(voltage: ElectricalPotential) {
    wristTalon.setVoltage(voltage.inVolts)
  }

  override fun setWristPosition(position: Angle, feedforward: ElectricalPotential) {
    wristTalon.setControl(
      PositionVoltage(position, slot = 0, feedforward = feedforward).positionVoltagePhoenix6
    )
  }
  override fun updateInputs(inputs: WristIO.WristIOInputs) {
    inputs.wristPostion = wristSensor.position
    inputs.wristVelocity = wristSensor.velocity
    inputs.wristAppliedVoltage = dutyCycle.value.volts
    inputs.wristStatorCurrent = statorCurrentSignal.value.amps
    inputs.wristSupplyCurrent = supplyCurrentSignal.value.amps
    inputs.wristTemperature = tempSignal.value.celsius
  }

  override fun setWristBrakeMode(brake: Boolean) {
    val motorOutputConfig = MotorOutputConfigs()

    if (brake) {
      motorOutputConfig.NeutralMode = NeutralModeValue.Brake
    } else {
      motorOutputConfig.NeutralMode = NeutralModeValue.Coast
    }
    wristTalon.configurator.apply(motorOutputConfig)
  }

  override fun zeroEncoder() {}
}
