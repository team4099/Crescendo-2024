package com.team4099.robot2023.subsystems.wrist

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.InvertedValue
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
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object WristIOTalon : WristIO {

  private val wristTalon: TalonFX = TalonFX(Constants.WRIST.WRIST_MOTOR_ID)

  private val wristConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  private val absoluteEncoder: CANcoder = CANcoder(Constants.WRIST.CANCODER_ID)
  private val absoluteEncoderConfiguration: MagnetSensorConfigs = MagnetSensorConfigs()

  var positionRequest = PositionVoltage(-1337.degrees, slot = 0, feedforward = -1337.volts)

  private val wristSensor =
    ctreAngularMechanismSensor(
      wristTalon,
      WristConstants.WRIST_GEAR_RATIO,
      WristConstants.VOLTAGE_COMPENSATION,
    )

  var statorCurrentSignal: StatusSignal<Double>
  var supplyCurrentSignal: StatusSignal<Double>
  var tempSignal: StatusSignal<Double>
  var dutyCycle: StatusSignal<Double>
  var motorVoltage: StatusSignal<Double>
  var motorTorque: StatusSignal<Double>
  var motorAcceleration: StatusSignal<Double>

  init {
    wristTalon.configurator.apply(TalonFXConfiguration())

    wristTalon.clearStickyFaults()

    absoluteEncoderConfiguration.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf
    absoluteEncoderConfiguration.SensorDirection = SensorDirectionValue.CounterClockwise_Positive
    absoluteEncoderConfiguration.MagnetOffset =
      WristConstants.ABSOLUTE_ENCODER_OFFSET.inDegrees / 180

    /*
    absoluteEncoder.configurator.apply(absoluteEncoderConfiguration)

    wristConfiguration.Feedback.FeedbackRemoteSensorID = absoluteEncoder.deviceID
    wristConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder

    wristConfiguration.Feedback.SensorToMechanismRatio = WristConstants.WRIST_GEAR_RATIO
    wristConfiguration.Feedback.RotorToSensorRatio = WristConstants.WRIST_ENCODER_GEAR_RATIO


     */
    wristConfiguration.Slot0.kP =
      wristSensor.proportionalPositionGainToRawUnits(WristConstants.PID.REAL_KP)
    wristConfiguration.Slot0.kI =
      wristSensor.integralPositionGainToRawUnits(WristConstants.PID.REAL_KI)
    wristConfiguration.Slot0.kD =
      wristSensor.derivativePositionGainToRawUnits(WristConstants.PID.REAL_KD)

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
    wristConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

    wristTalon.configurator.apply(wristConfiguration)

    statorCurrentSignal = wristTalon.statorCurrent
    supplyCurrentSignal = wristTalon.supplyCurrent
    tempSignal = wristTalon.deviceTemp
    dutyCycle = wristTalon.dutyCycle
    motorVoltage = wristTalon.motorVoltage
    motorTorque = wristTalon.torqueCurrent
    motorAcceleration = wristTalon.acceleration

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
    wristTalon.setControl(VoltageOut(voltage.inVolts))
  }

  override fun setWristPosition(position: Angle, feedforward: ElectricalPotential) {
    positionRequest.setFeedforward(feedforward)
    positionRequest.setPosition(position)
    wristTalon.setControl(positionRequest.positionVoltagePhoenix6)
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
      motorTorque, motorVoltage, dutyCycle, supplyCurrentSignal, tempSignal, motorAcceleration
    )
  }

  override fun updateInputs(inputs: WristIO.WristIOInputs) {

    updateSignals()

    inputs.wristPosition = wristSensor.position
    inputs.wristAcceleration =
      (motorAcceleration.value * WristConstants.WRIST_GEAR_RATIO).rotations.perSecond.perSecond
    inputs.wristVelocity = wristSensor.velocity
    // TODO fix unit for torque
    inputs.wristTorque = motorTorque.value
    inputs.wristAppliedVoltage = motorVoltage.value.volts
    inputs.wristDutyCycle = dutyCycle.value.volts
    inputs.wristStatorCurrent = statorCurrentSignal.value.amps
    inputs.wristSupplyCurrent = supplyCurrentSignal.value.amps
    inputs.wristTemperature = tempSignal.value.celsius

    if (inputs.wristPosition < WristConstants.WRIST_MIN_ROTATION) {
      wristTalon.setPosition(wristSensor.positionToRawUnits(WristConstants.WRIST_MIN_ROTATION))
    }
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

  override fun zeroEncoder(encoderAngle: Angle) {
    wristTalon.setPosition(wristSensor.positionToRawUnits(encoderAngle))
  }
}
