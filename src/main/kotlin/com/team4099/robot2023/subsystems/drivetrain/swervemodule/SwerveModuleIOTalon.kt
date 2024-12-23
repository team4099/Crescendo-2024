package com.team4099.robot2023.subsystems.drivetrain.swervemodule

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.falconspin.Falcon500
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import com.team4099.utils.threads.PhoenixOdometryThread
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.RobotController
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.Value
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.ctreLinearMechanismSensor
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
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecond
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecondPerSecond
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
import java.lang.Math.PI
import java.util.Queue

class SwerveModuleIOTalon(
  private val steeringFalcon: TalonFX,
  private val driveFalcon: TalonFX,
  private val potentiometer: AnalogInput,
  private val zeroOffset: Angle,
  override val label: String
) : SwerveModuleIO {
  private val steeringSensor =
    ctreAngularMechanismSensor(
      steeringFalcon, 1.0, DrivetrainConstants.STEERING_COMPENSATION_VOLTAGE
    )

  private val driveSensor =
    ctreLinearMechanismSensor(
      driveFalcon,
      DrivetrainConstants.DRIVE_SENSOR_GEAR_RATIO,
      DrivetrainConstants.WHEEL_DIAMETER,
      DrivetrainConstants.DRIVE_COMPENSATION_VOLTAGE
    )

  // motor params
  private val steeringConfiguration: TalonFXConfiguration = TalonFXConfiguration()
  private val driveConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  private val potentiometerOutput: Double
    get() {
      return if (label == Constants.Drivetrain.FRONT_RIGHT_MODULE_NAME ||
        label == Constants.Drivetrain.BACK_RIGHT_MODULE_NAME
      ) {
        potentiometer.voltage / RobotController.getVoltage5V() * 2.0 * PI
      } else {
        2 * PI - potentiometer.voltage / RobotController.getVoltage5V() * 2.0 * Math.PI
      }
    }

  val driveStatorCurrentSignal: StatusSignal<Double>
  val driveSupplyCurrentSignal: StatusSignal<Double>
  val steeringStatorCurrentSignal: StatusSignal<Double>
  val steeringSupplyCurrentSignal: StatusSignal<Double>
  val steeringPosition: StatusSignal<Double>
  val driveTempSignal: StatusSignal<Double>
  val steeringTempSignal: StatusSignal<Double>
  val drivePositionQueue: Queue<Double>
  val steeringPositionQueue: Queue<Double>

  init {
    driveFalcon.configurator.apply(TalonFXConfiguration())
    steeringFalcon.configurator.apply(TalonFXConfiguration())

    driveFalcon.clearStickyFaults()
    steeringFalcon.clearStickyFaults()

    steeringConfiguration.Slot0.kP =
      steeringSensor.proportionalPositionGainToRawUnits(DrivetrainConstants.PID.STEERING_KP)
    steeringConfiguration.Slot0.kI =
      steeringSensor.integralPositionGainToRawUnits(DrivetrainConstants.PID.STEERING_KI)
    steeringConfiguration.Slot0.kD =
      steeringSensor.derivativePositionGainToRawUnits(DrivetrainConstants.PID.STEERING_KD)
    steeringConfiguration.Slot0.kV =
      steeringSensor.velocityFeedforwardToRawUnits(DrivetrainConstants.PID.STEERING_KFF)
    steeringConfiguration.MotionMagic.MotionMagicCruiseVelocity =
      steeringSensor.velocityToRawUnits(DrivetrainConstants.STEERING_VEL_MAX)
    steeringConfiguration.MotionMagic.MotionMagicAcceleration =
      steeringSensor.accelerationToRawUnits(DrivetrainConstants.STEERING_ACCEL_MAX)
    steeringConfiguration.CurrentLimits.SupplyCurrentLimit =
      DrivetrainConstants.STEERING_SUPPLY_CURRENT_LIMIT.inAmperes

    steeringConfiguration.ClosedLoopGeneral.ContinuousWrap = true
    steeringConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    steeringConfiguration.Feedback.SensorToMechanismRatio =
      1 / DrivetrainConstants.STEERING_SENSOR_GEAR_RATIO

    steeringConfiguration.MotorOutput.NeutralMode =
      NeutralModeValue.Brake // change back to coast maybe?
    steeringFalcon.inverted = true

    steeringFalcon.configurator.apply(steeringConfiguration)

    driveConfiguration.Slot0.kP =
      driveSensor.proportionalVelocityGainToRawUnits(DrivetrainConstants.PID.DRIVE_KP)
    driveConfiguration.Slot0.kI =
      driveSensor.integralVelocityGainToRawUnits(DrivetrainConstants.PID.DRIVE_KI)
    driveConfiguration.Slot0.kD =
      driveSensor.derivativeVelocityGainToRawUnits(DrivetrainConstants.PID.DRIVE_KD)
    driveConfiguration.Slot0.kV = DrivetrainConstants.PID.DRIVE_KV.inVoltsPerMetersPerSecond
    driveConfiguration.Slot0.kA =
      DrivetrainConstants.PID.DRIVE_KA.inVoltsPerMetersPerSecondPerSecond
    //      driveSensor.velocityFeedforwardToRawUnits(DrivetrainConstants.PID.DRIVE_KFF)
    driveConfiguration.CurrentLimits.SupplyCurrentLimit =
      DrivetrainConstants.DRIVE_SUPPLY_CURRENT_LIMIT.inAmperes
    driveConfiguration.CurrentLimits.SupplyCurrentThreshold =
      DrivetrainConstants.DRIVE_THRESHOLD_CURRENT_LIMIT.inAmperes
    driveConfiguration.CurrentLimits.SupplyTimeThreshold =
      DrivetrainConstants.DRIVE_TRIGGER_THRESHOLD_TIME.inSeconds
    driveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    driveConfiguration.CurrentLimits.StatorCurrentLimit =
      DrivetrainConstants.DRIVE_STATOR_CURRENT_LIMIT.inAmperes
    driveConfiguration.CurrentLimits.StatorCurrentLimitEnable = true // TODO tune

    driveConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
    driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast
    driveFalcon.configurator.apply(driveConfiguration)

    driveStatorCurrentSignal = driveFalcon.statorCurrent
    driveSupplyCurrentSignal = driveFalcon.supplyCurrent
    steeringStatorCurrentSignal = steeringFalcon.statorCurrent
    steeringSupplyCurrentSignal = steeringFalcon.supplyCurrent
    driveTempSignal = driveFalcon.deviceTemp
    steeringTempSignal = steeringFalcon.deviceTemp
    steeringPosition = steeringFalcon.position

    drivePositionQueue =
      PhoenixOdometryThread.getInstance().registerSignal(driveFalcon, driveFalcon.position)
    steeringPositionQueue =
      PhoenixOdometryThread.getInstance()
        .registerSignal(steeringFalcon, steeringFalcon.getPosition())

    MotorChecker.add(
      "Drivetrain",
      "Drive",
      MotorCollection(
        mutableListOf(Falcon500(driveFalcon, "$label Drive Motor")),
        DrivetrainConstants.DRIVE_SUPPLY_CURRENT_LIMIT,
        90.celsius,
        DrivetrainConstants.DRIVE_SUPPLY_CURRENT_LIMIT - 30.amps,
        110.celsius
      )
    )

    MotorChecker.add(
      "Drivetrain",
      "Steering",
      MotorCollection(
        mutableListOf(Falcon500(steeringFalcon, "$label Steering Motor")),
        DrivetrainConstants.STEERING_SUPPLY_CURRENT_LIMIT,
        90.celsius,
        DrivetrainConstants.STEERING_SUPPLY_CURRENT_LIMIT - 10.amps,
        110.celsius
      )
    )
  }

  fun updateSignals() {
    BaseStatusSignal.refreshAll(
      driveStatorCurrentSignal,
      driveSupplyCurrentSignal,
      steeringSupplyCurrentSignal,
      steeringStatorCurrentSignal,
      driveTempSignal,
      steeringTempSignal,
      steeringPosition
    )
  }

  override fun updateInputs(inputs: SwerveModuleIO.SwerveModuleIOInputs) {
    updateSignals()

    inputs.driveAppliedVoltage = (driveFalcon.get() * RobotController.getBatteryVoltage()).volts
    inputs.steeringAppliedVoltage =
      (steeringFalcon.get() * RobotController.getBatteryVoltage()).volts

    inputs.driveStatorCurrent = driveStatorCurrentSignal.value.amps
    inputs.driveSupplyCurrent = driveSupplyCurrentSignal.value.amps
    inputs.steeringStatorCurrent = steeringStatorCurrentSignal.value.amps
    inputs.steeringSupplyCurrent = steeringSupplyCurrentSignal.value.amps

    Logger.recordOutput(
      "$label/drivePosition",
      driveFalcon.position.value *
        (PI) *
        DrivetrainConstants.WHEEL_DIAMETER.inMeters *
        DrivetrainConstants.DRIVE_SENSOR_GEAR_RATIO
    )
    Logger.recordOutput("$label/drivePositionUnits", driveSensor.position.inMeters)
    inputs.drivePosition = driveSensor.position
    inputs.steeringPosition = steeringSensor.position
    Logger.recordOutput("$label/rawSteeringValue", steeringFalcon.position.value)
    Logger.recordOutput("$label/rawSteeringValue", steeringFalcon.position.value)

    steeringFalcon.position.value

    inputs.driveVelocity = driveSensor.velocity
    inputs.steeringVelocity = steeringSensor.velocity

    // processor temp is also something we may want to log ?
    inputs.driveTemp = driveTempSignal.value.celsius
    inputs.steeringTemp = steeringTempSignal.value.celsius

    inputs.odometryDrivePositions = listOf(inputs.drivePosition)
    inputs.odometrySteeringPositions = listOf(inputs.steeringPosition)

    //    inputs.odometryDrivePositions =
    //      drivePositionQueue
    //      .stream()
    //      .map { value: Double ->
    //        (
    //          DrivetrainConstants.WHEEL_DIAMETER * PI * value /
    // DrivetrainConstants.DRIVE_SENSOR_GEAR_RATIO
    //          )
    //      }
    //      .toList() as
    //      List<Length>
    //    inputs.odometrySteeringPositions =
    //      steeringPositionQueue
    //      .stream()
    //      .map { value: Double ->
    //        (value / DrivetrainConstants.STEERING_SENSOR_GEAR_RATIO).rotations
    //      }
    //      .toList() as
    //      List<Angle>
    drivePositionQueue.clear()
    steeringPositionQueue.clear()

    inputs.potentiometerOutputRaw =
      potentiometer.voltage / RobotController.getVoltage5V() * 2.0 * Math.PI
    inputs.potentiometerOutputRadians = potentiometerOutput.radians

    Logger.recordOutput(
      "$label/potentiometerRadiansWithOffset",
      (inputs.potentiometerOutputRadians - zeroOffset).inRadians
    )

    Logger.recordOutput("$label/motorOutput", driveFalcon.get())
  }

  override fun setSteeringSetpoint(angle: Angle) {
    Logger.recordOutput("$label/steeringSetpointDegrees", angle.inDegrees)
    steeringFalcon.setControl(
      PositionDutyCycle(
        steeringSensor.positionToRawUnits(angle),
        steeringSensor.velocityToRawUnits(0.0.radians.perSecond),
        DrivetrainConstants.FOC_ENABLED,
        0.0,
        0,
        false,
        false,
        false
      )
    )
  }

  override fun setClosedLoop(
    steering: Angle,
    speed: LinearVelocity,
    acceleration: LinearAcceleration
  ) {
    val feedforward = DrivetrainConstants.PID.DRIVE_KS * speed.sign
    driveFalcon.setControl(
      VelocityVoltage(
        driveSensor.velocityToRawUnits(speed),
        driveSensor.accelerationToRawUnits(acceleration),
        DrivetrainConstants.FOC_ENABLED,
        feedforward.inVolts,
        0,
        false,
        false,
        false
      )
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
    driveFalcon.setControl(
      DutyCycleOut(
        speed / DrivetrainConstants.DRIVE_SETPOINT_MAX,
        DrivetrainConstants.FOC_ENABLED,
        false,
        false,
        false
      )
    )
    setSteeringSetpoint(steering)
  }

  override fun resetModuleZero() {
    println("Absolute Potentiometer Value $label ($potentiometerOutput")
  }

  override fun zeroSteering(isInAutonomous: Boolean) {
    steeringFalcon.setPosition(
      steeringSensor.positionToRawUnits(
        if (label == Constants.Drivetrain.FRONT_RIGHT_MODULE_NAME ||
          label == Constants.Drivetrain.BACK_RIGHT_MODULE_NAME
        ) {
          (potentiometerOutput.radians - zeroOffset)
        } else {
          (2 * PI).radians - (potentiometerOutput.radians - zeroOffset)
        }
      )
    )

    drivePositionQueue.clear()
    steeringPositionQueue.clear()
    Logger.recordOutput("$label/zeroPositionRadians", steeringSensor.position.inRadians)
    println("Loading Zero for Module $label (${steeringFalcon.position.value})")
  }

  override fun zeroDrive() {
    driveFalcon.setPosition(0.0)
    drivePositionQueue.clear()
    steeringPositionQueue.clear()
  }

  override fun configureDrivePID(
    kP: ProportionalGain<Velocity<Meter>, Volt>,
    kI: IntegralGain<Velocity<Meter>, Volt>,
    kD: DerivativeGain<Velocity<Meter>, Volt>,
    kV: Value<Fraction<Volt, Velocity<Meter>>>,
    kA: Value<Fraction<Volt, Velocity<Velocity<Meter>>>>
  ) {
    val PIDConfig = Slot0Configs()

    PIDConfig.kP = driveSensor.proportionalVelocityGainToRawUnits(kP)
    PIDConfig.kI = driveSensor.integralVelocityGainToRawUnits(kI)
    PIDConfig.kD = driveSensor.derivativeVelocityGainToRawUnits(kD)
    PIDConfig.kV = kV.inVoltsPerMetersPerSecond
    PIDConfig.kA = kA.inVoltsPerMetersPerSecondPerSecond

    driveFalcon.configurator.apply(PIDConfig)
  }

  override fun configureSteeringPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    val PIDConfig = Slot0Configs()

    PIDConfig.kP = steeringSensor.proportionalPositionGainToRawUnits(kP)
    PIDConfig.kI = steeringSensor.integralPositionGainToRawUnits(kI)
    PIDConfig.kD = steeringSensor.derivativePositionGainToRawUnits(kD)
    PIDConfig.kV = 0.0

    steeringFalcon.configurator.apply(PIDConfig)
  }

  override fun configureSteeringMotionMagic(
    maxVel: AngularVelocity,
    maxAccel: AngularAcceleration
  ) {

    val motionMagicConfig = MotionMagicConfigs()

    motionMagicConfig.MotionMagicCruiseVelocity = steeringSensor.velocityToRawUnits(maxVel)
    motionMagicConfig.MotionMagicAcceleration = steeringSensor.accelerationToRawUnits(maxAccel)

    steeringFalcon.configurator.apply(motionMagicConfig)
  }

  override fun setDriveBrakeMode(brake: Boolean) {
    val motorOutputConfig = MotorOutputConfigs()

    if (brake) {
      motorOutputConfig.NeutralMode = NeutralModeValue.Brake
    } else {
      motorOutputConfig.NeutralMode = NeutralModeValue.Coast
    }
    motorOutputConfig.Inverted = InvertedValue.Clockwise_Positive
    driveFalcon.configurator.apply(motorOutputConfig)
  }

  override fun setSteeringBrakeMode(brake: Boolean) {
    val motorOutputConfig = MotorOutputConfigs()

    if (brake) {
      motorOutputConfig.NeutralMode = NeutralModeValue.Brake
    } else {
      motorOutputConfig.NeutralMode = NeutralModeValue.Coast
    }
    steeringFalcon.configurator.apply(motorOutputConfig)
    // motor output configs might overwrite invert?
    steeringFalcon.inverted = true
  }

  override fun runCharacterization(input: ElectricalPotential) {
    if (label == Constants.Drivetrain.FRONT_LEFT_MODULE_NAME) {
      driveFalcon.setControl(DutyCycleOut(-input.inVolts / 12.0))
    } else {
      driveFalcon.setControl(DutyCycleOut(input.inVolts / 12.0))
    }
  }
}
