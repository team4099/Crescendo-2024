package com.team4099.robot2023.subsystems.drivetrain.swervemodule

import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import com.team4099.robot2023.subsystems.falconspin.SimulatedMotor
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.PIDController
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRotations
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond

class SwerveModuleIOSim(override val label: String) : SwerveModuleIO {
  // Use inverses of gear ratios because our standard is <1 is reduction
  val driveMotorSim: FlywheelSim =
    FlywheelSim(
      DCMotor.getNEO(1),
      1 / DrivetrainConstants.DRIVE_SENSOR_GEAR_RATIO,
      DrivetrainConstants.DRIVE_WHEEL_INERTIA.inKilogramsMeterSquared
    )

  val steerMotorSim =
    FlywheelSim(
      DCMotor.getNEO(1),
      1 / DrivetrainConstants.STEERING_SENSOR_GEAR_RATIO,
      DrivetrainConstants.STEERING_WHEEL_INERTIA.inKilogramsMeterSquared
    )

  var turnRelativePosition = 0.0.radians
  var turnAbsolutePosition =
    (Math.random() * 2.0 * Math.PI).radians // getting a random value that we zero to
  var driveVelocity = 0.0.meters.perSecond

  private val driveFeedback =
    PIDController(
      DrivetrainConstants.PID.SIM_DRIVE_KP,
      DrivetrainConstants.PID.SIM_DRIVE_KI,
      DrivetrainConstants.PID.SIM_DRIVE_KD,
      Constants.Universal.LOOP_PERIOD_TIME
    )
  private val driveFeedForward =
    SimpleMotorFeedforward(
      DrivetrainConstants.PID.SIM_DRIVE_KS,
      DrivetrainConstants.PID.SIM_DRIVE_KV,
      DrivetrainConstants.PID.SIM_DRIVE_KA
    )

  private val steeringFeedback =
    PIDController(
      DrivetrainConstants.PID.SIM_STEERING_KP,
      DrivetrainConstants.PID.SIM_STEERING_KI,
      DrivetrainConstants.PID.SIM_STEERING_KD,
      Constants.Universal.LOOP_PERIOD_TIME
    )

  init {
    steeringFeedback.enableContinuousInput(-Math.PI.radians, Math.PI.radians)
    steeringFeedback.errorTolerance = DrivetrainConstants.ALLOWED_STEERING_ANGLE_ERROR

    MotorChecker.add(
      "Drivetrain",
      "Drive",
      MotorCollection(
        mutableListOf(SimulatedMotor(driveMotorSim, "$label Drive Motor")),
        65.amps,
        90.celsius,
        45.amps,
        100.celsius
      )
    )

    MotorChecker.add(
      "Drivetrain",
      "Steering",
      MotorCollection(
        mutableListOf(SimulatedMotor(steerMotorSim, "$label Steering Motor")),
        65.amps,
        90.celsius,
        45.amps,
        100.celsius
      )
    )
  }

  var driveAppliedVolts = 0.0.volts
  var turnAppliedVolts = 0.0.volts

  override fun updateInputs(inputs: SwerveModuleIO.SwerveModuleIOInputs) {
    driveMotorSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    steerMotorSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    val angleDifference: Angle =
      (steerMotorSim.angularVelocityRadPerSec * Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
        .radians
    turnAbsolutePosition += angleDifference
    turnRelativePosition += angleDifference

    // constrains it to 2pi radians
    while (turnAbsolutePosition < 0.radians) {
      turnAbsolutePosition += (2.0 * Math.PI).radians
    }
    while (turnAbsolutePosition > (2.0 * Math.PI).radians) {
      turnAbsolutePosition -= (2.0 * Math.PI).radians
    }

    // s = r * theta -> d/2 * rad/s = m/s
    driveVelocity =
      (DrivetrainConstants.WHEEL_DIAMETER / 2 * driveMotorSim.angularVelocityRadPerSec).perSecond

    // pi * d * rotations = distance travelled
    inputs.drivePosition +=
      DrivetrainConstants.WHEEL_DIAMETER *
      Math.PI *
      (
        driveMotorSim.angularVelocityRadPerSec *
          Constants.Universal.LOOP_PERIOD_TIME.inSeconds
        )
        .radians
        .inRotations
    inputs.steerPosition = turnAbsolutePosition

    inputs.driveVelocity = driveVelocity
    inputs.steerVelocity = steerMotorSim.angularVelocityRadPerSec.radians.perSecond

    inputs.driveAppliedVoltage = driveAppliedVolts
    inputs.supplyCurrentDrive = driveMotorSim.currentDrawAmps.amps
    inputs.statorCurrentDrive =
      (-1337).amps // no way to get applied voltage to motor so can't actually calculate stator
    // current

    inputs.driveTemp = (-1337).celsius
    inputs.steerTemp = (-1337).celsius

    inputs.steerAppliedVoltage = turnAppliedVolts
    inputs.supplyCurrentSteer = steerMotorSim.currentDrawAmps.amps
    inputs.statorCurrentSteer =
      (-1337).amps // no way to get applied voltage to motor so can't actually calculate stator
    // current

    inputs.potentiometerOutputRadians = turnAbsolutePosition
    inputs.potentiometerOutputRaw = turnAbsolutePosition.inRadians

    inputs.driveOdometryPos = listOf(inputs.drivePosition)
    inputs.steerOdometryPos = listOf(inputs.steerPosition)

    // Setting a more accurate simulated voltage under load
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        inputs.supplyCurrentDrive.inAmperes + inputs.supplyCurrentSteer.inAmperes
      )
    )
  }

  // helper functions to clamp all inputs and set sim motor voltages properly
  private fun setDriveVoltage(volts: ElectricalPotential) {
    driveAppliedVolts = clamp(volts, -12.0.volts, 12.0.volts)
    driveMotorSim.setInputVoltage(driveAppliedVolts.inVolts)
  }

  private fun setSteeringVoltage(volts: ElectricalPotential) {
    turnAppliedVolts = clamp(volts, -12.0.volts, 12.0.volts)
    steerMotorSim.setInputVoltage(turnAppliedVolts.inVolts)
  }

  override fun setSteeringSetpoint(angle: Angle) {
    val feedback = steeringFeedback.calculate(turnAbsolutePosition, angle)
    Logger.recordOutput("Drivetrain/PID/steeringFeedback", feedback.inVolts)
    Logger.recordOutput("Drivetrain/PID/kP", steeringFeedback.proportionalGain.inVoltsPerDegree)
    Logger.recordOutput("Drivetrain/PID/kI", steeringFeedback.integralGain.inVoltsPerDegreeSeconds)
    Logger.recordOutput(
      "Drivetrain/PID/kD", steeringFeedback.derivativeGain.inVoltsPerDegreePerSecond
    )
    setSteeringVoltage(feedback)
  }

  override fun setClosedLoop(
    steering: Angle,
    speed: LinearVelocity,
    acceleration: LinearAcceleration
  ) {
    Logger.recordOutput("$label/desiredDriveSpeedMPS", speed.inMetersPerSecond)
    val feedforward = driveFeedForward.calculate(speed, acceleration)
    val feedback = driveFeedback.calculate(driveVelocity, speed)

    setDriveVoltage(feedforward + feedback)
    setSteeringSetpoint(steering)
  }

  override fun setOpenLoop(steering: Angle, speed: LinearVelocity) {
    setDriveVoltage(
      RoboRioSim.getVInVoltage().volts * (speed / DrivetrainConstants.DRIVE_SETPOINT_MAX)
    )
    setSteeringSetpoint(steering)
  }

  override fun resetModuleZero() {
    println("Resetting your module's 0 doesn't do anything meaningful in sim :(")
  }

  override fun zeroDrive() {
    println("Zero drive do anything meaningful in sim")
  }

  override fun zeroSteering(isInAutonomous: Boolean) {
    turnAbsolutePosition = 0.0.radians
  }

  override fun configureDrivePID(
    kP: ProportionalGain<Velocity<Meter>, Volt>,
    kI: IntegralGain<Velocity<Meter>, Volt>,
    kD: DerivativeGain<Velocity<Meter>, Volt>,
    // kV: Value<Fraction<Volt, Velocity<Meter>>>,
    // kA: Value<Fraction<Volt, Velocity<Velocity<Meter>>>>
  ) {
    driveFeedback.setPID(kP, kI, kD)
  }

  override fun configureSteerPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    steeringFeedback.setPID(kP, kI, kD)
  }

  override fun setDriveBrakeMode(brake: Boolean) {
    DriverStation.reportError("Can't set brake mode in simulation", true)
  }

  override fun configSteerMotionMagic(maxVel: AngularVelocity, maxAccel: AngularAcceleration) {
    DriverStation.reportError("Can't configure motion magic in simulation", true)
  }

  override fun runCharacterization(input: ElectricalPotential) {
    val appliedVolts = MathUtil.clamp(input.inVolts, -12.0, 12.0)
    driveMotorSim.setInputVoltage(appliedVolts)
  }
}
