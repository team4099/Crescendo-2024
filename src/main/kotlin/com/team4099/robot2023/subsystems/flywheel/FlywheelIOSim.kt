package com.team4099.robot2023.subsystems.flywheel

import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FlywheelConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute

object FlywheelIOSim : FlywheelIO {
  private val flywheelRightSim: FlywheelSim =
    FlywheelSim(
      DCMotor.getKrakenX60Foc(2),
      FlywheelConstants.RIGHT_MOTOR_REVOLUTIONS_PER_FLYWHEEL_REVOLUTIONS,
      FlywheelConstants.INERTIA.inKilogramsMeterSquared
    )

  private val flywheelLeftSim: FlywheelSim =
    FlywheelSim(
      DCMotor.getKrakenX60Foc(2),
      FlywheelConstants.LEFT_GEAR_RATIO,
      FlywheelConstants.INERTIA.inKilogramsMeterSquared
    )

  private var appliedRightVoltage = 0.0.volts
  private var appliedLeftVoltage = 0.0.volts

  private val rightFlywheelController =
    PIDController(
      FlywheelConstants.PID.RIGHT_SIM_KP,
      FlywheelConstants.PID.RIGHT_SIM_KI,
      FlywheelConstants.PID.RIGHT_SIM_KD
    )

  private val leftFlywheelController =
    PIDController(
      FlywheelConstants.PID.LEFT_SIM_KP,
      FlywheelConstants.PID.LEFT_SIM_KI,
      FlywheelConstants.PID.LEFT_SIM_KD
    )

  override fun updateInputs(inputs: FlywheelIO.FlywheelIOInputs) {
    flywheelLeftSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    flywheelRightSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.leftFlywheelVelocity = flywheelLeftSim.getAngularVelocityRPM().rotations.perMinute
    inputs.leftFlywheelAppliedVoltage = appliedLeftVoltage
    inputs.leftFlywheelSupplyCurrent = 0.amps
    inputs.leftFlywheelStatorCurrent = flywheelLeftSim.currentDrawAmps.amps
    inputs.leftFlywheelTemperature = 0.0.celsius

    inputs.rightFlywheelVelocity = flywheelRightSim.getAngularVelocityRPM().rotations.perMinute
    inputs.rightFlywheelAppliedVoltage = appliedRightVoltage
    inputs.rightFlywheelSupplyCurrent = 0.amps
    inputs.rightFlywheelStatorCurrent = flywheelRightSim.currentDrawAmps.amps
    inputs.rightFlywheelTemperature = 0.0.celsius

    inputs.isSimulated = true
  }

  override fun setFlywheelVoltage(
    voltageRight: ElectricalPotential,
    voltageLeft: ElectricalPotential
  ) {
    appliedRightVoltage = voltageRight
    flywheelRightSim.setInputVoltage(
      clamp(
        voltageRight,
        -FlywheelConstants.VOLTAGE_COMPENSATION,
        FlywheelConstants.VOLTAGE_COMPENSATION
      )
        .inVolts
    )

    appliedLeftVoltage = voltageLeft
    flywheelLeftSim.setInputVoltage(
      clamp(
        voltageLeft,
        -FlywheelConstants.VOLTAGE_COMPENSATION,
        FlywheelConstants.VOLTAGE_COMPENSATION
      )
        .inVolts
    )
  }

  override fun setFlywheelVelocity(
    rightVelocity: AngularVelocity,
    leftVelocity: AngularVelocity,
    feedforwardLeft: ElectricalPotential,
    feedforwardRight: ElectricalPotential
  ) {
    val rightFeedback =
      rightFlywheelController.calculate(
        flywheelRightSim.getAngularVelocityRPM().rotations.perMinute, rightVelocity
      )
    val leftFeedback =
      leftFlywheelController.calculate(
        flywheelLeftSim.getAngularVelocityRPM().rotations.perMinute, leftVelocity
      )

    setFlywheelVoltage(rightFeedback + feedforwardRight, leftFeedback + feedforwardLeft)

    appliedRightVoltage = rightFeedback + feedforwardRight
    appliedLeftVoltage = leftFeedback + feedforwardLeft
  }

  override fun setFlywheelBrakeMode(brake: Boolean) {}

  override fun configPID(
    rightkP: ProportionalGain<Velocity<Radian>, Volt>,
    rightkI: IntegralGain<Velocity<Radian>, Volt>,
    rightkD: DerivativeGain<Velocity<Radian>, Volt>,
    leftkP: ProportionalGain<Velocity<Radian>, Volt>,
    leftkI: IntegralGain<Velocity<Radian>, Volt>,
    leftkD: DerivativeGain<Velocity<Radian>, Volt>
  ) {
    rightFlywheelController.setPID(rightkP, rightkI, rightkD)

    leftFlywheelController.setPID(leftkP, leftkI, leftkD)
  }
}
