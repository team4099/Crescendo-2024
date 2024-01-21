package com.team4099.robot2023.subsystems.flywheel

import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FlywheelConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute

object FlywheelIOSim : FlywheelIO {
  private val flywheelSim: FlywheelSim =
    FlywheelSim(
      DCMotor.getNEO(1),
      FlywheelConstants.LEFT_GEAR_RATIO,
      FlywheelConstants.INERTIA.inKilogramsMeterSquared
    )

  private var appliedVoltage = 0.volts

  private val flywheelController =
    PIDController(
      FlywheelConstants.PID.SIM_KP, FlywheelConstants.PID.SIM_KI, FlywheelConstants.PID.SIM_KD
    )

  override fun updateInputs(inputs: FlywheelIO.FlywheelIOInputs) {

    flywheelSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.leftFlywheelVelocity = flywheelSim.getAngularVelocityRPM().rotations.perMinute
    inputs.leftFlywheelVelocity = flywheelSim.getAngularVelocityRPM().rotations.perMinute
    inputs.leftFlywheelAppliedVoltage = appliedVoltage
    inputs.leftFlywheelSupplyCurrent = 0.amps
    inputs.leftFlywheelStatorCurrent = flywheelSim.currentDrawAmps.amps
    inputs.leftFlywheelTemperature = 0.0.celsius

    inputs.rightFlywheelVelocity = flywheelSim.getAngularVelocityRPM().rotations.perMinute
    inputs.rightFlywheelVelocity = flywheelSim.getAngularVelocityRPM().rotations.perMinute
    inputs.rightFlywheelAppliedVoltage = appliedVoltage
    inputs.rightFlywheelSupplyCurrent = 0.amps
    inputs.rightFlywheelStatorCurrent = flywheelSim.currentDrawAmps.amps
    inputs.rightFlywheelTemperature = 0.0.celsius

    inputs.isSimulated = true
  }

  override fun setFlywheelVoltage(leftVoltage: ElectricalPotential) {
    appliedVoltage = leftVoltage
    flywheelSim.setInputVoltage(
      clamp(
        leftVoltage,
        -FlywheelConstants.VOLTAGE_COMPENSATION,
        FlywheelConstants.VOLTAGE_COMPENSATION
      )
        .inVolts
    )
  }

  override fun setFlywheelVelocity(velocity: AngularVelocity, feedforward: ElectricalPotential) {
    val feedback =
      flywheelController.calculate(
        flywheelSim.getAngularVelocityRPM().rotations.perMinute, velocity
      )
    setFlywheelVoltage(feedback + feedforward)
  }

  override fun setFlywheelBrakeMode(brake: Boolean) {}
}
