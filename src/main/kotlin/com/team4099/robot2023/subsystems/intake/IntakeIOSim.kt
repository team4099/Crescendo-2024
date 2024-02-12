package com.team4099.robot2023.subsystems.intake

import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.IntakeConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute

object IntakeIOSim : IntakeIO {
  private val rollerSim: FlywheelSim =
    FlywheelSim(
      DCMotor.getNEO(1), IntakeConstants.ROLLER_GEAR_RATIO, IntakeConstants.ROLLER_INERTIA
    )

  private var appliedVoltage = 0.volts
  init {}

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {

    rollerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.rollerVelocity = rollerSim.getAngularVelocityRPM().rotations.perMinute
    inputs.rollerAppliedVoltage = appliedVoltage
    inputs.rollerSupplyCurrent = 0.amps
    inputs.rollerStatorCurrent = rollerSim.currentDrawAmps.amps
    inputs.rollerTemp = 0.0.celsius

    inputs.isSimulated = true
  }

  override fun setRollerVoltage(voltage: ElectricalPotential) {
    appliedVoltage = voltage
    rollerSim.setInputVoltage(
      clamp(voltage, -IntakeConstants.VOLTAGE_COMPENSATION, IntakeConstants.VOLTAGE_COMPENSATION)
        .inVolts
    )
  }

  override fun setRollerBrakeMode(brake: Boolean) {}
}
