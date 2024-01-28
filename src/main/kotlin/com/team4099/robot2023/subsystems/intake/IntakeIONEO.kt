package com.team4099.robot2023.subsystems.intake

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.IntakeConstants
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import com.team4099.robot2023.subsystems.falconspin.Neo
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor
import kotlin.math.absoluteValue

object IntakeIONEO : IntakeIO {
  private val rollerSparkMax =
    CANSparkMax(Constants.Intake.ROLLER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val rollerSensor =
    sparkMaxAngularMechanismSensor(
      rollerSparkMax, IntakeConstants.ROLLER_GEAR_RATIO, IntakeConstants.VOLTAGE_COMPENSATION
    )

  init {
    rollerSparkMax.restoreFactoryDefaults()
    rollerSparkMax.clearFaults()

    rollerSparkMax.enableVoltageCompensation(IntakeConstants.VOLTAGE_COMPENSATION.inVolts)
    rollerSparkMax.setSmartCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT.inAmperes.toInt())
    rollerSparkMax.inverted = IntakeConstants.ROLLER_MOTOR_INVERTED

    rollerSparkMax.idleMode = CANSparkMax.IdleMode.kCoast

    rollerSparkMax.burnFlash()

    MotorChecker.add(
      "Ground Intake",
      "Roller",
      MotorCollection(
        mutableListOf(Neo(rollerSparkMax, "Roller Motor")),
        IntakeConstants.ROLLER_CURRENT_LIMIT,
        70.celsius,
        30.amps,
        90.celsius
      ),
    )
  }

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
    inputs.rollerVelocity = rollerSensor.velocity
    inputs.rollerAppliedVoltage = rollerSparkMax.busVoltage.volts * rollerSparkMax.appliedOutput
    inputs.rollerStatorCurrent = rollerSparkMax.outputCurrent.amps

    // BusVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BusVoltage
    // SupplyCurrent = (percentOutput * BusVoltage / BusVoltage) * StatorCurrent =
    // percentOutput * statorCurrent
    inputs.rollerSupplyCurrent =
      inputs.rollerStatorCurrent * rollerSparkMax.appliedOutput.absoluteValue
    inputs.rollerTemp = rollerSparkMax.motorTemperature.celsius

    Logger.recordOutput(
      "Intake/rollerMotorOvercurrentFault",
      rollerSparkMax.getFault(CANSparkMax.FaultID.kOvercurrent)
    )
    Logger.recordOutput("Intake/busVoltage", rollerSparkMax.busVoltage)
  }

  /**
   * Sets the roller motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the roller motor to
   */
  override fun setRollerVoltage(voltage: ElectricalPotential) {
    rollerSparkMax.setVoltage(
      clamp(voltage, -IntakeConstants.VOLTAGE_COMPENSATION, IntakeConstants.VOLTAGE_COMPENSATION)
        .inVolts
    )
  }

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  override fun setRollerBrakeMode(brake: Boolean) {
    if (brake) {
      rollerSparkMax.idleMode = CANSparkMax.IdleMode.kBrake
    } else {
      rollerSparkMax.idleMode = CANSparkMax.IdleMode.kCoast
    }
  }
}
