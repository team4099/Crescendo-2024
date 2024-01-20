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
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.perMinute

object FlywheelIOSim: FlywheelIO {
    private val leftFlywheelSim: FlywheelSim = FlywheelSim(
        DCMotor.getNEO(1),
        FlywheelConstants.LEFT_GEAR_RATIO,
        FlywheelConstants.LEFT_INERTIA.inKilogramsMeterSquared
    )


    private val rightFlywheelSim: FlywheelSim = FlywheelSim(
        DCMotor.getNEO(1),
        FlywheelConstants.RIGHT_GEAR_RATIO,
        FlywheelConstants.RIGHT_INERTIA.inKilogramsMeterSquared
    )

    private var leftAppliedVoltage = 0.volts
    private var rightAppliedVoltage = 0.volts

    private val flywheelController =
        PIDController(
            FlywheelConstants.PID.SIM_KP,
            FlywheelConstants.PID.SIM_KI,
            FlywheelConstants.PID.SIM_KD
        )

    override fun updateInputs(inputs: FlywheelIO.FlywheelIOInputs) {

        leftFlywheelSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
        rightFlywheelSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

        inputs.leftFlywheelVelocity = leftFlywheelSim.getAngularVelocityRPM().rotations.perMinute
        inputs.leftFlywheelVelocity = leftFlywheelSim.getAngularVelocityRPM().rotations.perMinute
        inputs.leftFlywheelAppliedVoltage = leftAppliedVoltage
        inputs.leftFlywheelSupplyCurrent = 0.amps
        inputs.leftFlywheelStatorCurrent = leftFlywheelSim.currentDrawAmps.amps
        inputs.leftFlywheelTemperature = 0.0.celsius

        inputs.rightFlywheelVelocity = rightFlywheelSim.getAngularVelocityRPM().rotations.perMinute
        inputs.rightFlywheelVelocity = rightFlywheelSim.getAngularVelocityRPM().rotations.perMinute
        inputs.rightFlywheelAppliedVoltage = rightAppliedVoltage
        inputs.rightFlywheelSupplyCurrent = 0.amps
        inputs.rightFlywheelStatorCurrent = rightFlywheelSim.currentDrawAmps.amps
        inputs.rightFlywheelTemperature = 0.0.celsius

        inputs.isSimulated = true
    }

    override fun setLeftFlywheelVoltage(leftVoltage: ElectricalPotential) {
        leftAppliedVoltage = leftVoltage
        leftFlywheelSim.setInputVoltage(
            clamp(
                leftVoltage,
                -FlywheelConstants.VOLTAGE_COMPENSATION,
                FlywheelConstants.VOLTAGE_COMPENSATION
            )
                .inVolts
        )
    }

    override fun setRightFlywheelVoltage(rightVoltage: ElectricalPotential) {
        leftAppliedVoltage = rightVoltage
        leftFlywheelSim.setInputVoltage(
            clamp(
                rightVoltage,
                -FlywheelConstants.VOLTAGE_COMPENSATION,
                FlywheelConstants.VOLTAGE_COMPENSATION
            )
                .inVolts
        )
    }

    override fun setLeftFlywheelVelocity(velocity: AngularVelocity, feedforward: ElectricalPotential) {
        val feedback = flywheelController.calculate(leftFlywheelSim.getAngularVelocityRPM().rotations.perMinute, velocity)
        setLeftFlywheelVoltage(feedback + feedforward)
    }

    override fun setRightFlywheelVelocity(velocity: AngularVelocity, feedforward: ElectricalPotential) {
        val feedback = flywheelController.calculate(rightFlywheelSim.getAngularVelocityRPM().rotations.perMinute, velocity)
        setRightFlywheelVoltage(feedback + feedforward)
    }

    override fun setLeftFlywheelBrakeMode(brake: Boolean) {}

    override fun setRightFlywheelBrakeMode(brake: Boolean) {}
}