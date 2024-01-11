package com.team4099.robot2023.subsystems.intake

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableNumber
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.IntakeConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj.RobotBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond

class Intake(private val io: IntakeIO) {
    val inputs = IntakeIO.IntakeIOInputs()

    lateinit var armFeedforward: ArmFeedforward

    private val kP = LoggedTunableValue("Intake/kP", Pair({it.inVoltsPerDegree}, {it.volts.perDegree}))
    private val kI = LoggedTunableValue("Intake/kI", Pair({it.inVoltsPerDegreeSeconds}, {it.volts.perDegreeSeconds}))
    private val kD = LoggedTunableValue("Intake/kD", Pair({it.inVoltsPerDegreePerSecond}, {it.volts.perDegreePerSecond}))

    object TunableGroundIntakeStates {
        val enableArm =
            LoggedTunableNumber(
                "Intake/enableArmIntake",
                IntakeConstants.ENABLE_ARM
            )
        val enableRotation =
            LoggedTunableNumber(
                "Intake/enableRotationIntake",
                IntakeConstants.ENABLE_ROTATION
            )
        val stowedUpAngle =
            LoggedTunableValue(
                "Intake/stowedUpAngle",
                IntakeConstants.STOWED_UP_ANGLE,
                Pair({ it.inDegrees }, { it.degrees })
            )
        val stowedDownAngle =
            LoggedTunableValue(
                "Intake/stowedDownAngle",
                IntakeConstants.STOWED_DOWN_ANGLE,
                Pair({ it.inDegrees }, { it.degrees })
            )
        val intakeVoltage =
            LoggedTunableValue(
                "Intake/intakeVoltage",
                IntakeConstants.INTAKE_VOLTAGE,
                Pair({ it.inVolts }, { it.volts })
            )
        val neutralVoltage =
            LoggedTunableValue(
                "Intake/neutralVoltage",
                IntakeConstants.NEUTRAL_VOLTAGE,
                Pair({ it.inVolts }, { it.volts })
            )
    }
}
