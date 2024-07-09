package com.team4099.robot2023.subsystems.drivetrain.swervemodule

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.DrivetrainConstants
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerMeters
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecond
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecondPerSecond
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perMeterPerSecond
import org.team4099.lib.units.derived.perMeterPerSecondPerSecond
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object SwerveTunableValues {

  val steerMaxAccel =
    LoggedTunableValue("Drivetrain/steerMaxAcceleration", DrivetrainConstants.STEERING_ACCEL_MAX)
  val steerMaxVelo =
    LoggedTunableValue("Drivetrain/steerMaxVelocity", DrivetrainConstants.STEERING_VEL_MAX)

  val steerkP =
    LoggedTunableValue(
      "Drivetrain/modulesteerkP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree })
    )
  val steerkI =
    LoggedTunableValue(
      "Drivetrain/modulesteerkI",
      Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  val steerkD =
    LoggedTunableValue(
      "Drivetrain/modulesteerkD",
      Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  val drivekP =
    LoggedTunableValue(
      "Drivetrain/drivekP",
      Pair({ it.inVoltsPerMetersPerSecond }, { it.volts.perMeterPerSecond })
    )
  val drivekI =
    LoggedTunableValue(
      "Drivetrain/drivekI",
      Pair({ it.inVoltsPerMeters }, { it.volts / (1.meters.perSecond * 1.seconds) })
    )
  val drivekD =
    LoggedTunableValue(
      "Drivetrain/drivekD",
      Pair({ it.inVoltsPerMetersPerSecondPerSecond }, { it.volts.perMeterPerSecondPerSecond })
    )

  val drivekV =
    LoggedTunableValue(
      "Drivetrain/drivekV",
      Pair({ it.inVoltsPerMetersPerSecond }, { it.volts.perMeterPerSecond })
    )

  val drivekA =
    LoggedTunableValue(
      "Drivetrain/drivekA",
      Pair({ it.inVoltsPerMetersPerSecondPerSecond }, { it.volts.perMeterPerSecondPerSecond })
    )
}
