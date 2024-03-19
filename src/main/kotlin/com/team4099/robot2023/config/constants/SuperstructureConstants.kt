package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.perMinute

object SuperstructureConstants {
  val distanceFlywheelSpeedTableReal =
    listOf(
      Pair(
        FieldConstants.subwooferX + FieldConstants.edgeOfBumperToCenter,
        3000.rotations.perMinute
      ),
      Pair(55.78.inches, 3000.rotations.perMinute),
      Pair(64.9.inches, 3000.rotations.perMinute),
      Pair(74.0.inches, 3000.rotations.perMinute),
      Pair(84.76.inches, 3000.rotations.perMinute),
      Pair(94.2.inches, 3500.rotations.perMinute),
      Pair(104.5.inches, 3500.rotations.perMinute),
      Pair(114.45.inches, 3500.rotations.perMinute),
      Pair(124.15.inches, 3500.rotations.perMinute),
      Pair(128.5.inches, 3500.rotations.perMinute),
      Pair(136.1.inches, 3500.rotations.perMinute),
      Pair(149.3.inches, 4000.rotations.perMinute),
    )

  val distanceWristAngleTableReal =
    listOf(
      Pair(FieldConstants.subwooferX + FieldConstants.edgeOfBumperToCenter, -35.degrees),
      Pair(55.78.inches, -20.degrees),
      Pair(64.9.inches, -16.degrees),
      Pair(74.0.inches, -14.degrees),
      Pair(84.76.inches, -10.degrees),
      Pair(94.2.inches, -7.degrees),
      Pair(104.5.inches, -4.degrees),
      Pair(114.45.inches, -2.degrees),
      Pair(124.15.inches, 0.degrees),
      Pair(128.5.inches, 0.4.degrees),
      Pair(136.1.inches, 1.0.degrees),
      Pair(149.3.inches, 3.6.degrees),
      Pair(0.inches, 0.degrees),
      Pair(0.inches, 0.degrees),
      Pair(0.inches, 0.degrees),
      Pair(0.inches, 0.degrees),
      Pair(0.inches, 0.degrees),
      Pair(0.inches, 0.degrees)
    )

  val distanceFlywheelSpeedTableSim =
    listOf(
      Pair(1.meters, 3000.rotations.perMinute),
      Pair(2.meters, 3500.rotations.perMinute),
      Pair(3.meters, 4000.rotations.perMinute),
      Pair(4.meters, 4500.rotations.perMinute),
      Pair(5.meters, 5000.rotations.perMinute),
      Pair(6.meters, 5500.rotations.perMinute),
      Pair(7.meters, 6000.rotations.perMinute),
      Pair(500.inches, 8000.rotations.perMinute)
    )

  val distanceWristAngleTableSim =
    listOf(
      Pair(46.5.inches, -35.degrees),
      Pair(75.inches, -22.degrees),
      Pair(117.inches, -11.5.degrees),
      Pair(149.inches, -6.5.degrees),
      Pair(191.inches, -2.degrees),
      Pair(253.inches, 1.5.degrees),
      Pair(321.inches, 4.degrees),
      Pair(500.inches, 10.degrees)
    )
}
