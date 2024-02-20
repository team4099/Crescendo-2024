package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.perMinute

object SuperstructureConstants {

    val distanceFlywheelSpeedTable = listOf(
        Pair(1.meters, 3000.rotations.perMinute),
        Pair(2.meters, 3500.rotations.perMinute),
        Pair(3.meters, 4000.rotations.perMinute),
        Pair(4.meters, 4500.rotations.perMinute),
        Pair(5.meters, 5000.rotations.perMinute),
        Pair(6.meters, 5500.rotations.perMinute),
        Pair(7.meters, 6000.rotations.perMinute),
        Pair(500.inches, 8000.rotations.perMinute)
    )

    val distanceWristAngleTable = listOf(
        Pair(46.5.inches, -35.degrees),
        Pair(75.inches, -22.degrees),
        Pair(117.inches, -11.5.degrees),
        Pair(149.inches, -6.5.degrees),
        Pair(191.inches, -2.degrees),
        Pair(253.inches, 1.5.degrees),
        Pair(321.inches, 4.degrees),
        Pair(500.inches , 10.degrees)
    )
}