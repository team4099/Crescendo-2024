package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.perMinute

object SuperstructureConstants {

    val distanceFlywheelSpeedTable = listOf(
        Pair(1.meters, 2000.rotations.perMinute),
        Pair(2.meters, 2000.rotations.perMinute),
        Pair(3.meters, 3000.rotations.perMinute),
        Pair(4.meters, 3000.rotations.perMinute),
        Pair(5.meters, 4000.rotations.perMinute),
        Pair(6.meters, 4000.rotations.perMinute),
        Pair(7.meters, 5000.rotations.perMinute)
    )

    val distanceWristAngleTable = listOf(
        Pair(1.meters, -35.degrees),
        Pair(2.meters, -30.degrees),
        Pair(3.meters, -25.degrees),
        Pair(4.meters, -20.degrees),
        Pair(5.meters, -15.degrees),
        Pair(6.meters, -10.degrees),
        Pair(7.meters, -5.degrees)
    )
}