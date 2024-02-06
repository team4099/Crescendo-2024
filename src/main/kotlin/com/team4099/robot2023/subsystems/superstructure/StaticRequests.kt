package com.team4099.robot2023.subsystems.superstructure

import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond

object StaticRequests {
    object Drivetrain {
        val openLoopToZero = Request.DrivetrainRequest.OpenLoop(
            0.0.radians.perSecond, Pair(0.0.meters.perSecond, 0.0.meters.perSecond)
        )
    }
}