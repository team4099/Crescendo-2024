package com.team4099.robot2023.util

import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.derived.degrees

abstract class FrameCoordinate(val x: Length, val y: Length) {

    fun toTransform2d(): Transform2d {
        return Transform2d(Translation2d(x, y), 0.degrees)
    }

    fun toPose2d(): Pose2d {
        return Pose2d(x, y, 0.degrees)
    }

    class OdometryCoordinate(x: Length, y: Length) : FrameCoordinate(x, y) {
        constructor (pose2d: Pose2d) : this(pose2d.translation.x, pose2d.translation.y)
    }

    class FieldCoordinate(x: Length, y: Length) : FrameCoordinate(x, y) {
        constructor (pose2d: Pose2d) : this(pose2d.translation.x, pose2d.translation.y)
    }

    companion object {
        lateinit var robotInOdometryFrameSupplier: () -> Pose2d
    }
}
