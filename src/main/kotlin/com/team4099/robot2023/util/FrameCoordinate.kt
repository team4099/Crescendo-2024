package com.team4099.robot2023.util

import com.team4099.lib.math.asPose2d
import com.team4099.lib.math.asTransform2d
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees

abstract class FrameCoordinate(val x: Length, val y: Length) {

    fun toTransform2d(): Transform2d {
        return Transform2d(Translation2d(x, y), 0.degrees)
    }

    fun toPose2d(): Pose2d {
        return Pose2d(x, y, 0.degrees)
    }

    class OdometryCoordinate(x: Length, y: Length) : FrameCoordinate(x, y) {
        private val pose2d = Pose2d(x, y, 0.degrees)

        constructor (pose2d: Pose2d) : this(pose2d.translation.x, pose2d.translation.y)

        fun toFieldCoordinate(): FieldCoordinate{
            return FieldCoordinate(pose2d.transformBy(odomTField()))
        }
    }

    class FieldCoordinate(x: Length, y: Length) : FrameCoordinate(x, y) {
        private val pose2d = Pose2d(x, y, 0.degrees)

        constructor (pose2d: Pose2d) : this(pose2d.translation.x, pose2d.translation.y)

        fun toOdometryCoordinate(): OdometryCoordinate {
            return OdometryCoordinate(odomTField().asPose2d().transformBy(pose2d.asTransform2d()))
        }
    }

    companion object {
        var odomTField: () -> Transform2d = { Transform2d(Translation2d(0.0.meters, 0.0.meters), 0.0.degrees)}
    }
}
