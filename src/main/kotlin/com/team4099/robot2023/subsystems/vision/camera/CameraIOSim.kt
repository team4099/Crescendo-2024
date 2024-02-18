package com.team4099.robot2023.subsystems.vision.camera

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.toDoubleArray
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.util.toPose3d
import com.team4099.robot2023.util.toTransform3d
import edu.wpi.first.networktables.DoubleArraySubscriber
import edu.wpi.first.networktables.NetworkTableInstance
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.radians

class CameraIOSim(val id: String) : CameraIO {
  private val poseSubscriber: DoubleArraySubscriber =
    NetworkTableInstance.getDefault()
      .getTable("AdvantageKit")
      .getSubTable("RealOutputs")
      .getDoubleArrayTopic(VisionConstants.POSE_TOPIC_NAME)
      .subscribe(DoubleArray(3))
  private val errorScaleConstant = 0.05

  override fun updateInputs(inputs: CameraIO.CameraInputs) {
    val drivePose = poseSubscriber.get(DoubleArray(3)).toPose2d().toPose3d()

    inputs.timestamp = Clock.fpgaTime - (Math.random() * 0.2).seconds
    val robotTtag = drivePose.relativeTo(FieldConstants.aprilTags[0].pose)
    inputs.frame = robotTtag.gaussianShenanigans()
    val estimatedRobotPose =
      FieldConstants.aprilTags[0].pose.transformBy(inputs.frame.toTransform3d())

    Logger.recordOutput(
      "Vision/$id/simEstimatedRobotPose",
      estimatedRobotPose.toPose2d().toDoubleArray().toDoubleArray()
    )
    Logger.recordOutput(
      "Vision/$id/simTagPose", FieldConstants.aprilTags[0].pose.toDoubleArray().toDoubleArray()
    )
    inputs.fps = Math.random() * 10 + 30
    inputs.usedTargets = listOf(0)
  }

  private fun DoubleArray.toPose2d(): Pose2d {
    return Pose2d(Translation2d(this[0].meters, this[1].meters), this[2].radians)
  }

  private fun Pose3d.gaussianShenanigans(): Pose3d {
    return Pose3d(
      Translation3d(
        this.translation.x * (1 - Math.random() * errorScaleConstant),
        this.translation.y * (1 - Math.random() * errorScaleConstant),
        this.translation.z * (1 - Math.random() * errorScaleConstant)
      ),
      Rotation3d(
        this.rotation.x * (1 - Math.random() * errorScaleConstant),
        this.rotation.y * (1 - Math.random() * errorScaleConstant),
        this.rotation.z * (1 - Math.random() * errorScaleConstant)
      )
    )
  }
}
