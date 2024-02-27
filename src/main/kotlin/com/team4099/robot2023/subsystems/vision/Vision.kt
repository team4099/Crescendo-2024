package com.team4099.robot2023.subsystems.vision

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.TunableNumber
import com.team4099.lib.vision.TimestampedVisionUpdate
import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.subsystems.vision.camera.CameraIO
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose2dWPILIB
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Pose3dWPILIB
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import java.util.function.Consumer
import java.util.function.Supplier
import kotlin.math.pow

class Vision(vararg cameras: CameraIO) : SubsystemBase() {
  val io: List<CameraIO> = cameras.toList()
  val inputs = List(io.size) { CameraIO.CameraInputs() }

  companion object {
    val ambiguityThreshold = 0.7
    val targetLogTime = 0.05.seconds
    val cameraPoses = VisionConstants.CAMERA_TRANSFORMS

    val xyStdDevCoeffecient = 0.05
    val thetaStdDevCoefficient = 1.5
  }

  private val xyStdDevCoefficient = TunableNumber("Vision/xystdev", xyStdDevCoeffecient)

  private val thetaStdDev = TunableNumber("Vision/thetaStdDev", thetaStdDevCoefficient)

  private var fieldFramePoseSupplier = Supplier<Pose2d> { Pose2d() }
  private var visionConsumer: Consumer<List<TimestampedVisionUpdate>> = Consumer {}
  private val lastFrameTimes = mutableMapOf<Int, Time>()
  private val lastTagDetectionTimes = mutableMapOf<Int, Time>()

  init {
    for (i in io.indices) {
      lastFrameTimes[i] = 0.0.seconds
    }
  }

  fun setDataInterfaces(
    fieldFramePoseSupplier: Supplier<Pose2d>,
    visionConsumer: Consumer<List<TimestampedVisionUpdate>>
  ) {
    this.fieldFramePoseSupplier = fieldFramePoseSupplier
    this.visionConsumer = visionConsumer
  }

  override fun periodic() {
    //    val tuningPosition = Pose3d(Pose3d(
    //      (43.125).inches,
    //      (108.375).inches,
    //      (18.22).inches,
    //      Rotation3d(0.0.radians, 0.0.radians, 0.0.radians)
    //    ).translation  + (Translation3d(45.625.inches, 1.3125.inches, 0.0.inches)),
    // Rotation3d()).toPose2d()
    //
    //    Logger.recordOutput("Vision/tuningPosition", tuningPosition.pose2d)

    val startTime = Clock.realTimestamp

    for (instance in io.indices) {
      io[instance].updateInputs(inputs[instance])
      Logger.processInputs("Vision/${VisionConstants.CAMERA_NAMES[instance]}", inputs[instance])
    }

    var fieldTCurrentRobotEstimate: Pose2d = fieldFramePoseSupplier.get()
    val robotPoses = mutableListOf<Pose2d>()
    val visionUpdates = mutableListOf<TimestampedVisionUpdate>()

    for (instance in io.indices) {

      lastFrameTimes[instance] = Clock.fpgaTime
      val timestamp = inputs[instance].timestamp
      val values = inputs[instance].frame

      var cameraPose: Pose3d? = inputs[instance].frame
      var robotPose: Pose2d? = cameraPose?.transformBy(cameraPoses[instance])?.toPose2d()

      if (cameraPose == null || robotPose == null) {
        continue
      }

      if ((robotPose.rotation - fieldFramePoseSupplier.get().rotation).absoluteValue > 7.degrees &&
        DriverStation.isEnabled()
      ) {
        continue
      }

      // Find all detected tag poses
      val tagPoses = inputs[instance].usedTargets.map { FieldConstants.fieldAprilTags[it].pose }

      // Calculate average distance to tag
      var totalDistance = 0.0.meters
      for (tagPose in tagPoses) {
        totalDistance += tagPose.translation.getDistance(cameraPose.translation)
      }
      val averageDistance = totalDistance / tagPoses.size

      // Add to vision updates
      val xyStdDev = xyStdDevCoefficient.get() * averageDistance.inMeters.pow(2) / tagPoses.size
      val thetaStdDev = thetaStdDev.get() * averageDistance.inMeters.pow(2) / tagPoses.size

      visionUpdates.add(
        TimestampedVisionUpdate(
          timestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
        )
      )
      robotPoses.add(robotPose)

      Logger.recordOutput(
        "Vision/${VisionConstants.CAMERA_NAMES[instance]}/latencyMS",
        (Clock.fpgaTime - timestamp).inMilliseconds
      )

      Logger.recordOutput(
        "Vision/${VisionConstants.CAMERA_NAMES[instance]}/estimatedRobotPose", robotPose.pose2d
      )

      Logger.recordOutput(
        "Vision/${VisionConstants.CAMERA_NAMES[instance]}/tagPoses",
        *tagPoses.map { it.pose3d }.toTypedArray()
      )

      if (inputs[instance].timestamp == 0.0.seconds) { // prolly wrong lol
        Logger.recordOutput(
          "Vision/${VisionConstants.CAMERA_NAMES[instance]}/estimatedRobotPose",
          Pose2dWPILIB.struct,
          Pose2d().pose2d
        )
      }

      if (Clock.fpgaTime - lastFrameTimes[instance]!! > targetLogTime) {
        Logger.recordOutput(
          "Vision/${VisionConstants.CAMERA_NAMES[instance]}/tagPoses",
          Pose3dWPILIB.struct,
          *arrayOf<Pose3dWPILIB>()
        )
      }

      val allTagPoses = mutableListOf<Pose3d>()
      //    for (detectionEntry in lastTagDetectionTimes.entries) {
      //      if (Clock.fpgaTime - detectionEntry.value < targetLogTime) {
      //        FieldConstants.getTagPose(detectionEntry.key)?.let { allTagPoses.add(it) }
      //      }
      //    }

      Logger.recordOutput(
        "Vision/allTagPoses", Pose3dWPILIB.struct, *allTagPoses.map { it.pose3d }.toTypedArray()
      )

      visionConsumer.accept(visionUpdates)

      Logger.recordOutput(
        "LoggedRobot/Subsystems/VisionLoopTimeMS",
        (Clock.realTimestamp - startTime).inMilliseconds
      )
    }
  }
}
