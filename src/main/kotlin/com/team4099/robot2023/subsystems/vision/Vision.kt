package com.team4099.robot2023.subsystems.vision

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.TunableNumber
import com.team4099.lib.logging.toDoubleArray
import com.team4099.lib.vision.TimestampedVisionUpdate
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.subsystems.vision.camera.CameraIO
import com.team4099.robot2023.util.FMSData
import com.team4099.robot2023.util.LimelightReading
import com.team4099.robot2023.util.rotateBy
import com.team4099.robot2023.util.toPose3d
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose2dWPILIB
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Pose3dWPILIB
import org.team4099.lib.geometry.Quaternion
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Rotation3dWPILIB
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.geometry.Translation3dWPILIB
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.tan
import java.util.function.Consumer
import java.util.function.Supplier
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.math.tan

class Vision(vararg cameras: CameraIO) : SubsystemBase() {
  val io: List<CameraIO> = cameras.toList()
  val inputs = List(io.size) { CameraIO.CameraInputs() }

  var drivetrainOdometry: () -> Pose2d = { Pose2d() }

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
      var tagTargets = inputs[instance].cameraTargets

      println(tagTargets.size)

      val cornerData = mutableListOf<Double>()

      for (tag in tagTargets) {
        if (
          (tag.fiducialId in intArrayOf(3, 4) && !FMSData.isBlue) ||
          (tag.fiducialId in intArrayOf(7, 8) && FMSData.isBlue)
        ) { // i made the tag IDS up

          for (corner in 0 until tag.detectedCorners.size){
            val px = tag.detectedCorners[corner].x
            val py = tag.detectedCorners[corner].y

            val px2 = tag.detectedCorners[corner+1].x
            val py2 = tag.detectedCorners[corner+1].y


            val nx = -(1.0 / (VisionConstants.CAMERA_OV2387.CAMERA_PX / 2)) * (px - (VisionConstants.CAMERA_OV2387.CAMERA_PX / 2) - 0.5)
            val ny = (1.0 / (VisionConstants.CAMERA_OV2387.CAMERA_PY / 2)) * ((VisionConstants.CAMERA_OV2387.CAMERA_PY / 2) - 0.5 - py)

            val nx2 = -(1.0 / (VisionConstants.CAMERA_OV2387.CAMERA_PX / 2)) * (px2 - (VisionConstants.CAMERA_OV2387.CAMERA_PX / 2) - 0.5)

            val cameraY = VisionConstants.CAMERA_OV2387.vpw / 2 * nx
            val cameraY2 = VisionConstants.CAMERA_OV2387.vpw / 2 * nx2
            val cameraZ = VisionConstants.CAMERA_OV2387.vph / 2 * ny
            val cameraX = 1

            val scaledYTagDist = FieldConstants.aprilTagWidth * ((px2 - px) / hypot((px2-px), (py2-py)))
            val scalar = scaledYTagDist / (cameraY2.meters - cameraY.meters).absoluteValue


            // good up


            var scaledPositionCameraSpace = Translation3d(
              cameraX.meters, cameraY.meters, cameraZ.meters
            ).times(scalar).rotateBy(cameraPoses[0].rotation.unaryMinus())

            Logger.recordOutput("\"Vision/${instance}/${tag.fiducialId}/${corner}/guesstimatedCameraPosewrtRobot", scaledPositionCameraSpace.toDoubleArray().toDoubleArray())

            Logger.recordOutput("Vision/odometryRotation", drivetrainOdometry.invoke().rotation.inDegrees)
            scaledPositionCameraSpace = scaledPositionCameraSpace.rotateBy(
              Rotation3d(
                0.degrees,
                0.degrees,
                drivetrainOdometry.invoke().rotation
              ))

//            if (drivetrainOdometry.invoke().rotation != null){
//              println("rotating")
//              println(drivetrainOdometry.invoke().rotation)
//              scaledPositionCameraSpace = scaledPositionCameraSpace.rotateBy(
//                Rotation3d(
//                  0.degrees,
//                  0.degrees,
//                  tag.yaw.degrees
//                )
//              )
//            }

            val detectionTransformation = Transform3d(
              scaledPositionCameraSpace,
              Rotation3d()
            )

            val flipTransform = Transform3d(
              Translation3d(),
              Rotation3d(0.degrees, 0.degrees, 180.degrees)
            )

            var estimatedRobotPose = Pose2d()



            if (tag.fiducialId in intArrayOf(3, 4)){
              estimatedRobotPose = FieldConstants.fieldAprilTags.get(tag.fiducialId).pose
                .transformBy(
                  Transform3d(
                    Translation3d(0.inches, 0.inches, -3.25.inches),
                    Rotation3d()
                  )
                )
                .transformBy(
                  flipTransform
                )
                .transformBy(
                  detectionTransformation.inverse()
                )
                .transformBy(
                  Transform3d(
                    cameraPoses[0].translation.unaryMinus(),
                    Rotation3d()
                  ))
                .toPose2d()
            } else {
              estimatedRobotPose = FieldConstants.fieldAprilTags.get(tag.fiducialId).pose
                .transformBy(
                  Transform3d(
                    Translation3d(0.inches, 0.inches, -3.25.inches),
                    Rotation3d()
                  )
                ).transformBy(
                  detectionTransformation.inverse()
                ).transformBy(
                  Transform3d(
                    cameraPoses[0].translation,
                    Rotation3d()
                  )
                ).toPose2d()
            }

            Logger.recordOutput("Vision/${instance}/${tag.fiducialId}/${corner}/guesstimatedPose", estimatedRobotPose.toDoubleArray().toDoubleArray())

            Logger.recordOutput("Vision/${instance}/${tag.fiducialId}/guesstimatedTag", FieldConstants.fieldAprilTags.get(tag.fiducialId).pose.toDoubleArray().toDoubleArray())
            //Logger.recordOutput("Vision/${instance}/${tag.fiducialId}/guesstimatedReal", Pose2d(15.meters, 5.meters, 70.degrees).toDoubleArray().toDoubleArray())

            cornerData.add(px)
            cornerData.add(py)
            cornerData.add(px2)
            cornerData.add(py2)

            break
          }
        }
      }

      Logger.recordOutput("Vision/cornerDetections/${instance}}", cornerData.toDoubleArray())
      Logger.recordOutput("Vision/realPose", Pose2d(537.212500.inches - 4.875.inches, 161.628.inches - 39.5.inches, 0.degrees).toDoubleArray().toDoubleArray())

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
