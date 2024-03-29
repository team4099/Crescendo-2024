package com.team4099.robot2023.subsystems.limelight

import com.team4099.lib.logging.TunableNumber
import com.team4099.lib.vision.TargetCorner
import com.team4099.lib.vision.TimestampedVisionUpdate
import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.util.LimelightReading
import com.team4099.robot2023.util.rotateBy
import com.team4099.robot2023.util.toPose3d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Rotation3dWPILIB
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.geometry.Translation3dWPILIB
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.tan
import java.util.function.Consumer
import kotlin.math.hypot
import kotlin.math.sqrt
import kotlin.math.tan

class LimelightVision(val io: LimelightVisionIO) : SubsystemBase() {
  val inputs = LimelightVisionIO.LimelightVisionIOInputs()

  var poseSupplier: () -> Pose2d = { Pose2d() }
  var visionConsumer: Consumer<List<TimestampedVisionUpdate>> = Consumer {}

  // i think we need this for camera project to irl coordinates
  val vpw = (2.0 * (VisionConstants.Limelight.HORIZONTAL_FOV / 2).tan)
  val vph = (2.0 * (VisionConstants.Limelight.VERITCAL_FOV / 2).tan)

  private val xyStdDevCoefficient = TunableNumber("LimelightVision/xystdev", 0.05)
  private val thetaStdDev = TunableNumber("LimelightVision/thetaStdDev", 0.75)

  fun solveTargetPoseFromAngle(
    currentPose: Pose2d,
    target: LimelightReading,
    targetHeight: Length
  ): Pose3d {
    val xyDistance = xyDistanceFromTarget(target, targetHeight)
    val distanceToTarget =
      hypot(
        xyDistance.inMeters,
        targetHeight.inMeters - VisionConstants.Limelight.LL_TRANSFORM.z.inMeters
      )
        .meters

    val targetTranslation =
      Translation3dWPILIB(
        distanceToTarget.inMeters,
        Rotation3dWPILIB(0.0, -target.ty.inRadians, -target.tx.inRadians)
      )

    Logger.recordOutput("LimelightVision/distanceToTarget", distanceToTarget.inMeters)

    // figure out which way this target is facing using yaw of robot and yaw of camera
    val targetRotation =
      Rotation3d(
        0.0.degrees,
        0.0.degrees,
        if (currentPose.rotation.rotateBy(VisionConstants.Limelight.LL_TRANSFORM.rotation.z) in
          0.degrees..180.degrees
        ) {
          // we are looking at a red node which is facing towards 180 degrees
          180.0.degrees
        } else {
          // we are looking at a blue node which is facing towards 0 degrees
          0.degrees
        }
      )

    return currentPose
      .toPose3d()
      .transformBy(VisionConstants.Limelight.LL_TRANSFORM)
      .transformBy(Transform3d(Translation3d(targetTranslation), targetRotation))
  }

  fun xyDistanceFromTarget(target: LimelightReading, targetHeight: Length): Length {
    var x = target.tx.tan
    var y = target.ty.tan
    var z = 1.0
    val normalVectorMagnitude = sqrt(x * x + y * y + z * z) // distance formula
    x /= normalVectorMagnitude
    y /= normalVectorMagnitude
    z /= normalVectorMagnitude

    val xPrime = x
    val yzPrime =
      Translation2d(y.meters, z.meters)
        .rotateBy(VisionConstants.Limelight.LL_TRANSFORM.rotation.y)
    val yPrime = yzPrime.x
    val zPrime = yzPrime.y

    val angleToGoal = Math.asin(yPrime.inMeters)
    val targetToCameraHeight = targetHeight - VisionConstants.Limelight.LL_TRANSFORM.z

    return targetToCameraHeight / tan(angleToGoal)
  }

  // based off of angles
  fun solveTargetPositionFromAngularOutput(
    tx: Angle,
    ty: Angle,
    currentPose: Pose2d,
    cameraTransform: Transform3d,
    targetHeight: Length
  ): Pose3d {
    val horizontalAngleFromCamera = -tx
    val verticalAngleFromCamera = -ty

    // rotation from robot to the target in frame
    val rotationFromTargetToCamera =
      Rotation3d(0.0.degrees, verticalAngleFromCamera, horizontalAngleFromCamera)

    val xDistanceFromTargetToCamera =
      (targetHeight - cameraTransform.z) / verticalAngleFromCamera.tan
    val yDistanceFromTargetToCamera = xDistanceFromTargetToCamera * horizontalAngleFromCamera.tan

    val translationFromTargetToCamera =
      Translation3d(
        xDistanceFromTargetToCamera,
        yDistanceFromTargetToCamera,
        targetHeight - cameraTransform.z
      )

    // figure out which way this target is facing using yaw of robot and yaw of camera
    val targetRotation =
      Rotation3d(
        0.0.degrees,
        0.0.degrees,
        if (currentPose.rotation.rotateBy(cameraTransform.rotation.z) in
          0.degrees..180.degrees
        ) {
          // we are looking at a red node which is facing towards 180 degrees
          180.0.degrees
        } else {
          // we are looking at a blue node which is facing 0 degrees
          0.degrees
        }
      )

    return Pose3d(
      currentPose.toPose3d().transformBy(cameraTransform).translation +
        translationFromTargetToCamera,
      targetRotation
    )
  }

  // based off of pixel coordinates
  private fun pixelCoordsToNormalizedPixelCoords(pixelCoords: CoordinatePair): CoordinatePair {
    // we're defining a coordinate pair to be where 0,0 is (horizontal fov / 2, vertical fov / 2)
    // and 1,1 is 1 pixel shifted both vertically and horizontally
    // note that pixel coordinates is returned where 0,0 is upper left, pos x is down, and pos y is
    // right but we want pos x to be right and pos y to be up
    return CoordinatePair(
      1 / (VisionConstants.Limelight.RES_WIDTH / 2) *
        (pixelCoords.x - VisionConstants.Limelight.RES_WIDTH - 0.5),
      1 / (VisionConstants.Limelight.RES_HEIGHT / 2) *
        (pixelCoords.x - VisionConstants.Limelight.RES_HEIGHT - 0.5)
    )
  }

  // need to make sure that all the corners are from the same target before passing into this
  // function
  fun solveTargetPositionFromCameraOutput(
    currentPose: Pose2d,
    pixelPosition: CoordinatePair,
    cameraTransform: Transform3d,
    targetHeight: Length
  ): Pose3d {
    val normalizedCoordinates = pixelCoordsToNormalizedPixelCoords(pixelPosition)

    // i think we wanna do what is there above because we are getting absolute position of the node
    // relative to the origin
    val targetRotation =
      currentPose
        .rotation
        .rotateBy(VisionConstants.Limelight.LL_TRANSFORM.rotation.z)
        .rotateBy(180.degrees) // assuming we are looking at the tape head on

    val cartesianPoseOfTarget =
      Pose3d(
        currentPose.x + (vpw / 2 * normalizedCoordinates.x).meters,
        currentPose.y + (vph / 2 * normalizedCoordinates.y).meters,
        targetHeight,
        Rotation3d(0.0.degrees, 0.0.degrees, targetRotation)
      )

    return cartesianPoseOfTarget
  }

  // assume top left, top right, bottom left, bottom right
  fun centerOfRectangle(corners: List<TargetCorner>): CoordinatePair? {
    var xPos: Double? = null
    var yPos: Double? = null
    if (corners.size == 4) {
      xPos = corners.map { it.x }.average()
      yPos = corners.map { it.y }.average()
    }

    return if (xPos != null && yPos != null) {
      CoordinatePair(xPos, yPos)
    } else {
      null
    }
  }

  fun setDataInterfaces(
    poseSupplier: () -> Pose2d,
    visionConsumer: Consumer<List<TimestampedVisionUpdate>>
  ) {
    this.poseSupplier = poseSupplier
    this.visionConsumer = visionConsumer
  }
}
