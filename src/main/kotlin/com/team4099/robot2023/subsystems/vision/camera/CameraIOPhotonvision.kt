package com.team4099.robot2023.subsystems.vision.camera

import com.team4099.robot2023.config.constants.FieldConstants
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Transform3dWPILIB
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.micro
import java.util.Optional

class CameraIOPhotonvision(override val id: String, override val robotTCamera: Transform3d) : CameraIO {

  private val photonEstimator: PhotonPoseEstimator
  private val camera: PhotonCamera
  private var lastEstTimestamp: Time = 0.0.seconds

  init {
    camera = PhotonCamera(id)
    photonEstimator =
      PhotonPoseEstimator(
        FieldConstants.wpilibFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camera,
        Transform3dWPILIB()
      )
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
  }

  override fun updateInputs(inputs: CameraIO.CameraInputs) {
    val visionEst: Optional<EstimatedRobotPose>? = photonEstimator.update()
    val poseEst = visionEst?.get()?.let { Pose3d(it.estimatedPose) }
    val latestTimestamp = visionEst?.get()?.timestampSeconds?.seconds
    if (latestTimestamp != null) {
      if ((latestTimestamp - lastEstTimestamp).absoluteValue > 10.micro.seconds) {
        inputs.fps = 1 / (latestTimestamp - lastEstTimestamp).inSeconds
        lastEstTimestamp = latestTimestamp
      }
    }

    if (visionEst != null) {
      inputs.usedTargets = visionEst.get().targetsUsed.map { it.fiducialId }
    }

    inputs.timestamp = lastEstTimestamp
    if (poseEst != null) {
      inputs.frame = poseEst
    }
  }
}
