package com.team4099.robot2023.subsystems.vision.camera

import com.team4099.robot2023.config.constants.FieldConstants
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.team4099.lib.around
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Pose3dWPILIB
import org.team4099.lib.geometry.Transform3dWPILIB
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.micro
import org.team4099.lib.units.milli
import java.util.*
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget


class CameraIOPhotonvision(private val identifier: String): CameraIO {

    private val photonEstimator: PhotonPoseEstimator
    private val camera: PhotonCamera
    private var lastEstTimestamp: Time = 0.0.seconds

    init {
        camera = PhotonCamera(identifier)
        photonEstimator = PhotonPoseEstimator(
            FieldConstants.wpilibFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Transform3dWPILIB()
        )
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)

    }

    override fun updateInputs(inputs: CameraIO.CameraInputs) {
        print(camera.hasTargets())
        //val latestResult = camera.latestResult
        //inputs.cameraTargets = latestResult.targets

        /*
        val visionEst: Optional<EstimatedRobotPose>? = photonEstimator.update()
        val poseEst = visionEst?.get()?.let { Pose3d(it.estimatedPose) }
        val latestTimestamp = visionEst?.get()?.timestampSeconds?.seconds
        if (latestTimestamp != null) {
            if ((latestTimestamp - lastEstTimestamp).absoluteValue > 10.micro.seconds) {
                inputs.fps = 1/(latestTimestamp - lastEstTimestamp).inSeconds
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

         */
    }
}