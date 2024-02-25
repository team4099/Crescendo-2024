package com.team4099.robot2023.subsystems.vision.camera

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.common.dataflow.structures.Packet
import org.photonvision.targeting.PhotonTrackedTarget
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Pose3dWPILIB
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds

interface CameraIO {
  class CameraInputs : LoggableInputs {
    var timestamp = 0.0.seconds
    var frame: Pose3d = Pose3d()
    var fps = 0.0
    var usedTargets: List<Int> = listOf<Int>()
    var cameraTargets = mutableListOf<PhotonTrackedTarget>()
    var indices = 0

    override fun toLog(table: LogTable?) {
      table?.put("timestampSeconds", timestamp.inSeconds)
      table?.put("frame", frame.pose3d)
      table?.put("fps", fps)
      table?.put("usedTargets", usedTargets.toIntArray())

      for (targetID in cameraTargets.indices) {
        val photonPacket = Packet(PhotonTrackedTarget.PACK_SIZE_BYTES)
        val byteArrayEncodedData = cameraTargets.get(targetID).populatePacket(photonPacket)
        table?.put("cameraTargets/${targetID}", byteArrayEncodedData.data)
      }

      table?.put("cameraTargets/indices", cameraTargets.size)


    }

    override fun fromLog(table: LogTable?) {
      table?.get("timestampSeconds", 0.0)?.let {
        timestamp = it.seconds
      }
      table?.get("frame", Pose3dWPILIB())?.let {
        frame = Pose3d(it.get(0))
      }
      table?.get("fps", 0.0)
      table?.get("usedTargets", intArrayOf())?.let {
        usedTargets = it.toList()
      }

      table?.get("cameraTargets/indices", 0)?.let {
        indices = it
      }

      cameraTargets = mutableListOf<PhotonTrackedTarget>()

      for (targetID in 0 until indices) {
        table?.get("cameraTargets/${targetID}", ByteArray(PhotonTrackedTarget.PACK_SIZE_BYTES))?.let {
          var trackedTarget = PhotonTrackedTarget()
          trackedTarget.createFromPacket(Packet(it))
          cameraTargets.add(trackedTarget)
        }
      }
    }
  }

  fun updateInputs(inputs: CameraInputs) {}
}