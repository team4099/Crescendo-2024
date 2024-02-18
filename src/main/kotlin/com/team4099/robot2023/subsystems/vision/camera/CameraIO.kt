package com.team4099.robot2023.subsystems.vision.camera

import com.team4099.lib.logging.toDoubleArray
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Pose3dWPILIB
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds

interface CameraIO {

  val id: String
  val robotTCamera: Transform3d
  class CameraInputs : LoggableInputs {
    var timestamp = 0.0.seconds
    var frame: Pose3d = Pose3d()
    var fps = 0.0
    var usedTargets: List<Int> = listOf<Int>()

    override fun toLog(table: LogTable?) {
      table?.put("timestampSeconds", timestamp.inSeconds)
      table?.put("frame", frame.toDoubleArray().toDoubleArray())
      table?.put("fps", fps)
      table?.put("usedTargets", usedTargets.toIntArray())
    }

    override fun fromLog(table: LogTable?) {
      table?.get("timestampSeconds", 0.0)?.let { timestamp = it.seconds }
      table?.get("frame", Pose3dWPILIB())?.let { frame = Pose3d(it[0]) }
      table?.get("fps", 0.0).let { fps = it ?: 0.0 }
      table?.get("usedTargets", IntArray(0)).let { usedTargets = it?.toList() ?: listOf() }
    }
  }

  fun updateInputs(inputs: CameraInputs) {}
}
