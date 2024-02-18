package com.team4099.robot2023.subsystems.vision.camera

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
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

    override fun toLog(table: LogTable?) {
      table?.put("timestampSeconds", timestamp.inSeconds)
      table?.put("frame", frame.pose3d)
      table?.put("fps", fps)
      table?.put("usedTargets", usedTargets)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("timestampSeconds", 0.0)?.let {
        timestamp = it.seconds
      }
      table?.get("frame", Pose3dWPILIB())?.let {
        frame = Pose3d(it)
      }
      table?.get("fps", 0.0)
      table?.get("usedTargets", listOf<Double>())
    }
  }

  fun updateInputs(inputs: CameraInputs) {}
}
