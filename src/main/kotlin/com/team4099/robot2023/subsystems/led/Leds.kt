package com.team4099.robot2023.subsystems.led

import com.team4099.robot2023.config.constants.LEDConstants
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Leds(val io: LedIO) : SubsystemBase() {
  var inputs = LedIO.LedIOInputs()

  var hasNote = false
  var subsystemsAtPosition = false
  var isIdle = true
  var batteryIsLow = false

  var state = LEDConstants.CandleState.NO_NOTE
    set(value) {
      io.setState(value)
      field = value
    }
  init {
    state = state
  }

  override fun periodic() {
    io.updateInputs(inputs)
    if (batteryIsLow && DriverStation.isDisabled()) {
      state = LEDConstants.CandleState.LOW_BATTERY
    } else if (hasNote) {
      if (isIdle) {
        state = LEDConstants.CandleState.HAS_NOTE
      } else if (subsystemsAtPosition) {
        state = LEDConstants.CandleState.CAN_SHOOT
      }
    } else {
      state = LEDConstants.CandleState.NO_NOTE
    }

    Logger.processInputs("LED", inputs)
    Logger.recordOutput("LED/state", state.name)
  }
}
