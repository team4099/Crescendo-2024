package com.team4099.robot2023.subsystems.led

import com.team4099.robot2023.config.constants.LEDConstants
import com.team4099.robot2023.util.FMSData
import edu.wpi.first.wpilibj.DriverStation
import org.littletonrobotics.junction.Logger

class Leds(val io: LedIO) {
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

  fun periodic() {
    io.updateInputs(inputs)
    if (batteryIsLow && DriverStation.isDisabled()) {
      state = LEDConstants.CandleState.LOW_BATTERY
    } else if (DriverStation.isDisabled()) {
      if (DriverStation.getAlliance().isPresent) {
        if (FMSData.isBlue) {
          state = LEDConstants.CandleState.BLUE
        } else {
          state = LEDConstants.CandleState.RED
        }
      } else {
        state = LEDConstants.CandleState.NOTHING
      }
    } else if (hasNote) {
      if (subsystemsAtPosition) {
        state = LEDConstants.CandleState.CAN_SHOOT
      } else {
        state = LEDConstants.CandleState.HAS_NOTE
      }
    } else {
      state = LEDConstants.CandleState.NO_NOTE
    }

    Logger.processInputs("LED", inputs)
    Logger.recordOutput("LED/state", state.name)
  }
}
