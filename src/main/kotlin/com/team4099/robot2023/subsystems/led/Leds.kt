package com.team4099.robot2023.subsystems.led

import com.team4099.robot2023.config.constants.LEDConstants
import com.team4099.robot2023.util.FMSData
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.derived.volts

class Leds(val io: LedIO) {
  var inputs = LedIO.LedIOInputs()

  var hasNote = false
  var subsystemsAtPosition = false
  var isAutoAiming = false
  var isAmping = false
  var seesGamePiece = false
  var seesTag = true

  var state = LEDConstants.CandleState.NO_NOTE
    set(value) {
      io.setState(value)
      field = value
    }

  fun periodic() {
    io.updateInputs(inputs)
    if (DriverStation.getAlliance().isEmpty) {
      io.batteryVoltage = RobotController.getBatteryVoltage().volts
      state = LEDConstants.CandleState.GOLD
      if (io.batteryVoltage < 12.3.volts) {
        state = LEDConstants.CandleState.LOW_BATTERY_WARNING
      } else {
        state = LEDConstants.CandleState.GOLD
      }
    } else if (DriverStation.isDisabled() && DriverStation.getAlliance().isPresent) {
      if (FMSData.isBlue) {
        state = LEDConstants.CandleState.BLUE
      } else {
        state = LEDConstants.CandleState.RED
      }
    } else if (hasNote) {
      if (isAutoAiming) {
        if (!seesTag) {
          state = LEDConstants.CandleState.NO_TAG
        } else if (!subsystemsAtPosition) {
          state = LEDConstants.CandleState.SEES_TAG
        } else {
          state = LEDConstants.CandleState.CAN_SHOOT
        }
      } else if (isAmping) {
        if (subsystemsAtPosition) {
          state = LEDConstants.CandleState.CAN_SHOOT
        } else {
          state = LEDConstants.CandleState.HAS_NOTE
        }
      }
    } else if (seesGamePiece) {
      state = LEDConstants.CandleState.SEES_NOTE
    } else {
      state = LEDConstants.CandleState.NO_NOTE
    }

    Logger.processInputs("LED", inputs)
    Logger.recordOutput("LED/state", state.name)
  }
}
