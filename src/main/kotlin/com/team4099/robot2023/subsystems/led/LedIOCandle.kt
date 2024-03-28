package com.team4099.robot2023.subsystems.led

import com.ctre.phoenix.led.CANdle
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.LEDConstants
import org.littletonrobotics.junction.Logger

object LedIOCandle : LedIO {

  private val ledController = CANdle(Constants.LED.LED_CANDLE_ID)
  private var lastState: LEDConstants.CandleState = LEDConstants.CandleState.NO_NOTE

  override fun updateInputs(inputs: LedIO.LedIOInputs) {
    inputs.ledState = lastState.name
  }

  override fun setState(newState: LEDConstants.CandleState) {
    Logger.recordOutput("LED/newState", newState)
    lastState = newState
    setCANdleState(newState)
  }

  private fun setCANdleState(state: LEDConstants.CandleState) {
    if (state.animation == null) {
      ledController.clearAnimation(0)
      ledController.setLEDs(state.r, state.g, state.b)
    } else {
      ledController.animate(state.animation, 0)
      ledController.setLEDs(state.r, state.g, state.b)
    }
  }
}
