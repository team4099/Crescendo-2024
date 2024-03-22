package com.team4099.robot2023.subsystems.led

import com.ctre.phoenix.led.CANdle
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.LEDConstants

object LedIOCandle : LedIO {

  private val ledController = CANdle(Constants.LED.LED_CANDLE_ID, Constants.Universal.CANIVORE_NAME)
  private var lastState: LEDConstants.BlinkinLEDState = LEDConstants.BlinkinLEDState.IDLE

  override fun updateInputs(inputs: LedIO.LedIOInputs) {
    inputs.ledState = lastState.name
  }

  override fun setState(newState: LEDConstants.BlinkinLEDState) {
    lastState = newState
    when (newState) {
      LEDConstants.BlinkinLEDState.IDLE -> setCANdleState(LEDConstants.CandleState.IDLE)
      LEDConstants.BlinkinLEDState.AUTO -> setCANdleState(LEDConstants.CandleState.AUTO)
      LEDConstants.BlinkinLEDState.HAS_NOTE -> setCANdleState(LEDConstants.CandleState.HAS_NOTE)
      LEDConstants.BlinkinLEDState.NO_NOTE -> setCANdleState(LEDConstants.CandleState.NO_NOTE)
    }
  }

  private fun setCANdleState(state: LEDConstants.CandleState) {
    if (state.animation == null) {
      ledController.setLEDs(state.r, state.g, state.b)
    } else {
      ledController.animate(state.animation)
    }
  }
}
