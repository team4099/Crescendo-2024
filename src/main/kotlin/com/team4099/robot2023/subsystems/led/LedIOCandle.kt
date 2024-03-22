package com.team4099.robot2023.subsystems.led

import com.ctre.phoenix.led.CANdle
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.LEDConstants

object LedIOCandle : LedIO {

  private val ledController = CANdle(Constants.LED.LED_CANDLE_ID, Constants.Universal.CANIVORE_NAME)
  private var lastState: LEDConstants.CandleState = LEDConstants.CandleState.NO_NOTE

  override fun updateInputs(inputs: LedIO.LedIOInputs) {
    inputs.ledState = lastState.name
  }

  override fun setState(newState: LEDConstants.CandleState) {
    lastState = newState
    when (newState) {
      LEDConstants.CandleState.LOW_BATTERY -> setCANdleState(LEDConstants.CandleState.LOW_BATTERY)
      LEDConstants.CandleState.NO_NOTE -> setCANdleState(LEDConstants.CandleState.NO_NOTE)
      LEDConstants.CandleState.HAS_NOTE -> setCANdleState(LEDConstants.CandleState.HAS_NOTE)
      LEDConstants.CandleState.CAN_SHOOT -> setCANdleState(LEDConstants.CandleState.CAN_SHOOT)
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
