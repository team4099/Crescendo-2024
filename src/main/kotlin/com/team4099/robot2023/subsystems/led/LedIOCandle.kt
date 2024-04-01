package com.team4099.robot2023.subsystems.led

import com.ctre.phoenix.led.CANdle
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.LEDConstants
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

object LedIOCandle : LedIO {

  private val ledController = CANdle(Constants.LED.LED_CANDLE_ID)
  private var lastState: LEDConstants.CandleState = LEDConstants.CandleState.NO_NOTE
  private var waveRuns = 0
  private var loopCycles = 0
  private var reverseLEDS = false
  private var finishedFade = false
  override var batteryVoltage: ElectricalPotential = 12.0.volts

  override fun updateInputs(inputs: LedIO.LedIOInputs) {
    inputs.ledState = lastState.name
  }

  override fun setState(newState: LEDConstants.CandleState) {
    Logger.recordOutput("LED/newState", newState)
    lastState = newState
    setCANdleState(newState)
  }

  private fun wave(
    state: LEDConstants.CandleState,
    defaultState: LEDConstants.CandleState,
    lengthOfWave: Int = 10,
    lengthOfGradient: Double = 5.0,
  ) {
    ledController.setLEDs(state.r, state.g, state.b, 0, waveRuns, lengthOfWave)

    // Set outer colors
    ledController.setLEDs(defaultState.r, defaultState.g, defaultState.b, 0, 0, waveRuns - lengthOfGradient.toInt())
    ledController.setLEDs(defaultState.r, defaultState.g, defaultState.b, 0, waveRuns + lengthOfWave + lengthOfGradient.toInt(), LEDConstants.LED_COUNT - waveRuns - lengthOfWave)

    // Set gradient colors
    if (lengthOfGradient > 0.0) {
      for ((step, ledIdx) in ((waveRuns - lengthOfGradient.toInt())..waveRuns).withIndex()) {
        ledController.setLEDs(
          (defaultState.r + (state.r - defaultState.r) * ((step + 1) / lengthOfGradient)).toInt(),
          (defaultState.g + (state.g - defaultState.g) * ((step + 1) / lengthOfGradient)).toInt(),
          (defaultState.b + (state.b - defaultState.b) * ((step + 1) / lengthOfGradient)).toInt(),
          0,
          ledIdx,
          1
        )
      }

      for ((step, ledIdx) in (waveRuns..(waveRuns + lengthOfGradient).toInt()).withIndex()) {
        ledController.setLEDs(
          (defaultState.r + (state.r - defaultState.r) * ((lengthOfGradient - (step + 1)) / lengthOfGradient)).toInt(),
          (defaultState.g + (state.g - defaultState.g) * ((lengthOfGradient - (step + 1)) / lengthOfGradient)).toInt(),
          (defaultState.b + (state.b - defaultState.b) * ((lengthOfGradient - (step + 1)) / lengthOfGradient)).toInt(),
          0,
          ledIdx,
          1
        )
      }
    }

    if (waveRuns >= LEDConstants.LED_COUNT - lengthOfWave) {
      reverseLEDS = true
    }
    else if (waveRuns < lengthOfWave){
      reverseLEDS = false
    }

    waveRuns += if (!reverseLEDS) 1 else -1
  }

  private fun gradient(
    state: LEDConstants.CandleState,
    otherState: LEDConstants.CandleState,
    lengthOfSolidEnds: Int = 10
  ) {
    ledController.setLEDs(state.r, state.g, state.b, 0, 0, lengthOfSolidEnds)
    ledController.setLEDs(otherState.r, otherState.g, otherState.b, 0, LEDConstants.LED_COUNT - lengthOfSolidEnds, lengthOfSolidEnds)

    for ((step, ledIdx) in (lengthOfSolidEnds until(LEDConstants.LED_COUNT - lengthOfSolidEnds)).withIndex()) {
      ledController.setLEDs(
        (state.r + (otherState.r - state.r) * ((step + 1) / (LEDConstants.LED_COUNT - lengthOfSolidEnds * 2.0))).toInt(),
        (state.g + (otherState.g - state.g) * ((step + 1) / (LEDConstants.LED_COUNT - lengthOfSolidEnds * 2.0))).toInt(),
        (state.b + (otherState.b - state.b) * ((step + 1) / (LEDConstants.LED_COUNT - lengthOfSolidEnds * 2.0))).toInt(),
        0,
        ledIdx,
        1
      )
    }
  }

  private fun progressBar(
    state: LEDConstants.CandleState,
    defaultState: LEDConstants.CandleState,
    percent: Double
  ) {
    ledController.setLEDs(state.r, state.g, state.b, 0, 0, (percent * LEDConstants.LED_COUNT).toInt())
    ledController.setLEDs(defaultState.r, defaultState.g, defaultState.b, 0, (percent * LEDConstants.LED_COUNT).toInt(), LEDConstants.LED_COUNT)
  }

  private fun fadeBetweenColors(
    state: LEDConstants.CandleState,
    otherState: LEDConstants.CandleState,
    loopCyclesToConverge: Int = 5
  ) {
    for ((ledIdx, numberOfLoopCycles) in (loopCyclesToConverge..loopCyclesToConverge + LEDConstants.LED_COUNT).withIndex()) {
      val calculatedR = (state.r + (otherState.r - state.r) * (if (loopCycles > numberOfLoopCycles) 1.0 else (loopCycles / numberOfLoopCycles.toDouble()))).toInt()
      val calculatedG = (state.g + (otherState.g - state.g) * (if (loopCycles > numberOfLoopCycles) 1.0 else (loopCycles / numberOfLoopCycles.toDouble()))).toInt()
      val calculatedB = (state.b + (otherState.b - state.b) * (if (loopCycles > numberOfLoopCycles) 1.0 else (loopCycles / numberOfLoopCycles.toDouble()))).toInt()
      ledController.setLEDs(
        calculatedR,
        calculatedB,
        calculatedG,
        0,
        ledIdx,
        1
      )
    }
  }

  private fun setCANdleState(state: LEDConstants.CandleState) {
    if (state == LEDConstants.CandleState.BATTERY_DISPLAY) {
      progressBar(
        LEDConstants.CandleState.BATTERY_DISPLAY,
        LEDConstants.CandleState.NOTHING,
        (batteryVoltage.inVolts - 11.5) / (LEDConstants.BATTERY_FULL_THRESHOLD.inVolts - 11.5)
      )
    }
    else if (state == LEDConstants.CandleState.BLUE) {
      ledController.clearAnimation(0)
      val otherState = LEDConstants.CandleState.MAGENTA
      fadeBetweenColors(state, otherState)
    }
    else if (state.animation == null) {
      ledController.clearAnimation(0)
      ledController.setLEDs(state.r, state.g, state.b)
    } else {
      ledController.animate(state.animation, 0)
      ledController.setLEDs(state.r, state.g, state.b)
    }
  }
}
