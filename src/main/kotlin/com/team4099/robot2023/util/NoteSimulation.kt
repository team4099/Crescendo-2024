package com.team4099.robot2023.util

import FieldConstants
import com.team4099.lib.hal.Clock
import org.littletonrobotics.junction.Logger
import com.team4099.lib.logging.toDoubleArray
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.config.constants.IntakeConstants
import com.team4099.robot2023.config.constants.WristConstants
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.Second
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond
import kotlin.math.absoluteValue
import kotlin.math.max

class NoteSimulation(val id: Int = 0, val offFieldPose: Pose3d = Pose3d(), val stagedPose: Pose3d = Pose3d()) {


    var currentPose = Pose3d()

    var xVel = 0.0.meters.perSecond
    var yVel = 0.0.meters.perSecond
    var zVel = 0.0.meters.perSecond

    var xAccel = 0.0.meters.perSecond.perSecond
    var yAccel = 0.0.meters.perSecond.perSecond
    var zAccel = 0.0.meters.perSecond.perSecond

    var lastXPos = 0.0.meters
    var lastYPos = 0.0.meters
    var lastZPos = 0.0.meters

    var lastXVel = 0.0.meters.perSecond
    var lastYVel = 0.0.meters.perSecond
    var lastZVel = 0.0.meters.perSecond

    var launchStartPose = Pose3d()
    var lastUpdateTime = Clock.fpgaTime
    var inclinationAngleTheta = 0.0.degrees
    var azimuthalAnglePhi = 0.0.degrees
    var launchVelocity = 0.meters.perSecond

    var currentState = NoteStates.OFF_FIELD
        set(value) {
            if (currentState == NoteStates.IN_ROBOT && value == NoteStates.IN_FLIGHT) {
                launchStartPose = getNoteInRobotPose()

                lastUpdateTime = Clock.fpgaTime
                inclinationAngleTheta = 90.degrees + (wristAngleSupplier() + WristConstants.NOTE_ANGLE_SIM_OFFSET)
                azimuthalAnglePhi =  poseSupplier().rotation

                lastXPos = launchStartPose.x
                lastYPos = launchStartPose.y
                lastZPos = launchStartPose.z

                launchVelocity = (FlywheelConstants.FLYWHEEL_SPEED_TRANSFER_PERCENTAGE * flywheelAngularVelocitySupplier().inRadiansPerSecond * FlywheelConstants.FLYWHEEL_RADIUS.inMeters).meters.perSecond

                lastXVel = launchVelocity * inclinationAngleTheta.sin * azimuthalAnglePhi.cos
                lastYVel = launchVelocity * inclinationAngleTheta.sin * azimuthalAnglePhi.sin
                lastZVel = launchVelocity * inclinationAngleTheta.cos


            }
            field = value
        }

    var poseSupplier: () -> Pose2d = { Pose2d() }

    var wristAngleSupplier: () -> Angle = {0.0.degrees}

    var elevatorHeightSupplier: () -> Length =  { 0.0.inches}

    var flywheelAngularVelocitySupplier: () -> AngularVelocity = {0.0.rotations.perMinute}

    fun periodic() {
        Logger.recordOutput("NoteData/${id}/inclinationAngle", inclinationAngleTheta.inDegrees)
        Logger.recordOutput("NoteData/${id}/azimuthalAngle", azimuthalAnglePhi.inDegrees)
        Logger.recordOutput("NoteData/${id}/launchVelocityMetersPerSecond", launchVelocity.inMetersPerSecond)
        Logger.recordOutput("NoteData/${id}/startPose", launchStartPose.toDoubleArray())
        Logger.recordOutput("NoteData/${id}/state", currentState.name)
        Logger.recordOutput("NoteSimulation/IntakePose", poseSupplier().transformBy(IntakeConstants.INTAKE_TRANSFORM).toDoubleArray())
        when (currentState) {
            NoteStates.OFF_FIELD -> currentPose = offFieldPose
            NoteStates.STAGED -> currentPose = stagedPose
            NoteStates.ON_FIELD -> {}
            NoteStates.IN_ROBOT -> {
                currentPose = getNoteInRobotPose()
            }
            NoteStates.IN_FLIGHT -> {
                xAccel = if (lastZPos > FieldConstants.noteThickness/2) 0.0.meters.perSecond.perSecond else -6.meters.perSecond.perSecond * azimuthalAnglePhi.cos.absoluteValue * lastXVel.sign
                yAccel = if (lastZPos > FieldConstants.noteThickness/2) 0.0.meters.perSecond.perSecond else -6.meters.perSecond.perSecond * azimuthalAnglePhi.sin.absoluteValue * lastYVel.sign
                zAccel = (Constants.Universal.gravity)
                update()

                if (xVel.absoluteValue < 0.1.meters.perSecond && yVel.absoluteValue < 0.1.meters.perSecond) {
                    currentState = NoteStates.ON_FIELD
                }
            }
        }
    }

    fun update() {
        val currentTime = Clock.fpgaTime
        val dt = (Clock.fpgaTime - lastUpdateTime)

        xVel = lastXVel + dt * xAccel
        yVel = lastYVel + dt * yAccel
        zVel = lastZVel + dt * zAccel

        val xPos = lastXPos + dt * (xVel + lastXVel) / 2
        val yPos = lastYPos + dt * (yVel + lastYVel) / 2
        val zPos = max((lastZPos + dt * (zVel + lastZVel)/2).inMeters, FieldConstants.noteThickness.inMeters/2).meters

        val rotation = if (zPos > FieldConstants.noteThickness / 2 ) launchStartPose.rotation else Rotation3d()

        lastUpdateTime = currentTime

        lastXPos = xPos
        lastYPos = yPos
        lastZPos = zPos

        lastXVel = xVel
        lastYVel = yVel
        lastZVel = zVel

        currentPose = Pose3d(xPos, yPos, zPos, rotation)

        Logger.recordOutput("NoteData/${id}/xVel", xVel.inMetersPerSecond)
        Logger.recordOutput("NoteData/${id}/yVel", yVel.inMetersPerSecond)
        Logger.recordOutput("NoteData/${id}/zVel", zVel.inMetersPerSecond)
        Logger.recordOutput("NoteData/${id}/xAccel", xAccel.inMetersPerSecondPerSecond)
        Logger.recordOutput("NoteData/${id}/yAccel", yAccel.inMetersPerSecondPerSecond)
        Logger.recordOutput("NoteData/${id}/zAccel", zAccel.inMetersPerSecondPerSecond)
    }

    fun getNoteInRobotPose(): Pose3d{
        return poseSupplier().toPose3d()
            .transformBy(
                Transform3d(
                    Translation3d(
                        0.013.meters - WristConstants.WRIST_AXIS_TO_NOTE_HOLD_POSITION * wristAngleSupplier().cos,
                        0.0.inches,
                        elevatorHeightSupplier() + 0.58.meters + WristConstants.WRIST_AXIS_TO_NOTE_HOLD_POSITION * wristAngleSupplier().sin
                    ),
                    Rotation3d(
                        0.0.degrees,
                        wristAngleSupplier() + WristConstants.NOTE_ANGLE_SIM_OFFSET,
                        0.0.degrees
                    )
                )
            )
    }

    fun canIntake(): Boolean {
        return (poseSupplier().transformBy(IntakeConstants.INTAKE_TRANSFORM) - currentPose.toPose2d()).translation.magnitude.meters < IntakeConstants.SIM_INTAKE_DISTANCE
    }


    enum class NoteStates {
            OFF_FIELD,
            STAGED,
            ON_FIELD,
            IN_ROBOT,
            IN_FLIGHT;
    }
}