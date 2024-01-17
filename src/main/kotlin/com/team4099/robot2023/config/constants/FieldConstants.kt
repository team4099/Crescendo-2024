package com.team4099.robot2023.config.constants

import edu.wpi.first.math.geometry.Rotation2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.base.inches

object FieldConstants {
    val fieldLength = 651.223.inches
    val fieldWidth = 323.277.inches
//    val wingX = 229.201.inches
//    val podiumX = 126.75.inches
//    val startingLineX = 74.111.inches
//    val ampCenter = Translation2d(72.455.inches, 322.996.inches)
//    val subwooferX = 3.feet + (1/8).inches
//
//    // corners (blue alliance origin)
//    val topRightSpeaker = Translation3d(
//      18.055.inches,
//      238.815.inches,
//      13.091.inches
//    )
//    val topLeftSpeaker = Translation3d(
//      18.055.inches,
//      197.765.inches,
//      83.091.inches
//    )
//    val bottomRightSpeaker = Translation3d(
//      0.inches,
//      238.815.inches,
//      78.324.inches
//    )
//    val bottomLeftSpeaker = Translation3d(
//      0.inches,
//      197.765.inches,
//      78.324.inches
//    )
//
//    /** Staging locations for each note  */
//    object StagingLocations {
//      val centerLineX = fieldLength / 2
//
//      // need to update
//      val centerLineFirstY = (29.638).inches
//      val centerLineSeparationY = (66).inches
//      val spikeX  = (114).inches
//
//      // need
//      val spikeFirstY: Double = Units.inchesToMeters(161.638)
//      val spikeSeparationY: Double = Units.inchesToMeters(57)
//      val centerlineTranslations: Array<Translation2d?> = arrayOfNulls<Translation2d>(5)
//      val spikeTranslations: Array<Translation2d?> = arrayOfNulls<Translation2d>(3)
//
//      init {
//        for (i in centerlineTranslations.indices) {
//          centerlineTranslations[i] = Translation2d(
//            centerlineX,
//            centerlineFirstY + i * centerlineSeparationY
//          )
//        }
//      }
//
//      init {
//        for (i in spikeTranslations.indices) {
//          spikeTranslations[i] = Translation2d(
//            spikeX,
//            spikeFirstY + i * spikeSeparationY
//          )
//        }
//      }
//    }
//
//    /** Each corner of the speaker *  */
//    object Speaker {
//      /** Center of the speaker opening (blue alliance)  */
//      val centerSpeakerOpening: Pose2d = Pose2d(0.0, fieldWidth - Units.inchesToMeters(104.0), Rotation2d())
//    }
//  }
}
