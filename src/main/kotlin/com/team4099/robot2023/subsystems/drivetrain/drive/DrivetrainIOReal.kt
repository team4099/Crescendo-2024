package com.team4099.robot2023.subsystems.drivetrain.drive

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.swervemodule.SwerveModule
import com.team4099.robot2023.subsystems.drivetrain.swervemodule.SwerveModuleIONeo

object DrivetrainIOReal : DrivetrainIO {
  override fun getSwerveModules(): List<SwerveModule> {
    return listOf(
      SwerveModule(
        SwerveModuleIONeo(
          CANSparkMax(
            Constants.Drivetrain.FRONT_LEFT_STEERING_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
          ),
          CANSparkMax(
            Constants.Drivetrain.FRONT_LEFT_DRIVE_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
          ),
          DrivetrainConstants.FRONT_LEFT_MODULE_ZERO,
          Constants.Drivetrain.FRONT_LEFT_MODULE_NAME
        )
      ),
      SwerveModule(
        SwerveModuleIONeo(
          CANSparkMax(
            Constants.Drivetrain.FRONT_RIGHT_STEERING_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
          ),
          CANSparkMax(
            Constants.Drivetrain.FRONT_RIGHT_DRIVE_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
          ),
          DrivetrainConstants.FRONT_RIGHT_MODULE_ZERO,
          Constants.Drivetrain.FRONT_RIGHT_MODULE_NAME
        )
      ),
      SwerveModule(
        SwerveModuleIONeo(
          CANSparkMax(
            Constants.Drivetrain.BACK_LEFT_STEERING_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
          ),
          CANSparkMax(
            Constants.Drivetrain.BACK_LEFT_DRIVE_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
          ),
          DrivetrainConstants.BACK_LEFT_MODULE_ZERO,
          Constants.Drivetrain.BACK_LEFT_MODULE_NAME
        )
      ),
      SwerveModule(
        //        object: SwerveModuleIO {
        //          override val label: String = Constants.Drivetrain.BACK_RIGHT_MODULE_NAME
        //        }
        SwerveModuleIONeo(
          CANSparkMax(
            Constants.Drivetrain.BACK_RIGHT_STEERING_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
          ),
          CANSparkMax(
            Constants.Drivetrain.BACK_RIGHT_DRIVE_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless
          ),
          DrivetrainConstants.BACK_RIGHT_MODULE_ZERO,
          Constants.Drivetrain.BACK_RIGHT_MODULE_NAME
        )
      )
    )
  }
}
