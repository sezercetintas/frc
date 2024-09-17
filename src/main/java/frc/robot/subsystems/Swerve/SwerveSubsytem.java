// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class SwerveSubsytem extends SubsystemBase {
  /** Creates a new SwerveSubsytem. */
  
  private final SwerveModule frontLeftModule = new SwerveModule(
    Constants.DriveConstants.kFrontLeftDriveMotorId,
    Constants.DriveConstants.kFrontLeftSteerMotorId,
    Constants.DriveConstants.kFrontLeftCANcoderId,
    Constants.DriveConstants.kFrontLeftDriveMotorInverted,
    Constants.DriveConstants.kFrontLeftSteerMotorInverted,
    Constants.DriveConstants.kFrontLeftCANcoderOffset,
    "Swerve"
  );

  private final SwerveModule frontRightModule = new SwerveModule(
    Constants.DriveConstants.kFrontRightDriveMotorId,
    Constants.DriveConstants.kFrontRightSteerMotorId,
    Constants.DriveConstants.kFrontRightCANcoderId,
    Constants.DriveConstants.kFrontRightDriveMotorInverted,
    Constants.DriveConstants.kFrontRightSteerMotorInverted,
    Constants.DriveConstants.kFrontRightCANcoderOffset,
    "Swerve"
  );

  private final SwerveModule backLeftModule = new SwerveModule(
    Constants.DriveConstants.kBackLeftDriveMotorId,
    Constants.DriveConstants.kBackLeftSteerMotorId,
    Constants.DriveConstants.kBackLeftCANcoderId,
    Constants.DriveConstants.kBackLeftDriveMotorInverted,
    Constants.DriveConstants.kBackLeftSteerMotorInverted,
    Constants.DriveConstants.kBackLeftCANcoderOffset,
    "Swerve"
  );
  private final SwerveModule backRightModule = new SwerveModule(
    Constants.DriveConstants.kBackRightDriveMotorId,
    Constants.DriveConstants.kBackRightSteerMotorId,
    Constants.DriveConstants.kBackRightCANcoderId,
    Constants.DriveConstants.kBackRightDriveMotorInverted,
    Constants.DriveConstants.kBackRightSteerMotorInverted,
    Constants.DriveConstants.kBackRightCANcoderOffset,
    "Swerve"
  );

  private final SwerveDriveKinematics driveKinematics = Constants.DriveConstants.kDriveKinematics;

  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
    driveKinematics,
    new Rotation2d(0),
    new SwerveModulePosition[]{
      frontLeftModule.getPosition(true),
      frontRightModule.getPosition(true),
      backLeftModule.getPosition(true),
      backRightModule.getPosition(true)
    }
  );


  private final Pigeon2 pidgey = new Pigeon2(Constants.DriveConstants.kPigeon2Id, "Swerve");

  private final StatusSignal<Double> yawStatusSignal = pidgey.getYaw().clone();
  private final StatusSignal<Double> angularVelocityStatusSignal = pidgey.getAngularVelocityZWorld().clone();
  private double yawDegrees;

  private CommandXboxController controller = new CommandXboxController(0);
  
  public SwerveSubsytem(CommandXboxController controller) {
    this.controller = controller;
  }

  public void setChassisSpeed(ChassisSpeeds desired){
    SwerveModuleState[] newModuleStates = driveKinematics.toSwerveModuleStates(desired);
    setModuleStates(newModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    driveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  public double getYawDegrees(){
    double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(yawStatusSignal, angularVelocityStatusSignal);
    return yawDegrees;
  }

  public void stop(){
    frontLeftModule.stopModule();
    frontRightModule.stopModule();
    backLeftModule.stopModule();
    backRightModule.stopModule();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometer.update(
      Rotation2d.fromDegrees(yawDegrees),
      new SwerveModulePosition[]{
      frontLeftModule.getPosition(false),
      frontRightModule.getPosition(false),
      backLeftModule.getPosition(false),
      backRightModule.getPosition(false)        
      }
    );

    SmartDashboard.putNumber("Yaw", getYawDegrees());

    SmartDashboard.putNumber("AbsEnc1", frontLeftModule.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("AbsEnc2", frontRightModule.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("AbsEnc3", backRightModule.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("AbsEnc4", backLeftModule.getAbsoluteEncoderRad());

    double logging_states[] = {
      frontLeftModule.m_targetState.angle.getRadians(),
      frontLeftModule.m_targetState.speedMetersPerSecond,
      frontRightModule.m_targetState.angle.getRadians(),
      frontRightModule.m_targetState.speedMetersPerSecond,  
      backLeftModule.m_targetState.angle.getRadians(),
      backLeftModule.m_targetState.speedMetersPerSecond,  
      backRightModule.m_targetState.angle.getRadians(),
      backRightModule.m_targetState.speedMetersPerSecond                                                  
    };

    SmartDashboard.putNumberArray("swervestates", logging_states);

    SmartDashboard.putNumber("robot x", odometer.getPoseMeters().getX());
    SmartDashboard.putNumber("robot y", odometer.getPoseMeters().getY());
    SmartDashboard.putNumber("robot angle", odometer.getPoseMeters().getRotation().getRadians());
  }
}
