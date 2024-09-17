// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.util.concurrent.TransferQueue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ModuleConstants{
    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1/ 6.12;
    public static final double kSteerMotorGearRatio = 1/ 12.8;

    public static final double kSlipCurrent = 400;
  }

  public static class DriveConstants{

    public static final double kTrackWidth = 0.53;
    // Distance between right and left wheels
    public static final double kWheelBase = 0.43;
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
     // Test   new TransferQueue.......

    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontRightDriveMotorId = 2;
    public static final int kBackLeftDriveMotorId = 3;
    public static final int kBackRightDriveMotorId = 4;

    public static final int kFrontLeftSteerMotorId = 5;
    public static final int kFrontRightSteerMotorId = 6;
    public static final int kBackLeftSteerMotorId = 7;
    public static final int kBackRightSteerMotorId = 8;

    public static final int kFrontLeftCANcoderId = 9;
    public static final int kFrontRightCANcoderId = 10;
    public static final int kBackLeftCANcoderId = 11;
    public static final int kBackRightCANcoderId = 12;

    public static final int kPigeon2Id = 13;

    public static final boolean kFrontLeftDriveMotorInverted = false;
    public static final boolean kFrontRightDriveMotorInverted = false;
    public static final boolean kBackLeftDriveMotorInverted = false;
    public static final boolean kBackRightDriveMotorInverted = false;

    public static final boolean kFrontLeftSteerMotorInverted = false;
    public static final boolean kFrontRightSteerMotorInverted = false;
    public static final boolean kBackLeftSteerMotorInverted = false;
    public static final boolean kBackRightSteerMotorInverted = false;

    public static final double kFrontLeftCANcoderOffset = 0;
    public static final double kFrontRightCANcoderOffset = 0;
    public static final double kBackLeftCANcoderOffset = 0;
    public static final double kBackRightCANcoderOffset = 0;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 10.0;
  }
}
