// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsytem;

public class SwerveDriveCommand extends Command {
  /** Creates a new SwerveDriveCommand. */
  private static final double DEADBAND = 0.1;
  private static final double ROTATION_COEFFICIENT = 0.75;
  private final SwerveSubsytem drivetrain;
  private final Supplier<Double> xSupplier;
  private final Supplier<Double> ySupplier;
  private final Supplier<Double> angularSupplier;
  private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
  .withDeadband(DEADBAND);

    public double xVelocity;
    public double yVelocity;


  public SwerveDriveCommand(
    SwerveSubsytem swerveSubsytem,
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    Supplier<Double> angulaSupplier
  ) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = swerveSubsytem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.angularSupplier = angulaSupplier;

    addRequirements(swerveSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xVelocity = m_driveRequest.withVelocityX(xSupplier.get()).VelocityX;
    yVelocity = m_driveRequest.withVelocityY(ySupplier.get()).VelocityY;

    SmartDashboard.putNumber("x", xVelocity);
    SmartDashboard.putNumber("y", yVelocity);

    double angle = m_driveRequest.withRotationalRate(angularSupplier.get()).RotationalRate;

    ChassisSpeeds setpoint = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angle, null);

    drivetrain.setChassisSpeed(setpoint);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
