// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class TurnToAngleCmd extends CommandBase {
  Swerve swerveBase;
  double angleDegrees;
  DoubleSupplier m_forward;
  DoubleSupplier m_strafe;
  DoubleSupplier m_speedReduction;
  private final SlewRateLimiter xLimiter, yLimiter;
  final double ANGULAR_P = 0.18;
  final double ANGULAR_D = 0.004;
  PIDController turnController = new PIDController(ANGULAR_P, 0.0, ANGULAR_D);
  double rotationSpeed;

  /** Creates a new TurnToAngleCmd. */
  public TurnToAngleCmd(Swerve swerveBase,
  double angleDegrees,
  DoubleSupplier forward,
  DoubleSupplier strafe,
  DoubleSupplier speedReduction) {
  this.swerveBase = swerveBase;
  this.angleDegrees = angleDegrees;
  m_forward = forward;
  m_strafe = strafe;
  m_speedReduction = speedReduction;
  turnController.enableContinuousInput(-180, 180);
  this.xLimiter = new SlewRateLimiter(Constants.Swerve.kTeleDriveMaxAccelerationUnitsPerSecond);
  this.yLimiter = new SlewRateLimiter(Constants.Swerve.kTeleDriveMaxAccelerationUnitsPerSecond);
    addRequirements(swerveBase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("set point", angleDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotationSpeed = turnController.calculate(swerveBase.getYaw().getDegrees(), angleDegrees);

    double fwdX = m_forward.getAsDouble();
    double fwdY = m_strafe.getAsDouble();

    // 2. Apply deadband
    fwdX = Math.abs(fwdX) > 0.1 ? fwdX : 0.0;
    fwdY = Math.abs(fwdY) > 0.1 ? fwdY : 0.0;


    // 3. Make the driving smoother
    fwdX = xLimiter.calculate(fwdX) * Constants.Swerve.kTeleDriveMaxSpeedMetersPerSecond * m_speedReduction.getAsDouble();
    fwdY = yLimiter.calculate(fwdY) * Constants.Swerve.kTeleDriveMaxSpeedMetersPerSecond * m_speedReduction.getAsDouble();


    swerveBase.drive(new Translation2d(-fwdX, -fwdY), rotationSpeed,true, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}