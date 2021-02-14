// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegreesGyroStartAngle extends CommandBase {
  private double m_speed;  
  private final double m_degrees;
  private final double m_startAngle;
  private final Drivetrain m_drive;
  private double m_turnEndpointAngle;
  private double m_vinniesError;

  /** Creates a new TurnDegreesGyro. */
  public TurnDegreesGyroStartAngle(double speed, double degrees, double startAngle, Drivetrain drive) {
    m_speed = speed;
    m_degrees = degrees;
    m_startAngle = startAngle;
    m_drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set motors to stop
    m_drive.arcadeDrive(0, 0);
    m_turnEndpointAngle = m_startAngle + m_degrees;
    m_vinniesError = m_turnEndpointAngle - m_drive.getGyroAngleZ();
  }

  //michele w
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vinniesError = m_turnEndpointAngle - m_drive.getGyroAngleZ();
    m_speed = m_vinniesError * Constants.turnPIDGyroD*-1;

    m_speed = Math.min(1, m_speed);
    m_speed = Math.max(-1, m_speed);

    m_drive.arcadeDrive(0, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_vinniesError) <= Constants.turnToleranceDeg;
  }
}
