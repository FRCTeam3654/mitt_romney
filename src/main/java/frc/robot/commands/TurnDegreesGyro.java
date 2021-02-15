// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegreesGyro extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_degrees;
  private double m_speed;
  private double m_turnEndpointAngle;
  private double m_vinniesError;

  /** Creates a new TurnDegreesGyro. */
  public TurnDegreesGyro(double speed, double degrees, Drivetrain drive) {
    m_degrees = degrees;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set motors to stop. reset gyro value for starting point 
    m_drive.resetGyro();
    m_turnEndpointAngle = m_drive.getGyroAngleZ() + m_degrees;
    m_vinniesError = m_turnEndpointAngle - m_drive.getGyroAngleZ();
    m_drive.arcadeDrive(0, 0);
  }

  //michele w
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vinniesError = m_turnEndpointAngle - m_drive.getGyroAngleZ();
    m_speed = m_vinniesError * Constants.turnPIDGyroD*-1;

    m_speed = Math.min(1, m_speed);
    m_speed = Math.max(-1, m_speed);

    if (m_speed> 0 && m_speed < 0.2) m_speed = 0.2;
    if (m_speed< 0 && m_speed > -0.2) m_speed = -0.2;
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
