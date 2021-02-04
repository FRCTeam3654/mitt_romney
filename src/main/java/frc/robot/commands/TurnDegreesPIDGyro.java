// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnDegreesPIDGyro extends PIDCommand {
  /** Creates a new TurnDegreesPIDGyro. */
  public TurnDegreesPIDGyro(double speed, double degrees, Drivetrain drive) {
    super(
        // The controller that the command will use
        new PIDController(Constants.turnPIDGyroP, Constants.turnPIDGyroI, Constants.turnPIDGyroD),
        // This should return the measurement
        drive::getGyroAngleZ,
        // This should return the setpoint (can also be a constant)
        degrees,
        // This uses the output
        output -> drive.arcadeDrive(0, output),
          // Use the output here
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.turnToleranceDeg, Constants.turnRateToleranceDegPerS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
