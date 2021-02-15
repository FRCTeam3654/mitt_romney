// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Drivetrain drivetrain) {
      double startingGyroAngle = drivetrain.getGyroAngleZ();

    addCommands(
        new DriveDistance(0.5, 18, drivetrain),
        new TurnDegreesGyroStartAngle(-0.5, 90, startingGyroAngle, drivetrain));
        //new DriveDistance(0.5, 18, drivetrain), //at yellow
        //new TurnDegreesGyroStartAngle(-0.5, 180, startingGyroAngle, drivetrain),
        //new DriveDistance(0.5, 15, drivetrain),
        //new TurnDegreesGyroStartAngle(-0.5, 270, startingGyroAngle, drivetrain),
        //new DriveDistance(0.5, 7, drivetrain),
        //new TurnDegreesGyroStartAngle(0.5, 180, startingGyroAngle, drivetrain),
        //new DriveDistance(0.5, 12, drivetrain), //at blue
       // new TurnDegreesGyroStartAngle(0.5, 90, startingGyroAngle, drivetrain),
       // new DriveDistance(0.5, 18, drivetrain),
       // new TurnDegreesGyroStartAngle(0.5, 0, startingGyroAngle, drivetrain),
       // new DriveDistance(0.5, 20, drivetrain)); //at red
  }
}
