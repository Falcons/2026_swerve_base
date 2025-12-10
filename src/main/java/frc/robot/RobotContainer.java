// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  private SwerveSubsystem drive = new SwerveSubsystem(DriveConstants.swerveDirectory);
  public RobotContainer() {
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> driverXbox.getLeftY() * -1,
    () -> driverXbox.getLeftX() * -1)
    .withControllerRotationAxis(driverXbox::getRightX)
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);
   //Clone's the angular velocity input stream and converts it to a robotRelative input stream
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
  .allianceRelativeControl(false);
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
