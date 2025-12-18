// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
  private SwerveSubsystem drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  private CommandXboxController driverController = new CommandXboxController(0);
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  public RobotContainer() {

    autoChooser.setDefaultOption("No Auto", Commands.print("No autonomous command configured"));
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
      new frc.robot.Commands.Drive.manualDrive(
        drive,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        driverController.rightBumper().getAsBoolean()
      )
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
