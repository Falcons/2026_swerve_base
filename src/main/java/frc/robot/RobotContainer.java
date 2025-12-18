// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
  private SwerveSubsystem drive = new SwerveSubsystem(DriveConstants.swerveDirectory);
  private CommandXboxController driverController = new CommandXboxController(0);
  private SendableChooser<Command> autoChooser;
  public RobotContainer() {
    NamedCommands.registerCommand("elevator L2", Commands.print("L2!!!!!"));

    autoChooser = AutoBuilder.buildAutoChooser();
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
