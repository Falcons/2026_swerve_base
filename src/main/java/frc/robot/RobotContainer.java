// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.zeroGyro;
import frc.robot.subsystems.SwerveSubsystem;
// import swervelib.SwerveInputStream;

public class RobotContainer {

  // initialize robot's subsystems
  private final SwerveSubsystem swerve = new SwerveSubsystem();

  private final CommandXboxController driver = new CommandXboxController(0);

  // make a chooser option to select autos
  SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // variables

  public RobotContainer() {

    // silence joystick warning if simulation
    if (Robot.isSimulation()){DriverStation.silenceJoystickConnectionWarning(true);}

    // add default commands (run when no other commands are running)
    swerve.setDefaultCommand(new TeleopDrive( 
      swerve, 
      () -> MathUtil.applyDeadband(-driver.getLeftX(), ControllerConstants.deadBand), 
      () -> MathUtil.applyDeadband(-driver.getLeftY(), ControllerConstants.deadBand), 
      () -> MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.deadBand), 
      () -> !driver.getHID().getLeftBumper()));
    
    // Command driveFieldOrientedDirectAngle = swerve.driveCommand(
    //     () -> MathUtil.applyDeadband(driver.getLeftY(), ControllerConstants.deadBand),
    //     () -> MathUtil.applyDeadband(driver.getLeftX(), ControllerConstants.deadBand),
    //     () -> driver.getRightX(),
    //     () -> driver.getRightY());
        
        // swerve.setDefaultCommand(driveFieldOrientedDirectAngle);

    // Configure the button bindings
    configureBindings();
  }

  // // MOVEMENT IN ROBOT CONTAINER 
  // // this is kept here for testing just in case the command based movment doesnt work
  // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
  //   swerve.getSwerveDrive(),() -> driver.getLeftY(), () -> driver.getLeftX())
  //   .withControllerRotationAxis(driver::getRightX)
  //   .deadband(ControllerConstants.deadBand).scaleTranslation(0.8).allianceRelativeControl(true);
  // // a very weird turning where the angle is set by the controller directly?
  // SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driver::getRightX, driver::getRightY)
  //   .headingWhile(true);
  // // command for that thingy
  // Command driveFieldOrientedAngle = swerve.driveFieldOriented(driveDirectAngle);
  // Command driveFieldOrientedAngularVelocity = swerve.driveFieldOriented(driveAngularVelocity);
  // }

  private void configureBindings() {
    driver.b().onTrue(new zeroGyro(swerve));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}