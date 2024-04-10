// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  CommandXboxController driverXbox = new CommandXboxController(0);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driverXbox.start().onTrue((new InstantCommand(swerve::zeroGyro)));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
