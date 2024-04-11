// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  CommandXboxController driverXbox = new CommandXboxController(0);


  public RobotContainer() {
    configureBindings();

    DriveCommand driveCmd = new DriveCommand(
      swerve,
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.02), // Y axis on joystick is X axis for FRC. Forward is postive-Y, so need to invert sign
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.02), // X axis on joystick is Y axis for FRC. Left is positive-X, so need to invert sign
      () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.08)
      ); // Rotation for FRC is CCW-positive, so need to invert sign
      swerve.setDefaultCommand(driveCmd);
  }

  private void configureBindings() {
    driverXbox.start().onTrue((new InstantCommand(swerve::zeroGyro)));

    // Reset pose-estimation when starting auton
    RobotModeTriggers.autonomous().onTrue(new InstantCommand(() -> {swerve.resetGyroTo(swerve.getPose().getRotation());}));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
