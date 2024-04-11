// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;

public class RobotContainer {

  SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  CommandXboxController driverXbox = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

    swerve.setDefaultCommand(
        swerve.driveCommand(
            () ->
                MathUtil.applyDeadband(
                    -driverXbox.getLeftY(),
                    0.02), // Y axis on joystick is X axis for FRC. Forward is postive-Y, so need to
            // invert sign
            () ->
                MathUtil.applyDeadband(
                    -driverXbox.getLeftX(),
                    0.02), // X axis on joystick is Y axis for FRC. Left is positive-X, so need to
            // invert sign
            () ->
                MathUtil.applyDeadband(
                    -driverXbox.getRightX(),
                    0.08))); // Rotation for FRC is CCW-positive, so need to invert sign));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auton Path", autoChooser);
  }

  private void configureBindings() {
    driverXbox.start().onTrue(swerve.zeroYawCommand());

    // Reset pose-estimation when starting auton
    RobotModeTriggers.autonomous().onTrue(swerve.resetGyroCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
