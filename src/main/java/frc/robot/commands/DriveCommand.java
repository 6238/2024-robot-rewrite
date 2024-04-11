package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {

  private final SwerveSubsystem subsys;

  private final DoubleSupplier vX, vY;
  private final DoubleSupplier rotationSpeed;

  public DriveCommand(
      SwerveSubsystem subsys, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier rotationSpeed) {
    this.subsys = subsys;

    this.vX = vX;
    this.vY = vY;

    this.rotationSpeed = rotationSpeed;

    addRequirements(this.subsys);
  }

  @Override
  public void execute() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    double sign = (ally.get() == Alliance.Blue) ? 1.0 : -1.0;
    // Read from joysticks
    double driveY = Math.pow(vY.getAsDouble(), 1);
    double driveX = Math.pow(vX.getAsDouble(), 1);
    double rotation = rotationSpeed.getAsDouble() * Constants.MAX_ANGULAR_VELOCITY;

    Translation2d translation =
        new Translation2d(sign * driveX * subsys.maximumSpeed, sign * driveY * subsys.maximumSpeed);
    subsys.drive(translation, rotation, true);
  }
}
