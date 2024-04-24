// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Arm.*;
import static java.util.Map.entry;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.Alert;
import frc.robot.telemetry.Alert.AlertType;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class ArmSubsystem extends SubsystemBase {

  // Initialize control requests
  // Position 0, at rest, no FOC, no FF, use config slot 0, always brake when in
  // deadband, no limiting
  private final PositionVoltage voltagePosition =
      new PositionVoltage(0, 0, false, 0, 0, false, false, false);
  // Brake request to set when neutral

  private double kP = 1.0;
  private double kI = 0.05;
  private double kD = 0.0001;
  public double setpoint = 0;

  private TalonFXConfiguration configs = new TalonFXConfiguration();

  private final TalonFX motor1 = new TalonFX(11);
  private final TalonFX motor2 = new TalonFX(12);
  private final TalonFX motor3 = new TalonFX(13);

  private final TalonSRX sensorTalon = new TalonSRX(14);

  private Alert encoderDisconnected = new Alert("Arm encoder is disconnected", AlertType.WARNING);
  private Alert falconFailed =
      new Alert("Arm TalonFX failed to configure, possibly disconnected", AlertType.ERROR);

  private static final Map<ArmStates, Double> ANGLES =
      Map.ofEntries(
          entry(ArmStates.TRANSFER, 60.0),
          entry(ArmStates.INTAKE, 22.0),
          entry(ArmStates.SHOOT, 45.0),
          entry(ArmStates.STOW, 85.0));

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    configs.Feedback.SensorToMechanismRatio = ARM_TO_MOTOR_RATIO;

    configs.Slot0.kP = kP;
    configs.Slot0.kI = kI;
    configs.Slot0.kD = kD;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 3;
    configs.Voltage.PeakReverseVoltage = -3;

    // Set motors 2 and 3 to follow motor 1
    motor2.setControl(new Follower(motor1.getDeviceID(), false));
    motor2.setNeutralMode(NeutralModeValue.Brake);
    motor3.setControl(new Follower(motor1.getDeviceID(), false));
    motor3.setNeutralMode(NeutralModeValue.Brake);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor1.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs to motor 1, error code: " + status.toString());
      falconFailed.set(true);
    }

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor2.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs to motor 2, error code: " + status.toString());
      falconFailed.set(true);
    }

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor3.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs to motor 3, error code: " + status.toString());
      falconFailed.set(true);
    }

    // Figure out our starting position in degrees, and set the Falcon's onboard
    // encoders to correspond

    if (sensorTalon.getSensorCollection().getPulseWidthRiseToRiseUs() <= 10_000) {
      // if we're getting a pulse width of less than 100 hertz (10k microseconds
      // rise-to-rise), there's
      // a good chance the encoder is disconnected
      encoderDisconnected.set(true);
    }

    sensorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    sensorTalon.configFeedbackNotContinuous(true, 0);
    double startingPosition =
        Math.floorMod(
            (long) ((sensorTalon.getSelectedSensorPosition() - ENCODER_ZERO) * 360 / 4096),
            (long) 360.0);
    motor1.setPosition(startingPosition);

    // Set the arm setpoint to be the same as the starting position so that the arm
    // doesn't move until we explicitly tell it to go somewhere else
    setpoint = startingPosition;
    SmartDashboard.putNumber("setpoint", setpoint);
  }

  private double getArmAngle(double dist) {
    double a = -1.22533;
    double b = 12.1372;
    double c = 30.6778;
    double angle = a * Math.pow(dist, 2) + b * dist + c;
    if (angle > 19 && angle < 110) {
      setpoint = angle;
      return angle;
    } else {
      return 75;
    }
  }

  public Command runPIDCommand() {
    return runOnce(
        () -> {
          motor1.setControl(voltagePosition.withPosition(setpoint));
        });
  }

  public Command autoSetAngleCommand(DoubleSupplier poseX, DoubleSupplier poseY) {
    // TODO: Should this be a runOnce or a run?
    return runOnce(
        () -> {
          this.autoSetAngle(poseX, poseY);
        });
  }

  private void autoSetAngle(DoubleSupplier poseX, DoubleSupplier poseY) {
    Optional<Alliance> ally = DriverStation.getAlliance();
    double speakerX = (ally.get() == Alliance.Blue) ? 0.0 : 16.579342;
    double dist = Math.hypot(poseY.getAsDouble() - 5.547868, poseX.getAsDouble() - speakerX);
    SmartDashboard.putNumber("distFromSpeaker", dist);
    motor1.setControl(voltagePosition.withPosition(getArmAngle(dist)));
    // motor1.setControl(voltagePosition.withPosition(SmartDashboard.getNumber("setpoint2",
    // 47)));
  }

  public Command runPIDwithAngle(ArmStates angle) {
    return runOnce(
        () -> {
          setpoint = ANGLES.get(angle);
          motor1.setControl(voltagePosition.withPosition(ANGLES.get(angle)));
        });
  }

  public Command increaseSetpointCommand() {
    return runOnce(
        () -> {
          this.setpoint += 5; // one more motor turn
        });
  }

  public Command decreaseSetpointCommand() {
    return Commands.runOnce(
        () -> {
          this.setpoint -= 5; // one more motor turn
        });
  }

  public void setAngle(Double angle) {
    this.setpoint = angle;
  }

  public Command setAngleCommand(Double angle) {
    return Commands.runOnce(
        () -> {
          this.setpoint = angle;
        });
    // don't use this.runOnce because it implicitly requires this, which is not what
    // we want (don't stop the loop to change the setpt)
  }

  public Command setAngleCommand(ArmStates state) {
    return Commands.runOnce(
        () -> {
          this.setpoint = ANGLES.get(state);
        });
    // don't use this.runOnce because it implicitly requires this, which is not what
    // we want (don't stop the loop to change the setpt)
  }

  public Command setAngleCommand(DoubleSupplier angle) {
    return Commands.runOnce(
        () -> {
          this.setpoint = angle.getAsDouble();
        });
    // don't use this.runOnce because it implicitly requires this, which is not what
    // we want (don't stop the loop to change the setpt)
  }

  public Command setBrakeCommand(boolean brake) {
    return Commands.runOnce(
            () -> {
              motor1.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
              motor2.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
              motor3.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            })
        .ignoringDisable(true);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
