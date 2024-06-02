// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.telemetry.Alert;
import frc.robot.telemetry.Alert.AlertType;

public class IntakeOuttakeSubsystem extends SubsystemBase {
  public CANSparkBase intakeMotor;
  public CANSparkBase outtakeTopMotor;
  public CANSparkBase outtakeBottomMotor;
  private SparkPIDController top_pidController;
  private SparkPIDController bottom_pidController;
  private SparkPIDController intake_pidController;
  public RelativeEncoder top_encoder;
  private RelativeEncoder bottom_encoder;
  private RelativeEncoder intake_encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private DigitalInput limitSwitch1 = new DigitalInput(9);
  private DigitalInput limitSwitch2 = new DigitalInput(8);
  private DigitalInput limitSwitch3 = new DigitalInput(7);

  private Alert motorFailed = new Alert("An intake/outtake motor failed to config", AlertType.ERROR);


  /** Creates a new IntakeOuttakeSubsystem. */
  public IntakeOuttakeSubsystem() {

    intakeMotor = new CANSparkMax(IDs.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    outtakeTopMotor = new CANSparkMax(IDs.OUTTAKE_TOP_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    outtakeBottomMotor = new CANSparkMax(IDs.OUTTAKE_BOTTOM_MOTOR, CANSparkLowLevel.MotorType.kBrushless);

    // Reset each motor so the configs are known-good
    if (intakeMotor.restoreFactoryDefaults() != REVLibError.kOk) {
      motorFailed.set(true);
    }
    if (outtakeTopMotor.restoreFactoryDefaults() != REVLibError.kOk) {
      motorFailed.set(true);
    }  
    if (outtakeBottomMotor.restoreFactoryDefaults() != REVLibError.kOk) {
      motorFailed.set(true);
    }  

    intakeMotor.setInverted(false);

    intakeMotor.setSmartCurrentLimit(80);
    outtakeTopMotor.setSmartCurrentLimit(40);
    outtakeTopMotor.setSecondaryCurrentLimit(20);
    outtakeBottomMotor.setSmartCurrentLimit(40);
    outtakeBottomMotor.setSecondaryCurrentLimit(20);

    intakeMotor.setIdleMode(IdleMode.kBrake);
    outtakeTopMotor.setIdleMode(IdleMode.kCoast);
    outtakeBottomMotor.setIdleMode(IdleMode.kCoast);

    top_pidController = outtakeTopMotor.getPIDController();
    bottom_pidController = outtakeBottomMotor.getPIDController();

    top_encoder = outtakeTopMotor.getEncoder();
    bottom_encoder = outtakeBottomMotor.getEncoder();
    intake_encoder = intakeMotor.getEncoder();

    kP = OuttakeGains.kP; 
    kI = OuttakeGains.kI;
    kD = OuttakeGains.kD;
    // SmartDashboard.putNumber("shooterKp", kP);
    // SmartDashboard.putNumber("shooterKi", kI);
    // SmartDashboard.putNumber("shooterKd", kD);
    kIz = OuttakeGains.kIz; 
    kFF = OuttakeGains.kFF; 
    kMaxOutput = OuttakeGains.kMaxOutput; 
    kMinOutput = OuttakeGains.kMinOutput;
    maxRPM = OuttakeGains.maxRPM;

    top_pidController.setP(kP);
    top_pidController.setI(kI);
    top_pidController.setD(kD);
    top_pidController.setIZone(kIz);
    top_pidController.setFF(kFF);
    top_pidController.setOutputRange(kMinOutput, kMaxOutput);

    bottom_pidController.setP(kP);
    bottom_pidController.setI(kI);
    bottom_pidController.setD(kD);
    bottom_pidController.setIZone(kIz);
    bottom_pidController.setFF(kFF);
    bottom_pidController.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("shooterRPM", 1000.0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command setMotorsCommand(double intake, double outtake) {
    return runOnce(() -> {
      intakeMotor.set(intake);
      SmartDashboard.putNumber("shooterSpeedSetpoint", outtake);
      top_pidController.setReference(-outtake, CANSparkMax.ControlType.kVelocity);
      bottom_pidController.setReference(outtake, CANSparkMax.ControlType.kVelocity);
    });
  }

  public Command startOutakeCommand() {
    return this.setMotorsCommand(0.0, Constants.Speeds.OUTTAKE_SPEED);

  }

  public Command stopCommand() {
    return this.setMotorsCommand(0, 0);
  }

  public Command ejectCommand() {
    return this.setMotorsCommand(-Constants.Speeds.INTAKE_SPEED, 0);
  }

  public Trigger stalled() {
    return new Trigger(() -> !limitSwitch1.get() || !limitSwitch2.get() || !limitSwitch3.get());
  }
}
