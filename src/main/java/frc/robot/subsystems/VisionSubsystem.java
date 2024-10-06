// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.vision.PopcornCamera;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class VisionSubsystem extends SubsystemBase {
  private AprilTagFieldLayout layout;
  private final Transform3d robotToCam_A =
      new Transform3d(
          new Translation3d(Inches.of(-13.5), Inches.of(-13.5), Inches.of(10.5)),
          new Rotation3d(
              Degrees.of(180).in(Radians),
              Degrees.of(-55).in(Radians),
              Degrees.of(210).in(Radians)));
  private final Transform3d robotToCam_B =
      new Transform3d(
          new Translation3d(Inches.of(-13.5), Inches.of(13.5), Inches.of(10.5)),
          new Rotation3d(
              Degrees.of(0).in(Radians), Degrees.of(-55).in(Radians), Degrees.of(150).in(Radians)));

  private PopcornCamera camA;

  private Optional<EstimatedRobotPose> poseA;
  private Optional<EstimatedRobotPose> poseB;

  private PopcornCamera camB;

  private SwerveSubsystem swerve;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerve) {

    this.swerve = swerve;

    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    camA = new PopcornCamera(Constants.Vision.PHOTONCAMERA_NAME_A, robotToCam_A, layout);
    camB = new PopcornCamera(Constants.Vision.PHOTONCAMERA_NAME_B, robotToCam_B, layout);
  }

  @Override
    public void periodic() {
        Optional<EstimatedRobotPose> poseA = camA.update();
        if (poseA.isPresent()) {
            swerve.addVisionPose(poseA.get(), Constants.VISION_STDDEV);
        }

        Optional<EstimatedRobotPose> poseB = camB.update();
        if (poseB.isPresent()) {
            swerve.addVisionPose(poseB.get(), Constants.VISION_STDDEV);
        }
    }

  public Trigger hasPose() {
    return new Trigger(() -> poseA.isPresent() || poseB.isPresent());
  }

  // TODO: Add a way to get the pose out
}
