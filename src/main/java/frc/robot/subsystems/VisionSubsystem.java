// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.function.BiConsumer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
@Logged
public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator poseEstimator;
  private final AprilTagFieldLayout fieldLayout;
  private final BiConsumer<Pose2d, Double> consumer;
  @NotLogged private final DriveSubsystem drive;
  private Pose3d estimated3dPose;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(BiConsumer<Pose2d, Double> consumer, DriveSubsystem drive, String photonCameraName,
  Transform3d cameraOffset) throws IOException{
    photonCamera = new PhotonCamera(photonCameraName);

    fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
