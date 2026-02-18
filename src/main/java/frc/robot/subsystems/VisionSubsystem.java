// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

@Logged
public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator poseEstimator;
  private final AprilTagFieldLayout fieldLayout;
  private final BiConsumer<Pose2d, Double> consumer;
  @NotLogged private final DriveSubsystem drive;
  private Pose3d estimated3dPose;
  
  // Hopper targeting data for Advantage Scope logging
  @Logged private double hopperYaw = 0.0;
  @Logged private double hopperPitch = 0.0;
  @Logged private double hopperDistance = 0.0;
  @Logged private double hopperAmbiguity = 1.0;
  @Logged private boolean hopperVisible = false;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(BiConsumer<Pose2d, Double> consumer, DriveSubsystem drive, String LeftSideCamera,
      Transform3d cameraOffset) throws IOException {
    photonCamera = new PhotonCamera(LeftSideCamera);

    // Log camera name and offset to help tune the mounting transform
    System.out.println("[VisionSubsystem] Camera name: " + LeftSideCamera + ", cameraOffset=" + cameraOffset);

    fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
    poseEstimator = new PhotonPoseEstimator(fieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        cameraOffset);
    estimated3dPose = new Pose3d();
    this.consumer = consumer;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!photonCamera.isConnected()) {
      hopperVisible = false;
      return;
    }

    var pipeLineResult = photonCamera.getLatestResult();
    boolean hasTargets = pipeLineResult.hasTargets();
    if (!hasTargets) {
      hopperVisible = false;
      return;
    }

    // Filter targets by ambiguity and distance (use previous estimated pose if available,
    // otherwise fall back to odometry). This avoids biasing distance checks to only the
    // odometry pose and lets the pose estimator act on multiple tags.
    Pose2d robotPoseForFiltering = (estimated3dPose != null) ? estimated3dPose.toPose2d() : drive.getPose();
    List<PhotonTrackedTarget> badTargets = new ArrayList<>();
    for (PhotonTrackedTarget target : pipeLineResult.targets) {
      var tagPose = fieldLayout.getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) {
        badTargets.add(target);
        continue;
      }
      var distanceToTag = PhotonUtils.getDistanceToPose(robotPoseForFiltering, tagPose.get().toPose2d());
      if (target.getPoseAmbiguity() > VisionConstants.kMaxAmbiguity || distanceToTag > VisionConstants.kMaxDistanceMeters) {
        badTargets.add(target);
      }
    }
    pipeLineResult.targets.removeAll(badTargets);

    // Update robot pose estimation first (so we prefer multi-tag pose estimation for
    // downstream calculations when available), then update hopper targeting using the
    // estimated pose if present.
    Optional<EstimatedRobotPose> poseResult = poseEstimator.update(pipeLineResult);
    if (poseResult.isPresent()) {
      EstimatedRobotPose estimatedPose = poseResult.get();
      estimated3dPose = estimatedPose.estimatedPose;
      // Provide the new estimate to the drive (caller) via the consumer
      consumer.accept(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
      // Use the estimator's pose for hopper distance and other calculations
      updateHopperTargeting(pipeLineResult, estimatedPose.estimatedPose.toPose2d());
    } else {
      // No pose estimate available this cycle; fall back to odometry-based calculations
      updateHopperTargeting(pipeLineResult, drive.getPose());
    }
  }

  /**
   * Update hopper targeting data from camera results
   */
  private void updateHopperTargeting(org.photonvision.targeting.PhotonPipelineResult result, Pose2d robotPose) {
    hopperVisible = false;
    hopperYaw = 0.0;
    hopperPitch = 0.0;
    hopperDistance = 0.0;
    hopperAmbiguity = 1.0;

    if (!result.hasTargets())
      return;

    // Find the hopper April tag
    for (PhotonTrackedTarget target : result.targets) {
      if (target.getFiducialId() == VisionConstants.kHopperTagId) {
        hopperVisible = true;
        hopperYaw = target.getYaw();
        hopperPitch = target.getPitch();
        hopperAmbiguity = target.getPoseAmbiguity();

        // Calculate distance from tag pose
        var tagPose = fieldLayout.getTagPose(target.getFiducialId());
        if (tagPose.isPresent()) {
          hopperDistance = PhotonUtils.getDistanceToPose(robotPose, tagPose.get().toPose2d());
        }
        return; // Only track the first hopper tag found
      }
    }
  }

  /**
   * Check if the camera can see the hopper April tag
   * @return true if hopper tag is visible
   */
  public boolean canSeeHopper() {
    return hopperVisible;
  }

  /**
   * Get the yaw (horizontal angle) to the hopper April tag
   * Positive yaw = target is to the right
   * Negative yaw = target is to the left
   * @return Yaw in degrees, or 0 if hopper not visible
   */
  public double getHopperYaw() {
    return hopperYaw;
  }

  /**
   * Get the pitch (vertical angle) to the hopper April tag
   * @return Pitch in degrees, or 0 if hopper not visible
   */
  public double getHopperPitch() {
    return hopperPitch;
  }

  /**
   * Get the distance to the hopper
   * @return Distance in meters
   */
  public double getHopperDistance() {
    return hopperDistance;
  }

  /**
   * Get the ambiguity of the hopper target (0 = perfect, 1 = ambiguous)
   * @return Ambiguity value
   */
  public double getHopperAmbiguity() {
    return hopperAmbiguity;
  }

  /**
   * Get the estimated 3D pose of the robot
   * @return Pose3d
   */
  public Pose3d getEstimated3dPose() {
    return estimated3dPose;
  }
}
