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
import edu.wpi.first.units.measure.Distance;
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

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(BiConsumer<Pose2d, Double> consumer, DriveSubsystem drive, String LeftSideCamera,
  Transform3d cameraOffset) throws IOException{
    photonCamera = new PhotonCamera(LeftSideCamera);

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
    if (!photonCamera.isConnected())
      return;

    var pipeLineResult = photonCamera.getLatestResult();
    boolean hasTargets = pipeLineResult.hasTargets();
    if (!hasTargets)
    return;

    List<PhotonTrackedTarget> badTargets = new ArrayList<>();
    for(PhotonTrackedTarget target : pipeLineResult.targets){
      var tagPose = fieldLayout.getTagPose(target.getFiducialId());
      var distanceToTag = PhotonUtils.getDistanceToPose(drive.getPose(),tagPose.get().toPose2d());
      if(target.getPoseAmbiguity()>0.35 || distanceToTag > VisionConstants.kMaxDistanceMeters){
        badTargets.add(target);
      }
    }
pipeLineResult.targets.removeAll(badTargets);

Optional<EstimatedRobotPose> poseResult = poseEstimator.update(pipeLineResult);
   boolean posePresent = poseResult.isPresent();
   if (!posePresent)  
   return; 

   EstimatedRobotPose estimatedPose = poseResult.get();

   estimated3dPose = estimatedPose.estimatedPose;
   consumer.accept(estimatedPose.estimatedPose.toPose2d(),estimatedPose.timestampSeconds);
    }

    public Pose3d getEstimated3dPose() {
      return estimated3dPose;
    }


}
