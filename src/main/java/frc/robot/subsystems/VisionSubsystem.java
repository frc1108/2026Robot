package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

@Logged
public class VisionSubsystem extends SubsystemBase {
  private static final AprilTagFieldLayout kFieldLayout = loadFieldLayout();

  private final List<PhotonCamera> m_photonCameras = new ArrayList<>();
  private final List<PhotonPoseEstimator> m_poseEstimators = new ArrayList<>();
  private final AprilTagFieldLayout m_fieldLayout;
  private final BiConsumer<Pose2d, Double> m_consumer;
  @NotLogged private final DriveSubsystem drive;

  private Pose3d estimated3dPose = new Pose3d();
  @Logged private Pose3d leftEstimated3dPose = new Pose3d();
  @Logged private Pose3d rightEstimated3dPose = new Pose3d();
  @Logged private Pose3d frontEstimated3dPose = new Pose3d();
  @Logged private Pose3d backEstimated3dPose = new Pose3d();
  @Logged private String lastEstimatorCameraName = "";

  @Logged private double hopperYaw = 0.0;
  @Logged private double hopperPitch = 0.0;
  @Logged private double hopperDistance = 0.0;
  @Logged private double hopperAmbiguity = 1.0;
  @Logged private boolean hopperVisible = false;
  @Logged private double autoAlignReferenceX = 0.0;
  @Logged private double autoAlignReferenceY = 0.0;
  @Logged private double autoAlignTargetHeadingDeg = 0.0;

  private static AprilTagFieldLayout loadFieldLayout() {
    try {
      return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException("Failed to load 2026 field layout", e);
    }
  }

  private static int getAllianceHopperTagId() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
        ? VisionConstants.kRedHopperTagId
        : VisionConstants.kBlueHopperTagId;
  }

  public VisionSubsystem(
      BiConsumer<Pose2d, Double> consumer,
      DriveSubsystem drive,
      String cameraName,
      Transform3d cameraOffset) throws IOException {
    m_fieldLayout = kFieldLayout;
    m_consumer = consumer;
    this.drive = drive;
    addPoseCamera(cameraName, cameraOffset);
  }

  public VisionSubsystem(BiConsumer<Pose2d, Double> consumer, DriveSubsystem drive) throws IOException {
    m_fieldLayout = kFieldLayout;
    m_consumer = consumer;
    this.drive = drive;

    addPoseCamera(
        VisionConstants.kLeftCameraName,
        new Transform3d(
            new Translation3d(
                VisionConstants.kLeftCameraOffsetX,
                VisionConstants.kLeftCameraOffsetY,
                VisionConstants.kLeftCameraOffsetZ),
            new Rotation3d(
                VisionConstants.kLeftCameraRotX,
                VisionConstants.kLeftCameraRotY,
                VisionConstants.kLeftCameraRotZ)));
    addPoseCamera(
        VisionConstants.kRightCameraName,
        new Transform3d(
            new Translation3d(
                VisionConstants.kRightCameraOffsetX,
                VisionConstants.kRightCameraOffsetY,
                VisionConstants.kRightCameraOffsetZ),
            new Rotation3d(
                VisionConstants.kRightCameraRotX,
                VisionConstants.kRightCameraRotY,
                VisionConstants.kRightCameraRotZ)));
    addPoseCamera(
        VisionConstants.kFrontSideCameraName,
        new Transform3d(
            new Translation3d(
                VisionConstants.kFrontSideCameraOffsetX,
                VisionConstants.kFrontSideCameraOffsetY,
                VisionConstants.kFrontSideCameraOffsetZ),
            new Rotation3d(
                VisionConstants.kFrontSideCameraRotX,
                VisionConstants.kFrontSideCameraRotY,
                VisionConstants.kFrontSideCameraRotZ)));
    addPoseCamera(
        VisionConstants.kBackCameraName,
        new Transform3d(
            new Translation3d(
                VisionConstants.kBackCameraOffsetX,
                VisionConstants.kBackCameraOffsetY,
                VisionConstants.kBackCameraOffsetZ),
            new Rotation3d(
                VisionConstants.kBackCameraRotX,
                VisionConstants.kBackCameraRotY,
                VisionConstants.kBackCameraRotZ)));
  }

  private void addPoseCamera(String cameraName, Transform3d cameraOffset) {
    m_photonCameras.add(new PhotonCamera(cameraName));
    m_poseEstimators.add(new PhotonPoseEstimator(
        m_fieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        cameraOffset));
  }

  @Override
  public void periodic() {
    publishCameraConnectionStatus();
    SmartDashboard.putNumber("Vision/RobotPoseX", drive.getPose().getX());
    SmartDashboard.putNumber("Vision/RobotPoseY", drive.getPose().getY());
    SmartDashboard.putNumber("Vision/RobotPoseDeg", drive.getPose().getRotation().getDegrees());

    Pose2d fallbackPose = drive.getPose();

    PhotonTrackedTarget bestHopperTarget = null;
    double bestHopperAmbiguity = Double.MAX_VALUE;
    Pose2d bestHopperPoseForDistance = fallbackPose;

    for (int i = 0; i < m_photonCameras.size(); i++) {
      PhotonCamera cam = m_photonCameras.get(i);
      PhotonPoseEstimator est = m_poseEstimators.get(i);

      if (!cam.isConnected()) {
        continue;
      }

      var result = cam.getLatestResult();
      if (!result.hasTargets()) {
        continue;
      }

      List<PhotonTrackedTarget> badTargets = new ArrayList<>();
      for (PhotonTrackedTarget target : result.targets) {
        var tagPose = m_fieldLayout.getTagPose(target.getFiducialId());
        if (tagPose.isEmpty() || target.getPoseAmbiguity() > VisionConstants.kMaxAmbiguity) {
          badTargets.add(target);
        }
      }
      result.targets.removeAll(badTargets);
      if (!result.hasTargets()) {
        continue;
      }

      Optional<EstimatedRobotPose> poseResult = est.update(result);
      if (poseResult.isPresent()) {
        EstimatedRobotPose estimatedPose = poseResult.get();
        Pose2d estimatedPose2d = estimatedPose.estimatedPose.toPose2d();

        double minDistanceToTag = Double.POSITIVE_INFINITY;
        for (PhotonTrackedTarget target : result.targets) {
          var tagPose = m_fieldLayout.getTagPose(target.getFiducialId());
          if (tagPose.isEmpty()) {
            continue;
          }
          double distanceToTag =
              PhotonUtils.getDistanceToPose(estimatedPose2d, tagPose.get().toPose2d());
          minDistanceToTag = Math.min(minDistanceToTag, distanceToTag);
        }

        estimated3dPose = estimatedPose.estimatedPose;
        boolean far = minDistanceToTag > VisionConstants.kMaxDistanceMeters;
        drive.addVisionMeasurementWithStdDevs(
            estimatedPose2d,
            estimatedPose.timestampSeconds,
            far ? VisionConstants.kVisionStdDevFarXY : VisionConstants.kVisionStdDevCloseXY,
            far ? VisionConstants.kVisionStdDevFarTheta : VisionConstants.kVisionStdDevCloseTheta);

        lastEstimatorCameraName = cam.getName();
        if (i == 0) {
          leftEstimated3dPose = estimatedPose.estimatedPose;
        } else if (i == 1) {
          rightEstimated3dPose = estimatedPose.estimatedPose;
        } else if (i == 2) {
          frontEstimated3dPose = estimatedPose.estimatedPose;
        } else if (i == 3) {
          backEstimated3dPose = estimatedPose.estimatedPose;
        }
      }

      for (PhotonTrackedTarget target : result.targets) {
        if (target.getFiducialId() == getAllianceHopperTagId()
            && target.getPoseAmbiguity() < bestHopperAmbiguity) {
          bestHopperAmbiguity = target.getPoseAmbiguity();
          bestHopperTarget = target;
          bestHopperPoseForDistance = estimated3dPose.toPose2d();
        }
      }
    }

    if (bestHopperTarget != null) {
      hopperVisible = true;
      hopperYaw = bestHopperTarget.getYaw();
      hopperPitch = bestHopperTarget.getPitch();
      hopperAmbiguity = bestHopperTarget.getPoseAmbiguity();
      var tagPose = m_fieldLayout.getTagPose(bestHopperTarget.getFiducialId());
      hopperDistance = tagPose.isPresent()
          ? PhotonUtils.getDistanceToPose(bestHopperPoseForDistance, tagPose.get().toPose2d())
          : 0.0;
    } else {
      hopperVisible = false;
      hopperYaw = 0.0;
      hopperPitch = 0.0;
      hopperDistance = 0.0;
      hopperAmbiguity = 1.0;
    }
  }

  private void publishCameraConnectionStatus() {
    for (PhotonCamera camera : m_photonCameras) {
      SmartDashboard.putBoolean("Vision/CameraConnected/" + camera.getName(), camera.isConnected());
    }
  }

  public boolean canSeeHopper() {
    return hopperVisible;
  }

  public double getHopperYaw() {
    return hopperYaw;
  }

  public double getHopperPitch() {
    return hopperPitch;
  }

  public double getHopperDistance() {
    return hopperDistance;
  }

  public double getHopperAmbiguity() {
    return hopperAmbiguity;
  }

  public OptionalDouble getHopperTargetHeadingDegrees(Pose2d robotPose) {
    Optional<Pose2d> hopperCenterPose = getHopperCenterPose();
    if (hopperCenterPose.isEmpty()) {
      return OptionalDouble.empty();
    }

    OptionalDouble targetRobotHeadingDeg = getTargetHeadingDegrees(robotPose, hopperCenterPose.get());
    if (targetRobotHeadingDeg.isEmpty()) {
      return OptionalDouble.empty();
    }

    Translation2d autoAlignReferenceField = getAutoAlignReferenceField(robotPose);
    autoAlignReferenceX = autoAlignReferenceField.getX();
    autoAlignReferenceY = autoAlignReferenceField.getY();
    autoAlignTargetHeadingDeg = targetRobotHeadingDeg.getAsDouble();
    return targetRobotHeadingDeg;
  }

  public static OptionalDouble getTargetHeadingDegrees(Pose2d robotPose, Pose2d targetPose) {
    Translation2d autoAlignReferenceField = getAutoAlignReferenceField(robotPose);
    double lineToTargetDeg = Math.toDegrees(Math.atan2(
        targetPose.getY() - autoAlignReferenceField.getY(),
        targetPose.getX() - autoAlignReferenceField.getX()));

    double shooterFacingDeg =
        robotPose.getRotation().getDegrees() + VisionConstants.getShooterAxisAngleDegrees();
    double shooterAimErrorDeg =
        MathUtil.inputModulus(lineToTargetDeg - shooterFacingDeg, -180.0, 180.0);

    double targetRobotHeadingDeg = MathUtil.inputModulus(
        robotPose.getRotation().getDegrees() + shooterAimErrorDeg,
        -180.0,
        180.0);
    return OptionalDouble.of(targetRobotHeadingDeg);
  }

  public static OptionalDouble getDistanceMetersToTarget(Pose2d robotPose, Pose2d targetPose) {
    Translation2d autoAlignReferenceField = getAutoAlignReferenceField(robotPose);
    return OptionalDouble.of(autoAlignReferenceField.getDistance(targetPose.getTranslation()));
  }

  public static Optional<Pose2d> getPoseFromTagOffset(
      int tagId,
      double forwardOffsetMeters,
      double leftOffsetMeters) {
    var tagPose = kFieldLayout.getTagPose(tagId);
    if (tagPose.isEmpty()) {
      return Optional.empty();
    }

    return Optional.of(tagPose.get().toPose2d().transformBy(
        new Transform2d(forwardOffsetMeters, leftOffsetMeters, Rotation2d.kZero)));
  }

  public static Optional<Pose2d> getAlliancePoseFromTagOffset(
      int blueTagId,
      int redTagId,
      double forwardOffsetMeters,
      double leftOffsetMeters) {
    int selectedTagId = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red
            ? redTagId
            : blueTagId;
    return getPoseFromTagOffset(selectedTagId, forwardOffsetMeters, leftOffsetMeters);
  }

  public OptionalDouble getHopperCenterDistanceMeters(Pose2d robotPose) {
    Optional<Pose2d> hopperCenterPose = getHopperCenterPose();
    if (hopperCenterPose.isEmpty()) {
      return OptionalDouble.empty();
    }

    return getDistanceMetersToTarget(robotPose, hopperCenterPose.get());
  }

  public static OptionalDouble getEstimatedHopperCenterDistanceMeters(Pose2d robotPose) {
    Optional<Pose2d> hopperCenterPose = getHopperCenterPose();
    if (hopperCenterPose.isEmpty()) {
      return OptionalDouble.empty();
    }

    return getDistanceMetersToTarget(robotPose, hopperCenterPose.get());
  }

  public static Optional<Pose2d> getEstimatedHopperCenterPose() {
    return getHopperCenterPose();
  }

  public static OptionalDouble getDistanceMetersToTagOffsetTarget(
      Pose2d robotPose,
      int tagId,
      double forwardOffsetMeters,
      double leftOffsetMeters) {
    Optional<Pose2d> targetPose = getPoseFromTagOffset(tagId, forwardOffsetMeters, leftOffsetMeters);
    if (targetPose.isEmpty()) {
      return OptionalDouble.empty();
    }
    return getDistanceMetersToTarget(robotPose, targetPose.get());
  }

  public static OptionalDouble getHeadingDegreesToTagOffsetTarget(
      Pose2d robotPose,
      int tagId,
      double forwardOffsetMeters,
      double leftOffsetMeters) {
    Optional<Pose2d> targetPose = getPoseFromTagOffset(tagId, forwardOffsetMeters, leftOffsetMeters);
    if (targetPose.isEmpty()) {
      return OptionalDouble.empty();
    }
    return getTargetHeadingDegrees(robotPose, targetPose.get());
  }

  private static Translation2d getAutoAlignReferenceField(Pose2d robotPose) {
    Translation2d autoAlignOffsetRobot = new Translation2d(
        VisionConstants.kAutoAlignCenterShiftForwardMeters,
        VisionConstants.kAutoAlignCenterShiftLeftMeters);
    return robotPose.getTranslation().plus(autoAlignOffsetRobot.rotateBy(robotPose.getRotation()));
  }

  private static Optional<Pose2d> getHopperCenterPose() {
    return getAlliancePoseFromTagOffset(
        VisionConstants.kBlueHopperTagId,
        VisionConstants.kRedHopperTagId,
        VisionConstants.kHopperCenterOffsetForwardMeters,
        VisionConstants.kHopperCenterOffsetLeftMeters);
  }

  public Pose3d getEstimated3dPose() {
    return estimated3dPose;
  }

  public Pose3d getLeftEstimated3dPose() {
    return leftEstimated3dPose;
  }

  public Pose3d getRightEstimated3dPose() {
    return rightEstimated3dPose;
  }

  public Pose3d getFrontEstimated3dPose() {
    return frontEstimated3dPose;
  }

  public Pose3d getBackEstimated3dPose() {
    return backEstimated3dPose;
  }

  public String getLastEstimatorCameraName() {
    return lastEstimatorCameraName;
  }
}
