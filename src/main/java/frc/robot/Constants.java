package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    public static final int kDrivingMotorPinionTeeth = 14;

    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class IntakeConstants {
    public static final int frontIntakecanid = 21;
    public static final int backIntakecanid = 22;
    public static final double intakeFrontVelocityRpm = 4000.0;
    public static final double slowIntakeFrontVelocityRpm = 1000.0;
    public static final double reverseIntakeFrontVelocityRpm = -4000.0;
    public static final double intakeBackVelocityRpm = -4000.0;
    public static final double slowIntakeBackVelocityRpm = -4000.0;
    public static final double reverseIntakeBackVelocityRpm = 4000.0;
  }

  public static final class TombConstants {
    public static final int frontTombCanId = 23;
    public static final int backTombCanId = 24;
    public static final int feederTombCanId = 25;
    public static final double backTombSpeed = 0.95;
    public static final double frontTombSpeed = -0.95;
    public static final double feederTombSpeed = -0.95;
    public static final double feederTombVelocityRpm = 5000.0;
    public static final double reverseFrontTombSpeed = 0.95;
    public static final double reverseBackTombSpeed = -0.95;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularSpeedRadiansPerSecondSquared);

    public static final boolean kUseFuelObjectAuto = false;
    public static final double kFuelAutoTimeoutSeconds = 4.0;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ShooterConstants {
    public static final int kLeftShooterCanId = 31;
    public static final int kRightShooterCanId = 32;

    public static final int kHoodMotorCanId = 33;
    public static final double kHoodGearRatio = 36.0;
    public static final double kMinHoodAngleDegrees = 0.0;
    public static final double kMaxHoodAngleDegrees = 45.0;

    public static final double kHoodP = 0.25;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.0;
    public static final double kHoodMaxClosedLoopOutput = 0.35;
    public static final double kHoodCommandToleranceDegrees = 0.5;

    public static final boolean kAutoZeroHoodOnStartup = true;
    public static final double kHoodStartupZeroDegrees = 0.0;

    public static final double kShooterFullRpm = 4200.0;
    public static final double kShooterSlowRpm = 2800.0;
    public static final double kShooterMaxRpm = 6000.0;

    // TalonFX velocity PID/FF gains for shooter wheels.
    public static final double kShooterVelocityP = 0.35;
    public static final double kShooterVelocityI = 0.0;
    public static final double kShooterVelocityD = 0.0;
    public static final double kShooterVelocityV = 0.16;

    public static final double kShooterVelocityS = 0.0;

    // Distance-based shooter RPM table (interpolated).
    public static final double[] kShooterDistanceMeters = {1.5, 2.5, 3.5, 4.5};
    public static final double[] kShooterDistanceRpm = {1700.0, 1950.0, 2350.0, 2800.0};

    // Distance-to-RPM smoothing for auto/assisted shooting.
    public static final double kAutoShooterDistanceFilterAlpha = 0.1;
    public static final double kAutoShooterDistanceDeadbandMeters = 0.05;
    public static final double kAutoShooterRpmSlewRatePerSec = 10000.0;
    public static final double kAutoShooterMinCommandStepRpm = 5.0;
    public static final double kAutoShooterUpdatePeriodSeconds = 0.02;

    public static final double[] kHoodDistanceMeters = {1.5, 2.5, 3.5, 4.5};
    public static final double[] kHoodAngleDegrees = {34.0, 28.0, 22.0, 17.0};

    public static final double kAutoHoodDistanceFilterAlpha = 0.1;
    public static final double kAutoHoodDistanceDeadbandMeters = 0.05;
    public static final double kAutoHoodAngleSlewRateDegPerSec = 20.0;
    public static final double kAutoHoodMinCommandStepDeg = 0.4;
    public static final double kAutoHoodUpdatePeriodSeconds = 0.12;
  }

  public static final class VisionConstants {
    // Camera names in PhotonVision.
    public static final String kLeftCameraName = "LeftSideCamera";
    public static final String kRightCameraName = "RightSideCamera";
    public static final String kFrontSideCameraName = "FrontSideCamera";
    public static final String kFrontFuelCameraName = "FrontFuelCamera";
    public static final String kFuelCameraName = kFrontFuelCameraName;

    // Hopper targeting.
    public static final int kHopperTagId = 26;
    public static final double kHopperCenterOffsetForwardMeters = -0.305;
    public static final double kHopperCenterOffsetLeftMeters = 0.0;

    // Shooter mount geometry in robot frame (+X forward, +Y left).
    public static final double kShooterOffsetForwardMeters = -0.2;
    public static final double kShooterOffsetLeftMeters = 0.2;
    public static final double kShooterYawOffsetDegrees = 90.0;

    // Left camera mount pose in robot frame.
    public static final double kLeftCameraOffsetX = 0.165;   // +X forward (meters)
    public static final double kLeftCameraOffsetY = 0.368; // +Y left (meters)
    public static final double kLeftCameraOffsetZ = 0.711; // +Z up (meters)
    public static final double kLeftCameraRotX = 0.0;      // roll about +X (radians)
    public static final double kLeftCameraRotY = 0.0;      // pitch about +Y (radians)
    public static final double kLeftCameraRotZ = 1.5708;   // yaw about +Z (radians), +90 deg

    // Right camera mount pose in robot frame.
    public static final double kRightCameraOffsetX = 0.0;
    public static final double kRightCameraOffsetY = -0.368;
    public static final double kRightCameraOffsetZ = 0.711;
    public static final double kRightCameraRotX = 0.0;
    public static final double kRightCameraRotY = 0.0;
    public static final double kRightCameraRotZ = -1.5708;

    // Front-side camera mount pose in robot frame (AprilTag camera).
    public static final double kFrontSideCameraOffsetX = 0.305;
    public static final double kFrontSideCameraOffsetY = 0.0;
    public static final double kFrontSideCameraOffsetZ = 0.711;
    public static final double kFrontSideCameraRotX = 0.0;
    public static final double kFrontSideCameraRotY = 0.0;
    public static final double kFrontSideCameraRotZ = 0.0;

    // Front fuel camera mount (robot frame).
    public static final double kFrontFuelCameraOffsetX = 0.305;
    public static final double kFrontFuelCameraOffsetY = 0.0;
    public static final double kFrontFuelCameraOffsetZ = 0.305;
    public static final double kFrontFuelCameraRotX = 0.0;
    public static final double kFrontFuelCameraRotY = 0.0;
    public static final double kFrontFuelCameraRotZ = 0.0;

    public static final double kMaxDistanceMeters = 10.0;
    public static final double kMaxAmbiguity = 0.35;

    public static final double kAimP = 0.03;
    public static final double kAimI = 0.0;
    public static final double kAimD = 0.002;
    public static final double kAimingToleranceDegrees = 2.0;
    public static final double kAimingTimeoutSeconds = 2.0;
    // Rotation command sign to match drivetrain convention.
    // Use 1.0 for normal PID direction, -1.0 only if your drivetrain is inverted.
    public static final double kAimRotationSign = 1.0;
    // Positive = aim more left, negative = aim more right.
    public static final double kAimHeadingTrimDegrees = -6;

    public static final double kBallExitSpeedMetersPerSecond = 9.0;
    public static final double kShotLeadGain = 1.0;
    public static final double kMaxShotLeadDegrees = 12.0;

    // Fuel-object follow tuning (PhotonVision object detection / YOLO).
    public static final double kFuelAimP = 0.03;
    public static final double kFuelAimI = 0.0;
    public static final double kFuelAimD = 0.001;
    public static final double kFuelApproachP = 0.09;
    public static final double kFuelTargetArea = 10.0;
    public static final double kFuelFinishArea = 13.0;
    public static final double kFuelMaxDriveSpeed = 0.45;
    public static final double kFuelMaxTurnSpeed = 0.5;
    public static final double kFuelSearchTurnSpeed = 0.22;
    public static final double kFuelYawToleranceDegrees = 3.0;

    // Fuel cluster selection:
    // detections within these yaw/pitch windows are grouped into one cluster.
    public static final double kFuelClusterYawToleranceDegrees = 6.0;
    public static final double kFuelClusterPitchToleranceDegrees = 6.0;

    // Fuel heatmap (Shuffleboard Field2d overlay):
    // grid resolution in field cells.
    public static final int kFuelHeatmapCellsX = 36;
    public static final int kFuelHeatmapCellsY = 18;
    // Requested field image/background name for Field2d widgets.
    public static final String kFuelHeatmapFieldBackground = "2026RebuiltWelded";
    // Master enable for heatmap publishing to dashboards.
    public static final boolean kEnableFuelHeatmap = true;
    // Limit dashboard update rate to reduce client-side lag.
    public static final double kFuelHeatmapPublishPeriodSeconds = 1.0;
    // How quickly old cells fade each second (0=no decay, 1=full clear every second).
    public static final double kFuelHeatmapDecayPerSecond = 0.35;
    // Number of top-intensity cells drawn as hotspot markers.
    public static final int kFuelHeatmapHotspotsToDisplay = 6;
    // Minimum cell value required before a hotspot marker is shown.
    public static final double kFuelHeatmapMinHotspotValue = 0.2;

    // Approximate range from object area:
    // estimatedDistanceMeters ~= kFuelAreaToDistanceScale / sqrt(area).
    // Tune scale first, then clamp min/max to keep projections sane.
    public static final double kFuelAreaToDistanceScale = 2.2;
    public static final double kFuelMinEstimatedDistanceMeters = 0.3;
    public static final double kFuelMaxEstimatedDistanceMeters = 4.0;

    // Camera startup re-init pulse:
    // run once after boot to force PhotonVision to re-apply camera settings.
    public static final boolean kRunCameraStartupReinitPulse = true;
    public static final double kCameraStartupReinitDelaySeconds = 2.0;
  }
}
