// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AimAtFieldPoseWhileDrivingCommand;
import frc.robot.commands.AimWhileDrivingCommand;
import frc.robot.commands.FollowFuelCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TombSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Optional;
import java.util.function.DoubleSupplier;

@Logged
public class RobotContainer {
  private static final String kDefaultAutoName = "BR";
  private static final double kDefaultHopperDistanceMeters = 2.5;

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final HoodSubsystem m_hood = new HoodSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LightsSubsystem m_lights = new LightsSubsystem();
  private final TombSubsystem m_tomb = new TombSubsystem();
  private VisionSubsystem m_vision;
  private boolean m_autoAimAtHopperEnabled = false;
  private Command m_activeAutoShootCommand;
  private Command m_activeAutoIntakeCommand;
  private Command m_activeAutoTombCommand;
  private Command m_activeHoodAutoZeroCommand;
  private final PIDController m_autoAimRotationPid =
      new PIDController(VisionConstants.kAimP, VisionConstants.kAimI, VisionConstants.kAimD);
  private SendableChooser<Command> m_autoChooser;
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  public RobotContainer() {
    try {
      m_vision = new VisionSubsystem(m_robotDrive::addVisionMeasurement, m_robotDrive);
    } catch (Exception e) {
      DriverStation.reportError("Failed to initialize VisionSubsystem: " + e.getMessage(), false);
    }

    m_shooter.setHoodSubsystem(m_hood);
    m_autoAimRotationPid.enableContinuousInput(-180.0, 180.0);
    m_autoAimRotationPid.setTolerance(VisionConstants.kAimingToleranceDegrees);
    configurePathPlannerNamedCommands();
    configureAutoChooser();
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  private void configureAutoChooser() {
    m_autoChooser = AutoBuilder.buildAutoChooser(kDefaultAutoName);
    Shuffleboard.getTab("Autonomous")
        .add("Auto Chooser", m_autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(5, 2);
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  private void configurePathPlannerNamedCommands() {
    DoubleSupplier hopperDistanceSupplier = createHopperDistanceSupplier();

    new EventTrigger("Shoot").onTrue(Commands.runOnce(() -> startAutoShootCommand(hopperDistanceSupplier)));
    new EventTrigger("StopShoot").onTrue(Commands.runOnce(this::stopAutoShootCommand));
    new EventTrigger("Intake").onTrue(Commands.runOnce(this::startAutoIntakeCommand));
    new EventTrigger("StopIntake").onTrue(Commands.runOnce(this::stopAutoIntakeCommand));
    new EventTrigger("Tomb").onTrue(Commands.runOnce(this::startAutoTombCommand));
    new EventTrigger("StopTomb").onTrue(Commands.runOnce(this::stopAutoTombCommand));
    new EventTrigger("AimAtHopperOn").onTrue(Commands.runOnce(() -> {
      m_autoAimAtHopperEnabled = true;
      m_autoAimRotationPid.reset();
      PPHolonomicDriveController.overrideRotationFeedback(this::getAutoAimRotationFeedbackRadPerSec);
    }));
    new EventTrigger("AimAtHopperOff").onTrue(Commands.runOnce(this::disableAutoAimOverride));

    NamedCommands.registerCommand(
        "AimAtHopperOn",
        Commands.runOnce(() -> {
          m_autoAimAtHopperEnabled = true;
          m_autoAimRotationPid.reset();
          PPHolonomicDriveController.overrideRotationFeedback(this::getAutoAimRotationFeedbackRadPerSec);
        }));
    NamedCommands.registerCommand(
        "AimAtHopperOff",
        Commands.runOnce(this::disableAutoAimOverride));
    NamedCommands.registerCommand(
        "AutoZeroHood",
        Commands.runOnce(this::scheduleHoodAutoZero));
    NamedCommands.registerCommand(
        "Shoot",
        Commands.runOnce(() -> startAutoShootCommand(hopperDistanceSupplier)));
    NamedCommands.registerCommand(
        "StopShoot",
        Commands.runOnce(this::stopAutoShootCommand));
    NamedCommands.registerCommand(
        "Intake",
        Commands.runOnce(this::startAutoIntakeCommand));
    NamedCommands.registerCommand(
        "StopIntake",
        Commands.runOnce(this::stopAutoIntakeCommand));
    NamedCommands.registerCommand(
        "Tomb",
        Commands.runOnce(this::startAutoTombCommand));
    NamedCommands.registerCommand(
        "StopTomb",
        Commands.runOnce(this::stopAutoTombCommand));

    new EventTrigger("AutoZeroHood").onTrue(Commands.runOnce(this::scheduleHoodAutoZero));
  }

  private double getAutoAimRotationFeedbackRadPerSec() {
    if (!m_autoAimAtHopperEnabled || m_vision == null) {
      return 0.0;
    }

    var targetHeadingDeg = m_vision.getHopperTargetHeadingDegrees(m_robotDrive.getPose());
    if (targetHeadingDeg.isEmpty()) {
      return 0.0;
    }

    double currentHeadingDeg = m_robotDrive.getPose().getRotation().getDegrees();
    double normalizedRotationCmd = MathUtil.clamp(
        m_autoAimRotationPid.calculate(currentHeadingDeg, targetHeadingDeg.getAsDouble()),
        -1.0,
        1.0);
    normalizedRotationCmd *= VisionConstants.kAimRotationSign;
    return normalizedRotationCmd * DriveConstants.kMaxAngularSpeed;
  }

  private void disableAutoAimOverride() {
    m_autoAimAtHopperEnabled = false;
    PPHolonomicDriveController.clearRotationFeedbackOverride();
  }

  public void clearAutoAimOverride() {
    disableAutoAimOverride();
  }

  public void stopAutoNamedCommands() {
    stopAutoShootCommand();
    stopAutoIntakeCommand();
    stopAutoTombCommand();
  }

  public void scheduleHoodAutoZero() {
    if (m_activeHoodAutoZeroCommand != null && m_activeHoodAutoZeroCommand.isScheduled()) {
      return;
    }
    m_activeHoodAutoZeroCommand = m_hood.autoZeroByCurrentCommand();
    CommandScheduler.getInstance().schedule(m_activeHoodAutoZeroCommand);
  }

  private void configureButtonBindings() {
    boolean hasVision = m_vision != null;
    DoubleSupplier hopperDistanceSupplier = createHopperDistanceSupplier();
    Command autoShootAlignCommand = hasVision
        ? Commands.parallel(
            new AimWhileDrivingCommand(m_vision, m_robotDrive, m_driverController),
            m_hood.autoHoodFromDistanceCommand(hopperDistanceSupplier),
            m_shooter.autoShootFromDistanceCommand(hopperDistanceSupplier))
        : Commands.parallel(
            m_hood.autoHoodFromDistanceCommand(hopperDistanceSupplier),
            m_shooter.autoShootFromDistanceCommand(hopperDistanceSupplier));
    Command tombActionCommand = Commands.parallel(
        m_tomb.tomb(),
        m_intake.backIntakeCommand());
    Command delayedBFireCommand = Commands.sequence(
        Commands.waitSeconds(0.5),
        Commands.parallel(
            m_tomb.tomb(),
            m_intake.jiggleIntakeCommand()));
    Command leftSideShotCommand = buildSideShotCommand(
        () -> VisionSubsystem.getAlliancePoseFromTagOffset(
            VisionConstants.kBlueLeftSideShotReferenceTagId,
            VisionConstants.kRedLeftSideShotReferenceTagId,
            VisionConstants.kSideShotOffsetForwardMeters,
            VisionConstants.kLeftSideShotOffsetLeftMeters));
    Command rightSideShotCommand = buildSideShotCommand(
        () -> VisionSubsystem.getAlliancePoseFromTagOffset(
            VisionConstants.kBlueRightSideShotReferenceTagId,
            VisionConstants.kRedRightSideShotReferenceTagId,
            VisionConstants.kSideShotOffsetForwardMeters,
            VisionConstants.kRightSideShotOffsetLeftMeters));

    if (hasVision) {
      m_driverController.rightBumper().whileTrue(Commands.parallel(
          new AimWhileDrivingCommand(m_vision, m_robotDrive, m_driverController),
          m_hood.autoHoodFromDistanceCommand(hopperDistanceSupplier)));
    } else {
      // Keep hood control available even if vision fails to initialize.
      m_driverController.rightBumper().whileTrue(
          m_hood.autoHoodFromDistanceCommand(hopperDistanceSupplier));
    }

    if (hasVision) {
      // Use measured distance to adjust shooter speed directly.
      m_driverController.rightTrigger().whileTrue(Commands.parallel(
          m_shooter.autoShootFromDistanceCommand(hopperDistanceSupplier)));
    } else {
      m_driverController.rightTrigger().whileTrue(
          m_shooter.autoShootFromDistanceCommand(hopperDistanceSupplier));
    }

    m_driverController.leftTrigger().whileTrue(m_intake.intake());
    m_driverController.leftBumper().whileTrue(m_intake.slowIntake());
    m_driverController.x().whileTrue(m_tomb.reverseFeederCommand());
    m_driverController.a().onTrue(Commands.runOnce(m_robotDrive::zeroHeading, m_robotDrive));
    if (hasVision) {
      m_driverController.povUp().whileTrue(new FollowFuelCommand(m_vision, m_robotDrive));
    }
    m_driverController.povLeft().whileTrue(leftSideShotCommand);
    m_driverController.povDown().whileTrue(m_intake.reverseIntake());
    m_driverController.povDown().whileTrue(m_tomb.reverseTomb());
    m_driverController.povRight().whileTrue(rightSideShotCommand);
    m_driverController.y().whileTrue(tombActionCommand);
    m_driverController.b().whileTrue(
        Commands.parallel(
            autoShootAlignCommand,
            delayedBFireCommand));
    m_driverController.start().onTrue(Commands.runOnce(this::scheduleHoodAutoZero));
    m_driverController.back().onTrue(Commands.runOnce(m_lights::cycleMode, m_lights));

    m_operatorController.rightBumper().whileTrue(m_hood.hoodUpCommand());
    m_operatorController.leftBumper().whileTrue(m_hood.hoodDownCommand());
    m_operatorController.a().whileTrue(m_tomb.reverseFeederCommand());
    m_operatorController.start().onTrue(Commands.runOnce(this::scheduleHoodAutoZero));

    Shuffleboard.getTab("Hood")
        .addBoolean("Operator/ConnectedPort1",
            () -> DriverStation.isJoystickConnected(OIConstants.kOperatorControllerPort));
    Shuffleboard.getTab("Hood")
        .addBoolean("Operator/IsXboxPort1",
            () -> DriverStation.getJoystickIsXbox(OIConstants.kOperatorControllerPort));
    Shuffleboard.getTab("Hood")
        .addBoolean("Operator/RB",
            () -> m_operatorController.rightBumper().getAsBoolean());
    Shuffleboard.getTab("Hood")
        .addBoolean("Operator/LB",
            () -> m_operatorController.leftBumper().getAsBoolean());
  }

  public Command getAutonomousCommand() {
    if (m_vision != null && AutoConstants.kUseFuelObjectAuto) {
      return new FollowFuelCommand(m_vision, m_robotDrive)
          .beforeStarting(this::stopAutoNamedCommands)
          .withTimeout(AutoConstants.kFuelAutoTimeoutSeconds)
          .beforeStarting(this::disableAutoAimOverride)
          .andThen(Commands.runOnce(this::disableAutoAimOverride))
          .andThen(Commands.runOnce(this::stopAutoNamedCommands));
    }

    if (m_autoChooser != null && m_autoChooser.getSelected() != null) {
      return m_autoChooser.getSelected()
          .beforeStarting(this::stopAutoNamedCommands)
          .beforeStarting(this::disableAutoAimOverride)
          .andThen(Commands.runOnce(this::disableAutoAimOverride))
          .andThen(Commands.runOnce(this::stopAutoNamedCommands));
    }
    return Commands.runOnce(() -> {
      disableAutoAimOverride();
      stopAutoNamedCommands();
    });
  }

  private DoubleSupplier createHopperDistanceSupplier() {
    return () -> VisionSubsystem.getEstimatedHopperCenterDistanceMeters(m_robotDrive.getPose())
        .orElse(kDefaultHopperDistanceMeters);
  }

  private DoubleSupplier createTargetDistanceSupplier(java.util.function.Supplier<Optional<Pose2d>> targetPoseSupplier) {
    return () -> targetPoseSupplier.get()
        .flatMap(
            pose -> VisionSubsystem.getDistanceMetersToTarget(m_robotDrive.getPose(), pose)
                .stream()
                .boxed()
                .findFirst())
        .orElse(kDefaultHopperDistanceMeters);
  }

  private Command buildSideShotCommand(java.util.function.Supplier<Optional<Pose2d>> targetPoseSupplier) {
    DoubleSupplier targetDistanceSupplier = createTargetDistanceSupplier(targetPoseSupplier);
    return Commands.parallel(
        new AimAtFieldPoseWhileDrivingCommand(m_robotDrive, m_driverController, targetPoseSupplier),
        m_shooter.autoShootFromDistanceCommand(
            targetDistanceSupplier,
            ShooterConstants.kSideShotDistanceMeters,
            ShooterConstants.kSideShotDistanceRpm),
        m_hood.setHoodAngleCommand(VisionConstants.kSideShotHoodAngleDegrees),
        Commands.sequence(
            Commands.waitSeconds(0.5),
            Commands.parallel(
                m_tomb.tomb(),
                m_intake.jiggleIntakeCommand())));
  }

  private void startAutoShootCommand(DoubleSupplier hopperDistanceSupplier) {
    stopAutoShootCommand();
    m_activeAutoShootCommand = m_shooter.autoShootFromDistanceCommand(hopperDistanceSupplier);
    CommandScheduler.getInstance().schedule(m_activeAutoShootCommand);
  }

  private void stopAutoShootCommand() {
    if (m_activeAutoShootCommand != null) {
      m_activeAutoShootCommand.cancel();
      m_activeAutoShootCommand = null;
    }
    m_shooter.stopShooter();
  }

  private void startAutoIntakeCommand() {
    stopAutoIntakeCommand();
    m_activeAutoIntakeCommand = m_intake.autoIntakeCommand();
    CommandScheduler.getInstance().schedule(m_activeAutoIntakeCommand);
  }

  private void stopAutoIntakeCommand() {
    if (m_activeAutoIntakeCommand != null) {
      m_activeAutoIntakeCommand.cancel();
      m_activeAutoIntakeCommand = null;
    }
    m_intake.stopIntakeMotors();
  }

  private void startAutoTombCommand() {
    stopAutoTombCommand();
    m_activeAutoTombCommand = m_tomb.tomb();
    CommandScheduler.getInstance().schedule(m_activeAutoTombCommand);
  }

  private void stopAutoTombCommand() {
    if (m_activeAutoTombCommand != null) {
      m_activeAutoTombCommand.cancel();
      m_activeAutoTombCommand = null;
    }
    m_tomb.stopTombMotors();
  }
}
