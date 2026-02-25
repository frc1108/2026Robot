// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AimWhileDrivingCommand;
import frc.robot.commands.FollowFuelCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TombSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

@Logged
public class RobotContainer {
  private static final String kDefaultAutoName = "Hopper Test";
  private static final double kDefaultHopperDistanceMeters = 2.5;

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final HoodSubsystem m_hood = new HoodSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final TombSubsystem m_tomb = new TombSubsystem();
  private VisionSubsystem m_vision;
  private boolean m_autoAimAtHopperEnabled = false;
  private final PIDController m_autoAimRotationPid =
      new PIDController(VisionConstants.kAimP, VisionConstants.kAimI, VisionConstants.kAimD);
  private SendableChooser<Command> m_autoChooser;
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    try {
      m_vision = new VisionSubsystem(m_robotDrive::addVisionMeasurement, m_robotDrive);
    } catch (Exception e) {
      System.err.println("Failed to initialize VisionSubsystem: " + e.getMessage());
      e.printStackTrace();
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
  }

  private double getAutoAimRotationFeedbackRadPerSec() {
    if (!m_autoAimAtHopperEnabled || m_vision == null) {
      return 0.0;
    }

    double currentHeadingDeg = m_robotDrive.getPose().getRotation().getDegrees();
    var targetHeadingDeg = m_vision.getHopperTargetHeadingDegrees(m_robotDrive.getPose());
    if (targetHeadingDeg.isEmpty()) {
      return 0.0;
    }

    double normalizedRotationCmd =
        MathUtil.clamp(m_autoAimRotationPid.calculate(currentHeadingDeg, targetHeadingDeg.getAsDouble()), -1.0, 1.0);
    return normalizedRotationCmd * DriveConstants.kMaxAngularSpeed;
  }

  private void disableAutoAimOverride() {
    m_autoAimAtHopperEnabled = false;
    PPHolonomicDriveController.clearRotationFeedbackOverride();
  }

  public void clearAutoAimOverride() {
    disableAutoAimOverride();
  }

  private void configureButtonBindings() {
    boolean hasVision = m_vision != null;

    if (hasVision) {
      m_driverController.rightBumper().whileTrue(new AimWhileDrivingCommand(m_vision, m_robotDrive, m_driverController));
    }

    if (hasVision) {
      // Keep hood angle synchronized with measured distance while shooting.
      m_driverController.rightTrigger().whileTrue(Commands.parallel(
          m_shooter.shootCommand(),
          m_hood.autoHoodFromDistanceCommand(
              () -> m_vision.getHopperCenterDistanceMeters(m_robotDrive.getPose()).orElse(kDefaultHopperDistanceMeters))));
    } else {
      m_driverController.rightTrigger().whileTrue(m_shooter.shootCommand());
    }

    m_driverController.leftTrigger().whileTrue(m_intake.intake());
    m_driverController.leftBumper().whileTrue(m_intake.slowIntake());
    m_driverController.x().whileTrue(m_shooter.slowShootCommand());
    if (hasVision) {
      m_driverController.povUp().whileTrue(new FollowFuelCommand(m_vision, m_robotDrive));
    }
    m_driverController.povDown().whileTrue(m_intake.reverseIntake());
    m_driverController.povDown().whileTrue(m_tomb.reverseTomb());
    m_driverController.y().whileTrue(m_tomb.tomb());
  }

  public Command getAutonomousCommand() {
    if (m_vision != null && AutoConstants.kUseFuelObjectAuto) {
      return new FollowFuelCommand(m_vision, m_robotDrive)
          .withTimeout(AutoConstants.kFuelAutoTimeoutSeconds)
          .beforeStarting(this::disableAutoAimOverride)
          .andThen(Commands.runOnce(this::disableAutoAimOverride));
    }

    if (m_autoChooser != null && m_autoChooser.getSelected() != null) {
      return m_autoChooser.getSelected()
          .beforeStarting(this::disableAutoAimOverride)
          .andThen(Commands.runOnce(this::disableAutoAimOverride));
    }
    return Commands.runOnce(this::disableAutoAimOverride);
  }
}
