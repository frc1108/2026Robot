// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AimWhileDrivingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TombSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
@Logged
public class RobotContainer {
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final HoodSubsystem m_hood = new HoodSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final TombSubsystem m_tomb = new TombSubsystem();
  private VisionSubsystem m_vision;
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    try {
      m_vision = new VisionSubsystem(m_robotDrive::addVisionMeasurement, m_robotDrive);
    } catch (Exception e) {
      System.err.println("Failed to initialize VisionSubsystem: " + e.getMessage());
      e.printStackTrace();
    }

    m_shooter.setHoodSubsystem(m_hood);
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

  private void configureButtonBindings() {
    if (m_vision != null) {
      m_driverController.a().whileTrue(new AimWhileDrivingCommand(m_vision, m_robotDrive, m_driverController));
    }

    if (m_vision != null) {
      m_driverController.rightTrigger().whileTrue(Commands.parallel(
          m_shooter.shootCommand(),
          m_hood.autoHoodFromDistanceCommand(
              () -> m_vision.getHopperCenterDistanceMeters(m_robotDrive.getPose()).orElse(2.5))));
      m_driverController.rightBumper().whileTrue(Commands.parallel(
          m_shooter.slowShootCommand(),
          m_hood.autoHoodFromDistanceCommand(
              () -> m_vision.getHopperCenterDistanceMeters(m_robotDrive.getPose()).orElse(2.5))));
    } else {
      m_driverController.rightTrigger().whileTrue(m_shooter.shootCommand());
      m_driverController.rightBumper().whileTrue(m_shooter.slowShootCommand());
    }

    m_driverController.leftTrigger().whileTrue(m_intake.intake());
    m_driverController.leftBumper().whileTrue(m_intake.slowIntake());
    m_driverController.povDown().whileTrue(m_intake.reverseIntake());
    m_driverController.povDown().whileTrue(m_tomb.reverseTomb());
    m_driverController.y().whileTrue(m_tomb.tomb());

    m_driverController.x().onTrue(m_hood.setHoodAngleCommand(15.0));
    m_driverController.b().onTrue(m_hood.setHoodAngleCommand(30.0));
  }

  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
