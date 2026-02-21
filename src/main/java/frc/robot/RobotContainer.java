// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
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
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final HoodSubsystem m_hood = new HoodSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final TombSubsystem m_tomb = new TombSubsystem();
  private VisionSubsystem m_vision;
  private boolean m_autoAimAtHopperEnabled = false;
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
    m_autoChooser = AutoBuilder.buildAutoChooser("Hopper Test");
    Shuffleboard.getTab("Autonomous")
        .add("Auto Chooser", m_autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(5, 2);
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, Button.kR1.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));

    // new JoystickButton(m_driverController, XboxController.Button.kStart.value)
    //     .onTrue(new InstantCommand(
    //         () -> m_robotDrive.zeroHeading(),
    //         m_robotDrive));

    m_driverController.rightTrigger().whileTrue(m_shooter.shootCommand());///rightTrigger().whileTrue(m_shooter.shootCommand());
    m_driverController.rightBumper().whileTrue(m_shooter.slowShootCommand());
    m_driverController.leftTrigger().whileTrue(m_intake.intake());
    m_driverController.povDown().whileTrue(m_intake.reverseIntake());
    m_driverController.povDown().whileTrue(m_tomb.reverseTomb());
    m_driverController.leftBumper().whileTrue(m_intake.slowIntake());
    m_driverController.y().whileTrue(m_tomb.tomb());
  }

  private void configurePathPlannerNamedCommands() {
    NamedCommands.registerCommand(
        "AimAtHopperOn",
        Commands.runOnce(() -> m_autoAimAtHopperEnabled = true));
    NamedCommands.registerCommand(
        "AimAtHopperOff",
        Commands.runOnce(() -> m_autoAimAtHopperEnabled = false));

    PPHolonomicDriveController.setRotationTargetOverride(() -> {
      if (!m_autoAimAtHopperEnabled || m_vision == null) {
        return Optional.empty();
      }

      return m_vision.getHopperTargetHeadingDegrees(m_robotDrive.getPose())
          .stream()
          .mapToObj(Rotation2d::fromDegrees)
          .findFirst();
    });
  }

  private void configureButtonBindings() {
    if (m_vision != null) {
      m_driverController.rightBumper().whileTrue(new AimWhileDrivingCommand(m_vision, m_robotDrive, m_driverController));
    }

    if (m_vision != null) {
      m_driverController.rightTrigger().whileTrue(Commands.parallel(
          m_shooter.shootCommand(),
          m_hood.autoHoodFromDistanceCommand(
              () -> m_vision.getHopperCenterDistanceMeters(m_robotDrive.getPose()).orElse(2.5))));
      //m_driverController.rightBumper().whileTrue(Commands.parallel(
          //m_shooter.slowShootCommand(),
          //m_hood.autoHoodFromDistanceCommand(
              //() -> m_vision.getHopperCenterDistanceMeters(m_robotDrive.getPose()).orElse(2.5))));
    } else {
      m_driverController.rightTrigger().whileTrue(m_shooter.shootCommand());
      //m_driverController.rightBumper().whileTrue(m_shooter.slowShootCommand());
    }

    m_driverController.leftTrigger().whileTrue(m_intake.intake());
    m_driverController.leftBumper().whileTrue(m_intake.slowIntake());
    if (m_vision != null) {
      m_driverController.povUp().whileTrue(new FollowFuelCommand(m_vision, m_robotDrive));
    }
    m_driverController.povDown().whileTrue(m_intake.reverseIntake());
    m_driverController.povDown().whileTrue(m_tomb.reverseTomb());
    m_driverController.y().whileTrue(m_tomb.tomb());
    //m_driverController.x().onTrue(m_hood.setHoodAngleCommand(15.0));
    //m_driverController.b().onTrue(m_hood.setHoodAngleCommand(30.0));
  }

  public Command getAutonomousCommand() {
    if (m_vision != null && AutoConstants.kUseFuelObjectAuto) {
      return new FollowFuelCommand(m_vision, m_robotDrive)
          .withTimeout(AutoConstants.kFuelAutoTimeoutSeconds);
    }

    if (m_autoChooser != null && m_autoChooser.getSelected() != null) {
      return m_autoChooser.getSelected();
    }
    return Commands.none();
  }
}
