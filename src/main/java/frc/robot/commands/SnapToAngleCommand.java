package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Rotates the robot to an absolute field heading (degrees). Uses the same PID
 * constants as the aiming command to keep tuning consistent.
 */
public class SnapToAngleCommand extends Command {
  private final DriveSubsystem m_drive;
  private final PIDController m_rotationPID;
  private final double m_targetHeading;

  public SnapToAngleCommand(DriveSubsystem drive) {
    this.m_drive = drive;
    this.m_rotationPID = new PIDController(VisionConstants.kAimP, VisionConstants.kAimI, VisionConstants.kAimD);
    m_rotationPID.enableContinuousInput(-180, 180);
    m_rotationPID.setTolerance(VisionConstants.kAimingToleranceDegrees);
    this.m_targetHeading = 0.0; // absolute heading to snap to
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // no-op; target heading is fixed
  }

  @Override
  public void execute() {
    double currentHeading = m_drive.getHeading();
    double rotationSpeed = m_rotationPID.calculate(currentHeading, m_targetHeading);
    rotationSpeed = Math.max(-1.0, Math.min(1.0, rotationSpeed));
    m_drive.drive(0, 0, rotationSpeed, false);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    // Keep running while held (RobotContainer uses whileTrue). Also allow early end when at setpoint.
    return m_rotationPID.atSetpoint();
  }
}
