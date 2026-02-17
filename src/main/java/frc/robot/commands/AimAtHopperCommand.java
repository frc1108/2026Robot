package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimAtHopperCommand extends Command {
  private final VisionSubsystem m_vision;
  private final DriveSubsystem m_drive;
  private final PIDController m_rotationPID;
  private double m_targetHeading = 0.0;

  /**
   * Aims the robot at the hopper using PhotonVision
   * The robot will rotate to face the hopper detected by the April tag
   */
  public AimAtHopperCommand(VisionSubsystem vision, DriveSubsystem drive) {
    this.m_vision = vision;
    this.m_drive = drive;
    this.m_rotationPID = new PIDController(
        VisionConstants.kAimP,
        VisionConstants.kAimI,
        VisionConstants.kAimD
    );
    
    // Enable continuous input for angle wrapping (-180 to 180)
    m_rotationPID.enableContinuousInput(-180, 180);
    m_rotationPID.setTolerance(VisionConstants.kAimingToleranceDegrees);
    
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Start the timing
    m_targetHeading = m_drive.getHeading();
  }

  @Override
  public void execute() {
    if (!m_vision.canSeeHopper()) {
      // If we can't see the hopper, stop rotating
      m_drive.drive(0, 0, 0, false);
      return;
    }
    
    // Get the current angle error to the hopper
    double hopperYaw = m_vision.getHopperYaw();
    double currentHeading = m_drive.getHeading();
    
    // Calculate the target heading needed to point at the hopper
    // The vision yaw is relative to the camera, so we add it to current heading
    m_targetHeading = currentHeading + hopperYaw;
    
    // Use PID to calculate the rotation speed required
    double rotationSpeed = m_rotationPID.calculate(currentHeading, m_targetHeading);
    
    // Cap the rotation speed
    rotationSpeed = Math.max(-1.0, Math.min(1.0, rotationSpeed));
    
    // Drive with no translation, only rotation
    // Parameters: xSpeed, ySpeed, rotationSpeed, fieldRelative
    m_drive.drive(0, 0, rotationSpeed, false);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends
    m_drive.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    // Finish when we're pointed at the hopper within tolerance
    if (!m_vision.canSeeHopper()) {
      return false; // Keep trying to find it
    }
    
    double currentHeading = m_drive.getHeading();
    return m_rotationPID.atSetpoint();
  }
}
