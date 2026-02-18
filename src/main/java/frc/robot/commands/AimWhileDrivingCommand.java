package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Drive while continuously aiming at the hopper using vision for rotation.
 * Left stick controls translation; rotation is controlled by PID to face the hopper.
 */
public class AimWhileDrivingCommand extends Command {
  private final VisionSubsystem m_vision;
  private final DriveSubsystem m_drive;
  private final CommandXboxController m_controller;
  private final PIDController m_rotationPID;

  public AimWhileDrivingCommand(VisionSubsystem vision, DriveSubsystem drive, CommandXboxController controller) {
    this.m_vision = vision;
    this.m_drive = drive;
    this.m_controller = controller;
    this.m_rotationPID = new PIDController(VisionConstants.kAimP, VisionConstants.kAimI, VisionConstants.kAimD);
    m_rotationPID.enableContinuousInput(-180, 180);
    m_rotationPID.setTolerance(VisionConstants.kAimingToleranceDegrees);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // No special init required
  }

  @Override
  public void execute() {
    // Read translation from left stick and apply deadband
    double xSpeed = -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband);
    double ySpeed = -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband);

    double rotationInputFallback = -MathUtil.applyDeadband(m_controller.getRightX(), OIConstants.kDriveDeadband);

    double rotation;
    if (m_vision.canSeeHopper()) {
      double hopperYaw = m_vision.getHopperYaw();
      double currentHeading = m_drive.getHeading();
      double targetHeading = currentHeading + hopperYaw; // Positive yaw = target to the right
      rotation = m_rotationPID.calculate(currentHeading, targetHeading);
      // Clamp to [-1,1]
      rotation = Math.max(-1.0, Math.min(1.0, rotation));
    } else {
      // Fallback to driver rotation if no target
      rotation = rotationInputFallback;
    }

    // Drive in field-relative mode so translation is still controlled by the driver
    m_drive.drive(xSpeed, ySpeed, rotation, true);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop rotation when command ends
    m_drive.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    // This is a whileTrue binding; return false so it only ends when released
    return false;
  }
}
