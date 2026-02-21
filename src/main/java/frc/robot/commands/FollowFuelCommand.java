package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FollowFuelCommand extends Command {
  private final VisionSubsystem m_vision;
  private final DriveSubsystem m_drive;
  private final PIDController m_turnPID;

  public FollowFuelCommand(VisionSubsystem vision, DriveSubsystem drive) {
    m_vision = vision;
    m_drive = drive;
    m_turnPID = new PIDController(
        VisionConstants.kFuelAimP,
        VisionConstants.kFuelAimI,
        VisionConstants.kFuelAimD);
    m_turnPID.setTolerance(VisionConstants.kFuelYawToleranceDegrees);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_turnPID.reset();
  }

  @Override
  public void execute() {
    if (m_vision.canSeeFuel()) {
      double yaw = m_vision.getFuelYaw();
      double area = m_vision.getFuelArea();

      double turn = m_turnPID.calculate(yaw, 0.0);
      turn = MathUtil.clamp(turn, -VisionConstants.kFuelMaxTurnSpeed, VisionConstants.kFuelMaxTurnSpeed);

      double drive = (VisionConstants.kFuelTargetArea - area) * VisionConstants.kFuelApproachP;
      drive = MathUtil.clamp(drive, -VisionConstants.kFuelMaxDriveSpeed, VisionConstants.kFuelMaxDriveSpeed);

      m_drive.drive(drive, 0.0, turn, false);
    } else {
      m_drive.drive(0.0, 0.0, VisionConstants.kFuelSearchTurnSpeed, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0.0, 0.0, 0.0, false);
  }

  @Override
  public boolean isFinished() {
    return m_vision.canSeeFuel()
        && m_vision.getFuelArea() >= VisionConstants.kFuelFinishArea
        && Math.abs(m_vision.getFuelYaw()) <= VisionConstants.kFuelYawToleranceDegrees;
  }
}
