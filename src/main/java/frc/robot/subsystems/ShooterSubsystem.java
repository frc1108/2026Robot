// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

@Logged
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_leftShooter;
  private final TalonFX m_rightShooter;
  private HoodSubsystem m_hood;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_leftShooter = new TalonFX(ShooterConstants.kLeftShooterCanId, "rio");
    m_rightShooter = new TalonFX(ShooterConstants.kRightShooterCanId, "rio");
  }

  /**
   * Set the hood subsystem (call this from RobotContainer)
   */
  public void setHoodSubsystem(HoodSubsystem hood) {
    m_hood = hood;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setShooterPower(double power) {
    m_leftShooter.set(power);
    m_rightShooter.set(-power);
  }

  /**
   * Stop the shooter
   */
  public void stopShooter() {
    setShooterPower(0.0);
  }
public Command shootCommand() {
return this.startEnd( () -> this.setShooterPower(0.90), () -> this.setShooterPower(0.0));
}
public Command slowShootCommand() {
return this.startEnd( () -> this.setShooterPower(0.5), () -> this.setShooterPower(0.0));
}

  /**
   * Get current shooter power
   */
  public double getShooterPower() {
    return m_leftShooter.get();
  }

  /**
   * Shoot at full power
   */
  public Command shootCommand() {
    return this.startEnd(
        () -> this.setShooterPower(ShooterConstants.kShooterFullSpeed),
        this::stopShooter
    );
  }

  /**
   * Shoot at slow/controlled speed
   */
  public Command slowShootCommand() {
    return this.startEnd(
        () -> this.setShooterPower(ShooterConstants.kShooterSlowSpeed),
        this::stopShooter
    );
  }

  /**
   * Shoot at full power with hood at specified angle
   */
  public Command shootWithHoodCommand(double hoodAngleDegrees) {
    if (m_hood == null) {
      return shootCommand(); // Fall back to normal shoot if hood not available
    }
    
    return Commands.parallel(
        shootCommand(),
        m_hood.setHoodAngleCommand(hoodAngleDegrees)
    );
  }

  /**
   * Shoot at slow speed with hood at specified angle
   */
  public Command slowShootWithHoodCommand(double hoodAngleDegrees) {
    if (m_hood == null) {
      return slowShootCommand(); // Fall back to normal shoot if hood not available
    }
    
    return Commands.parallel(
        slowShootCommand(),
        m_hood.setHoodAngleCommand(hoodAngleDegrees)
    );
  }
}
