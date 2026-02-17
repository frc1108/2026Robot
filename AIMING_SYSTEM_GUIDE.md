# Auto-Aiming System Implementation Guide

## Overview
This system enables your robot to automatically aim and shoot at the hopper using PhotonVision to detect April tags. The robot rotates using its swerve drive while the shooter remains stationary. You adjust the shot distance using shooter speed or hood angle.

## Components Created

### 1. **VisionSubsystem** (`src/main/java/frc/robot/subsystems/VisionSubsystem.java`)
- Handles all camera interactions with PhotonVision
- Detects the hopper's April tag and provides targeting data:
  - `canSeeHopper()` - Check if hopper is visible
  - `getHopperYaw()` - Horizontal angle to hopper (degrees)
  - `getHopperPitch()` - Vertical angle to hopper (degrees)
  - `getHopperDistance()` - Distance to hopper (meters)
  - `getHopperAmbiguity()` - Confidence of detection (0-1)

### 2. **HoodSubsystem** (`src/main/java/frc/robot/subsystems/HoodSubsystem.java`)
- Controls the hood angle using Neo 550 motor (CAN ID 33 by default)
- Features PID-based angle control with range [0°, 45°]
- Methods:
  - `setHoodAngle(degrees)` - Set hood to specific angle
  - `getHoodAngle()` - Get current hood angle
  - `isAtTarget(angle, tolerance)` - Check if at target angle
  - `setHoodAngleCommand(degrees)` - Create a command for hood positioning

### 3. **AimAtHopperCommand** (`src/main/java/frc/robot/commands/AimAtHopperCommand.java`)
- Automatically rotates the robot to face the hopper
- Uses PID control to smoothly adjust robot heading
- Stops when pointing at hopper within tolerance (±2° by default)
- Non-blocking - robot can move and shoot while aiming

### 4. **Updated ShooterSubsystem**
- Integrated with HoodSubsystem
- Additional commands for coordinated shooting:
  - `shootWithHoodCommand(angle)` - Shoot at full power with hood at specific angle
  - `slowShootWithHoodCommand(angle)` - Shoot at slow speed with hood at specific angle

## Configuration Requirements

### PhotonVision Setup
1. **Camera Name**: Update in `Constants.VisionConstants.kCameraName`
   - Must match the name configured in PhotonVision UI
   - Default: `"shooter_camera"`

2. **Hopper April Tag ID**: Update `Constants.VisionConstants.kHopperTagId`
   - Default: `5` - **CHANGE THIS TO YOUR HOPPER'S TAG ID**
   - Verify in your field layout (First Robotics AprilTag scheme)

### Hardware Configuration
1. **Hood Motor CAN ID**: `Constants.ShooterConstants.kHoodMotorCanId`
   - Default: `33` (Neo 550)
   - Adjust if using different CAN ID

2. **Hood Gear Ratio**: `Constants.ShooterConstants.kHoodGearRatio`
   - Default: `50.0` - **TUNE THIS BASED ON YOUR ACTUAL GEARBOX RATIO**
   - Used to convert motor position to degrees

3. **Hood Angle Limits**: 
   - Min: `Constants.ShooterConstants.kMinHoodAngleDegrees` (default 0°)
   - Max: `Constants.ShooterConstants.kMaxHoodAngleDegrees` (default 45°)

### Tuning Constants
In `Constants.java`, adjust these for your robot:

**Hood PID** (for steady hood positioning):
```java
public static final double kHoodP = 0.5;      // Proportional gain
public static final double kHoodI = 0.0;      // Integral gain
public static final double kHoodD = 0.1;      // Derivative gain
```

**Vision Aiming PID** (for robot rotation):
```java
public static final double kAimP = 0.02;      // Proportional gain
public static final double kAimI = 0.0;       // Integral gain
public static final double kAimD = 0.001;     // Derivative gain
```

**Aiming Tolerance & Timeout**:
```java
public static final double kAimingToleranceDegrees = 2.0;    // Must be within 2° to aim complete
public static final double kAimingTimeoutSeconds = 2.0;      // Max time to spend aiming
```

## Controller Buttons

Default Xbox controller mappings in `RobotContainer`:

| Button | Action |
|--------|--------|
| **A** | Aim at hopper (hold to maintain aiming) |
| **X** | Set hood to 15° (low angle shot) |
| **B** | Set hood to 30° (mid angle shot) |
| **Right Trigger** | Shoot at full power |
| **Right Bumper** | Shoot at slow/controlled speed |
| **Left Trigger** | Intake |
| **Left Bumper** | Slow intake |
| **Y** | Tomb forward |
| **D-Pad Down** | Reverse intake & tomb |

## How to Use

### Basic Aiming and Shooting
1. **Hold button A** to aim at the hopper (robot rotates)
2. **Use X or B buttons** to set hood angle based on distance
3. **Hold right trigger** to shoot once aimed

### Advanced Usage
For more sophisticated control, you can:

1. **Combine commands** - Use `shootWithHoodCommand()` to shoot and adjust hood simultaneously:
   ```java
   m_driverController.rightTrigger().whileTrue(
       m_shooter.shootWithHoodCommand(25.0) // Full power at 25° hood
   );
   ```

2. **Fine-tune hood** during matches by adjusting X/B presets or using analog stick:
   ```java
   m_driverController.rightStickY().onTrue(
       m_hood.setHoodAngleCommand(someCalculatedAngle)
   );
   ```

3. **Monitor vision data** - Check SmartDashboard values:
   - Hopper visible (boolean)
   - Hopper angle (degrees)
   - Distance to hopper (meters)

## Tuning Guide

### Hood Angle Tuning
1. Start with hood at 0° (vertical)
2. Test various angles to find optimal distances:
   - 0-15°: Close range shots
   - 15-25°: Mid range
   - 25-40°: Long range
3. Adjust PID gains if hood oscillates or moves slowly

### Aiming Speed Tuning
- **Too slow**: Increase `kAimP` in VisionConstants
- **Overshoots**: Increase `kAimD` (derivative dampening)
- **Oscillates**: Decrease `kAimP` or increase `kAimD`

### Distance-Based Hood Adjustment (Advanced)
After measuring distances and successful hood angles, you can implement automatic hood adjustment:

```java
double hopperDistance = m_vision.getHopperDistance();
double hoodAngle = 10.0 + (hopperDistance * 5.0); // Example formula
m_hood.setHoodAngle(hoodAngle);
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Camera not detected | Verify camera name in PhotonVision matches `kCameraName` constant |
| Hood doesn't move | Check Neo 550 motor CAN ID and power connections |
| Robot doesn't aim | Verify hopper April tag ID is correct; check PhotonVision detections |
| Aiming overshoots | Increase `kAimD`, decrease `kAimP` |
| Hood drifts or oscillates | Tune `kHoodP`, verify gearbox ratio |
| Erratic behavior | Ensure gyro is calibrated; check gyro orientation |

## Important Notes
- The aiming command assumes the shooter is fixed relative to the robot frame
- PhotonVision latency affects aiming accuracy; typically 20-40ms
- Gyro drift can accumulate; reset heading at match start
- Test all hood angles with actual balls before competition
- Hood motor should have appropriate braking to maintain position

## Next Steps
1. Deploy code and verify compilation
2. Configure PhotonVision camera in UI
3. Update hopper April tag ID in Constants
4. Calibrate hood gear ratio with actual mechanism
5. Tune all PID constants for your robot's dynamics
6. Test aiming and shooting in practice
