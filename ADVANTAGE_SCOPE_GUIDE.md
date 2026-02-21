# Advantage Scope Integration Guide

## Overview
This implementation integrates **Advantage Scope** (a data logging and visualization tool) with your vision-based aiming system. Advantage Scope automatically logs all `@Logged` annotated fields and allows you to visualize them in real-time or replay from logs.

## What Gets Logged

### From VisionSubsystem
All vision-related data is automatically logged via Advantage Scope:

| Field | Type | Description |
|-------|------|-------------|
| `hopperVisible` | boolean | Camera can see hopper April tag |
| `hopperYaw` | double | Horizontal angle to hopper (degrees) |
| `hopperPitch` | double | Vertical angle to hopper (degrees) |
| `hopperDistance` | double | Distance to hopper (meters) |
| `hopperAmbiguity` | double | Target detection confidence (0-1) |

### From Other Subsystems
- **DriveSubsystem**: Robot pose, module positions, gyro heading
- **ShooterSubsystem**: Shooter power status
- **HoodSubsystem**: Current hood angle
- **IntakeSubsystem**: Intake status
- **TombSubsystem**: Tomb mechanism status

## Running Advantage Scope

### Setup
1. **Install Advantage Scope**:
   - Download from [Advantage Scope GitHub](https://github.com/Mechanical-Advantage/AdvantageScope)
   - Or install via https://advantagescope.org/

2. **Enable Data Logging**:
   - Code already has logging enabled in `Robot.java` via `DataLogManager.start()`
   - Logs are stored on robot RoboRIO at `/logs/`

### Live Viewing (WiFi Connection)
1. Connect to robot WiFi network
2. Open Advantage Scope
3. Click **Connect** → **Network Tables**
4. Enter robot IP: `10.0.0.2`
5. Click **Connect**
6. Use the **Live** tab to view real-time data

### Replay (Offline Analysis)
1. Connect RoboRIO via USB
2. Extract log files from `/logs/` directory
3. In Advantage Scope: **File** → **Open Log**
4. Select the `.rlog` or `.wpilog` file
5. Use timeline scrubbing to review matches

## Visualizing Vision Data

### Camera View
1. Add **Camera** widget to Advantage Scope
2. Configure with camera feed from PhotonVision

### Graph Plot
1. Click **+** to add visualization
2. Select **Line Graph**
3. Add fields:
   - `Hopper Yaw` - Shows targeting error over time
   - `Hopper Distance` - Distance trajectory
   - `Hopper Visible` - Boolean indicator of target tracking

### Example Dashboard for Debugging:
```
┌─────────────────────────────────────┐
│    HOPPER TARGETING - LINE GRAPH    │
├─────────────────────────────────────┤
│ ┌──────────────────────────────┐   │
│ │                              │   │
│ │  Hopper Yaw (error)          │   │
│ │  Hopper Pitch               │   │
│ │  Hopper Distance            │   │
│ │                              │   │
│ └──────────────────────────────┘   │
└─────────────────────────────────────┘

┌──────────────────┐  ┌──────────────────┐
│  HOPPER VISIBLE  │  │ HOPPER AMBIGUITY │
│                  │  │                  │
│  YES / NO        │  │  0.0 - 1.0       │
│                  │  │                  │
└──────────────────┘  └──────────────────┘
```

## Tuning Vision-Based Aiming Using Logs

### 1. Analyze Aiming Accuracy
1. Review `Hopper Yaw` graph during test matches
2. Check if yaw oscillates or converges smoothly
3. Verify `AimAtHopperCommand` completes quickly

**If yaw oscillates:**
- Decrease `VisionConstants.kAimP`
- Increase `VisionConstants.kAimD`

**If aiming is slow:**
- Increase `VisionConstants.kAimP`
- Decrease `VisionConstants.kAimD`

### 2. Monitor Distance Estimation
1. Compare `hopperDistance` logs with actual measured distances
2. Verify camera offset constants in `VisionConstants`
3. Check for realistic distance values

### 3. Verify Target Tracking
1. Monitor `hopperVisible` boolean
2. Check `hopperAmbiguity` stays below 0.35
3. Verify frames per second (check for dropouts)

### 4. Debug Hood Positioning
1. Create graph with hood angle vs. distance
2. Correlate successful shots with hood angles
3. Adjust hood angle presets in `RobotContainer` based on data

## Tips for Effective Logging

### Log Naming Convention
Advantage Scope uses forward slashes for hierarchy:
```
/Drive/Heading
/Vision/HopperYaw
/Vision/HopperDistance
/Shooter/Power
/Shooter/Hood/Angle
```

### Export Data
1. Right-click on data in Advantage Scope
2. Select **Export** → **CSV** for spreadsheet analysis
3. Use Excel/Google Sheets to correlate shooting success with:
   - Hopper distance
   - Hood angle  
   - Shooter speed
   - Aiming time

### Replay Speed Control
- **Play Button**: Real-time playback
- **1.0x, 2.0x, 4.0x**: Speed multipliers
- **Scrub Timeline**: Jump to specific moments
- **Frame By Frame**: `← →` arrow keys

## Performance Considerations

### Data Recording Overhead
- Logging uses ~1-2MB per match (~2.5 minutes)
- RoboRIO has ~1GB storage - plenty of room for multiple matches
- Network overhead: <1% of bandwidth

### Optimization Tips
1. **Selective Logging**: Only log what you need
   - Vision data: Always
   - Raw sensor values: Useful during tuning, can disable in comp
   
2. **Reduction Options**:
   - Default: All data every 20ms
   - Can change via `DataLogManager` settings

### Network Bandwidth
- Over WiFi: ~0.5 Mbps with Network Tables
- No significant impact on robot performance
- Safe to stream live data during matches

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "Cannot connect to robot" | Verify WiFi connection; check IP (`10.0.0.2`) |
| No vision data in logs | Verify `@Logged` annotations present; check VisionSubsystem initialization |
| Hopper distance always 0 | Check camera offset constants; verify April tag detection |
| Advantage Scope crashes on large logs | Split logs into smaller files; update Advantage Scope |
| Data appears delayed | Reduce refresh rate in Advantage Scope; check WiFi signal |

## Field Testing Workflow

### Pre-Match
1. Connect Advantage Scope
2. Verify all sensors logging (check Live tab)
3. Test vision targeting with dummy hopper
4. Record baseline log file

### During Match
1. Keep Advantage Scope connected (Live tab)
2. Monitor hopper visibility and distance
3. Watch aiming accuracy in real-time

### Post-Match
1. Export match log for analysis
2. Graph aiming accuracy vs. successful shots
3. Identify tuning opportunities
4. Update constants if needed

## Advanced: Custom Widgets

You can create custom visualization dashboards in Advantage Scope:

1. **Field 2D**: Show robot pose + AprilTag positions
2. **Mechanism 2D**: Visualize hood angle relative to shooter
3. **Number Display**: Real-time feedback on key metrics
4. **Boolean Indicator**: Visual indicator of hopper tracking status

## Resources
- [Advantage Scope Documentation](https://docs.advantagescope.org/)
- [WPILib DataLogManager](https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html)
- [2024+ Logging Best Practices](https://docs.wpilib.org/en/stable/docs/software/telemetry/what-is-telemetry.html)
