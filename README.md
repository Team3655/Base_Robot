# Base Robot - FRC Project Template

**Team 3655 - The Tractor Technicians**

This repository serves as the foundational architecture for all robot projects for Team 3655. It implements a modern, maintainable codebase using industry-standard patterns and tools.

**Note:** This codebase incorporates code and architectural patterns from FRC 6328 (Mechanical Advantage), who are credited for their excellent work and open-source contributions to the FRC community.

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Key Components](#key-components)
- [Design Decisions](#design-decisions)
- [Getting Started](#getting-started)
- [Extending the Base](#extending-the-base)

## Overview

This base robot project provides:

- **Swerve Drive** with full odometry and pose estimation
- **Vision Integration** with AprilTag support and pose estimation
- **Command-Based Architecture** using WPILib's command framework
- **IO Layer Pattern** for hardware abstraction
- **AdvantageKit Logging** for data recording and replay
- **PathPlanner Integration** for autonomous path following

### Technologies & Libraries

- **WPILib** - Core FRC framework
- **AdvantageKit** - Advanced logging and replay capabilities
- **PathPlanner** - Autonomous path planning and following

## Architecture

### IO Layer Pattern

The codebase uses an **IO Layer Pattern** (recommended by [AdvantageKit](https://docs.advantagekit.org/category/data-flow)) to abstract hardware implementations. This pattern provides several key benefits:

1. **Hardware Abstraction**: Subsystems don't know about specific hardware implementations
2. **Easy Testing**: Can swap real hardware for sim implementations
3. **Replay Support**: Empty IO implementations for log replay (AdvantageKit injects data from logs)
4. **Maintainability**: Hardware changes only affect IO classes
5. **Data Flow**: Enables AdvantageKit's logging and replay system to work seamlessly

**Note:** This pattern is part of AdvantageKit's recommended architecture for proper data flow and log replay. See the [AdvantageKit Data Flow documentation](https://docs.advantagekit.org/category/data-flow) for more details.

#### How It Works

Each subsystem that interacts with hardware has:

- An **IO Interface** (e.g., `ModuleIO`, `GyroIO`, `VisionIO`) that defines the contract
- **Concrete Implementations** for different hardware:
  - Real hardware: `ModuleIOTalonFX`, `GyroIOPigeon2`, `VisionIOLimelight`
  - Simulation: `ModuleIOSim`, `VisionIOSim`
  - Replay: Empty implementations (no-op)

The subsystem takes IO implementations as constructor parameters, allowing different implementations to be injected based on the robot mode (real, sim, or replay).

```java
// In RobotContainer.java - Different implementations based on mode
switch (Constants.currentMode) {
  case REAL:
    drive = new DriveSubsystem(
        new GyroIOPigeon2(),
        new ModuleIOTalonFX(0),
        // ...
    );
    break;
  case SIM:
    drive = new DriveSubsystem(
        new GyroIO() {},  // Empty implementation
        new ModuleIOSim(),
        // ...
    );
    break;
}
```

### Command-Based Framework

The project uses WPILib's **Command-Based Programming** model:

- **Subsystems** (`DriveSubsystem`, `VisionSubsystem`) - Represent robot hardware/functionality
- **Commands** (`DriveCommands`) - Represent actions the robot can perform
- **Triggers** - Button/input bindings that trigger commands
- **CommandScheduler** - Manages command execution

Commands are composed, scheduled, and can interrupt each other. This makes the code modular and easy to reason about.

### Robot State Management

`RobotState` is a **singleton** that manages the robot's pose estimation and the state of its mechanisms:

- **Odometry**: Uses wheel encoders and gyro for continuous pose tracking
- **Vision Fusion**: Incorporates vision measurements with uncertainty
- **Pose Estimator**: Combines odometry and vision using WPILib's `SwerveDrivePoseEstimator`

This centralized state management ensures all subsystems have access to the same pose information.

## Key Components

### Core Classes

#### `Robot.java`

The main robot class that extends `LoggedRobot` (from AdvantageKit). Handles:

- Robot lifecycle (init, periodic, disabled, autonomous, teleop, test)
- AdvantageKit logger initialization
- Command scheduler execution

**Key Points:**

- Uses `LoggedRobot` instead of `TimedRobot` for AdvantageKit integration
- Logger is configured based on mode (REAL, SIM, REPLAY)
- Command scheduler runs in `robotPeriodic()`

#### `RobotContainer.java`

The central container that:

- Initializes all subsystems with appropriate IO implementations
- Configures button bindings and controller mappings
- Sets up autonomous command chooser
- Manages driver-specific control schemes

**Driver Modes:**

- `MAIN` - Competition driver using CommandNXT controllers
- `PROGRAMMING` - Development using Xbox controllers
- `MACBOOK` - Simulation/testing on MacBook

**Important:** Always instantiate subsystems here, not in `Robot.java`. This keeps initialization logic centralized.

#### `RobotState.java`

Singleton class managing robot pose estimation:

- **`addOdometryMeasurement()`** - Updates pose from wheel encoders and gyro
- **`addVisionMeasurement()`** - Incorporates vision-based pose estimates
- **`getEstimatedPose()`** - Returns the best estimate (odometry + vision)
- **`getOdometryPose()`** - Returns odometry-only pose
- **`zeroHeading()`** - Resets robot heading to 0° while preserving position

**Why Singleton?**

- Single source of truth for robot pose
- Prevents multiple pose estimators from diverging
- All subsystems can access the same pose data

#### `Constants.java`

Central configuration file containing:

- **`currentMode`** - REAL, SIM, or REPLAY (auto-detected)
- **`currentDriver`** - Which driver control scheme to use
- **`currentRobot`** - COMPBOT or PROTOBOT (for different hardware configs)
- **CAN Bus Configuration** - CANivore name and bus setup

**Best Practices:**

- All constants should be `public static final`
- Use enums for related constants
- Don't put functional code here - only constants

### Subsystems

#### `DriveSubsystem`

Manages the swerve drive base:

**Key Features:**

- 4 swerve modules (FL, FR, BL, BR)
- Gyro integration for heading
- PathPlanner AutoBuilder integration
- SysId routines for characterization
- Odometry thread for high-frequency updates

**Important Methods:**

- **`runVelocity(ChassisSpeeds)`** - Sets robot velocity (field-relative or robot-relative)
- **`sysIdQuasistatic/Dynamic()`** - Returns SysId test commands

**Odometry Updates:**

- Uses `PhoenixOdometryThread` for high-frequency (250Hz) encoder sampling
- Locks prevent race conditions during odometry updates
- Multiple samples per cycle for accurate pose estimation

#### `VisionSubsystem`

Handles vision processing and pose estimation:

**Key Features:**

- Supports multiple cameras
- AprilTag detection and pose estimation
- Pose filtering (rejects invalid measurements)
- Standard deviation calculation based on tag distance and count

**Pose Filtering:**

- Rejects poses with high ambiguity
- Rejects poses outside field boundaries
- Different distance limits for single vs. multi-tag observations
- Tighter tolerances for MegaTag2 observations

**Integration:**

- Vision measurements are added to `RobotState` with calculated uncertainty
- The pose estimator automatically weights vision vs. odometry based on uncertainty

### Commands

#### `DriveCommands`

Factory class for drive-related commands:

- **`joystickDrive()`** - Field-relative drive command with deadband and input shaping
- **`feedforwardCharacterization()`** - Measures kS and kV for drive motors
- **`wheelRadiusCharacterization()`** - Measures actual wheel radius
- **`pathFindToPose()`** - Dynamic pathfinding to a target pose

**Input Processing:**

- Deadband applied to joystick inputs
- Input shaping (squaring) for better low-speed control
- Field-relative conversion with alliance flipping
- Slow mode when trigger/button held

### Utilities

#### `CommandNXT.java`

Wrapper for KBSIM Gladiator NXT controllers:

- Provides named methods for all buttons and axes
- Integrates with command framework via `CommandGenericHID`
- Handles multi-stage triggers (fireStage1, fireStage2)

#### `PathPlannerUtil.java`

Utilities for PathPlanner integration:

- Writes robot configuration to PathPlanner settings file
- Ensures PathPlanner matches robot constants

#### `PhysicsUtil.java`

Physics calculations:

- Moment of inertia estimation

## Design Decisions

### Why IO Layer Pattern?

**Problem:** Traditional FRC code mixes hardware-specific code with subsystem logic, making it hard to:

- Test without hardware
- Switch between hardware implementations
- Replay logs without hardware (AdvantageKit needs this structure)

**Solution:** IO interfaces abstract hardware details. Subsystems work with interfaces, not concrete hardware. This is the [recommended pattern from AdvantageKit](https://docs.advantagekit.org/category/data-flow) for proper data flow.

**Benefits:**

- Easy simulation - just swap IO implementations
- Log replay works without hardware (AdvantageKit injects logged data)
- Hardware changes isolated to IO classes
- Testable subsystems
- Enables AdvantageKit's logging and replay features

**Credit:** This pattern is part of AdvantageKit's architecture. See their [Data Flow documentation](https://docs.advantagekit.org/category/data-flow) for the official explanation.

### Why Singleton for RobotState?

**Problem:** Multiple pose estimators would diverge and cause inconsistencies.

**Solution:** Single `RobotState` instance ensures all subsystems use the same pose data.

**Benefits:**

- Single source of truth
- Consistent pose across all subsystems
- Easier debugging (one place to check pose)

### Why Command-Based Framework?

**Problem:** Traditional imperative code becomes hard to manage as complexity grows.

**Solution:** Command-based framework provides:

- Modular, composable actions
- Built-in scheduling and interruption
- Clear separation of concerns

**Benefits:**

- Commands can be composed (parallel, sequential, etc.)
- Easy to add/remove functionality
- Built-in safety (subsystem requirements)

### Why AdvantageKit?

**Problem:** Standard WPILib logging is limited and doesn't support replay.

**Solution:** AdvantageKit provides:

- High-frequency logging
- Log replay for testing
- Advanced data visualization
- Deterministic timestamps

**Benefits:**

- Debug issues from competition matches
- Test code changes against real match data
- Characterize robot behavior

### Why PathPlanner?

**Problem:** Manual trajectory generation is time-consuming and error-prone.

**Solution:** PathPlanner provides:

- Visual path editor
- Automatic path optimization
- Dynamic pathfinding
- Easy autonomous creation

**Benefits:**

- Faster autonomous development
- Better path optimization
- Visual debugging

## Getting Started

### Setting Up a New Robot Project

1. **Clone this repository** as your starting point

2. **Update Constants:**

   - Set `currentDriver` to match your driver setup
   - Set `currentRobot` if you have multiple robot configurations
   - Update CAN bus name if not using "ctre"

3. **Configure Drive Constants:**

   - Update `DriveConstants.java` with your robot's physical parameters:
     - Track width (X and Y)
     - Wheel radius
     - Gear ratios
     - Mass and moment of inertia
     - Encoder offsets

4. **Configure Vision:**

   - Update `VisionConstants.java` with camera transforms
   - Adjust filtering thresholds as needed

5. **Set Up Controllers:**

   - Update `RobotContainer.java` with your controller ports
   - Configure button bindings in `configureButtonBindings()`

6. **Test in Simulation:**
   - Run in SIM mode first to verify everything works
   - Use AdvantageKit visualization to debug

### Running the Robot

**Real Robot:**

- Deploy code to robot
- `Constants.currentMode` will automatically be `REAL`
- Logger will write to USB stick at `/U/logs`

**Simulation:**

- Set `Constants.simMode = Mode.SIM`
- Run simulation in VS Code or WPILib
- Logger outputs to NetworkTables

## Extending the Base

### Adding a New Subsystem

1. **Create IO Interface:**

```java
public interface MySubsystemIO {
  @AutoLog
  public static class MySubsystemIOInputs {
    // Define inputs here
  }

  public default void updateInputs(MySubsystemIOInputs inputs) {}
  // Define other methods
}
```

2. **Create IO Implementations:**

   - `MySubsystemIOReal.java` - Real hardware implementation
   - `MySubsystemIOSim.java` - Simulation implementation

3. **Create Subsystem:**

```java
public class MySubsystem extends SubsystemBase {
  private final MySubsystemIO io;
  private final MySubsystemIOInputsAutoLogged inputs;

  public MySubsystem(MySubsystemIO io) {
    this.io = io;
    this.inputs = new MySubsystemIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Inputs/MySubsystem", inputs);
    // Subsystem logic here
  }
}
```

4. **Add to RobotContainer:**

```java
// In constructor, add to switch statement
case REAL:
  mySubsystem = new MySubsystem(new MySubsystemIOReal());
  break;
case SIM:
  mySubsystem = new MySubsystem(new MySubsystemIOSim());
  break;
```

### Adding Commands

1. **Create command factory class** (like `DriveCommands.java`):

```java
public class MySubsystemCommands {
  public static Command myCommand(MySubsystem subsystem) {
    return Commands.run(() -> {
      // Command logic
    }, subsystem);
  }
}
```

2. **Bind to buttons in RobotContainer:**

```java
controller.button().onTrue(MySubsystemCommands.myCommand(mySubsystem));
```

### Adding Autonomous Routines

1. **Create paths in PathPlanner** (If using the GUI, it automatically saves to `src/main/deploy/pathplanner/`)

2. **AutoBuilder automatically creates commands** - they appear in the auto chooser

3. **For custom autonomous commands**, add to auto chooser:

```java
autoChooser.addOption("My Auto", MyAutoCommand);
```

### Modifying Drive Behavior

**To change drive characteristics:**

- Update PID constants in `DriveConstants.java`
- Modify feedforward constants (kS, kV)
- Adjust max speeds/accelerations

**To change control scheme:**

- Modify `DriveCommands.joystickDrive()` for input processing
- Update button bindings in `RobotContainer.configureButtonBindings()`

**To add new drive commands:**

- Add methods to `DriveCommands.java`
- Bind to buttons/triggers in `RobotContainer`

### Working with RobotState

**Getting the current pose:**

```java
Pose2d currentPose = RobotState.getInstance().getPose();
Rotation2d heading = RobotState.getInstance().getRotation();
```

**Resetting pose:**

```java
// Reset to specific pose
RobotState.getInstance().resetPose(new Pose2d(x, y, rotation));

// Zero heading only (preserve position)
RobotState.getInstance().zeroHeading();
```

**Adding vision measurements:**

```java
// Usually done in VisionSubsystem, but if needed:
RobotState.getInstance().addVisionMeasurement(
    new VisionMeasurement(timestamp, pose, stdDevs)
);
```

## Best Practices

### Code Organization

- **One subsystem per physical component** - Keep subsystems focused
- **Commands in separate classes** - Don't put command logic in subsystems
- **Constants in Constants classes** - Don't hardcode values
- **IO implementations separate** - Hardware code in IO classes only

### Naming Conventions

- **Subsystems**: `*Subsystem` (e.g., `DriveSubsystem`)
- **IO Interfaces**: `*IO` (e.g., `ModuleIO`)
- **IO Implementations**: `*IO*` (e.g., `ModuleIOTalonFX`, `ModuleIOSim`)
- **Commands**: `*Commands` (factory class) or `*Command` (single command)
- **Constants**: `*Constants` (e.g., `DriveConstants`)

### Logging

- **Use `@AutoLog`** on IO input classes for automatic logging
- **Use `@AutoLogOutput`** for subsystem outputs
- **Log meaningful data** - Don't log everything, but log what you need to debug
- **Use descriptive keys** - Organize logs hierarchically (e.g., `Drive/SwerveStates/Measured`)

### Testing

- **Test in simulation first** - Catch issues before deploying to robot
- **Use AdvantageKit replay** - Test code changes against real match data
- **Characterize subsystems** - Use SysId and characterization commands
- **Log everything** - You can't debug what you didn't log

## Additional Resources

- [WPILib Documentation](https://docs.wpilib.org/)
- [AdvantageKit Documentation](https://github.com/Mechanical-Advantage/AdvantageKit)
- [AdvantageKit Data Flow](https://docs.advantagekit.org/category/data-flow) - Official explanation of the IO layer pattern
- [PathPlanner Documentation](https://pathplanner.dev/)
- [CTRE Phoenix 6 Documentation](https://v6.docs.ctr-electronics.com/)

## Credits & License

**Team 3655 - The Tractor Technicians** maintains this base robot template for use across all our robot projects.

This project incorporates code and architectural patterns from **FRC 6328 (Mechanical Advantage)**.
