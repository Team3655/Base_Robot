package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.VisionConstants;

import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.CommandNXT;

/**
 * The central container for the robot that manages subsystems, IO implementations, and controls.
 * 
 * <p>This class is the heart of the robot's architecture. It:
 * <ul>
 *   <li>Initializes all subsystems with appropriate IO implementations based on robot mode</li>
 *   <li>Configures button bindings and controller mappings</li>
 *   <li>Sets up autonomous command chooser</li>
 *   <li>Manages driver-specific control schemes</li>
 * </ul>
 * 
 * <p><b>IO Layer Pattern:</b> This class demonstrates the IO layer pattern recommended by
 * AdvantageKit. Different IO implementations are instantiated based on {@link Constants#currentMode}:
 * <ul>
 *   <li>REAL: Hardware IO implementations (e.g., {@link ModuleIOTalonFX}, {@link GyroIOPigeon2})</li>
 *   <li>SIM: Simulation IO implementations (e.g., {@link ModuleIOSim})</li>
 *   <li>REPLAY: Empty IO implementations (AdvantageKit injects data from logs)</li>
 * </ul>
 * 
 * <p><b>Subsystem Initialization:</b> All subsystems should be created here, not in {@link Robot}.
 * This keeps initialization logic centralized and makes it easy to see the robot's structure.
 * 
 * <p><b>Control Schemes:</b> Supports multiple driver configurations via {@link Constants#currentDriver}:
 * <ul>
 *   <li>MAIN: Competition driver using CommandNXT controllers</li>
 *   <li>PROGRAMMING: Development using Xbox controllers</li>
 *   <li>MACBOOK: Simulation/testing on MacBook with different axis mappings</li>
 * </ul>
 * 
 * @see <a href="https://docs.advantagekit.org/category/data-flow">AdvantageKit Data Flow</a>
 */
public class RobotContainer {

  private final RobotState robotState = RobotState.getInstance();

  // Subsystems
  private final DriveSubsystem drive;

  @SuppressWarnings("unused")
  // Vision does not have any direct commands, so it is "unused" in this file
  // However, it must be initialized to run properly
  private final VisionSubsystem vision;

  // Programming controller
  private final CommandXboxController programmingController = new CommandXboxController(5);

  // Driver controller
  private final CommandNXT mainTranslation = new CommandNXT(0);
  private final CommandNXT mainRotation = new CommandNXT(1);

  // Operator controller
  @SuppressWarnings("unused")
  private final CommandGenericHID tractorController = new CommandGenericHID(4);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private double driveMultiplier = 0.4;

  // region Subsystem init
  /**
   * Constructs the RobotContainer and initializes all subsystems.
   * 
   * <p>This constructor:
   * <ol>
   *   <li>Creates subsystems with appropriate IO implementations based on robot mode</li>
   *   <li>Sets up PathPlanner AutoBuilder for autonomous routines</li>
   *   <li>Adds SysId and characterization commands to auto chooser</li>
   *   <li>Configures button bindings for the selected driver</li>
   * </ol>
   * 
   * <p><b>IO Implementation Selection:</b> The switch statement on {@link Constants#currentMode}
   * demonstrates the IO layer pattern. Each mode uses different IO implementations, but the
   * subsystems remain unchanged. This is why the IO layer pattern is so powerful - subsystems
   * are hardware-agnostic.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      // These implementations read from and write to actual hardware
      case REAL:
        drive = new DriveSubsystem(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));

        vision = new VisionSubsystem(
            new VisionIOLimelight("limelight"));
        break;

      // Sim robot, instantiate physics sim IO implementations
      // These implementations use physics simulation models instead of real hardware
      case SIM:
        drive = new DriveSubsystem(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());

        vision = new VisionSubsystem(
            new VisionIOSim("left", VisionConstants.LEFT_ROBOT_TO_CAMERA));
        break;

      // Replayed robot, disable IO implementations
      // Empty implementations are used - AdvantageKit injects data from log files
      // This allows replaying match data without any hardware present
      default:
        drive = new DriveSubsystem(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        vision = new VisionSubsystem(
            new VisionIO() {
            });
        break;
    }

    // region Autonomous Commands

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Configures button bindings and control schemes based on the selected driver.
   * 
   * <p>This method sets up:
   * <ul>
   *   <li>Default commands for subsystems (e.g., joystick drive)</li>
   *   <li>Button-triggered commands (e.g., zero heading, stop with X)</li>
   *   <li>Operator controls for mechanisms</li>
   * </ul>
   * 
   * <p>The control scheme is selected via {@link Constants#currentDriver}. Each driver
   * may use different controllers or have different button mappings based on their
   * preferences or the development environment.
   * 
   * <p><b>Default Commands:</b> Use {@link SubsystemBase#setDefaultCommand(Command)} to
   * set commands that run continuously when no other command requires the subsystem.
   * 
   * <p><b>Button Bindings:</b> Use trigger methods (e.g., {@code button().onTrue()}) to
   * bind commands to button events. These are managed automatically by the CommandScheduler.
   */
  private void configureButtonBindings() {

    //region Driver controls
    switch (Constants.currentDriver) {
      case MAIN:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -mainTranslation.StickYAxis() * 1.0,
                () -> -mainTranslation.StickXAxis() * 1.0,
                () -> -mainRotation.StickXAxis() * 0.7,
                1,
                mainTranslation.fireStage1()
                    .or(mainTranslation.fireStage2())));

        mainTranslation.B1().onTrue(Commands.runOnce(robotState::zeroHeading));

        mainTranslation.A2().whileTrue(Commands.run(() -> drive.stopWithX(), drive));
        break;

      // Programming uses Xbox controllers
      case PROGRAMMING:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> programmingController.getLeftY(),
                () -> programmingController.getLeftX(),
                () -> -programmingController.getRightX(),
                1,
                programmingController.leftBumper()));

        break;

      // When running sim on a Macbook, the controls are different than an Xbox
      // controller running a real robot
      case MACBOOK:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> programmingController.getRawAxis(1),
                () -> programmingController.getRawAxis(0),
                () -> -programmingController.getRawAxis(2),
                driveMultiplier,
                programmingController.leftTrigger()));

        programmingController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
        programmingController.button(12).onTrue(Commands.runOnce(robotState::zeroHeading));
        break;
    }

    // region Operator controls

    /**
     * This is where you would define button bindings and controls for our operator
     * board,
     * by default it has nothing since operator controls the robot's mechanisms and
     * noe the drive base
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
