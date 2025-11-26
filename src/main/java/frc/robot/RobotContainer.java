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
 * This class is where the robot and its subsystems should be declared.
 * It will also handle button pressing and autonomous logic.
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
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
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
   * Function to define controls based on currently set driver
   */
  private void configureButtonBindings() {
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
