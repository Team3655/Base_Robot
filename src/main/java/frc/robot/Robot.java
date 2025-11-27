package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The main robot class that manages the robot lifecycle and AdvantageKit logging.
 * 
 * <p>This class extends {@link LoggedRobot} (from AdvantageKit) instead of {@link TimedRobot}
 * to enable advanced logging and replay capabilities. The robot lifecycle follows WPILib's
 * standard structure with initialization and periodic methods for each mode.
 * 
 * <p><b>Key Responsibilities:</b>
 * <ul>
 *   <li>Initialize AdvantageKit logger based on robot mode (REAL, SIM, REPLAY)</li>
 *   <li>Instantiate RobotContainer (which creates all subsystems)</li>
 *   <li>Run CommandScheduler every periodic cycle</li>
 *   <li>Handle mode transitions (autonomous, teleop, disabled, test)</li>
 * </ul>
 * 
 * <p><b>AdvantageKit Integration:</b>
 * <ul>
 *   <li>REAL mode: Logs to USB stick at "/U/logs" and NetworkTables</li>
 *   <li>SIM mode: Logs to NetworkTables for visualization</li>
 *   <li>REPLAY mode: Reads from log files and injects data into IO implementations</li>
 * </ul>
 * 
 * <p><b>Important:</b> Do not instantiate subsystems here. All subsystem creation
 * should happen in {@link RobotContainer}.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  /**
   * Initializes the robot when first started up.
   * 
   * <p>This method:
   * <ul>
   *   <li>Starts WPILib DataLogManager for basic logging</li>
   *   <li>Configures AdvantageKit logger based on robot mode</li>
   *   <li>Records build metadata (Git SHA, date, branch, etc.)</li>
   *   <li>Sets up data receivers (USB logging, NetworkTables, replay source)</li>
   *   <li>Creates RobotContainer (which initializes all subsystems)</li>
   * </ul>
   * 
   * <p>The logger configuration is critical for AdvantageKit's functionality.
   * Different modes require different data receivers to enable logging and replay.
   */
  @Override
  public void robotInit() {

    DataLogManager.start();

    // The most important part of the code
    DriverStation.silenceJoystickConnectionWarning(true);

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /**
   * Called periodically during all robot modes (disabled, autonomous, teleop, test).
   * 
   * <p>This method runs the CommandScheduler, which is the heart of the command-based
   * framework. The scheduler:
   * <ul>
   *   <li>Polls button/trigger states</li>
   *   <li>Schedules newly-triggered commands</li>
   *   <li>Executes running commands</li>
   *   <li>Removes finished or interrupted commands</li>
   *   <li>Calls subsystem periodic() methods</li>
   * </ul>
   * 
   * <p><b>Critical:</b> This must be called every periodic cycle (every 20ms) for
   * the command-based framework to function. Without this, no commands will run
   * and subsystems won't update.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
