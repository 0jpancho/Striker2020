/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivebase.DiffDrive;
import frc.robot.commands.shooter.OpenLoopShooting;
import frc.robot.commands.shooter.VeloShooting;
import frc.robot.commands.indexer.SimpleFeed;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // private Joystick driverLeft = new Joystick(0);
  // private Joystick driverRight = new Joystick(1);

  private XboxController driver = new XboxController(0);

  private final SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(0.5);

  private final DriveBase m_driveBase = new DriveBase();
  private final Shooter m_shooter = new Shooter();
  private final Indexer m_indexer = new Indexer();

  ShuffleboardTab sensorTab = Shuffleboard.getTab("Sensors");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    DoubleSupplier forward = () -> m_forwardLimiter.calculate(driver.getY(Hand.kLeft));
    DoubleSupplier rot = () -> m_rotLimiter.calculate(driver.getX(Hand.kRight));

    m_driveBase.setDefaultCommand(new DiffDrive(m_driveBase, forward, rot));

    m_shooter.setDefaultCommand(new OpenLoopShooting(m_shooter, driver.getTriggerAxis(Hand.kRight)));

    Shuffleboard.getTab("Sensors").add("Heading", m_driveBase.getHeadingDegrees()).withSize(1, 1).withPosition(0, 0);
    Shuffleboard.getTab("Sensors").add("Navx Calibrating", m_driveBase.navxAlive()).withSize(1, 1).withPosition(1, 0);
    Shuffleboard.getTab("Sensors").add("Heading", m_driveBase.getHeadingDegrees()).withSize(1, 1).withPosition(0, 1);
    Shuffleboard.getTab("Sensors").add("Amps", m_driveBase.getTotalAmps()).withSize(1, 1).withPosition(1, 1);
    Shuffleboard.getTab("Sensors").add("Left Meters", m_driveBase.leftMetersTraveled).withSize(1, 1).withPosition(2, 0);
    Shuffleboard.getTab("Sensors").add("Left M/S", m_driveBase.leftMetersPerSec).withSize(1, 1).withPosition(3, 0);
    Shuffleboard.getTab("Sensors").add("Right Meters", m_driveBase.rightMetersTraveled).withSize(1, 1).withPosition(2, 1);
    Shuffleboard.getTab("Sensors").add("Right M/S", m_driveBase.rightMetersPerSec).withSize(1, 1).withPosition(3, 1);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton veloShootButton = new JoystickButton(driver, Constants.GamepadIDs.kGamepadButtonShoulderR);
    JoystickButton runIndexer = new JoystickButton(driver, Constants.GamepadIDs.kGamepadButtonShoulderL);

    veloShootButton
        .whileHeld(new VeloShooting(m_shooter, driver.getTriggerAxis(Hand.kRight), driver.getBumper(Hand.kRight)));

    runIndexer.whileHeld(new SimpleFeed(m_indexer, 0.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
   * public Command getAutonomousCommand() { // An ExampleCommand will run in
   * autonomous return null; }
   */
}
