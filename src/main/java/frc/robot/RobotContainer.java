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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivebase.DiffDrive;
import frc.robot.commands.shooter.OpenLoopShooting;
import frc.robot.commands.shooter.VeloShooting;
import frc.robot.commands.indexer.SimpleFeed;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.Limelight;
import frc.robot.vision.ControlMode.LedMode;

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

  //Subsystems
  private final Drivebase m_drivebase = new Drivebase();
  private final Shooter m_shooter = new Shooter();
  private final Indexer m_indexer = new Indexer();
  private final Limelight m_limelight = new Limelight();

  //Commands


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_limelight.setPipeline(1);

    DoubleSupplier forward = () -> m_forwardLimiter.calculate(driver.getY(Hand.kLeft));
    DoubleSupplier rot = () -> m_rotLimiter.calculate(driver.getX(Hand.kRight));

    m_drivebase.setDefaultCommand(new DiffDrive(m_drivebase, forward, rot));

    SmartDashboard.putNumber("Heading", m_drivebase.getHeadingDegrees());
    SmartDashboard.putNumber("Heading", m_drivebase.getHeadingDegrees());
    SmartDashboard.putBoolean("Navx Calibrating", m_drivebase.navxAlive());
    SmartDashboard.putBoolean("NavX Alive", m_drivebase.navxAlive());
    SmartDashboard.putNumber("Left MTraveled", m_drivebase.leftMetersTraveled);
    SmartDashboard.putNumber("Left MPerSec", m_drivebase.leftMetersPerSec);
    SmartDashboard.putNumber("Right MTraveled", m_drivebase.rightMetersTraveled);
    SmartDashboard.putNumber("Right MPerSec", m_drivebase.rightMetersPerSec);
    SmartDashboard.putNumber("ShooterL Velo", m_shooter.leftMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("ShooterR Velo", m_shooter.rightMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter Setpoint", m_shooter.getMotorVal());
    
    SmartDashboard.updateValues();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  
    Button A = new JoystickButton(driver, 1);
    Button B = new JoystickButton(driver, 2);

    /*
    Button X = new JoystickButton(driver, 3);
    Button Y = new JoystickButton(driver, 4);
    Button LB = new JoystickButton(driver, 5);
    Button RB = new JoystickButton(driver, 6);
    Button Start = new JoystickButton(driver, 7);
    Button Select = new JoystickButton(driver, 8);
    */

     A.whileHeld(new OpenLoopShooting(m_shooter, 1));
     //A.whileHeld(new VeloShooting(m_shooter, 200));
     B.whileHeld(new SimpleFeed(m_indexer, 0.75));
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
