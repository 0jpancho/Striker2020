/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.commands.climber.RunClimber;
import frc.robot.commands.drivebase.DiffDrive;
import frc.robot.commands.drivebase.RawArcadeDrive;
import frc.robot.commands.shooter.VeloShooting;
import frc.robot.commands.indexer.RunIndexerSimple;
import frc.robot.commands.intake.RunIntake;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.Limelight;

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

  private XboxController m_driver = new XboxController(0);
  private XboxController m_operator = new XboxController(1);

  // Subsystems
  private final Drivebase m_drivebase = new Drivebase();
  private final Intake m_intake = new Intake();
  private final Indexer m_indexer = new Indexer();
  private final Shooter m_shooter = new Shooter();
  private final Climber m_climber = new Climber();

  private final Limelight m_limelight = new Limelight();

  // Commands
  private final DiffDrive m_diffDriveCommand = new DiffDrive(m_drivebase, m_driver);

  //private final RawArcadeDrive m_rawArcadeDrive = new RawArcadeDrive(m_drivebase, driver);

  private final RunIntake m_runIntakeCommand = new RunIntake(m_intake);
  private final RunIndexerSimple m_runIndexerCommand = new RunIndexerSimple(m_indexer, Constants.Indexer.kPower);
  private final VeloShooting m_veloShootingCommand = new VeloShooting(m_shooter, Constants.Shooter.kTestRPM);
  private final RunClimber m_runClimberCommand = new RunClimber(m_climber, m_driver);

  Dashboard m_dashboard;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_dashboard = new Dashboard(m_drivebase, m_shooter);

    // Configure the button bindings
    configureButtonBindings();

    m_limelight.setPipeline(1);

    m_drivebase.setDefaultCommand(m_diffDriveCommand);
    //m_drivebase.setDefaultCommand(m_rawArcadeDrive);

    m_climber.setDefaultCommand(m_runClimberCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


    /**
     * 
     * Driver Bindings 
     * 
     */

    Button A = new JoystickButton(m_driver, 1);
    Button B = new JoystickButton(m_driver, 2);
    Button X = new JoystickButton(m_driver, 3);
    Button Y = new JoystickButton(m_driver, 4);
    
    
    Button LB = new JoystickButton(m_driver, 5); 
    Button RB = new JoystickButton(m_driver, 6); 
    Button Start = new JoystickButton(m_driver, 7);
    Button Select = new JoystickButton(m_driver, 8);
    
    LB.whileHeld(m_runIntakeCommand);
    RB.whileHeld(m_runIndexerCommand);
    
    A.whileHeld(m_veloShootingCommand);
    X.whileHeld(m_runClimberCommand);

    /**
     * 
     * Operator Bindings
     * 
     */


    Button opA = new JoystickButton(m_operator, 1);
    Button opB = new JoystickButton(m_operator, 2);
    Button opX = new JoystickButton(m_operator, 3);
    Button opY = new JoystickButton(m_operator, 4);
    
    
    Button opLB = new JoystickButton(m_operator, 5); 
    Button opRB = new JoystickButton(m_operator, 6); 
    Button opStart = new JoystickButton(m_operator, 7);
    Button opSelect = new JoystickButton(m_operator, 8);
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
