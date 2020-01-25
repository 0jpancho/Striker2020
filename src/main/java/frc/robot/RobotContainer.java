/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DiffDrive;
import frc.robot.subsystems.DriveBase;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Joystick driver = new Joystick(0);

  private final DriveBase m_DriveBase = new DriveBase();


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    /*
    m_DriveBase.setDefaultCommand(
      new ArcadeDrive(m_DriveBase, driver::getY, driver::getX)
    );
    */
    
    //m_DriveBase.testMotors(driver.getRawButton(2), driver.getRawButton(4), driver.getRawButton(3), driver.getRawButton(5));
    
    
    double forward = driver.getY() * Constants.DriveConstants.kMaxSpeed;
    double rot = driver.getX() * Constants.DriveConstants.kMaxAngularSpeed;

    m_DriveBase.setDefaultCommand(
      new DiffDrive(m_DriveBase, forward, rot)
    );
    
    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
  */
}
