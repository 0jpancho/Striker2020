/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.teleop;

import frc.robot.subsystems.DriveTrain;

import java.util.Set;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * An example command that uses an example subsystem.
 */
public class DiffDrive implements Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private DriveTrain m_DriveTrain = new DriveTrain();

  private final DifferentialDriveKinematics m_Kinematics = 
    new DifferentialDriveKinematics(0.51);

  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_DriveTrain.getHeading());

  private final DifferentialDrive drive = 
    new DifferentialDrive(m_DriveTrain.leftMaster, m_DriveTrain.rightMaster);
  
  
  public DiffDrive(DriveTrain subsystem) {
    m_DriveTrain = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public Set<Subsystem> getRequirements() {
    // TODO Auto-generated method stub
    return null;
  }

  public void updateOdemetry(){
    //m_odometry.update(m_DriveTrain.getHeading(),, rightDistanceMeters)
  }
}
