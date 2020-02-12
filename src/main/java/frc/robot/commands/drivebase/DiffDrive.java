/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivebase;

import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DiffDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  
  private DriveBase m_DriveBase = new DriveBase();
  
  private DoubleSupplier forward;
  private DoubleSupplier rot;
  
  public DiffDrive(DriveBase subsystem, DoubleSupplier forward, DoubleSupplier rot) {
    m_DriveBase = subsystem;
    this.forward = forward;
    this.rot = rot;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveBase.resetEncoders();
    m_DriveBase.resetHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double inputForward = forward.getAsDouble();
    double inputRot = rot.getAsDouble();

    if (Math.abs(inputForward) < 0.1){
      inputForward = 0;
    } 

    if (Math.abs(inputRot) < 0.1){
      inputRot = 0;
    }

    SmartDashboard.putNumber("Input Forward", forward.getAsDouble());
    SmartDashboard.putNumber("Input Rotation", rot.getAsDouble());

    m_DriveBase.updateOdometry();

    m_DriveBase.differentialDrive(-inputForward * Constants.DriveConstants.kMaxSpeed, 
                                  -inputRot * Constants.DriveConstants.kMaxAngularSpeed);
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
}
