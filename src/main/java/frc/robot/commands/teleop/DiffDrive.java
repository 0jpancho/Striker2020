/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.teleop;

import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DiffDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  
  private DriveBase m_DriveBase = new DriveBase();

  private final DifferentialDriveKinematics m_kinematics = 
    new DifferentialDriveKinematics(Constants.DriveConstants.kTrackWidth);

  private final DifferentialDriveOdometry m_odometry = 
    new DifferentialDriveOdometry(m_DriveBase.getHeading());

  
  private final PIDController m_LPID = new PIDController(0.1, 0, 0);
  private final PIDController m_RPID = new PIDController(0.1, 0, 0);
  
  double forward = 0;
  double rot = 0;
  
  public DiffDrive(DriveBase subsystem, double forward, double rot) {
    m_DriveBase = subsystem;
    this.forward = forward;
    this.rot = rot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveBase.resetEncoders();
    m_DriveBase.resetHeading();

    SmartDashboard.putData("Left PID Controller", m_LPID);
    SmartDashboard.putData("Right PID Controller", m_RPID);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateOdometry();

    drive(forward, rot);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
    double leftOutput = m_LPID.calculate(m_DriveBase.getLeftVelo(), 
      speeds.leftMetersPerSecond);
    double rightOutput = m_RPID.calculate(m_DriveBase.getRightVelo(),
      speeds.rightMetersPerSecond);

    m_DriveBase.leftMaster.set(ControlMode.PercentOutput, leftOutput);
    m_DriveBase.rightMaster.set(ControlMode.PercentOutput, rightOutput);
  }

  public void drive(double forward, double rot){
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(forward, 0.0, rot));
    setSpeeds(wheelSpeeds);

  }

  public void updateOdometry(){
    m_odometry.update(m_DriveBase.getHeading(), m_DriveBase.getLeftVelo(), m_DriveBase.getRightVelo());
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
