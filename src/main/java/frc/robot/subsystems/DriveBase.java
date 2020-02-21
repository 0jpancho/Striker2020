/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveBase extends SubsystemBase {

  PowerDistributionPanel pdp = new PowerDistributionPanel();

  public final WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.DriveConstants.kLeftMasterID);

  private final WPI_TalonSRX leftSlave = new WPI_TalonSRX(Constants.DriveConstants.kLeftFollowerID);

  public final WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.DriveConstants.kRightMasterID);

  private final WPI_TalonSRX rightSlave = new WPI_TalonSRX(Constants.DriveConstants.kRightFollowerID);

  private TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();

  private final AHRS navx;

  private final PIDController m_LPID = new PIDController(Constants.DriveConstants.kDriveGains.kP, 
                                                         Constants.DriveConstants.kDriveGains.kI,
                                                         Constants.DriveConstants.kDriveGains.kD);

  private final PIDController m_RPID = new PIDController(Constants.DriveConstants.kDriveGains.kP, 
                                                         Constants.DriveConstants.kDriveGains.kI,
                                                         Constants.DriveConstants.kDriveGains.kD);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      Constants.DriveConstants.kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1);

  private double leftMetersPerSec;
  private double rightMetersPerSec;

  private double leftMetersTraveled;
  private double rightMetersTraveled;

  private ControlMode controlMode = ControlMode.PercentOutput;
  private double motorVal;

  public DriveBase() {
    // Rest configs back to default - prevents conflicts
    leftMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();

    rightMaster.configFactoryDefault();
    rightSlave.configFactoryDefault();

    // Set current limit
    motorConfig.continuousCurrentLimit = 40;
    motorConfig.peakCurrentDuration = 0; 

    motorConfig.nominalOutputForward = 0;
    motorConfig.nominalOutputReverse = 0;
    motorConfig.peakOutputForward = 1;
    motorConfig.peakOutputReverse = -1;

    //Vals set to config to minimize clutter. Expandable if needed
    leftMaster.configAllSettings(motorConfig);
    leftSlave.configAllSettings(motorConfig);

    rightMaster.configAllSettings(motorConfig);
    rightSlave.configAllSettings(motorConfig);

    // Set followers to masters
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    // Configure motor inversions/sensor phase
    leftMaster.setInverted(false);
    leftSlave.setInverted(InvertType.FollowMaster);
    leftMaster.setSensorPhase(false);

    rightMaster.setInverted(true);
    rightSlave.setInverted(InvertType.FollowMaster);
    rightMaster.setSensorPhase(true);

    // Set neutral mode
    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);

    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);

    // Config encoders
    leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
    rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);

    //Set update period to prevent stale data
    leftMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);

    rightMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);

    navx = new AHRS(SPI.Port.kMXP);

    m_odometry = new DifferentialDriveOdometry(getHeadingPose());

    navx.enableLogging(false);

    // Reset sensors
    resetEncoders();
    resetHeading();
  }

  @Override
  public void periodic() {

    leftMaster.set(controlMode, motorVal);
    rightMaster.set(controlMode, motorVal);
      
    leftMetersPerSec = getLVeloTicks() * (Constants.DriveConstants.kWheelCircumferenceMeters / 
                                          Constants.DriveConstants.kEncoderResolution);

    rightMetersPerSec = getRVeloTicks() * (Constants.DriveConstants.kWheelCircumferenceMeters / 
                                           Constants.DriveConstants.kEncoderResolution);

  
    SmartDashboard.putNumber("Left Enc Velo FPS", leftMetersPerSec);
    SmartDashboard.putNumber("Right Enc Velo FPS", rightMetersPerSec);
    
    SmartDashboard.putNumber("Left Distance", leftMetersTraveled);
    SmartDashboard.putNumber("Right Distance", rightMetersTraveled);

    SmartDashboard.putNumber("Heading", navx.getYaw());
    SmartDashboard.putBoolean("NavX Calibrating", navx.isCalibrating());
    SmartDashboard.putBoolean("NavX Alive", navx.isConnected());

    SmartDashboard.putData(m_LPID);
    SmartDashboard.putData(m_RPID);

    SmartDashboard.putNumber("Robot Current Draw", pdp.getTotalCurrent());
  }

  public void configMotors(ControlMode controlMode, double motorVal){
    this.controlMode = controlMode;
    this.motorVal = motorVal;
  }

  public void arcadeDrive(DoubleSupplier forward, DoubleSupplier rotation){
      leftMaster.set(ControlMode.PercentOutput, -forward.getAsDouble() + rotation.getAsDouble());
      rightMaster.set(ControlMode.PercentOutput, -forward.getAsDouble() - rotation.getAsDouble());
  }

  public void tankDrive(DoubleSupplier left, DoubleSupplier right){
    leftMaster.set(ControlMode.PercentOutput, -left.getAsDouble());
    rightMaster.set(ControlMode.PercentOutput, -right.getAsDouble());
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = m_LPID.calculate(getLVeloTicks(),
        speeds.leftMetersPerSecond);
    final double rightOutput = m_RPID.calculate(getRVeloTicks(),
        speeds.rightMetersPerSecond);
    leftMaster.setVoltage(leftOutput + leftFeedforward);
    rightMaster.setVoltage(rightOutput + rightFeedforward);
  }

  public void differentialDrive(double forward, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(forward, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  public Rotation2d getHeadingPose() {
    double angle = navx.getYaw();
    return Rotation2d.fromDegrees(angle);
  }

  public double getHeadingDegrees(){
    return navx.getYaw();
  }

  public void resetHeading(){
    navx.zeroYaw();
  }

  public void updateOdometry() {
    leftMetersTraveled = getLPosTicks() * (Constants.DriveConstants.kWheelCircumferenceMeters / Constants.DriveConstants.kEncoderResolution);
    rightMetersTraveled = getRPosTicks() * (Constants.DriveConstants.kWheelCircumferenceMeters / Constants.DriveConstants.kEncoderResolution);
    m_odometry.update(getHeadingPose(), leftMetersTraveled, rightMetersTraveled);
  }

  public void resetEncoders(){
    leftMaster.getSensorCollection().setQuadraturePosition(0, 20);
    rightMaster.getSensorCollection().setQuadraturePosition(0, 20);
  }

  public WPI_TalonSRX getLeftMaster() {
    return leftMaster;
  }

  public WPI_TalonSRX getRightMaster(){
    return rightMaster;
  }

  public int getLVeloTicks(){
    return -leftMaster.getSensorCollection().getQuadratureVelocity();
  }

  public int getLPosTicks(){
    return -leftMaster.getSensorCollection().getQuadraturePosition();
  }

  public int getRVeloTicks(){
    return rightMaster.getSensorCollection().getQuadratureVelocity();
  }

  public int getRPosTicks(){
    return rightMaster.getSensorCollection().getQuadraturePosition();
  }
}