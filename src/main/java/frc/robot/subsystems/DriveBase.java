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
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivebase extends SubsystemBase {

	// private PowerDistributionPanel pdp = new PowerDistributionPanel();

	public final WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.Drive.kLeftMasterID);

	private final WPI_VictorSPX leftSlave = new WPI_VictorSPX(Constants.Drive.kLeftFollowerID);

	public final WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.Drive.kRightMasterID);

	private final WPI_VictorSPX rightSlave = new WPI_VictorSPX(Constants.Drive.kRightFollowerID);

	private TalonSRXConfiguration masterConfig = new TalonSRXConfiguration();
	private VictorSPXConfiguration slaveConfig = new VictorSPXConfiguration();

	private final AHRS navx;

	private final PIDController m_LPID = new PIDController(Constants.Drive.kDriveGains.kP,
			Constants.Drive.kDriveGains.kI, Constants.Drive.kDriveGains.kD);

	private final PIDController m_RPID = new PIDController(Constants.Drive.kDriveGains.kP,
			Constants.Drive.kDriveGains.kI, Constants.Drive.kDriveGains.kD);

	private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
			Constants.Drive.kTrackWidth);

	private final DifferentialDriveOdometry m_odometry;

	// Gains are for example purposes only - must be determined for your own robot!
	SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1.06, 6.16, 1.43);

	public double leftMetersPerSec;
	public double rightMetersPerSec;

	public double leftMetersTraveled;
	public double rightMetersTraveled;

	private ControlMode mode = ControlMode.PercentOutput;
	private double motorVal;

	public Drivebase() {

		navx = new AHRS(SPI.Port.kMXP);
		navx.enableLogging(false);

		m_odometry = new DifferentialDriveOdometry(getHeadingPose());

		// Reset sensors
		resetHeading();
		resetOdometry();
		// Rest configs back to default - prevents conflicts
		leftMaster.configFactoryDefault();
		leftSlave.configFactoryDefault();

		rightMaster.configFactoryDefault();
		rightSlave.configFactoryDefault();

		// Set current limit
		masterConfig.continuousCurrentLimit = 40;
		masterConfig.peakCurrentDuration = 0;

		masterConfig.nominalOutputForward = 0;
		masterConfig.nominalOutputReverse = 0;
		masterConfig.peakOutputForward = 1;
		masterConfig.peakOutputReverse = -1;

		slaveConfig.nominalOutputForward = 0;
		slaveConfig.nominalOutputReverse = 0;
		slaveConfig.peakOutputForward = 1;
		slaveConfig.peakOutputReverse = -1;

		// Vals set to config to minimize clutter. Expandable if needed
		leftMaster.configAllSettings(masterConfig);
		leftSlave.configAllSettings(slaveConfig);

		rightMaster.configAllSettings(masterConfig);
		rightSlave.configAllSettings(slaveConfig);

		// Set followers to masters
		leftSlave.follow(leftMaster);
		rightSlave.follow(rightMaster);

		// Configure motor inversions/sensor phase
		
		leftMaster.setSensorPhase(true);
		leftMaster.setInverted(false);
		leftSlave.setInverted(InvertType.FollowMaster);

		rightMaster.setSensorPhase(true);
		rightMaster.setInverted(true);
		rightSlave.setInverted(InvertType.FollowMaster);
	

		// Set neutral mode
		setBrakeMode(NeutralMode.Brake);
		
		// Config encoders
		leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
		rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);

		// Set update period to prevent stale data
		leftMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
		leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
		leftMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);

		rightMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
		rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
		rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);

		System.out.println("Drivebase Initialized");
	}

	@Override
	public void periodic() {
		leftMaster.set(mode, motorVal);
		rightMaster.set(mode, motorVal);

		updateOdometry();
	}

	public void configMotors(ControlMode controlMode, double motorVal) {
		this.mode = controlMode;
		this.motorVal = motorVal;
	}

	public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
		final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
		final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

		final double leftOutput = m_LPID.calculate(
				getLVeloTicks() * (10.0 / Constants.Drive.kEncoderResolution) * Constants.Drive.kCircumferenceMeters,
				speeds.leftMetersPerSecond);
		final double rightOutput = m_RPID.calculate(
				getRVeloTicks() * (10.0 / Constants.Drive.kEncoderResolution) * Constants.Drive.kCircumferenceMeters,
				speeds.rightMetersPerSecond);

		leftMaster.setVoltage(leftOutput + leftFeedforward);
		rightMaster.setVoltage(rightOutput + rightFeedforward);
	}

	public void differentialDrive(double forward, double rot) {
		var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(forward, 0.0, rot));
		setSpeeds(wheelSpeeds);
	}

	public void updateOdometry() {
		leftMetersTraveled = getLPosTicks()
				* (Constants.Drive.kCircumferenceMeters / Constants.Drive.kEncoderResolution);
		rightMetersTraveled = getRPosTicks()
				* (Constants.Drive.kCircumferenceMeters / Constants.Drive.kEncoderResolution);
		m_odometry.update(getHeadingPose(), leftMetersTraveled, rightMetersTraveled);
	}

	public Rotation2d getHeadingPose() {
		double angle = navx.getYaw();
		return Rotation2d.fromDegrees(angle);
	}

	public void resetOdometry() {
		resetEncoders();
		resetHeading();
		m_odometry.resetPosition(new Pose2d(), getHeadingPose());
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public float getHeadingDegrees() {
		return navx.getYaw();
	}

	public boolean navxCalibrating() {
		return navx.isCalibrating();
	}

	public boolean navxAlive() {
		return navx.isConnected();
	}

	public void resetHeading() {
		System.out.println("Heading Reset");
		navx.zeroYaw();
	}

	public void resetEncoders() {
		System.out.println("Encoders Reset");
		leftMaster.getSensorCollection().setQuadraturePosition(0, 20);
		rightMaster.getSensorCollection().setQuadraturePosition(0, 20);
	}

	public void setBrakeMode(NeutralMode mode) {
		leftMaster.setNeutralMode(mode);
		leftSlave.setNeutralMode(mode);

		rightMaster.setNeutralMode(mode);
		rightSlave.setNeutralMode(mode);
	}

	public WPI_TalonSRX getLeftMaster() {
		return leftMaster;
	}

	public WPI_TalonSRX getRightMaster() {
		return rightMaster;
	}

	public int getLVeloTicks() {
		return -leftMaster.getSelectedSensorVelocity();
	}

	public int getLPosTicks() {
		return -leftMaster.getSelectedSensorPosition();
	}

	public int getRVeloTicks() {
		return rightMaster.getSelectedSensorVelocity();
	}

	public int getRPosTicks() {
		return rightMaster.getSelectedSensorPosition();
	}

	public double getLeftMetersPerSec() {
		return getLVeloTicks() * (Constants.Drive.kCircumferenceMeters / Constants.Drive.kEncoderResolution);
	}

	public double getLeftMetersTraveled() {
		return getLPosTicks() * (Constants.Drive.kCircumferenceMeters / Constants.Drive.kEncoderResolution);
	}

	public double getRightMetersPerSec() {
		return getRVeloTicks() * (Constants.Drive.kCircumferenceMeters / Constants.Drive.kEncoderResolution);
	}

	public double getRightMetersTraveled() {
		return getRPosTicks() * (Constants.Drive.kCircumferenceMeters / Constants.Drive.kEncoderResolution);
	}
}