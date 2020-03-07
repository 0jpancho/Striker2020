/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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
import edu.wpi.first.wpilibj.SlewRateLimiter;
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

	private final PIDController m_LPID = new PIDController(Constants.Drive.kVeloGains.kP, Constants.Drive.kVeloGains.kI,
			Constants.Drive.kVeloGains.kD);

	private final PIDController m_RPID = new PIDController(Constants.Drive.kVeloGains.kP, Constants.Drive.kVeloGains.kI,
			Constants.Drive.kVeloGains.kD);

	private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
			Constants.Drive.kTrackWidth);

	private final DifferentialDriveOdometry m_odometry;

	private SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(3);
	private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

	SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1.06, 6.16, 1.43);

	public double leftMetersPerSec;
	public double rightMetersPerSec;

	public double leftMetersTraveled;
	public double rightMetersTraveled;
	
	public Drivebase() {

		navx = new AHRS(SPI.Port.kMXP);
		navx.enableLogging(true);

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
		masterConfig.continuousCurrentLimit = 30;
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
		leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

		rightMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
		rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
		rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
		rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

		System.out.println("Drivebase Initialized");

		m_LPID.reset();
		m_RPID.reset();
	}

	@Override
	public void periodic() {
		updateOdometry();
	}

	public void arcadeDrive(double forward, double rot) {
		leftMaster.set(ControlMode.PercentOutput, -forward + rot);
		rightMaster.set(ControlMode.PercentOutput, -forward - rot);
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

	/*
	public void differentialDrive(double forward, double rot) {
		var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(forward, 0.0, rot));
		setSpeeds(wheelSpeeds);
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
	*/

	public void arcadeDriveVelo(double forward, double rot, boolean useSquares) {
		var xSpeed = forward;
		var zRot = rot;

		if (useSquares) {
			xSpeed *= Math.abs(xSpeed);
			zRot *= Math.abs(zRot);
		}

		xSpeed = m_forwardLimiter.calculate(forward);
		zRot = m_rotLimiter.calculate(rot);

		xSpeed *= Constants.Drive.kRawMaxSpeed;
		zRot *= Constants.Drive.kAdjustedAngularSpeed;

		var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, zRot));

		var leftFeedForwardVolts = m_feedforward.calculate(wheelSpeeds.leftMetersPerSecond);
		var rightFeedForwardVolts = m_feedforward.calculate(wheelSpeeds.rightMetersPerSecond);

		leftMaster.set(ControlMode.Velocity, metersPerSecToEdgesPerDecisec(wheelSpeeds.leftMetersPerSecond),
				DemandType.ArbitraryFeedForward, leftFeedForwardVolts / 12);
		rightMaster.set(ControlMode.Velocity, metersPerSecToEdgesPerDecisec(wheelSpeeds.rightMetersPerSecond),
				DemandType.ArbitraryFeedForward, rightFeedForwardVolts / 12);
	}

	public void updateOdometry() {
		leftMetersTraveled = getLPosTicks()
				* (Constants.Drive.kCircumferenceMeters / Constants.Drive.kEncoderResolution);
		rightMetersTraveled = getRPosTicks()
				* (Constants.Drive.kCircumferenceMeters / Constants.Drive.kEncoderResolution);
		m_odometry.update(getHeadingPose(), leftMetersTraveled, rightMetersTraveled);
	}

	public Rotation2d getHeadingPose() {
		return Rotation2d.fromDegrees(-getHeadingDegrees());
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
		return leftMaster.getSelectedSensorVelocity();
	}

	public int getLPosTicks() {
		return leftMaster.getSelectedSensorPosition();
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

	/**
	 * Converts from encoder edges to meters.
	 * 
	 * @param steps encoder edges to convert
	 * @return meters
	 */
	public static double edgesToMeters(int steps) {
		return (Constants.Drive.kCircumferenceMeters / Constants.Drive.kEncoderResolution) * steps;
	}

	/**
	 * Converts from encoder edges per 100 milliseconds to meters per second.
	 * 
	 * @param stepsPerDecisec edges per decisecond
	 * @return meters per second
	 */
	public static double edgesPerDecisecToMetersPerSec(int stepsPerDecisec) {
		return edgesToMeters(stepsPerDecisec * 10);
	}

	/**
	 * Converts from meters to encoder edges.
	 * 
	 * @param meters meters
	 * @return encoder edges
	 */
	public static double metersToEdges(double meters) {
		return (meters / Constants.Drive.kCircumferenceMeters) * Constants.Drive.kEncoderResolution;
	}

	/**
	 * Converts from meters per second to encoder edges per 100 milliseconds.
	 * 
	 * @param metersPerSec meters per second
	 * @return encoder edges per decisecond
	 */
	public static double metersPerSecToEdgesPerDecisec(double metersPerSec) {
		return metersToEdges(metersPerSec) * .1d;
	}

	public void stop() {
		leftMaster.set(0);
		rightMaster.set(0);
		m_forwardLimiter.reset(0);
		m_rotLimiter.reset(0);
	}
}
