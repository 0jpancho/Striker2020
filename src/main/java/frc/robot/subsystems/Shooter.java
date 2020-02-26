package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Shooter implements Subsystem {

    public final WPI_TalonSRX leftMotor = new WPI_TalonSRX(Constants.Shooter.kLShooterID);
    public final WPI_TalonSRX rightMotor = new WPI_TalonSRX(Constants.Shooter.kRShooterID);

    private TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();

    private ControlMode mode = ControlMode.Disabled;
    private double motorVal;

    public Shooter() {

        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();

        motorConfig.slot0.kP = Constants.Shooter.kGains.kP;
        motorConfig.slot0.kI = Constants.Shooter.kGains.kI;
        motorConfig.slot0.kD = Constants.Shooter.kGains.kD;
        motorConfig.slot0.kF = Constants.Shooter.kGains.kF;

        motorConfig.nominalOutputForward = 0;
        motorConfig.nominalOutputReverse = 0;
        motorConfig.peakOutputForward = 1;
        motorConfig.peakOutputReverse = -1;

        motorConfig.continuousCurrentLimit = 35;
        motorConfig.peakCurrentDuration = 0;

        leftMotor.configAllSettings(motorConfig);
        rightMotor.configAllSettings(motorConfig);

        leftMotor.setSensorPhase(false);
        rightMotor.setSensorPhase(false);

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        leftMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.Shooter.kPIDLoopIdx,
                Constants.Shooter.kTimeoutMs);
        rightMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.Shooter.kPIDLoopIdx,
                Constants.Shooter.kTimeoutMs);

        leftMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 20);

        rightMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        rightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        rightMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
        rightMotor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 20);

        resetEncoders();

        System.out.println("Shooter Initialized");
    }

    @Override
    public void periodic() {

        leftMotor.set(this.mode, this.motorVal);
    }

    public void setMotors(ControlMode mode, double motorVal) {
        leftMotor.set(mode, motorVal);
        rightMotor.set(mode, motorVal);
    }

    public double getMotorVal() {
        return motorVal;
    }

    public void resetEncoders() {
        leftMotor.setSelectedSensorPosition(0, Constants.Drive.kTimeoutMs, Constants.Shooter.kTimeoutMs);
        rightMotor.setSelectedSensorPosition(0, Constants.Drive.kTimeoutMs, Constants.Shooter.kTimeoutMs);
    }

    public void setBrake(boolean isEnabled) {
        leftMotor.setInverted(isEnabled);
        rightMotor.setInverted(isEnabled);
    }

    public WPI_TalonSRX getLeft(){
        return leftMotor;
    }

    public WPI_TalonSRX getRight(){
        return rightMotor;
    }
}