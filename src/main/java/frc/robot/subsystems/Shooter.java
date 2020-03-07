package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.util.Units;
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

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        leftMotor.setSensorPhase(false);
        rightMotor.setSensorPhase(false);

        leftMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.Shooter.kPIDLoopIdx,
                Constants.Shooter.kTimeoutMs);
        rightMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.Shooter.kPIDLoopIdx,
                Constants.Shooter.kTimeoutMs);

        leftMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
        leftMotor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 20);

        rightMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        rightMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
        rightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        rightMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
        rightMotor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 20);

        resetEncoders();

        System.out.println("Shooter Initialized");
    }

    @Override
    public void periodic() {
        
    }
    
    public double getMotorVal() {
        return motorVal;
    }

    public void resetEncoders() {
        leftMotor.getSensorCollection().setQuadraturePosition(0, Constants.Drive.kTimeoutMs);
        rightMotor.getSensorCollection().setQuadraturePosition(0, Constants.Drive.kTimeoutMs);
    }

    public void setBrake(NeutralMode mode) {
        leftMotor.setNeutralMode(mode);
        rightMotor.setNeutralMode(mode);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public WPI_TalonSRX getLeftMotor() {
        return leftMotor;
    }

    public WPI_TalonSRX getRightMotor() {
        return rightMotor;
    }

    public int getLeftVeloTicks() {
        return leftMotor.getSelectedSensorVelocity();
    }

    public int getRightVeloTicks() {
        return rightMotor.getSelectedSensorVelocity();
    }

    public double getDistToTarget(double targetY) {
        return (Constants.Limelight.kTargetHeight - Constants.Limelight.kAngleDegrees)
                / Math.tan(Units.degreesToRadians((Constants.Limelight.kAngleDegrees) - targetY));
    }
}