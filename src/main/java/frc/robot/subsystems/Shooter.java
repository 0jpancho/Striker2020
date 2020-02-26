package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.vision.Limelight;

public class Shooter implements Subsystem {

    public final WPI_TalonSRX left = new WPI_TalonSRX(Constants.Shooter.kLShooterID);
    public final WPI_TalonSRX right = new WPI_TalonSRX(Constants.Shooter.kRShooterID);

    private TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();

    private Limelight limelight = new Limelight();

    private ControlMode mode = ControlMode.Disabled;
    private double motorVal;

    public Shooter() {

        left.configFactoryDefault();
        right.configFactoryDefault();

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

        left.configAllSettings(motorConfig);
        right.configAllSettings(motorConfig);

        left.setSensorPhase(false);
        right.setSensorPhase(false);

        left.setInverted(false);
        right.setInverted(true);

        left.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.Shooter.kPIDLoopIdx,
                Constants.Shooter.kTimeoutMs);
        right.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.Shooter.kPIDLoopIdx,
                Constants.Shooter.kTimeoutMs);

        left.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        left.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        left.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
        left.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 20);

        right.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        right.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        right.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
        right.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 20);

        resetEncoders();

        System.out.println("Shooter Initialized");
    }

    @Override
    public void periodic() {

        left.set(this.mode, this.motorVal);
    }

    public void setMotors(ControlMode mode, double motorVal) {
        left.set(mode, motorVal);
        right.set(mode, motorVal);
    }

    public double getMotorVal() {
        return motorVal;
    }

    public void resetEncoders() {
        left.getSensorCollection().setQuadraturePosition(0, Constants.Shooter.kTimeoutMs);
        right.getSensorCollection().setQuadraturePosition(0, Constants.Shooter.kTimeoutMs);
    }

    public void setBrake(boolean isEnabled) {
        left.setInverted(isEnabled);
        right.setInverted(isEnabled);
    }

    public WPI_TalonSRX getLeft(){
        return left;
    }

    public WPI_TalonSRX getRight(){
        return right;
    }

    public Limelight getLimelight() {
        return this.limelight;
    }
}