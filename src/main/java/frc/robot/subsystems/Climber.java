package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class  Climber extends SubsystemBase {

    private Spark winchMotor, liftMotor;

    public Climber() {
        winchMotor = new Spark(Constants.Climber.kWinchID);
        liftMotor = new Spark(Constants.Climber.kLiftID);
    }

    @Override
    public void periodic() {

    }

    public void setWinchPower(double power) {
        winchMotor.set(power);
    }

    public void setLiftPower(DoubleSupplier power) {
        liftMotor.set(power.getAsDouble());
    }
}