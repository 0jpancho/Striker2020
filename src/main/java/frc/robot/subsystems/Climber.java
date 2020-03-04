package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class  Climber extends SubsystemBase {

    private Spark winchMotor;
    private Talon liftMotor;

    public Climber() {
        winchMotor = new Spark(Constants.Winch.kWinchID);
        liftMotor = new Talon(Constants.Winch.kLiftID);
    }

    @Override
    public void periodic() {

    }

    public void setWinchPower(double power) {
        winchMotor.set(power);
    }

    public void setLiftPower(double power) {
        liftMotor.set(power);
    }
}