package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private Spark climberMotor;
    private Talon liftMotor;

    public Climber() {
        climberMotor = new Spark(Constants.Climber.kClimberID);
        liftMotor = new Talon(Constants.Climber.kLiftID);
    }

    @Override
    public void periodic() {

    }

    public void setClimberPower(double power) {
        climberMotor.set(power);
    }

    public void setLiftPower(double power) {
        liftMotor.set(power);
    }
}