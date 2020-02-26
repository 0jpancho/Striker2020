package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private Spark climberMotor;

    public Climber() {
        climberMotor = new Spark(Constants.Climber.kClimberID);
    }

    @Override
    public void periodic() {

    }

    public void setPower(double power) {
        climberMotor.set(power);
    }
}