package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private Spark intakeMotor;

    public Intake() {

        intakeMotor = new Spark(Constants.Intake.kIntakeID);

    }

    @Override
    public void periodic() {

    }

    public void setPower(double power) {
        intakeMotor.set(power);
    }
}