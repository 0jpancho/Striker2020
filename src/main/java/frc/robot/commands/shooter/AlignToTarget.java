package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.vision.Limelight;

public class AlignToTarget extends CommandBase {

    private NetworkTable m_table;
    private String m_tableName;

    private Drivebase m_drive;
    private Limelight m_limelight;
    
    double kPAim = -2f;
    double kPDist = -0.1f;
    double kMinAimPower = 0.05f;

    public AlignToTarget(Drivebase drive, Limelight limelight) {
        m_drive = drive;
        m_limelight = limelight;

        m_limelight.setPipeline(0);

        m_tableName = "limelight";
        m_table = NetworkTableInstance.getDefault().getTable(m_tableName);

        m_table.getEntry("pipeline").setValue(0);

        

        addRequirements(drive, limelight);
    }

    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        NetworkTableEntry tx = m_table.getEntry("tx");
        double x = tx.getDouble(0.0);

        NetworkTableEntry ty = m_table.getEntry("ty");
        double y = ty.getDouble(0.0);

        SmartDashboard.putNumber("Limelight X", x);
        SmartDashboard.putNumber("Limelight Y", y);


        double headingError = x;
        double distError = y;

        System.out.println("Heading Error" + headingError);

        double rotCorrect = 0.0f;

        if (headingError > 1) {
            rotCorrect = kPAim * headingError - kMinAimPower;
        }

        else if (headingError < 1) {
            rotCorrect = kPAim * headingError + kMinAimPower;
        }

        double distCorrect = kPDist* distError;

        double leftPow =+ rotCorrect + distCorrect;
        double rightPow =- rotCorrect + distCorrect;

        m_drive.getLeftMaster().set(ControlMode.PercentOutput, leftPow);
        m_drive.getRightMaster().set(ControlMode.PercentOutput, rightPow);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.configMotors(ControlMode.PercentOutput, 0);
        //m_limelight.setPipeline(1);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}