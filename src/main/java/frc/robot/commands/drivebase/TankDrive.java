package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class TankDrive extends CommandBase{

    private final DriveBase m_DriveBase;
    private final DoubleSupplier left, right;


    public TankDrive(DriveBase subsystem, DoubleSupplier left, DoubleSupplier right){
        this.m_DriveBase = subsystem;
        this.left = left;
        this.right = right;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        m_DriveBase.tankDrive(left, right);
    }    

    @Override
    public boolean isFinished() {
        return false;
    }
}