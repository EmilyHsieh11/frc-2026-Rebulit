package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeMove extends Command {
    private final Intake m_intake;
    private final double m_pivotPos; // 這裡傳入 0
    private final double m_rollerVolts;
    private final Shooter m_indexer;
    

    public IntakeMove(Intake intake,double pivotPos, double rollerVolts, Shooter indexer) {
        m_intake = intake;
        m_pivotPos = pivotPos;
        m_rollerVolts = rollerVolts;
        m_indexer = indexer;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_intake.setPivotPosition(m_pivotPos);
        m_intake.setRollerVoltage(m_rollerVolts);
        m_indexer.runIndexerMotor(-1.5);
    }
    
    @Override
    public void end(boolean interrupted) { 
        m_intake.setRollerVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
