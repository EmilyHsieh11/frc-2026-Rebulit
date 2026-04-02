package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeMove extends Command {
    private final Intake m_intake;
    private final double m_pivotPos;
    private final double m_rollerVolts;

    public IntakeMove(Intake intake, double pivotPos, double rollerVolts) {
        m_intake = intake;
        m_pivotPos = pivotPos;
        m_rollerVolts = rollerVolts;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        
        m_intake.setPivotPosition(m_pivotPos);
    }

    @Override
    public void execute() {
        
        m_intake.setRollerVoltage(m_rollerVolts);
    }

    @Override
    public boolean isFinished() {

        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        
        m_intake.setRollerVoltage(0);
    }
}