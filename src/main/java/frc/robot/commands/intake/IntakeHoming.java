package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeHoming extends Command {
    private final Intake m_intake;
    private int m_detectionCounter = 0;

    public IntakeHoming(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.setPivotVoltage(-0.8); // 緩慢往回縮
        
        if (m_intake.getPivotCurrent() > 10.0) { // 如果電流升高代表撞到底了
            m_detectionCounter++;
        } else {
            m_detectionCounter = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return m_detectionCounter > 5; // 持續 0.1 秒高電流就結束
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        if (!interrupted) {
            m_intake.resetPivotEncoder(); // 設為物理 0 點
        }
    }
}