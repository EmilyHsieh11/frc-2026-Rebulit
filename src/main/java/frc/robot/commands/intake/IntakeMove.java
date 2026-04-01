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
        // 在開始時設定一次目標位置即可
        m_intake.setPivotPosition(m_pivotPos);
    }

    @Override
    public void execute() {
        // 滾輪電壓持續輸出（吸球時需要一直轉）
        m_intake.setRollerVoltage(m_rollerVolts);
    }

    @Override
    public boolean isFinished() {
        // 如果你希望「到達位置就結束指令」，可以取消註解下面這行：
        // return Math.abs(m_intake.getCurrentPosition() - m_pivotPos) < 0.1;
        
        // 但通常我們會讓它 return false，直到按鈕放開或被下一個指令取代
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        // 當指令結束或被切換（按下另一個按鈕）時，關閉滾輪
        m_intake.setRollerVoltage(0);
        
        // 注意：這裡不要呼叫 setPivotPosition(0)，否則支架會彈回去
    }
}