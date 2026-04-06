package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorMove extends Command {
    private final Elevator m_elevator;
    private final double m_goalHeight;

    public ElevatorMove(Elevator elevator, double goalHeight) {
        this.m_elevator = elevator;
        this.m_goalHeight = goalHeight;

        // 宣告需要使用這個子系統，避免多個指令同時搶奪電梯控制權
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        // 告訴電梯：現在要去這個高度
        m_elevator.setGoal(m_goalHeight);
    }

    @Override
    public void execute() {
        // 這裡不需要寫東西！
        // 因為 Elevator.java 的 periodic() 每一幀都會自動讀取 targetHeight 並執行 Motion Magic
    }

    @Override
    public boolean isFinished() {
        // 當電梯位置與目標誤差小於一定範圍時（例如 1 公分），指令結束
        // 這裡建議在 Elevator.java 寫一個 isAtGoal()，或者直接在此判斷：
        double error = Math.abs(m_elevator.getElevatorHeight() - m_goalHeight);
        return error < 0.015; // 假設單位是公尺，小於 1cm 算到達
    }

    @Override
    public void end(boolean interrupted) {
        // 指令結束時，通常不需要呼叫 stop()
        // 因為我們希望電梯到達後，PID 能繼續鎖在原地撐住機器人
        // 如果呼叫 stop() 且裡面是 set(0)，電梯會因為重力直接滑下來！
    }
}