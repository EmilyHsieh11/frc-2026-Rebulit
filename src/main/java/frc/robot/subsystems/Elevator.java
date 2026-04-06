package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private final TalonFX leftElevator = new TalonFX(ElevatorConstants.leftElevatorid);

    // 控制請求物件
    private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0);
    private double targetHeight = 0;

    // 歸零狀態旗標
    private boolean isHoming = false;

    public Elevator() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // 基本設定
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 電流限制：吊掛整台機器的生命線
        config.CurrentLimits.StatorCurrentLimit = 80.0; 
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
// 軟體限位：防止齒條脫軌
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = heightToRotations(ElevatorConstants.kMaxHeight);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0; 
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // Motion Magic 平滑化
        config.MotionMagic.MotionMagicCruiseVelocity = 10; 
        config.MotionMagic.MotionMagicAcceleration = 20;
        config.MotionMagic.MotionMagicJerk = 100;

        // PID 與 前饋
        config.Slot0.kP = ElevatorConstants.elevatorkp;
        config.Slot0.kI = ElevatorConstants.elevatorki;
        config.Slot0.kD = ElevatorConstants.elevatorkd;
        config.Slot0.kG = 0.5; // 重力補償，讓它吊著時不會往下滑
        config.Slot0.kV = 0.12;

        leftElevator.getConfigurator().apply(config);

        // 初始化重設 (假設開機就在底部)
        leftElevator.setPosition(0);
    }

    // --- 單位換算工具 ---
    private double heightToRotations(double height) {
        return (height / (2 * Math.PI * ElevatorConstants.drumRadius)) * ElevatorConstants.GearRatio;
    }

    public double getElevatorHeight() {
        double rotations = leftElevator.getPosition().getValueAsDouble();
        return (rotations / ElevatorConstants.GearRatio) * (2 * Math.PI * ElevatorConstants.drumRadius);
    }

    // --- 控制方法 ---
    public void setGoal(double goalHeight) {
        targetHeight = MathUtil.clamp(goalHeight, 0, ElevatorConstants.kMaxHeight);
    }

    public void stop() {
        setGoal(getElevatorHeight());
    }

    public void setManualSpeed(double speed) {
        leftElevator.set(speed);
    }

    // --- 歸零指令邏輯 (給明天同學用的按鈕功能) ---
    public Command elevatorHomingCommand() {
        return Commands.sequence(
            // 1. 切換到歸零模式 (停用 Motion Magic)
            Commands.runOnce(() -> {
                this.isHoming = true;
                System.out.println("Elevator: Starting Homing...");
            }),
// 2. 緩慢往下跑，直到撞到底部 (電流超過 40A)
            Commands.run(() -> leftElevator.setVoltage(-1.5), this) // 往下給 1.5V
                .until(() -> leftElevator.getStatorCurrent().getValueAsDouble() > 40.0)
                .withTimeout(3.0), // 最多執行 3 秒防止意外

            // 3. 歸零完成後的動作
            Commands.runOnce(() -> {
                leftElevator.setVoltage(0);
                leftElevator.setPosition(0); // 將此處設為 0 點
                this.targetHeight = 0;
                this.isHoming = false; // 回復 Motion Magic 控制
                System.out.println("Elevator: Homing Complete!");
            }, this)
        ).withName("ElevatorHoming");
    }

    @Override
    public void periodic() {
        // 如果不在歸零模式，就持續執行 Motion Magic 定位
        if (!isHoming) {
            leftElevator.setControl(m_mmRequest.withPosition(heightToRotations(targetHeight)));
        }

        // 監控數據
        SmartDashboard.putNumber("Elevator/實際高度(m)", getElevatorHeight());
        SmartDashboard.putNumber("Elevator/目標高度(m)", targetHeight);
        SmartDashboard.putNumber("Elevator/馬達電流(A)", leftElevator.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Elevator/歸零中", isHoming);
    }
}
