package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    // 定義三顆馬達：調整角度(Hood)、射球(Shooter)、進彈(Indexer)
    private final TalonFX hoodMotor = new TalonFX(ShooterConstants.khoodMotorId);
    private final TalonFX shooterMotor = new TalonFX(ShooterConstants.kshooterMotorId);
    private final TalonFX indexerMotor = new TalonFX(ShooterConstants.kindexerMotorId);

    // 【新東西】Motion Magic 的「指令包裹」。我們把目標位置塞進去，馬達就會自己跑。
    private final MotionMagicVoltage m_hoodRequest = new MotionMagicVoltage(0);
    
    private double targetAngle = 0; // 紀錄我們現在「想要」的角度
    private boolean isHoming = false; // 紀錄現在是不是正在手動歸零（如果是，自動控制要讓開）

    public Shooter() {
        /** 1. 調整角度馬達 (Hood) 的設定 **/
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // 沒電時要鎖死，不然支架會掉下來

        // --- Motion Magic 的核心參數 (就像是幫馬達寫導航設定) ---
        hoodConfig.Slot0.kP = 25.0; // 力量系數：數值越大，馬達追目標追得越兇（通常 10-50 之間）
        hoodConfig.Slot0.kG = ShooterConstants.kG; // 重力補償：用來抵銷支架重量，讓它不會因為重力掉下去
        hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // 補償方式：這告訴馬達這是一個「手臂」結構

        // 設定移動速度與加速度 (這決定了動作「絲滑」的程度)
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 10; // 最快每秒跑 10 圈
        hoodConfig.MotionMagic.MotionMagicAcceleration = 20;   // 加速到最高速需要多久
        hoodConfig.MotionMagic.MotionMagicJerk = 100;         // 讓加速過程變圓滑，不會抖一下

        hoodMotor.getConfigurator().apply(hoodConfig);

        /** 2. 射球與進彈馬達設定 (保持 Coast 模式，射完球讓它慢慢停) **/
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.CurrentLimits.StatorCurrentLimit = 70.0;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterMotor.getConfigurator().apply(shooterConfig);

        indexerMotor.getConfigurator().apply(new TalonFXConfiguration());
    }

    // 將馬達讀到的「圈數」換算成我們看得懂的「角度」
    public double getHoodMotorAngleDegrees() {
        double rotations = hoodMotor.getPosition().getValueAsDouble();
        return rotations * ShooterConstants.gearRatio * 360;
    }

    // 將我們設定的「角度」換算回馬達要跑的「圈數」
    private double degreesToRotations(double degrees) {
        return degrees / (ShooterConstants.gearRatio * 360);
    }

    // 讓外部 (如 Command) 來設定現在要幾度
    public void setTargetAngle(double degrees) {
        // 使用 Clamp 限制範圍，防止程式寫錯讓馬達轉過頭撞壞
        targetAngle = MathUtil.clamp(degrees, -1, ShooterConstants.hoodBiggestAngle);
    }

    public void stopHoodMotor() {
        hoodMotor.stopMotor();
        // 為了防止 periodic 裡面又把馬達拉回舊的目標，停下時把目標設為「現在這裡」
        targetAngle = getHoodMotorAngleDegrees(); 
    }

    // 把目前的物理位置強行定義為 0 度（歸零）
    public void resetShooterAngle() {
        hoodMotor.setPosition(0);
        targetAngle = 0;
    }

    // 設定射球馬達的電壓 (0-12V)
    public void shooterSetVoltage(double volts) {
        shooterMotor.setVoltage(volts);
    }

    // 一鍵發射：同時開射球和進彈馬達
    public void shootshoot(boolean state) {
        if (state) {
            shooterMotor.setVoltage(ShooterConstants.shooterSpeed);
            indexerMotor.setVoltage(ShooterConstants.indexerSpeed);
        } else {
            shooterMotor.setVoltage(0);
            indexerMotor.setVoltage(0);
        }
    }

    // 自動瞄準邏輯：根據距離查表決定角度
    public void autoAim(double distance) {
        if (distance < 0.5) {
            targetAngle = ShooterConstants.closestAngle;
        } else {
            // 從 Constants 裡面的表格拿對應的角度
            Double val = ShooterConstants.table.get(distance);
            if (val != null) targetAngle = val;
        }
    }

    // 檢查有沒有準：誤差小於 1 度就算對準了
    public boolean isReadyToShoot() {
        return Math.abs(getHoodMotorAngleDegrees() - targetAngle) < 1.0;
    }
    /** 讓外部 Command 可以讀取目前的目標角度 **/
    public double getTargetAngle() {
        return targetAngle;
    }

    /** 停止射球馬達的方法 **/
    public void stopShooterMotor() {
        shooterMotor.stopMotor();
    }

    // 當你要手動推馬達歸零時，要呼叫這個把自動控制暫時關掉
    public void setHomingMode(boolean homing) {
        this.isHoming = homing;
    }
    /** 
     * 手動驅動 Hood 馬達（歸零專用）
     * @param volts 給予的電壓，正值往上，負值往下
     */
    public void driveManual(double volts) {
        hoodMotor.setVoltage(volts);
    }

    /** 
     * 檢查馬達是否撞到底部（堵轉電流偵測）
     * 當馬達撞到硬擋塊轉不動時，電流會大幅上升
     */
    public boolean isStalled() {
        // 取得馬達的定子電流 (Stator Current)
        // 如果電流超過 20A，代表馬達撞到底了，轉不動了
        return hoodMotor.getStatorCurrent().getValueAsDouble() > 20.0;
    }

    @Override
    public void periodic() {
        // 在儀表板顯示數值，方便除錯
        SmartDashboard.putNumber("Shooter/實際角度", getHoodMotorAngleDegrees());
        SmartDashboard.putNumber("Shooter/目標角度", targetAngle);
        SmartDashboard.putBoolean("Shooter/準不準", isReadyToShoot());

        // 【最關鍵的邏輯】
        // 只要現在不是在歸零，我們就每 20ms 告訴馬達：「去目標圈數！」
        if (!isHoming) {
            double targetRotations = degreesToRotations(targetAngle);
            // 這一行就是啟動馬達內部的 Motion Magic 控制
            hoodMotor.setControl(m_hoodRequest.withPosition(targetRotations));
        }
    }
}