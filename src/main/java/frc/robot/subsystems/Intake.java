package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;;

public class Intake extends SubsystemBase {
    
    // 定義馬達：TalonFX 內建了編碼器感應功能
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.RollerId);
    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.PivotId);

    // 控制請求物件
    private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);

    private boolean m_hasHomed = false; 

    public Intake() {
        // --- Roller (滾輪) 配置 ---
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
        rollerMotor.getConfigurator().apply(rollerConfig);

        // --- Pivot (支架旋轉) 配置 ---
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        
        // PID 與 物理補償參數 (來自你的 Constants)
        pivotConfig.Slot0.kP = IntakeConstants.intakekp; 
        pivotConfig.Slot0.kV = IntakeConstants.intakekv;
        pivotConfig.Slot0.kG = IntakeConstants.intakekg;
        pivotConfig.Slot0.kS = IntakeConstants.intakeks;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Motion Magic 平滑移動設定
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 10; 
        pivotConfig.MotionMagic.MotionMagicAcceleration = 20;
        pivotConfig.MotionMagic.MotionMagicJerk = 50; 

        // 電流限制保護 (防止撞擊時馬達燒毀)
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 80;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // 停止時自動鎖死

        // --- Soft Limit 安全限制 (重要修改) ---
        // 既然推到底設為 0，往下放是正數：
        // Reverse (-0.5)：允許往內縮一點點緩衝，不准硬撞車架
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.8; 
        
        // Forward (11.0)：防止支架掉到地板，請確保此值 > 吸球位 (Lowered)
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10.25;  

        pivotMotor.getConfigurator().apply(pivotConfig);    
    }

    /** 
     * 手動歸零方法：
     * 對準物理零點後按下 Start 鍵觸發。
     */
    public void resetPivotEncoder() {
        pivotMotor.setPosition(0); // 強制馬達內部編碼器歸零
        m_hasHomed = true; 
        System.out.println("Intake 狀態：已歸零至物理死點");
    }

    /** 獲取目前位置（馬達轉動圈數） */
    public double getCurrentPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    /** 設定滾輪電壓 */
    public void setRollerVoltage(double volts) {
        rollerMotor.setControl(rollerVoltageRequest.withOutput(volts));
    }

    /** 使用 Motion Magic 移動到指定位置 */
    public void setPivotPosition(double rotations) {
        pivotMotor.setControl(pivotPositionRequest.withPosition(rotations));
    }

    /** 手動控制 Pivot 電壓 (測試/診斷用) */
    public void setPivotVoltage(double volts) {
        pivotMotor.setControl(pivotVoltageRequest.withOutput(volts));
    }

    public double getPivotCurrent() {
        return pivotMotor.getSupplyCurrent().getValueAsDouble();
    }

    public boolean hasHomed() {
        return m_hasHomed;
    }

    public void stop() {
        rollerMotor.stopMotor();
        pivotMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // 在 Elastic / SmartDashboard 監控數據
        SmartDashboard.putNumber("Intake/Pivot Position", getCurrentPosition());
        SmartDashboard.putNumber("Intake/Pivot Current", getPivotCurrent());
        SmartDashboard.putBoolean("Intake/Has Homed", m_hasHomed);
    }
}