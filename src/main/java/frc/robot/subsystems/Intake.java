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
    
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.RollerId);
    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.PivotId);

    private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);

    private boolean m_hasHomed = false; 

    public Intake() {
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
        rollerMotor.getConfigurator().apply(rollerConfig);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        
        pivotConfig.Slot0.kP = IntakeConstants.intakekp; 
        pivotConfig.Slot0.kV = IntakeConstants.intakekv;
        pivotConfig.Slot0.kG = IntakeConstants.intakekg;
        pivotConfig.Slot0.kS = IntakeConstants.intakeks;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;


        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 10; 
        pivotConfig.MotionMagic.MotionMagicAcceleration = 20;
        pivotConfig.MotionMagic.MotionMagicJerk = 50; 

        pivotConfig.CurrentLimits.SupplyCurrentLimit = 80;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        // 牆壁
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.8; 

        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10.25;  

        pivotMotor.getConfigurator().apply(pivotConfig);    
    }
    // 按start歸零馬達
    // 待修改(自動時要在最深的位置的時候自動歸零)
    public void resetPivotEncoder() {
        pivotMotor.setPosition(0);
        m_hasHomed = true; 
        System.out.println("Intake 狀態：已歸零至物理死點");
    }

    public double getCurrentPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public void setRollerVoltage(double volts) {
        rollerMotor.setControl(rollerVoltageRequest.withOutput(volts));
    }


    public void setPivotPosition(double rotations) {
        pivotMotor.setControl(pivotPositionRequest.withPosition(rotations));
    }

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
        SmartDashboard.putNumber("Intake/Pivot Position", getCurrentPosition());
        SmartDashboard.putNumber("Intake/Pivot Current", getPivotCurrent());
        SmartDashboard.putBoolean("Intake/Has Homed", m_hasHomed);
    }
}