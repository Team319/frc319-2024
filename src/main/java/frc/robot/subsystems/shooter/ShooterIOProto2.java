package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOProto2 implements ShooterIO{

    private final TalonFX shooterTop = new TalonFX(31); 
    private final TalonFX shooterBottom = new TalonFX(32);

    private final TalonFX feedLeft = new TalonFX(33);
    private final TalonFX feedRight = new TalonFX(34);


    public ShooterIOProto2(){

        shooterTop.setInverted(true);
        feedLeft.setInverted(true);
        shooterBottom.setInverted(false);
        feedRight.setInverted(false);
        
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {}

    @Override
    public void setFeedVoltage(double ffVolts) {
        feedLeft.setVoltage(ffVolts);
        feedRight.setVoltage(ffVolts);
        updateRPM();
        System.err.println("ff voltage: " + ffVolts);
    }

    @Override
    public void setVoltages(double leftShooterVolts, double rightShooterVolts, double feedVolts) {
        shooterTop.setVoltage(leftShooterVolts);
        shooterBottom.setVoltage(rightShooterVolts);
        setFeedVoltage(feedVolts);
        updateRPM();
    }

    @Override
    public void setShooterVelocity(double velocityRadPerSec, double ffVolts) {
        shooterTop.setControl(
            new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec), 0.0, true, ffVolts, 0, false, false, false));
        shooterBottom.setControl(
            new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec), 0.0, true, ffVolts, 0, false, false, false));
        updateRPM();
    }

    @Override
    public void setFeedVelocity(double velocityRadPerSec, double ffVolts) {
        feedLeft.setControl(
            new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec), 0.0, true, ffVolts, 0, false, false, false));
        feedRight.setControl(
            new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec), 0.0, true, ffVolts, 0, false, false, false));
        updateRPM();
    }

    @Override
    public void stop() {
        shooterTop.stopMotor();
        shooterBottom.stopMotor();
        feedLeft.stopMotor();
        feedRight.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) { 
        var shooterConfig = new Slot0Configs();
        shooterConfig.kP = kP;
        shooterConfig.kI = kI;
        shooterConfig.kD = kD;
        
        shooterTop.getConfigurator().apply(shooterConfig);
        shooterBottom.getConfigurator().apply(shooterConfig);
        feedLeft.getConfigurator().apply(shooterConfig);
        feedRight.getConfigurator().apply(shooterConfig);
    }

    public void updateRPM(){
    SmartDashboard.putNumber("leftShooter rpm",shooterTop.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("rightShooter rpm",shooterBottom.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("feed rpm",feedLeft.getVelocity().getValueAsDouble());
    }


    
}
