package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOReal implements ShooterIO{

    private final TalonFX shooterLeft = new TalonFX(21); 
    private final TalonFX shooterRight = new TalonFX(22);

    private final TalonFX feedLeft = new TalonFX(24);
    private final TalonFX feedRight = new TalonFX(23);


    public ShooterIOReal(){

        shooterLeft.setInverted(true);
        feedLeft.setInverted(true);
        shooterRight.setInverted(false);
        feedRight.setInverted(false);
        
    }

    public void updateInputs(ShooterIOInputs inputs) {}

    public void setVoltages(double leftShooterVolts, double rightShooterVolts,
                            double feedVolts) {
                                        shooterLeft.setVoltage(leftShooterVolts);
                                        shooterRight.setVoltage(rightShooterVolts);
                                        feedLeft.setVoltage(feedVolts);
                                        feedRight.setVoltage(feedVolts);

                                    updateRPM();
                                    }
    
    public void setLeftShooterVoltage(double velocityRadPerSec, double ffVolts) {}

    public void setRightShooterVoltage(double velocityRadPerSec, double ffVolts) {}

    public void setLeftFeedVoltage(double velocityRadPerSec, double ffVolts) {}

    public void setRightFeedVoltage(double velocityRadPerSec, double ffVolts) {}

    public void setVelocity(double velocityRadPerSec, double ffVolts) {}

    public void stop() {}

    public void configurePID(double kP, double kI, double kD) {}

    public void updateRPM(){
    SmartDashboard.putNumber("leftShooter rpm", shooterLeft.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("rightShooter rpm",shooterRight.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("feed rpm",feedLeft.getVelocity().getValueAsDouble()*60);
    }


    
}