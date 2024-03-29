package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

    private FlywheelSim leftShooterSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.5, 0.004);
    private FlywheelSim rightShooterSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.5, 0.004);

    private DCMotorSim leftFeedSim = new DCMotorSim(DCMotor.getFalcon500(1), 1.5, 0.004);
    private DCMotorSim rightFeedSim = new DCMotorSim(DCMotor.getFalcon500(1), 1.5, 0.004);

    private PIDController pid = new PIDController(0.0, 0.0, 0.0);

    private boolean closedLoop = false;
    private double ffVolts = 0.0;
    private double leftShooterAppliedVolts = 0.0;
    private double rightShooterAppliedVolts = 0.0;
    //private double leftFeedAppliedVolts = 0.0;
    private double rightFeedAppliedVolts = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        if (closedLoop) {
            leftShooterAppliedVolts =
                    MathUtil.clamp(pid.calculate(leftShooterSim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
            leftShooterSim.setInputVoltage(leftShooterAppliedVolts);

            rightFeedAppliedVolts =
                    MathUtil.clamp(pid.calculate(rightShooterSim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
            rightShooterSim.setInputVoltage(rightFeedAppliedVolts);
        }

        leftShooterSim.update(0.02);
        rightShooterSim.update(0.02);
        
        leftFeedSim.update(0.02);
        rightFeedSim.update(0.02);

        inputs.leftFlywheelVelocityRadPerSec = leftShooterSim.getAngularVelocityRadPerSec();
        inputs.rightFlywheelVelocityRadPerSec = rightShooterSim.getAngularVelocityRadPerSec();
        
        inputs.leftShooterAppliedVolts = leftShooterAppliedVolts;
        inputs.rightShooterAppliedVolts = rightShooterAppliedVolts;
        //inputs.leftFeedAppliedVolts = leftFeedAppliedVolts;
        //inputs.rightFeedAppliedVolts = rightFeedAppliedVolts;
        
        //inputs.leftFlywheelCurrentAmps = new double[] {leftShooterSim.getCurrentDrawAmps()};
        //inputs.rightFlywheelCurrentAmps = new double[] {rightShooterSim.getCurrentDrawAmps()};

        //inputs.leftFeedCurrentAmps = new double[] {leftFeedSim.getCurrentDrawAmps()};
        //inputs.rightFeedCurrentAmps = new double[] {rightFeedSim.getCurrentDrawAmps()};
    }

    @Override
    public void setVoltages(double leftFlywheelVolts, double rightFlywheelVolts,
                            double feedVolts) {
        
        closedLoop = false;
        leftShooterAppliedVolts = 0.0;
        rightShooterAppliedVolts = 0.0;
       // leftFeedAppliedVolts = 0.0;
        rightFeedAppliedVolts = 0.0;
        
        leftShooterSim.setInputVoltage(leftFlywheelVolts);
        rightShooterSim.setInputVoltage(rightFlywheelVolts);
        leftFeedSim.setInputVoltage(feedVolts);
        rightFeedSim.setInputVoltage(feedVolts);
        
    }

    @Override
    public void setLeftShooterVelocity(double velocityRadPerSec, double ffVolts) {
        closedLoop = true;
        pid.setSetpoint(velocityRadPerSec);
        this.ffVolts = ffVolts;
    }

    @Override
    public void setRightShooterVelocity(double velocityRadPerSec, double ffVolts) {
        closedLoop = true;
        pid.setSetpoint(velocityRadPerSec);
        this.ffVolts = ffVolts;
    }

    @Override
    public void setShooterVelocity(double velocityRadPerSec, double ffVolts) {
        closedLoop = true;
        pid.setSetpoint(velocityRadPerSec);
        this.ffVolts = ffVolts;
    }

    @Override
    public void stop() {
        setVoltages(0.0, 0.0, 0.0);
    }

    @Override
    public void configureFlywheelPID(double kP, double kI, double kD) {
        pid.setPID(kP, kI, kD);
    }
    
}
