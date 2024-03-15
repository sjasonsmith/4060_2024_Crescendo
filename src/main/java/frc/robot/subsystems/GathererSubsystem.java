// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.Constants.*;

public class GathererSubsystem extends SubsystemBase {
    private final CANSparkMax gathererLiftMotor = new CANSparkMax(GATHERER_ROLLER_MOTOR, MotorType.kBrushed);
    private final CANSparkMax gathererRollerMotor = new CANSparkMax(GATHERER_ROLLER_MOTOR, MotorType.kBrushless);
    private RelativeEncoder gathererLiftEncoder;
    private SparkPIDController gathererLiftPIDController;
    private SparkPIDController gathererRollerPIDController;


    private static double LIFTER_RAISED_SETPOINT = 0;
    private static double LIFTER_LOWERED_SETPOINT = 120;
    private double gathererLiftSetpoint = LIFTER_RAISED_SETPOINT;

    private static double ROLLER_FEED_RPM = 240.0;

    /** Creates a new GathererSubsystem. */
    public GathererSubsystem() {
        // Configure lift motor as follows:
        // 1. Encoder with 8192 counts per revolution, 360 absolute encoder conversion factor
        // 2. Softawre limits: Reverse Limit = 0, Forward Limit = 180
        // 3. Ramp Rate to 100% power: 0.2 seconds
        // 4. Set the motor to brake mode
        // 5. Smart current limit: 10A when stalled, 30A when running
        // 6. PID Slot 1, P = 0.02, I = 0.0, D = 0.0, F = 0.0
        gathererLiftMotor.restoreFactoryDefaults();
        gathererLiftEncoder = gathererLiftMotor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 8192);
        gathererLiftEncoder.setPositionConversionFactor(360);
        gathererLiftMotor.setOpenLoopRampRate(0.2);
        gathererLiftMotor.setClosedLoopRampRate(0.2);
        gathererLiftMotor.setSmartCurrentLimit(10, 30);
        gathererLiftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        gathererLiftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        gathererLiftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        gathererLiftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        gathererLiftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 180);
        
        gathererLiftPIDController = gathererLiftMotor.getPIDController();
        gathererLiftPIDController.setP(0.02);
        gathererLiftPIDController.setI(0.0);
        gathererLiftPIDController.setD(0.0);
        gathererLiftPIDController.setFF(0.0);
        
        gathererLiftEncoder.setPosition(LIFTER_RAISED_SETPOINT); // Reset in case we rebooted
        gathererLiftPIDController.setReference(LIFTER_RAISED_SETPOINT, ControlType.kPosition);

        // Configure roller motor as follows:
        // 1. NEO Brushless motor
        // 2. Brake Mode
        // 3. Smart current limit: 10A when stalled, 30A when running
        // 4. Ramp Rate to 100% power: 0.2 seconds
        // 5. Configure encoder for a 1:26 gear ratio
        // Smart Velocity Control Parameters:
        // 1. PID Slot 0, P = 0.001, I = 0.000013, D = 0.0, F = 0.0
        // 2. Max Velocity = 360 RPM
        // 3. Max Acceleration = 360 RPM/s
        gathererRollerMotor.restoreFactoryDefaults();
        gathererRollerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        gathererRollerMotor.setSmartCurrentLimit(10, 30);
        gathererRollerMotor.setOpenLoopRampRate(0.2);
        gathererRollerMotor.setClosedLoopRampRate(0.2);
        gathererRollerMotor.getEncoder().setPositionConversionFactor(1.0/26.0);

        gathererRollerPIDController = gathererRollerMotor.getPIDController();
        gathererRollerPIDController.setP(0.001);
        gathererRollerPIDController.setI(0.000013);
        gathererRollerPIDController.setD(0.0);
        gathererRollerPIDController.setFF(0.0);
        gathererRollerPIDController.setSmartMotionMaxVelocity(360, 0);
        gathererRollerPIDController.setSmartMotionMaxAccel(360, 0);
        gathererRollerMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setLifterSetpoint(double setpoint) {
        gathererLiftSetpoint = setpoint;
        gathererLiftPIDController.setReference(setpoint, ControlType.kPosition);
    }


    public void raiseLifter() {
        setLifterSetpoint(LIFTER_RAISED_SETPOINT);
    }

    public void lowerLifter() {
        setLifterSetpoint(LIFTER_LOWERED_SETPOINT);
    }

    public void stopLifter() {
        gathererLiftMotor.stopMotor();
    }

    public Boolean isLifterAtSetpoint() {
        double threshold = 5.0;
        return Math.abs(gathererLiftSetpoint - gathererLiftEncoder.getPosition()) < threshold;
    }

    public void feedIn() {
        gathererRollerPIDController.setReference(ROLLER_FEED_RPM, ControlType.kSmartVelocity);
    }

    public void feedOut() {
        gathererRollerPIDController.setReference(-ROLLER_FEED_RPM, ControlType.kSmartVelocity);
    }

    public void stopFeeding() {
        gathererRollerMotor.stopMotor();
    }
}
