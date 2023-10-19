// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** follower */
  private TalonFX left_motor;
  /** leader */
  private TalonFX right_motor;

  private final ShuffleboardTab ElevatorTab = Shuffleboard.getTab("Elevator");

<<<<<<< HEAD
    private double targetHeight;
    private double motorPower;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
   left_motor = new TalonFX(7);
   right_motor = new TalonFX(6);
    
   right_motor.configFactoryDefault();
=======
  private double targetHeight;  

  private double motorPower;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    left_motor = new TalonFX(7);
    right_motor = new TalonFX(6);
   
    right_motor.configFactoryDefault();
>>>>>>> main
    left_motor.configFactoryDefault();

    right_motor.clearStickyFaults();
    left_motor.clearStickyFaults();

    right_motor.configForwardSoftLimitThreshold(
        0, 0); // this is the bottom limit, we stop AT the bottom
    // right_motor.configReverseSoftLimitThreshold(
    //     -heightToTicks(24), 0); // this is the top limit, we stop at the very top

    right_motor.configForwardSoftLimitEnable(true, 0);
    right_motor.configReverseSoftLimitEnable(true, 0);

    right_motor.configOpenloopRamp(.5);

    left_motor.setSelectedSensorPosition(0);
    right_motor.setSelectedSensorPosition(0);

    // make sure we hold our height when we get disabled
    right_motor.setNeutralMode(NeutralMode.Coast);
    left_motor.setNeutralMode(NeutralMode.Coast);

   

    targetHeight = 0;

    motorPower = 0;

    ElevatorTab.addNumber("Current Motor Power", () -> this.motorPower);
    ElevatorTab.addNumber("Target Height", () -> this.targetHeight);

    
    ElevatorTab.addNumber("Left Motor Speed", left_motor::getSelectedSensorVelocity);
    ElevatorTab.addNumber("Right Motor Speed", right_motor::getSelectedSensorVelocity);

<<<<<<< HEAD
    targetHeight = 0;

    ElevatorTab.addNumber("Current Motor Powet", () -> this.motorPower);
    ElevatorTab.addNumber("target height", () -> this.targetHeight);

=======
>>>>>>> main
    // ElevatorTab.addNumber("height", () -> this.currentHeight);
    // ElevatorTab.addNumber("target height", () -> this.targetHeight);
    // ElevatorTab.addNumber("right motor sensor value", this::getHeight);

  }

<<<<<<< HEAD
public void setMotorPower(double motorPower){
  this.motorPower = MathUtil.clamp(motorPower, 0d, 0.25);
  
}

  @Override
  public void periodic() {
    right_motor.set(TalonFXControlMode.PercentOutput, motorPower);

=======
  public void setMotorPower(double motorPower){
    this.motorPower = MathUtil.clamp(motorPower, -0.25, 0.25);
  }

  @Override
  public void periodic() {
   
    left_motor.set(TalonFXControlMode.PercentOutput, motorPower);
    right_motor.follow(left_motor);
    // left_motor.follow(right_motor);
    // right_motor.set(TalonFXControlMode.PercentOutput, motorPower);
    // left_motor.follow(right_motor);

    //left_motor.set(TalonFXControlMode.PercentOutput, motorPower);
>>>>>>> main
  }


}