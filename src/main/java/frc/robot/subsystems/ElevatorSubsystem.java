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
import frc.robot.Constants;
public class ElevatorSubsystem extends SubsystemBase {
  /** follower */
  private TalonFX left_motor;
  /** leader */
  private TalonFX right_motor;

  private final ShuffleboardTab ElevatorTab = Shuffleboard.getTab("Elevator");

  private double currentHeight;
  private double targetHeight;  

  private double motorPower;

  private PIDController controller;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    left_motor = new TalonFX(7);
    right_motor = new TalonFX(6);
   
    right_motor.configFactoryDefault();
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

    right_motor.follow(left_motor);

    targetHeight = 0;

    motorPower = 0;

    controller = new PIDController(0, 0, 0);

    ElevatorTab.addNumber("Current Motor Power", () -> this.motorPower);
    ElevatorTab.addNumber("Target Height", () -> this.targetHeight);

    
    ElevatorTab.addNumber("Left Motor Speed", left_motor::getSelectedSensorVelocity);
    ElevatorTab.addNumber("Right Motor Speed", right_motor::getSelectedSensorVelocity);

    // ElevatorTab.addNumber("height", () -> this.currentHeight);
    // ElevatorTab.addNumber("target height", () -> this.targetHeight);
    // ElevatorTab.addNumber("right motor sensor value", this::getHeight);

  }

  public void setMotorPower(double motorPower){
    this.motorPower = MathUtil.clamp(motorPower, -0.25, 0.25);
  }

  public double inchesToTicks(double inches){

    return inches * Constants.Elevator.TICKS_PER_REVOLUTION / (Constants.Elevator.GEAR_RATIO * Constants.Elevator.GEAR_CIRCUMFERENCE); 
  }

  public double ticksToInches(double ticks){
    return (ticks * Constants.Elevator.GEAR_RATIO * Constants.Elevator.GEAR_CIRCUMFERENCE) / Constants.Elevator.TICKS_PER_REVOLUTION;
  }

  public void setHeight(double targetHeight){
    this.targetHeight = targetHeight;

    controller.setSetpoint(targetHeight);
  }

  @Override
  public void periodic() {

    currentHeight = ticksToInches(-left_motor.getSelectedSensorPosition());
   
    motorPower = controller.calculate(currentHeight);


    left_motor.set(TalonFXControlMode.PercentOutput, motorPower);
   
  }


}