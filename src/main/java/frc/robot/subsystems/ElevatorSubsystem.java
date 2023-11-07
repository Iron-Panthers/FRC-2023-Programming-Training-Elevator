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
import frc.robot.Constants.Elevator;
public class ElevatorSubsystem extends SubsystemBase {
  /** follower */
  private TalonFX left_motor;
  /** leader */
  private TalonFX right_motor;
  private PIDController pidController;
  private final ShuffleboardTab ElevatorTab = Shuffleboard.getTab("Elevator");

  private double targetHeight;
  private double motorPower;
  private double inPerTick;
  private double minHeight;
  private double maxHeight;
  private double currentHeight;

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
    right_motor.setNeutralMode(NeutralMode.Brake);
    left_motor.setNeutralMode(NeutralMode.Brake);

    right_motor.follow(left_motor);
    
    targetHeight = 0;
    minHeight = 0;
    maxHeight = 20;
    motorPower = 0;
  pidController = new PIDController(0.4,0, 0.01); 
  pidController.setTolerance(0.2,0.001);
    ElevatorTab.addNumber("Current Motor Power", () -> this.motorPower);
    ElevatorTab.addNumber("Target Height", () -> this.targetHeight);
    ElevatorTab.add(pidController);

    ElevatorTab.addNumber("Left Motor Speed", left_motor::getSelectedSensorVelocity);
    ElevatorTab.addNumber("Right Motor Speed", right_motor::getSelectedSensorVelocity);
    ElevatorTab.addNumber("Position Error", pidController::getPositionError);

// hi nora - sincerely, evelyn =D
    // ElevatorTab.addNumber("height", () -> this.currentHeight);
    // ElevatorTab.addNumber("target height", () -> this.targetHeight);
    // ElevatorTab.addNumber("right motor sensor value", this::getHeight);



  }
 public void setMotorPower(double x){
  System.out.println("hello");
 }

   public static double inchesToTicks(double height) {
    return height * ((Elevator.GEAR_RATIO * Elevator.TICKS_PER_REVOLUTION) / (Elevator.GEAR_CIRCUMFERENCE));
  }
  public static double ticksToInches(double ticks) {
    return (ticks * Elevator.GEAR_CIRCUMFERENCE) / (Elevator.TICKS_PER_REVOLUTION * Elevator.GEAR_RATIO);
  }
                      
                                                                               
  public void setTargetHeight(double targetHeight) {     
    this.targetHeight = targetHeight;                                   
    pidController.setSetpoint(this.targetHeight); }  
          
  public double getCurrentHeight(){
    currentHeight = ticksToInches(-left_motor.getSelectedSensorPosition());
   
    return currentHeight;

  }                                           
                                                                               
  // Sets the goal of the pid controller                                       
                                          
@Override
  public void periodic() {


    // left_motor.follow(right_motor);
    // right_motor.set(TalonFXControlMode.PercentOutput, motorPower);
    // left_motor.follow(right_motor);

    //left_motor.set(TalonFXControlMode.PercentOutput, motorPower);


        motorPower = 0.07+ (pidController.calculate(getCurrentHeight()));
    if (!pidController.atSetpoint()){
      if (currentHeight<5){
        left_motor.set(TalonFXControlMode.PercentOutput, -(MathUtil.clamp(motorPower,-0.2,0.2)));
      }
      else{
        left_motor.set(TalonFXControlMode.PercentOutput, -(MathUtil.clamp(motorPower,-0.5,0.5)));
      }
        
    }
    // left_motor.set(TalonFXControlMode.PercentOutput, -(0.1));
  }

}