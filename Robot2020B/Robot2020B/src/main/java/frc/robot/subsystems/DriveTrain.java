/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;




public class Drivtraij extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */



   private TalonSRX DriveLF;
   private TalonSRX DriveRF;
   private TalonSRX DriveLB;
   private TalonSRX DriveRB;

   private static final int joysick = 1;
  

  public ExampleSubsystem() {
    final TalonSRX DriveLF = new TalonSRX(1);
    DriveLF.configGactoryDefault();
    DriveLF.setInverted(true);
    DriveLF.setSensorPhase(true);
    DriveLF.ConfigNeutralDeadband(kdDeadBand);
    DriveLF.setNeutralMode(NeutralMode.Coast);

    DriveLF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kdTimeoutMS);
    DriveLF.setsStatusFramePeriod(StatusFrameEnhanced.Status_13_Bade_Base_PIDF0, 10, kdTimeoutMs);

    DriveLF.configNominalOutputFoward(0, kdTimeoutMs);
    DriveLF.configNominalOutputReverse(0,kdTimeoutMs);
    DriveLF.configPeakOutputForward(1, kdTimeoutMS);
    DriveLF.configPeakOutputReverse(-1, kdTimeoutMs);
    
    
    final TalonSRX DriveRF = new TalonSRX(2);
    DriveRF.configGactoryDefault();
    DriveRF.setInverted(true);
    DriveRF.setSensorPhase(true);
    DriveRF.ConfigNeutralDeadband(kdDeadBand);
    DriveRF.setNeutralMode(NeutralMode.Coast);

    DriveRF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kdTimeoutMS);
    DriveRF.setsStatusFramePeriod(StatusFrameEnhanced.Status_13_Bade_Base_PIDF0, 10, kdTimeoutMs);

    DriveRF.configNominalOutputFoward(0, kdTimeoutMs);
    DriveRF.configNominalOutputReverse(0,kdTimeoutMs);
    DriveRF.configPeakOutputForward(1, kdTimeoutMS);
    DriveRF.configPeakOutputReverse(-1, kdTimeoutMs);

    final TalonSRX DriveRB = new TalonSRX(3);
    DriveRB.configGactoryDefault();
    DriveRB.setInverted(true);
    DriveRB.setSensorPhase(true);
    DriveRB.ConfigNeutralDeadband(kdDeadBand);
    DriveRB.setNeutralMode(NeutralMode.Coast);

    DriveRB.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kdTimeoutMS);
    DriveRB.setsStatusFramePeriod(StatusFrameEnhanced.Status_13_Bade_Base_PIDF0, 10, kdTimeoutMs);

    DriveRB.configNominalOutputFoward(0, kdTimeoutMs);
    DriveRB.configNominalOutputReverse(0,kdTimeoutMs);
    DriveRB.configPeakOutputForward(1, kdTimeoutMS);
    DriveRB.configPeakOutputReverse(-1, kdTimeoutMs);



    final TalonSRX DriveLB = new TalonSRX(4);
    DriveLB.configGactoryDefault();
    DriveLB.setInverted(true);
    DriveLB.setSensorPhase(true);
    DriveLB.ConfigNeutralDeadband(kdDeadBand);
    DriveLB.setNeutralMode(NeutralMode.Coast);

    DriveLB.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kdTimeoutMS);
    DriveLB.setsStatusFramePeriod(StatusFrameEnhanced.Status_13_Bade_Base_PIDF0, 10, kdTimeoutMs);

    DriveLB.configNominalOutputFoward(0, kdTimeoutMs);
    DriveLB.configNominalOutputReverse(0,kdTimeoutMs);
    DriveLB.configPeakOutputForward(1, kdTimeoutMS);
    DriveLB.configPeakOutputReverse(-1, kdTimeoutMs);



  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void DriveWithJoy() { 
        
        
    double Jy=-1*Robot.oi.xboxP.getY(Hand.kLeft)*1;
    double Jx=Robot.oi.xboxP.getX(Hand.kLeft)*1;
    double Jz=Robot.oi.xboxP.getX(Hand.kRight)*1;


    
    double r = Math.hypot(Jx, Jy);
    double robotAngle = Math.atan2(Jy, Jx) - Math.PI / 4;
    double rightX = Jz;
    var LF = r * Math.cos(robotAngle) + rightX;
    double RF = r * Math.sin(robotAngle) - rightX;
    double LR = r * Math.sin(robotAngle) + rightX;
    double RR = r * Math.cos(robotAngle) - rightX;
    System.out.println("LF:" + LF + "  RF:" + RF );
    System.out.println("LR:" + LR + "  RR:" + RR );
    System.out.println(" ");

    SmartDashboard.putNumber("LF", LF);
    SmartDashboard.putNumber("RF", RF);
    SmartDashboard.putNumber("LR", LR);
    SmartDashboard.putNumber("RR", RR);

    //leftFront.setPower(v1); old old from origianl math code example
    //rightFront.setPower(v2);
    //leftRear.setPower(v3)
    //rightRear.setPower(v4);
    
    
   
   // DriveRF.set(RF);         
    //DriveLR.set(LR);
    //DriveRR.set(RR);

    DriveLF.set(ControlMode.PercentOutput, LF, DemandType.ArbitraryFeedForward, 0);  // this was oringaly used for arcade drive     
    DriveLR.set(ControlMode.PercentOutput, LR, DemandType.ArbitraryFeedForward, 0);  // this was oringaly used for arcade drive
   DriveRF.set(ControlMode.PercentOutput, RF, DemandType.ArbitraryFeedForward, 0);  // this was oringaly used for arcade drive
   DriveRR.set(ControlMode.PercentOutput, RR, DemandType.ArbitraryFeedForward, 0);  // this was oringaly used for arcade drive


  }
   public void ApplyMotorPower(double L, double R) {                    // Tankdrive Percent output
        // This modifies power applied to the motors directly.

        // Speed reduction
        // Original design:  Both triggers is reduction2, one trigger is reduction 1.
        // This allows either trigger to be used for reduction 1 and both results in reduction 2.
        if(Robot.oi.rightStick.getTrigger()&&Robot.oi.leftStick.getTrigger()){
            L=L*Constants.DRIVE_POWER_REDUCTION2;
            R=R*Constants.DRIVE_POWER_REDUCTION2;
        } else if(Robot.oi.rightStick.getTrigger()||Robot.oi.leftStick.getTrigger()){
            L=L*Constants.DRIVE_POWER_REDUCTION;
            R=R*Constants.DRIVE_POWER_REDUCTION;
        }


        //Apply the power to the drive.
        DriveLF.set(ControlMode.PercentOutput, L, DemandType.ArbitraryFeedForward, 0);  // this was oringaly used for arcade drive
        DriveRF.set(ControlMode.PercentOutput, R, DemandType.ArbitraryFeedForward, 0);  // With the zero at the end I don't think it does anything different   

    }
}
