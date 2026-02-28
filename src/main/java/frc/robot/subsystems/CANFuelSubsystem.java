package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax backRoller;
  private final SparkMax frontRoller;
  private final SparkMax launcherRoller;
  private final RelativeEncoder shooterEncoder;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    backRoller = new SparkMax(BACK_INTAKE_MOTOR_ID, MotorType.kBrushed);
    frontRoller = new SparkMax(FRONT_INTAKE_MOTOR_ID, MotorType.kBrushed);
    launcherRoller = new SparkMax(LAUNCHER_MOTOR_ID, MotorType.kBrushed);

    shooterEncoder = launcherRoller.getEncoder();

    SmartDashboard.putNumber("Intaking Back", INTAKING_BACK_VOLTAGE);
    SmartDashboard.putNumber("Intaking Front", INTAKING_FRONT_VOLTAGE);
    SmartDashboard.putNumber("Launching Intake", LAUNCHING_INTAKING_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin up", SPIN_UP_VOLTAGE);

    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(CURRENT_LIMIT);
    backRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(CURRENT_LIMIT);
    launcherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake() {
    backRoller.setVoltage(SmartDashboard.getNumber("Intaking Back", INTAKING_BACK_VOLTAGE));
    frontRoller.setVoltage(-1 * SmartDashboard.getNumber("Intaking Front", INTAKING_FRONT_VOLTAGE));
    launcherRoller.setVoltage(SmartDashboard.getNumber("Launching Intake", LAUNCHING_INTAKING_VOLTAGE));
  }

  public void eject() {
    backRoller.setVoltage(-1 * SmartDashboard.getNumber("Intaking Back", INTAKING_BACK_VOLTAGE));
    frontRoller.setVoltage(SmartDashboard.getNumber("Intaking Front", INTAKING_FRONT_VOLTAGE));
    launcherRoller.setVoltage(SmartDashboard.getNumber("Launching Intake", LAUNCHING_INTAKING_VOLTAGE));
  }

  public void launch() {
    backRoller.setVoltage(-1 * SmartDashboard.getNumber("Launching Intake", LAUNCHING_INTAKING_VOLTAGE));
    frontRoller.setVoltage(-1 * SmartDashboard.getNumber("Launching Intake", LAUNCHING_INTAKING_VOLTAGE));
    launcherRoller.setVoltage(SmartDashboard.getNumber("Launching launcher", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  public void stop() {
    backRoller.set(0);
    frontRoller.set(0);
    launcherRoller.set(0);
  }

  public void spinUp() {
    backRoller.setVoltage(SmartDashboard.getNumber("Spin up", SPIN_UP_VOLTAGE));
    frontRoller.setVoltage(SmartDashboard.getNumber("Spin up", SPIN_UP_VOLTAGE));
    launcherRoller.setVoltage(SmartDashboard.getNumber("Launching launcher", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  public double getShooterRPM() {
    return shooterEncoder.getVelocity();
  }

  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  public Command launchCommand() {
    return this.run(() -> launch());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
  }
}