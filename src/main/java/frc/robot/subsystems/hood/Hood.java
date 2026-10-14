/* (C) RoboLancers 2026 */
package frc.robot.subsystems.hood;
//whole bunch of imports
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;

public class Hood extends SubsystemBase {

  @Logged private TalonFX hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
  @Logged private DigitalInput hoodLimitSwitch = new DigitalInput(HoodConstants.kLimitSwitchID);
  @Logged private CANdi magneticLimitSwitch = new CANdi(0);
  private Angle targetAngle = HoodConstants.kStartingAngle;

  public PIDController pidController = new PIDController(0, 0, 0);

  public ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0);

  public Hood() {
    configureMotors();
    setHoodPID(HoodConstants.kP, HoodConstants.kD, HoodConstants.kG);
    magneticLimitSwitch.getConfigurator().apply(candiConfigs);
  }

  public void configureMotors() {

    TalonFXConfiguration hoodMotorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(HoodConstants.kHoodStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(HoodConstants.kHoodSupplyCurrentLimit)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        HoodConstants.kHoodMotorInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(HoodConstants.kHoodMotorMaxVelocity)
                    .withMotionMagicAcceleration(HoodConstants.kHoodMotorMaxAcceleration))
            .withSlot0(
                new Slot0Configs()
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(HoodConstants.kHoodGearRatio));

    hoodMotor.getConfigurator().apply(hoodMotorConfigs);
  }

  //Sets up the CAN motors
  public CANdiConfiguration candiConfigs =
      new CANdiConfiguration()
          .withDigitalInputs(
              new DigitalInputsConfigs().withS1CloseState(S1CloseStateValue.CloseWhenLow));

  //Sets the PID values for the hood motor controller and the feedforward value for the arm feedforward controller.
  public void setHoodPID(double kP, double kD, double kG) {
    pidController.setP(kP);
    pidController.setD(kD);
    armFeedforward.setKg(kG);
  }

  //Sets the target angle for the hood to move to.
  public void setTargetAngle(Angle angle) {
    this.targetAngle = angle;
  }

  //Moves the hood to the desired angle using PID controls and an arm feedforward value
  public void goToAngle(Angle angle) {
    hoodMotor.setVoltage(
        pidController.calculate(getAngle().in(Degrees), angle.in(Degrees))
            + armFeedforward.calculate(angle.in(Degrees), 0));
  }

  //Calculates the score angle based on the distance to the hub.
  public Angle getScoreAngle(Distance hubDistance) {
    double pitch = 0;
    pitch = -37.69619 + 9.35374 * Math.log(hubDistance.in(Inches));
    return Degrees.of(pitch);
  }

  //Returns the current angle of the hood, also logs the value to the dashboard.
  @Logged(name = "hoodPitch")
  public Angle getAngle() {
    Angle angle = hoodMotor.getPosition().getValue();
    return angle;
  }

  //Returns the current velocity of the hood, also logs the value to the dashboard.
  @Logged(name = "hoodVelocity")
  public AngularVelocity getVelocity() {
    return hoodMotor.getVelocity().getValue();
  }

  //checks whther the homed velocity is within a set  tolerence value
  public boolean isHomedVelocity() {
    return Math.abs(getVelocity().in(RPM) - HoodConstants.kHomingVelocityFloor.in(RPM))
        <= HoodConstants.kHomingVelocityTolerance;
  }

  //checks whther the homed is greater than or equal to the set current ceiling value
  public boolean isHomedCurrent() {
    return getCurrent().in(Amps) >= HoodConstants.kCurrentCeiling.in(Amps);
  }

  //checks whether the hood is at the homed position by checking the state of the magnetic limit switch
  public boolean atHomedPosition() {
    return magneticLimitSwitch.getS1Closed(true).getValue();
  }

  //Zeros the encoder for the hood motor
  public void zeroEncoder() {
    hoodMotor.setPosition(HoodConstants.kZeroPosition);
  }

  //Sets the voltage for the hood motor
  public void runVolts(Voltage volts) {
    hoodMotor.setVoltage(volts.in(Volts));
  }

  //Checks and logs whether the hood is at the target angle within a set tolerance value
  @Logged(name = "hoodAtTargetAngle")
  public boolean atTargetAngle() {
    boolean atTargetAngle =
        Math.abs(getAngle().in(Degrees) - targetAngle.in(Degrees))
            < HoodConstants.kAngleTolerance.in(Degrees);
    return atTargetAngle;
  }

//Checks whether the hood is at a specific angle
  public boolean atAngle(Angle angle) {
    boolean atAngle =
        Math.abs(getAngle().in(Degrees) - angle.in(Degrees))
            < HoodConstants.kAngleTolerance.in(Degrees);
    return atAngle;
  }

  //Sets the PID and feedforward values for the hood motor and moves the hood to a specific angle
  public void tune(double kP, double kD, double kG, double targetAngle) {
    setHoodPID(kP, kD, kG);
    goToAngle(Degrees.of(targetAngle));
  }

  TunableConstant kHomingVoltage = new TunableConstant("Homing voltage", 0);

  public Command homeHoodMagnetic() {
    return run(() -> hoodMotor.setVoltage(kHomingVoltage.get()))
        .until(() -> atHomedPosition())
        .andThen(() -> zeroEncoder());
  }

  //logs the target angle for the hood
  @Logged(name = "hoodTargetAngle")
  public Angle getTargetAngle() {
    return this.targetAngle;
  }

  //logs the voltage for the hood
  @Logged(name = "hoodVoltage")
  public Voltage getVoltage() {
    return hoodMotor.getMotorVoltage().getValue();
  }

  //logs the current for the hood
  @Logged(name = "hoodCurrent")
  public Current getCurrent() {
    return hoodMotor.getStatorCurrent().getValue();
  }

  //logs the position of the hood limit switch
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Angle", hoodMotor.getPosition().getValue().in(Degrees));
    SmartDashboard.putBoolean("Hood Is At Homed Position", atHomedPosition());
    SmartDashboard.putNumber("Hood Voltage", getVoltage().in(Volts));
  }
}
