package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Drivetrain extends Subsystem {

    private TalonSRX leftTalon;
    private TalonSRX rightTalon;

    private AHRS navxGyro;

    private double x, y, theta;
    private Drivetrain instance;
    private double lastPos, currentPos;

    public Drivetrain() {
        navxGyro = new AHRS(SPI.Port.kMXP);
        //System.out.println(SPI.Port.kMXP);

        while (!navxGyro.isConnected()){};
        for(int i=0;i<100000;i++);
        while (navxGyro.isCalibrating()){};
        while (navxGyro.isRotating()){};
        for(int i=0;i<100000;i++);
        navxGyro.reset();

        x = 0;
        y = 0;
        theta = 0;
        //System.out.println("Yaw: " + Math.toRadians(navxGyro.getYaw()));
        //System.out.println("Pitch: " + navxGyro.getPitch());
        //System.out.println("Roll: " + navxGyro.getRoll());

        leftTalon = new TalonSRX(2);
        rightTalon = new TalonSRX(3);
        setupTalons();
        navxGyro.resetDisplacement();

        lastPos = (leftTalon.getSelectedSensorPosition() + rightTalon.getSelectedSensorPosition()) / 2;
        //System.out.println("Yaw: " + Math.toRadians(navxGyro.getYaw()));
        //System.out.println("Pitch: " + navxGyro.getPitch());
        //System.out.println("Roll: " + navxGyro.getRoll());

        Notifier odoThread = new Notifier(() ->

        {
            currentPos = (leftTalon.getSelectedSensorPosition() + rightTalon.getSelectedSensorPosition()) / 2;
            System.out.println("Left Enc:" + leftTalon.getSelectedSensorPosition());
            System.out.println("Right Enc: " + rightTalon.getSelectedSensorPosition());

            double dPos = Units.ticksToFeet(currentPos - lastPos);
            //System.out.println("Is rotating: " + navxGyro.isRotating());
            if (!navxGyro.isCalibrating())
                theta = Math.toRadians(navxGyro.getYaw());
            x += Math.cos(theta) * dPos;
            y -= Math.sin(theta) * dPos; //If y position is returning negative, then change this to y+=
            lastPos = currentPos;

        });

        odoThread.startPeriodic(0.01);

    }

    public void setupTalons(){
        rightTalon.configFactoryDefault();
        leftTalon.configFactoryDefault();

        rightTalon.setInverted(true);
        leftTalon.setInverted(false);
        rightTalon.setSensorPhase(true);
        leftTalon.setSensorPhase(true);
        rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        rightTalon.setNeutralMode(NeutralMode.Brake);
        leftTalon.setNeutralMode(NeutralMode.Brake);
        rightTalon.configOpenloopRamp(0);
        leftTalon.configOpenloopRamp(0);
        rightTalon.configClosedloopRamp(0);
        leftTalon.configClosedloopRamp(0);
        rightTalon.configPeakOutputForward(1);
        leftTalon.configPeakOutputForward(1);
        rightTalon.configPeakOutputReverse(-1);
        leftTalon.configPeakOutputReverse(-1);

        rightTalon.setSelectedSensorPosition(0);
        leftTalon.setSelectedSensorPosition(0);

        rightTalon.config_kP(0, Constants.kP);
        rightTalon.config_kI(0, Constants.kI);
        rightTalon.config_kD(0, Constants.kD);
        rightTalon.config_kF(0, Constants.kF);

        leftTalon.config_kP(0, Constants.kP);
        leftTalon.config_kI(0, Constants.kI);
        leftTalon.config_kD(0, Constants.kD);
        leftTalon.config_kF(0, Constants.kF);
    }

    public Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;

    }

    public enum DrivetrainSide {
        left, right;
    }

    public TalonSRX getTalon(DrivetrainSide side) {
        if (side.equals(DrivetrainSide.left)) {
            return leftTalon;
        } else if (side.equals(DrivetrainSide.right)) {
            return rightTalon;
        } else {
            return null;
        }
    }

    public void setSpeeds(double left, double right) {
        leftTalon.set(ControlMode.PercentOutput, left);
        rightTalon.set(ControlMode.PercentOutput, right);
    }

    public void setFPS(double left, double right) {
        leftTalon.set(ControlMode.Velocity, Units.feetToTicks(left) / 10);
        rightTalon.set(ControlMode.Velocity, Units.feetToTicks(right) / 10);
    }

    public Odometry getOdo() {
        return new Odometry(x, y, theta);
    }

    @Override
    protected void initDefaultCommand() {

    }

}
