package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
        public boolean intakeState = false;
        public double gamePiecesCollected = 0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(PivotIOInputs inputs) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {}

    /** Run closed loop to reach position. */
    public default void seekPosition(double position) {}

    /** Stop in open loop. */
    public default void stop() {}

    /** Set velocity PID constants. */
    public default void configurePID(double kP, double kI, double kD) {}

    public default void setRunning(boolean runIntake) {}

    public default void setIntakeState(boolean state) {}
}
