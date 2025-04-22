package frc.robot.subsystems;

public interface MotorDef {
    // public MotorSrx(String name, int Id, int followId, boolean logging);

    public double getPos();

    public void enableLimitSwitch(boolean forward, boolean reverse);

    public boolean getForwardLimitSwitch();

    public boolean getReverseLimitSwitch();

    public void setPos(double position);

    public void setVelocity(double velocity);

    public void setInverted(boolean invert);

    public void setLogging(boolean value);

    public double getLastSpeed();

    public double getActualVelocity();

    public void forcePercentMode();

    // void periodic();

    public void logPeriodic();

    /*
     * Peak Current and Duration must be exceeded before current limit is activated.
     * When activated, current will be limited to Continuous Current. Set Peak
     * Current params to 0 if desired behavior is to immediately current-limit.
     */
    public void setCurrentLimit(int peakAmps, int continousAmps, int durationMilliseconds);

    public void updateSmart();

    public void setSpeed(double speed);

    public void setSpeedAbsolute(double speed);

    public void stopMotor();

    public void zeroEncoder();

    public void setEncoderPosition(double position);

    public void setVelocityPID(PID pid);

    public double getMotorVoltage();

    public void PIDToMotor(PID pid, int slot, int timeout);

    public double getError();

    public void logMotorVCS();

    public String getMotorVCS();

    public double getMotorCurrent();

    public void setSensorPhase(boolean phase);

    // Config the sensor used for Primary PID and sensor direction
    public void setPositionPID(int pidIdx, PID pid);

    public void setRampClosedLoop(double rate);

    public void setRampOpenLoop(double rate);
}
