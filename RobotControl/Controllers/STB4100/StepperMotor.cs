public class StepperMotor
{
    public int StepsPerRev { get; private set; }
    public double GearRatio { get; }
    public int Pin { get; }
    public double StartingAngle { get; }

    public int CurrentSteps { get; set; }
    public int Steps { get; set; }

    public double AngleToSteps => (StepsPerRev * GearRatio) / 360.0;
    public double StepsToAngle => 360.0 / (StepsPerRev * GearRatio);

    public double TargetAngle { get; private set; }
    public bool InvertDirection { get; set; }

    public StepperMotor(int pin, int stepsPerRev, double gearRatio,  double startingAngle)
    {
        StepsPerRev = stepsPerRev;
        GearRatio = gearRatio;
        Pin = pin;
        StartingAngle = startingAngle;

        TargetAngle = startingAngle;
    }

    public void Reconfigure(int stepsPerRev)
    {
        StepsPerRev = stepsPerRev;
    }

    public double CurrentAngle => CurrentSteps * StepsToAngle;
    public int TargetSteps => (int)(TargetAngle * AngleToSteps);
    public int StepError => TargetSteps - CurrentSteps;

    public void SetTarget(double angle) => TargetAngle = angle;
    public void OverwriteTarget(double angle)
    {
        // Clear the current location of the motor to this new angle
        TargetAngle = angle;
        CurrentSteps = TargetSteps;
    }
    public void Stop() => TargetAngle = CurrentAngle;

}
