public class StepperMotor
{
    public int StepsPerRev { get; }
    public double GearRatio { get; }
    public int Pin { get; }
    public double StartingAngle { get; }

    public int CurrentSteps { get; set; }
    public int Steps { get; set; }

    public double AngleToSteps { get; }
    public double StepsToAngle { get; }

    public double TargetAngle { get; private set; }

    public StepperMotor(int pin, int stepsPerRev, double gearRatio,  double startingAngle)
    {
        StepsPerRev = stepsPerRev;
        GearRatio = gearRatio;
        Pin = pin;
        StartingAngle = startingAngle;
        AngleToSteps = (stepsPerRev * gearRatio) / 360.0;
        StepsToAngle = 360.0 / (stepsPerRev * gearRatio);

        TargetAngle = startingAngle;
    }

    public double CurrentAngle => CurrentSteps * StepsToAngle;
    public int TargetSteps => (int)(TargetAngle * AngleToSteps);
    public int StepError => TargetSteps - CurrentSteps;

    public void SetTarget(double angle) => TargetAngle = angle;
    public void Stop() => TargetAngle = CurrentAngle;
}
