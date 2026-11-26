using Controller.RobotControl.Conveyors;

namespace RobotControl.Tests.Conveyors;

public class ConveyorCalibrationTests
{
    // A belt at 30° whose counter over-reads by 2 %: the mark moves 0.98 mm per reported mm.
    private const double Angle = 30, K = 0.98;
    private static readonly (double X, double Y) Origin = (250, -40);
    private static double Cos => Math.Cos(Angle * Math.PI / 180);
    private static double Sin => Math.Sin(Angle * Math.PI / 180);

    private static ConveyorCalibrationSample Mark(double travel, double t0 = 1000, double noise = 0) => new()
    {
        X = Origin.X + Cos * K * (travel - t0) + noise,
        Y = Origin.Y + Sin * K * (travel - t0) - noise,
        Z = 12,
        TravelMm = travel,
    };

    [Fact]
    public void TwoSamplesGiveTheDirectionOriginAndScale()
    {
        var r = ConveyorCalibration.Compute([Mark(1000), Mark(1200)], null, currentScale: 1.0);
        Assert.Equal(Origin.X, r.Frame.X, 3);
        Assert.Equal(Origin.Y, r.Frame.Y, 3);
        Assert.Equal(12, r.Frame.Z, 3);
        Assert.Equal(Angle, r.Frame.RZ, 3);
        Assert.Equal(K, r.Scale, 5);
        Assert.Equal(0, r.ResidualMm, 3);
        Assert.Null(r.SensorOffsetMm);
    }

    [Fact]
    public void TheNewScaleIsRelativeToTheScaleTheTravelWasReportedWith()
    {
        var r = ConveyorCalibration.Compute([Mark(1000), Mark(1300)], null, currentScale: 1.5);
        Assert.Equal(1.5 * K, r.Scale, 5);
    }

    [Fact]
    public void ACountThatRunsBackwardsGivesANegativeScaleAndKeepsTheDirection()
    {
        // The mark moves forward (+30°) while the reported travel goes down.
        var a = Mark(1000);
        var b = Mark(1200);
        b.TravelMm = 800;
        var r = ConveyorCalibration.Compute([a, b], null);
        Assert.Equal(Angle, r.Frame.RZ, 3);
        Assert.Equal(-K, r.Scale, 5);
    }

    [Fact]
    public void ThreeSamplesFitALineAndReportTheResidual()
    {
        var r = ConveyorCalibration.Compute([Mark(1000), Mark(1100, noise: 0.3), Mark(1250)], null);
        Assert.Equal(Angle, r.Frame.RZ, 0);
        Assert.InRange(r.ResidualMm, 0.05, 0.5);
        Assert.Equal(K, r.Scale, 2);
    }

    [Fact]
    public void TheSensorOffsetIsThePartXLessTheTravelSinceTheLatch()
    {
        // The part was latched at travel 2000 and the belt then ran 150 reported mm (147 real mm);
        // the robot finds it 400 mm along the belt from the origin.
        var s = new ConveyorCalibrationSample
        {
            X = Origin.X + Cos * 400, Y = Origin.Y + Sin * 400, Z = 12, TravelMm = 2150, LatchTravelMm = 2000,
        };
        var r = ConveyorCalibration.Compute([Mark(1000), Mark(1200)], s);
        Assert.Equal(400 - K * 150, r.SensorOffsetMm!.Value, 3);
    }

    [Fact]
    public void UnusableSamplesAreRefused()
    {
        Assert.Throws<ArgumentException>(() => ConveyorCalibration.Compute([Mark(1000)], null));
        var still = Mark(1000);
        Assert.Throws<ArgumentException>(() => ConveyorCalibration.Compute([still, Mark(1000)], null));       // belt did not move
        var moved = new ConveyorCalibrationSample { X = still.X, Y = still.Y, Z = still.Z, TravelMm = 1300 };
        Assert.Throws<ArgumentException>(() => ConveyorCalibration.Compute([still, moved], null));            // mark did not move
        Assert.Throws<ArgumentException>(() => ConveyorCalibration.Compute([Mark(1000), Mark(1200)],
            new ConveyorCalibrationSample { X = 1, Y = 2, TravelMm = 3 }));                                    // no latch travel
    }
}
