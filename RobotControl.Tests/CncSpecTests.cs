using Controller.RobotControl;

namespace RobotControl.Tests;

public class CncSpecTests
{
    private static CncSpec ContourSpec(double blendRadius = 1) => new()
    {
        SafeZ = 10,
        ActiveZ = -1,
        ActiveSpeed = 40,
        ActiveAccel = 800,
        ActiveDecel = 900,
        TravelSpeed = 150,
        TravelAccel = 1500,
        TravelDecel = 1600,
        BlendRadius = blendRadius,
        Paths = [[0, 0, 20, 0, 20, 20, 0, 20]], // open square path, 4 points
    };

    [Fact]
    public void HolesGenerateApproachAndThreadPairs()
    {
        var spec = new CncSpec
        {
            SafeZ = 5,
            Holes = [new CncHole { X = 1, Y = 2 }, new CncHole { X = 3, Y = 4 }],
            HoleDepth = -12, ThreadPitch = 1.25, HolePeck = true, HolePeckDepth = 3, ThreadReverseOut = true,
        };

        var steps = Controller.RobotControl.ProgramExecutor.GenerateCncSteps(spec);

        Assert.Equal(4, steps.Count);
        Assert.Equal(StepType.MoveL, steps[0].Type);
        Assert.Equal(1, steps[0].OverrideX);
        Assert.Equal(5, steps[0].OverrideZ);
        Assert.Equal(StepType.ThreadMove, steps[1].Type);
        Assert.Equal(-12, steps[1].ThreadDistance);
        Assert.Equal(3, steps[1].ThreadPeckDepth);
        Assert.Equal(StepType.MoveL, steps[2].Type);
        Assert.Equal(3, steps[2].OverrideX);
    }

    [Fact]
    public void ContourGeneratesTravelPlungeActiveRetract()
    {
        var steps = Controller.RobotControl.ProgramExecutor.GenerateCncSteps(ContourSpec());

        // travel + plunge + 3 active moves + retract
        Assert.Equal(6, steps.Count);
        Assert.All(steps, s => Assert.Equal(StepType.MoveL, s.Type));

        // Travel at safe Z to the start point, at travel dynamics
        Assert.Equal(0, steps[0].OverrideX);
        Assert.Equal(10, steps[0].OverrideZ);
        Assert.Equal(150, steps[0].Speed);
        Assert.Equal(1500, steps[0].Accel);
        Assert.Equal(1600, steps[0].Decel);
        Assert.Null(steps[0].Blend);

        // Plunge to active Z at active speed/accel/decel
        Assert.Equal(-1, steps[1].OverrideZ);
        Assert.Equal(40, steps[1].Speed);
        Assert.Equal(800, steps[1].Accel);
        Assert.Equal(900, steps[1].Decel);
        Assert.Null(steps[1].Blend);

        // Active moves are blended at active Z with the active dynamics
        foreach (var mv in steps.Skip(2).Take(3))
        {
            Assert.Equal(-1, mv.OverrideZ);
            Assert.Equal(40, mv.Speed);
            Assert.Equal(800, mv.Accel);
            Assert.Equal(900, mv.Decel);
            Assert.True(mv.Blend);
            Assert.Equal(1, mv.BlendRadius);
        }
        Assert.Equal(0, steps[4].OverrideX);
        Assert.Equal(20, steps[4].OverrideY);

        // Retract is unblended so the continuous run stops on the last point,
        // and lifts away at travel dynamics
        Assert.Equal(10, steps[5].OverrideZ);
        Assert.Equal(0, steps[5].OverrideX);
        Assert.Equal(20, steps[5].OverrideY);
        Assert.Equal(150, steps[5].Speed);
        Assert.Null(steps[5].Blend);
    }

    [Fact]
    public void ZeroBlendRadiusDisablesBlending()
    {
        var steps = Controller.RobotControl.ProgramExecutor.GenerateCncSteps(ContourSpec(blendRadius: 0));
        Assert.All(steps, s => Assert.Null(s.Blend));
        Assert.All(steps, s => Assert.Null(s.BlendRadius));
    }

    [Fact]
    public void HolesAndContoursCombineInOneSpec()
    {
        var spec = ContourSpec();
        spec.Holes = [new CncHole { X = 50, Y = 50 }];
        spec.HoleDepth = -5;

        var steps = Controller.RobotControl.ProgramExecutor.GenerateCncSteps(spec);

        // 2 hole steps first, then 6 contour steps
        Assert.Equal(8, steps.Count);
        Assert.Equal(StepType.ThreadMove, steps[1].Type);
        Assert.Equal("Contour 1 (4 pts)", steps[2].Name);
    }

    [Fact]
    public void CountStepsMatchesGeneratedCount()
    {
        var spec = ContourSpec();
        spec.Holes = [new CncHole { X = 1, Y = 1 }];
        var block = new ProgramStep { Id = "cnc", Type = StepType.CncProgram, CncSpec = spec };

        int counted = Controller.RobotControl.ProgramExecutor.CountSteps([block]);
        int generated = Controller.RobotControl.ProgramExecutor.GenerateCncSteps(spec).Count;

        // +1 for the CncProgram block itself
        Assert.Equal(generated + 1, counted);
    }

    [Fact]
    public void SpecExpressionsAttachToGeneratedSteps()
    {
        var spec = ContourSpec();
        spec.Expressions = new()
        {
            ["safeZ"] = "$clear",
            ["activeZ"] = "$z",
            ["activeSpeed"] = "$feed",
            ["blendRadius"] = "$r",
            ["travelSpeed"] = "$rapid",
        };

        var steps = Controller.RobotControl.ProgramExecutor.GenerateCncSteps(spec);

        // Travel: safeZ→overrideZ, travelSpeed→speed
        Assert.Equal("$clear", steps[0].Expressions!["overrideZ"]);
        Assert.Equal("$rapid", steps[0].Expressions!["speed"]);
        // Plunge: activeZ→overrideZ, activeSpeed→speed, no blend expr
        Assert.Equal("$z", steps[1].Expressions!["overrideZ"]);
        Assert.Equal("$feed", steps[1].Expressions!["speed"]);
        Assert.False(steps[1].Expressions!.ContainsKey("blendRadius"));
        // Active moves additionally carry the blend radius expression
        Assert.Equal("$r", steps[2].Expressions!["blendRadius"]);
        // Retract uses the travel expressions again
        Assert.Equal("$clear", steps[5].Expressions!["overrideZ"]);
    }

    [Fact]
    public void BlendRadiusExpressionEnablesBlendFlag()
    {
        var spec = ContourSpec(blendRadius: 0);
        spec.Expressions = new() { ["blendRadius"] = "$r" };
        var steps = Controller.RobotControl.ProgramExecutor.GenerateCncSteps(spec);
        // Blend must be on so the radius expression gets evaluated at run time
        Assert.True(steps[2].Blend);
    }

    [Fact]
    public void DrillGeneratesPeckPassesFromSafeZ()
    {
        var spec = new CncSpec
        {
            SafeZ = 5,
            Holes = [new CncHole { X = 10, Y = 20 }],
            HoleOp = "drill",
            HoleDepth = -12,
            HolePeck = true,
            HolePeckDepth = 5,
            ActiveSpeed = 30,
            TravelSpeed = 100,
        };

        var steps = Controller.RobotControl.ProgramExecutor.GenerateCncSteps(spec);

        // approach + 3 passes (5, 10, 12) × (plunge + retract) = 7 steps
        Assert.Equal(7, steps.Count);
        Assert.All(steps, s => Assert.Equal(StepType.MoveL, s.Type));

        // Depth measured from safe Z, like ThreadMove: safeZ 5 + (-5, -10, -12)
        Assert.Equal(0, steps[1].OverrideZ);
        Assert.Equal(5, steps[2].OverrideZ);   // retract
        Assert.Equal(-5, steps[3].OverrideZ);
        Assert.Equal(-7, steps[5].OverrideZ);  // final pass to full depth
        Assert.Equal(5, steps[6].OverrideZ);

        // Plunge at active dynamics, retract at travel dynamics
        Assert.Equal(30, steps[1].Speed);
        Assert.Equal(100, steps[2].Speed);
    }

    [Fact]
    public void DrillWithoutPeckIsSinglePlunge()
    {
        var spec = new CncSpec
        {
            SafeZ = 5,
            Holes = [new CncHole { X = 0, Y = 0 }],
            HoleOp = "drill",
            HoleDepth = -10,
        };
        var steps = Controller.RobotControl.ProgramExecutor.GenerateCncSteps(spec);
        Assert.Equal(3, steps.Count); // approach + plunge + retract
        Assert.Equal(-5, steps[1].OverrideZ);
        Assert.Equal(5, steps[2].OverrideZ);
    }

    [Fact]
    public void DrillStructuralExpressionResolvesAtBlockStart()
    {
        var spec = new CncSpec
        {
            SafeZ = 0,
            Holes = [new CncHole { X = 0, Y = 0 }],
            HoleOp = "drill",
            HoleDepth = -3,
            Expressions = new() { ["holeDepth"] = "$d" },
        };
        // Resolver supplies the runtime value of $d = -10
        var steps = Controller.RobotControl.ProgramExecutor.GenerateCncSteps(
            spec, null, (key, fallback) => key == "holeDepth" ? -10 : fallback);
        Assert.Equal(3, steps.Count);
        Assert.Equal(-10, steps[1].OverrideZ);
    }

    [Fact]
    public void CountStepsMatchesDrillGeneration()
    {
        var spec = new CncSpec
        {
            SafeZ = 5,
            Holes = [new CncHole { X = 1, Y = 1 }, new CncHole { X = 2, Y = 2 }],
            HoleOp = "drill",
            HoleDepth = -12,
            HolePeck = true,
            HolePeckDepth = 5,
        };
        var block = new ProgramStep { Id = "cnc", Type = StepType.CncProgram, CncSpec = spec };
        int counted   = Controller.RobotControl.ProgramExecutor.CountSteps([block]);
        int generated = Controller.RobotControl.ProgramExecutor.GenerateCncSteps(spec).Count;
        Assert.Equal(generated + 1, counted);
    }

    [Fact]
    public void ThreadExpressionsMapToThreadFields()
    {
        var spec = new CncSpec
        {
            SafeZ = 5,
            Holes = [new CncHole { X = 1, Y = 2 }],
            HoleDepth = -12,
            Expressions = new() { ["holeDepth"] = "$depth", ["threadPitch"] = "$pitch" },
        };
        var steps = Controller.RobotControl.ProgramExecutor.GenerateCncSteps(spec);
        Assert.Equal("$depth", steps[1].Expressions!["threadDistance"]);
        Assert.Equal("$pitch", steps[1].Expressions!["threadPitch"]);
    }

    [Fact]
    public void CurrentOriginAnchorShiftsAllCoordinates()
    {
        var spec = ContourSpec();
        spec.Holes = [new CncHole { X = 1, Y = 2 }];
        spec.HoleDepth = -3;
        spec.Expressions = new() { ["activeZ"] = "$z" };
        var anchor = new Vector6(100, 50, 20, 0, 0, 0);

        var steps = Controller.RobotControl.ProgramExecutor.GenerateCncSteps(spec, anchor);

        // Hole approach shifted by the anchor
        Assert.Equal(101, steps[0].OverrideX);
        Assert.Equal(52, steps[0].OverrideY);
        Assert.Equal(30, steps[0].OverrideZ);    // safeZ 10 + anchor 20

        // Contour travel/plunge/active shifted (contour steps start at index 2)
        Assert.Equal(100, steps[2].OverrideX);   // 0 + 100
        Assert.Equal(30, steps[2].OverrideZ);
        Assert.Equal(19, steps[3].OverrideZ);    // activeZ -1 + 20
        Assert.Equal(120, steps[4].OverrideX);   // 20 + 100

        // Z expressions are wrapped so they stay anchor-relative
        Assert.Equal("($z) + 20", steps[3].Expressions!["overrideZ"]);
    }

    [Fact]
    public void DegeneratePathsAreSkipped()
    {
        var spec = new CncSpec { SafeZ = 5, Paths = [[1, 2], [], null!] };
        var steps = Controller.RobotControl.ProgramExecutor.GenerateCncSteps(spec);
        Assert.Empty(steps);
    }

    [Fact]
    public void ResumeIndexNeverGoesBackwards()
    {
        // Path: start (0,0,0) → wp0 (100,0,0) → wp1 (100,100,0)
        var start = new Vector6(0, 0, 0, 0, 0, 0);
        var wps = new List<Vector6>
        {
            new(100, 0, 0, 0, 0, 0),
            new(100, 100, 0, 0, 0, 0),
        };

        // Stopped halfway along the LONG first leg → resume toward wp0, not back to start
        Assert.Equal(0, Controller.RobotControl.ProgramExecutor.FindResumeIndex(new Vector6(50, 0.1, 0, 0, 0, 0), start, wps));
        // Stopped halfway up the second leg → resume toward wp1
        Assert.Equal(1, Controller.RobotControl.ProgramExecutor.FindResumeIndex(new Vector6(100, 50, 0, 0, 0, 0), start, wps));
        // Stopped just before the corner (still on leg 0) → resume toward wp0
        Assert.Equal(0, Controller.RobotControl.ProgramExecutor.FindResumeIndex(new Vector6(99, 0.5, 0, 0, 0, 0), start, wps));
        // Stopped right at the start → first leg, resume toward wp0
        Assert.Equal(0, Controller.RobotControl.ProgramExecutor.FindResumeIndex(start, start, wps));
        // Stopped at the very end → last waypoint
        Assert.Equal(1, Controller.RobotControl.ProgramExecutor.FindResumeIndex(new Vector6(100, 100, 0, 0, 0, 0), start, wps));
    }
}
