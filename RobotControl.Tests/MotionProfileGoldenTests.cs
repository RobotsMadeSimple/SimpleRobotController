using System.Runtime.CompilerServices;
using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.MotionProfilers;

namespace RobotControl.Tests;

/// <summary>
/// Pins the trapezoid motion profiles bit for bit. The golden file was captured from the
/// profilers before the S-curve option existed; with the default (trapezoid) profile every
/// sample must still be the identical double. Regenerate only on purpose:
/// <c>GOLDEN_WRITE=1 dotnet test --filter MotionProfileGoldenTests</c>.
/// </summary>
public class MotionProfileGoldenTests
{
    private const string FileName = "motionprofile-trapezoid.golden.json";

    private static string Hex(double d) => BitConverter.DoubleToInt64Bits(d).ToString("X16");

    /// <summary>Sample times: an odd step so phase boundaries are hit off-grid, past the end too.</summary>
    private static IEnumerable<double> Times(double until)
    {
        for (double t = 0; t <= until; t += 0.0137) yield return t;
    }

    internal static Dictionary<string, List<string>> Capture()
    {
        var cases = new Dictionary<string, List<string>>();

        // Normalised scalar profile (MoveL / MoveJ): trapezoid, triangular, asymmetric, start velocity.
        foreach (var (name, v, a, d, v0) in new[]
        {
            ("scalar-trap",      0.5, 2.0, 2.0, 0.0),
            ("scalar-tri",       5.0, 3.0, 3.0, 0.0),
            ("scalar-asym",      0.8, 4.0, 1.5, 0.0),
            ("scalar-start-vel", 0.7, 2.0, 3.0, 0.3),
        })
        {
            var p = new ScalarMotionProfiler();
            p.Setup(v, a, d, v0);
            var list = new List<string>();
            foreach (var t in Times(4.0))
            {
                double s = p.Calc(t, out double vel);
                list.Add(Hex(s));
                list.Add(Hex(vel));
            }
            cases[name] = list;
        }

        // Distance profile (blended paths).
        foreach (var (name, dist, v, a, d) in new[]
        {
            ("distance-trap", 300.0, 100.0, 400.0, 400.0),
            ("distance-tri",   20.0, 200.0, 300.0, 300.0),
            ("distance-asym", 150.0,  80.0, 500.0, 120.0),
        })
        {
            var p = new DistanceScalarProfiler();
            p.Setup(dist, v, a, d);
            cases[name] = Times(4.0).Select(t => Hex(p.Sample(t))).ToList();
        }

        // Vector6 profiler (the whole MoveL/MoveJ sample, interpolated pose).
        foreach (var (name, from, to, v, a, d) in new[]
        {
            ("vector6-line",  new Vector6(100, 50, 200, 0, 0, 10), new Vector6(350, -80, 120, 0, 0, 95), 150.0, 600.0, 600.0),
            ("vector6-joint", new Vector6(10, 300, 250, 0, 0, 0),  new Vector6(95, 410, 180, 0, 0, -45), 90.0, 300.0, 200.0),
        })
        {
            var p = new Vector6MotionProfiler(from, to, v, a, d);
            var list = new List<string>();
            foreach (var t in Times(4.0))
            {
                var q = p.SampleAt(t);
                list.Add(Hex(q.X)); list.Add(Hex(q.Y)); list.Add(Hex(q.Z)); list.Add(Hex(q.RZ));
            }
            cases[name] = list;
        }

        // Blended path: four corners, one radius each, an exact stop at the end.
        {
            var pts = new List<Vector6>
            {
                new(0, 0, 100), new(120, 0, 100), new(120, 90, 100), new(240, 90, 160), new(240, 200, 160),
            };
            var radii = new List<double> { 0, 15, 25, 10, 0 };
            var p = new ContinuousPathingProfiler(pts, radii, 120, 500, 400);
            var list = new List<string>();
            foreach (var t in Times(5.0))
            {
                var q = p.SampleAt(t);
                list.Add(Hex(q.X)); list.Add(Hex(q.Y)); list.Add(Hex(q.Z));
            }
            cases["continuous-blend"] = list;
        }

        return cases;
    }

    private static string SourceDir([CallerFilePath] string path = "") => Path.GetDirectoryName(path)!;

    [Fact]
    public void Trapezoid_profiles_are_bit_identical_to_the_golden_capture()
    {
        var now = Capture();
        if (Environment.GetEnvironmentVariable("GOLDEN_WRITE") == "1")
        {
            File.WriteAllText(Path.Combine(SourceDir(), "Golden", FileName),
                JsonSerializer.Serialize(now, new JsonSerializerOptions { WriteIndented = false }));
            return;
        }

        var path = Path.Combine(AppContext.BaseDirectory, "Golden", FileName);
        Assert.True(File.Exists(path), $"missing {FileName}");
        var golden = JsonSerializer.Deserialize<Dictionary<string, List<string>>>(File.ReadAllText(path))!;

        Assert.Equal(golden.Keys.OrderBy(k => k), now.Keys.OrderBy(k => k));
        foreach (var (name, want) in golden)
        {
            var got = now[name];
            Assert.Equal(want.Count, got.Count);
            for (int i = 0; i < want.Count; i++)
                Assert.True(want[i] == got[i], $"{name}[{i}]: golden {want[i]} != now {got[i]}");
        }
    }
}
