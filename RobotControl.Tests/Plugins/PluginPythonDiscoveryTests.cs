using System.Runtime.InteropServices;
using Controller.RobotControl.Plugins;

namespace RobotControl.Tests.Plugins;

/// <summary>Finding a Python interpreter when PATH does not carry one (service launches).</summary>
public class PluginPythonDiscoveryTests
{
    [Theory]
    [InlineData("Python312", "3.12")]
    [InlineData("Python3.11", "3.11")]
    [InlineData("Python39-32", "3.9")]
    [InlineData("python313", "3.13")]
    public void PythonFolderVersion_parses_install_folder_names(string folder, string expected) =>
        Assert.Equal(Version.Parse(expected), PluginProcessLauncher.PythonFolderVersion(folder));

    [Theory]
    [InlineData("Python")]
    [InlineData("Launcher")]
    [InlineData("Python3")]
    [InlineData("MyPython312")]
    public void PythonFolderVersion_rejects_other_folders(string folder) =>
        Assert.Null(PluginProcessLauncher.PythonFolderVersion(folder));

    [Fact]
    public void FindPythonInStandardFolders_prefers_the_newest_version_and_skips_missing_roots()
    {
        string root = Path.Combine(Path.GetTempPath(), "src-py-" + Guid.NewGuid().ToString("N"));
        try
        {
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
            {
                Directory.CreateDirectory(Path.Combine(root, "Python311"));
                Directory.CreateDirectory(Path.Combine(root, "Python313"));
                Directory.CreateDirectory(Path.Combine(root, "Python39"));
                File.WriteAllText(Path.Combine(root, "Python311", "python.exe"), "");
                File.WriteAllText(Path.Combine(root, "Python39",  "python.exe"), "");
                // Python313 has no python.exe (half-removed install) → the next newest wins.
                var found = PluginProcessLauncher.FindPythonInStandardFolders(
                    [Path.Combine(root, "does-not-exist"), root]);
                Assert.Equal(Path.Combine(root, "Python311", "python.exe"), found);
            }
            else
            {
                Directory.CreateDirectory(root);
                File.WriteAllText(Path.Combine(root, "python3"), "");
                var found = PluginProcessLauncher.FindPythonInStandardFolders(
                    [Path.Combine(root, "does-not-exist"), root]);
                Assert.Equal(Path.Combine(root, "python3"), found);
            }
        }
        finally
        {
            if (Directory.Exists(root)) Directory.Delete(root, true);
        }
    }

    [Fact]
    public void FindPythonInStandardFolders_returns_null_when_nothing_matches()
    {
        string root = Path.Combine(Path.GetTempPath(), "src-py-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(root);
        try { Assert.Null(PluginProcessLauncher.FindPythonInStandardFolders([root])); }
        finally { Directory.Delete(root, true); }
    }
}
