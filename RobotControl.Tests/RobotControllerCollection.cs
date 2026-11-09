using Xunit;

namespace RobotControl.Tests;

/// <summary>
/// Test classes that construct a <c>RobotController</c> share this collection so they never run in
/// parallel with each other (or anything else). A RobotController's repositories load/save shared
/// JSON data files in the test working directory (e.g. locals.json), so concurrent construction
/// races on those files; serializing these classes removes that flakiness.
/// </summary>
[CollectionDefinition("RobotController", DisableParallelization = true)]
public sealed class RobotControllerCollection { }
