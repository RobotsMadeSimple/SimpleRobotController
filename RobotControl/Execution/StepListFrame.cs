namespace Controller.RobotControl.Execution
{
    /// <summary>What a step list on the frame stack is, which decides what happens when it runs out.</summary>
    internal enum FrameKind
    {
        /// <summary>The program body, an if/else branch or a routine: popped when done.</summary>
        Plain,
        /// <summary>A CNC block's generated steps: popping it clears the toolpath preview.</summary>
        Cnc,
        /// <summary>A counted loop body (count 0 = infinite): re-pushed until the count runs out.</summary>
        CountLoop,
        /// <summary>A while-loop body: re-pushed while its condition holds.</summary>
        WhileLoop,
        /// <summary>A for-each body: re-pushed once per element of the source list.</summary>
        ForEach,
    }

    /// <summary>
    /// One step list being executed — the program body, a loop body, a branch, a routine or
    /// a CNC block — and the executor's position in it. Created through the named factories;
    /// only the fields of its <see cref="Kind"/> are meaningful.
    /// </summary>
    internal sealed class StepListFrame
    {
        private StepListFrame(FrameKind kind, List<ProgramStep> steps)
        {
            Kind  = kind;
            Steps = steps;
        }

        public FrameKind         Kind  { get; }
        public List<ProgramStep> Steps { get; }
        public int               Index { get; set; }

        /// <summary>Loop frames count toward <see cref="FrameStack.LoopDepth"/>.</summary>
        public bool IsLoop => Kind is FrameKind.CountLoop or FrameKind.WhileLoop or FrameKind.ForEach;

        // ── Multi-tick step state ─────────────────────────────────────────────
        // Used by Wait, blocking SetOutput pulses, HttpRequest and HttpReceive. Only the step
        // at Index uses it, and that step holds the frame until it finishes.
        public bool WaitStarted { get; set; }
        public long WaitStartMs { get; set; }

        // ── Loop state ────────────────────────────────────────────────────────

        /// <summary>CountLoop / ForEach: variable receiving the 0-based iteration ("" = none).</summary>
        public string IndexVar { get; private init; } = "";

        /// <summary>CountLoop: iterations still to run including this one (int.MaxValue = infinite).</summary>
        public int LoopRemaining { get; set; }
        /// <summary>CountLoop: total iterations (int.MaxValue = infinite).</summary>
        public int LoopTotal { get; private init; }

        /// <summary>WhileLoop: the condition re-checked after each pass.</summary>
        public ConditionGroup? WhileCondition { get; private init; }

        /// <summary>ForEach: element count, current element, the list and the value variable.</summary>
        public int    ForEachCount        { get; private init; }
        public int    ForEachCurrentIndex { get; set; }
        public string ForEachSourceVar    { get; private init; } = "";
        public string ForEachValueVar     { get; private init; } = "";

        // ── Factories ─────────────────────────────────────────────────────────

        public static StepListFrame Plain(List<ProgramStep> steps) => new(FrameKind.Plain, steps);

        public static StepListFrame Cnc(List<ProgramStep> steps) => new(FrameKind.Cnc, steps);

        public static StepListFrame CountLoop(List<ProgramStep> steps, int remaining, int total, string indexVar) =>
            new(FrameKind.CountLoop, steps) { LoopRemaining = remaining, LoopTotal = total, IndexVar = indexVar };

        public static StepListFrame WhileLoop(List<ProgramStep> steps, ConditionGroup condition) =>
            new(FrameKind.WhileLoop, steps) { WhileCondition = condition };

        public static StepListFrame ForEach(List<ProgramStep> steps, int count, int currentIndex,
                                            string sourceVar, string valueVar, string indexVar) =>
            new(FrameKind.ForEach, steps)
            {
                ForEachCount        = count,
                ForEachCurrentIndex = currentIndex,
                ForEachSourceVar    = sourceVar,
                ForEachValueVar     = valueVar,
                IndexVar            = indexVar,
            };

        /// <summary>A fresh frame for this loop's next pass (same kind and settings, index 0).</summary>
        public StepListFrame NextPass() => Kind switch
        {
            FrameKind.CountLoop => CountLoop(Steps, LoopRemaining, LoopTotal, IndexVar),
            FrameKind.WhileLoop => WhileLoop(Steps, WhileCondition!),
            FrameKind.ForEach   => ForEach(Steps, ForEachCount, ForEachCurrentIndex,
                                           ForEachSourceVar, ForEachValueVar, IndexVar),
            _ => throw new InvalidOperationException($"{Kind} frame has no next pass"),
        };
    }

    /// <summary>
    /// The executor's stack of step lists. Keeps <see cref="LoopDepth"/> — the number of loop
    /// frames on the stack, which decides whether a completed step counts toward overall
    /// progress — in step with every push and pop, so no caller maintains it by hand.
    /// </summary>
    internal sealed class FrameStack
    {
        private readonly Stack<StepListFrame> _stack = new();

        public int Count     => _stack.Count;
        public int LoopDepth { get; private set; }

        public StepListFrame Peek() => _stack.Peek();

        public void Push(StepListFrame frame)
        {
            _stack.Push(frame);
            if (frame.IsLoop) LoopDepth++;
        }

        public StepListFrame Pop()
        {
            var frame = _stack.Pop();
            if (frame.IsLoop) LoopDepth--;
            return frame;
        }

        /// <summary>Innermost first — index 0 is the current frame.</summary>
        public StepListFrame[] ToArray() => _stack.ToArray();

        public void Clear()
        {
            _stack.Clear();
            LoopDepth = 0;
        }
    }
}
