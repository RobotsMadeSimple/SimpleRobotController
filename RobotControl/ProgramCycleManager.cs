using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl
{
    /// <summary>
    /// Thread-safe store for all program instances known to the robot controller.
    /// The external control program owns the lifecycle (create / update / run);
    /// the mobile app reads state and sets action flags.
    /// </summary>
    internal class ProgramCycleManager
    {
        private readonly Dictionary<string, ProgramModel> _programs = new();
        private readonly object _lock = new();

        // ── Registration ─────────────────────────────────────────────────────

        /// <summary>
        /// Upserts programs from a SetAvailablePrograms command.
        /// For existing programs only non-null fields are overwritten so that
        /// live status is not clobbered when the external program re-registers.
        /// </summary>
        public void SetAvailablePrograms(List<ProgramUpdateParams> programs)
        {
            lock (_lock)
            {
                foreach (var p in programs)
                {
                    if (_programs.TryGetValue(p.Name, out var existing))
                    {
                        if (p.Description != null) existing.Description = p.Description;
                        if (p.Image        != null) existing.Image       = Convert.FromBase64String(p.Image);
                    }
                    else
                    {
                        _programs[p.Name] = new ProgramModel
                        {
                            Name        = p.Name,
                            Description = p.Description ?? "",
                            Image       = p.Image != null ? Convert.FromBase64String(p.Image) : null
                        };
                    }
                }
            }
        }

        // ── Status updates ───────────────────────────────────────────────────

        /// <summary>
        /// Applies a sparse status update to the named program.
        /// Only non-null / non-empty fields are written. A non-empty
        /// StepDescription is also appended to the program's log (capped at 5 000 entries).
        /// </summary>
        public void ApplyStatusUpdate(ProgramCycleUpdate update)
        {
            lock (_lock)
            {
                if (!_programs.TryGetValue(update.ProgramName, out var program))
                    return;

                if (update.ProgramStatus.HasValue)     program.Status               = update.ProgramStatus.Value;
                if (update.CurrentStepNumber.HasValue) program.CurrentStepNumber    = update.CurrentStepNumber.Value;
                if (update.MaxStepCount.HasValue)      program.MaxStepCount         = update.MaxStepCount.Value;
                if (update.ErrorDescription   != null) program.ErrorDescription     = update.ErrorDescription;
                if (update.WarningDescription != null) program.WarningDescription   = update.WarningDescription;

                if (!string.IsNullOrEmpty(update.StepDescription))
                {
                    program.CurrentStepDescription = update.StepDescription;
                    if (update.ShouldLog)
                    {
                        program.StepLogs.Add(update.StepDescription);
                        if (program.StepLogs.Count > 5000)
                            program.StepLogs.RemoveAt(0);
                    }
                }
            }
        }

        // ── Action flags ─────────────────────────────────────────────────────

        /// <summary>
        /// Sets one action flag on the named program so the external control
        /// program can react on its next poll.
        /// </summary>
        public void SetFlag(string programName, string flag)
        {
            lock (_lock)
            {
                if (!_programs.TryGetValue(programName, out var program))
                    return;

                switch (flag)
                {
                    case "Start": program.Start = true; break;
                    case "Stop":  program.Stop  = true; break;
                    case "Reset": program.Reset = true; break;
                    case "Abort": program.Abort = true; break;
                }
            }
        }

        /// <summary>
        /// Fully resets a program back to the Ready state, clearing all progress,
        /// descriptions, and error/warning fields. Used by built-program Reset and Abort.
        /// </summary>
        public void ResetToReady(string programName, int maxStepCount)
        {
            lock (_lock)
            {
                if (!_programs.TryGetValue(programName, out var program))
                    return;

                program.Status                 = ProgramStatus.Ready;
                program.CurrentStepNumber      = 0;
                program.MaxStepCount           = maxStepCount;
                program.CurrentStepDescription = "";
                program.ErrorDescription       = "";
                program.WarningDescription     = "";
            }
        }

        /// <summary>
        /// Clears all four action flags on the named program so the external
        /// control program can signal it has consumed them.
        /// </summary>
        public void ClearActions(string programName)
        {
            lock (_lock)
            {
                if (!_programs.TryGetValue(programName, out var program))
                    return;

                program.Start = false;
                program.Stop  = false;
                program.Reset = false;
                program.Abort = false;
            }
        }

        // ── Queries ──────────────────────────────────────────────────────────

        /// <summary>
        /// Returns a summary of every program — excludes StepLogs and Image bytes
        /// so that the robot status broadcast stays small.
        /// </summary>
        public List<object> GetProgramsSummary()
        {
            lock (_lock)
            {
                return _programs.Values.Select(p => (object)new
                {
                    name                   = p.Name,
                    description            = p.Description,
                    status                 = p.Status.ToString(),
                    currentStepDescription = p.CurrentStepDescription,
                    currentStepNumber      = p.CurrentStepNumber,
                    maxStepCount           = p.MaxStepCount,
                    errorDescription       = p.ErrorDescription,
                    warningDescription     = p.WarningDescription,
                    start                  = p.Start,
                    stop                   = p.Stop,
                    reset                  = p.Reset,
                    abort                  = p.Abort
                }).ToList();
            }
        }

        /// <summary>
        /// Returns a map of programName → base-64 image string (null if the program
        /// has no image set) for all registered programs.
        /// </summary>
        public Dictionary<string, string?> GetAllImages()
        {
            lock (_lock)
            {
                return _programs.ToDictionary(
                    kv => kv.Key,
                    kv => kv.Value.Image != null ? Convert.ToBase64String(kv.Value.Image) : null
                );
            }
        }

        /// <summary>
        /// Returns a slice of a program's log list using a half-open [start, end) range.
        /// Omit start to read from the beginning; omit end to read to the current tail.
        /// Designed for lazy loading — the client can track the last index it received
        /// and pass it as the next start to pull only new entries.
        /// </summary>
        public List<string> GetProgramLogs(string programName, int? start, int? end)
        {
            lock (_lock)
            {
                if (!_programs.TryGetValue(programName, out var program))
                    return new List<string>();

                var logs = program.StepLogs;
                int s = Math.Clamp(start ?? 0,          0, logs.Count);
                int e = Math.Clamp(end   ?? logs.Count, s, logs.Count);
                return logs.GetRange(s, e - s);
            }
        }

        /// <summary>Returns the total number of log entries for a program (useful for paging).</summary>
        public int GetLogCount(string programName)
        {
            lock (_lock)
            {
                return _programs.TryGetValue(programName, out var p) ? p.StepLogs.Count : 0;
            }
        }
    }
}
