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
        public void RemoveProgram(string name)
        {
            lock (_lock) { _programs.Remove(name); }
        }

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

                if (update.ProgramStatus.HasValue)
                {
                    var incoming        = update.ProgramStatus.Value;
                    var isProgressOnly  = incoming == ProgramStatus.Running || incoming == ProgramStatus.Starting || incoming == ProgramStatus.Finishing;
                    var isTerminal      = program.Status == ProgramStatus.Stopped || program.Status == ProgramStatus.Complete || program.Status == ProgramStatus.Error;
                    if (!isProgressOnly || !isTerminal)
                        program.Status  = incoming;
                }
                if (update.CurrentStepNumber.HasValue) program.CurrentStepNumber    = update.CurrentStepNumber.Value;
                if (update.MaxStepCount.HasValue)      program.MaxStepCount         = update.MaxStepCount.Value;
                if (update.ErrorDescription      != null)  program.ErrorDescription     = update.ErrorDescription;
                if (update.WarningDescription    != null)  program.WarningDescription   = update.WarningDescription;
                if (update.CurrentPointName != null)
                {
                    program.CurrentPointName = update.CurrentPointName;
                    // Clear all offsets so the new move starts clean — HasValue checks below re-apply any that are configured
                    program.CurrentOffsetX = program.CurrentOffsetY = program.CurrentOffsetZ = null;
                    program.CurrentOffsetRX = program.CurrentOffsetRY = program.CurrentOffsetRZ = null;
                    program.CurrentToolOffsetX = program.CurrentToolOffsetY = program.CurrentToolOffsetZ = null;
                    program.CurrentToolOffsetRX = program.CurrentToolOffsetRY = program.CurrentToolOffsetRZ = null;
                }
                if (update.CurrentOffsetX     .HasValue) program.CurrentOffsetX    = update.CurrentOffsetX;
                if (update.CurrentOffsetY     .HasValue) program.CurrentOffsetY    = update.CurrentOffsetY;
                if (update.CurrentOffsetZ     .HasValue) program.CurrentOffsetZ    = update.CurrentOffsetZ;
                if (update.CurrentOffsetRX    .HasValue) program.CurrentOffsetRX   = update.CurrentOffsetRX;
                if (update.CurrentOffsetRY    .HasValue) program.CurrentOffsetRY   = update.CurrentOffsetRY;
                if (update.CurrentOffsetRZ    .HasValue) program.CurrentOffsetRZ   = update.CurrentOffsetRZ;
                if (update.CurrentToolOffsetX .HasValue) program.CurrentToolOffsetX  = update.CurrentToolOffsetX;
                if (update.CurrentToolOffsetY .HasValue) program.CurrentToolOffsetY  = update.CurrentToolOffsetY;
                if (update.CurrentToolOffsetZ .HasValue) program.CurrentToolOffsetZ  = update.CurrentToolOffsetZ;
                if (update.CurrentToolOffsetRX.HasValue) program.CurrentToolOffsetRX = update.CurrentToolOffsetRX;
                if (update.CurrentToolOffsetRY.HasValue) program.CurrentToolOffsetRY = update.CurrentToolOffsetRY;
                if (update.CurrentToolOffsetRZ.HasValue) program.CurrentToolOffsetRZ = update.CurrentToolOffsetRZ;

                if (!string.IsNullOrEmpty(update.StepDescription))
                {
                    program.CurrentStepDescription = update.StepDescription;
                    if (update.ShouldLog)
                    {
                        program.StepLogs.Add(update.StepDescription);
                        if (program.StepLogs.Count > 5000)
                        {
                            program.StepLogs.RemoveAt(0);
                            program.LogBaseIndex++;   // keep absolute paging stable
                        }
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
        /// Resets any programs in the supplied list that are Stopped or Complete back to Ready.
        /// Programs that are actively Running/Starting are left untouched.
        /// Called when a new built program starts so stale terminal states are cleared.
        /// </summary>
        public void ResetTerminatedToReady(IEnumerable<(string name, int maxStepCount)> programs)
        {
            lock (_lock)
            {
                foreach (var (name, maxSteps) in programs)
                {
                    if (!_programs.TryGetValue(name, out var p)) continue;
                    if (p.Status != ProgramStatus.Stopped && p.Status != ProgramStatus.Complete) continue;

                    p.Status                 = ProgramStatus.Ready;
                    p.CurrentStepNumber      = 0;
                    p.MaxStepCount           = maxSteps;
                    p.CurrentStepDescription = "";
                    p.ErrorDescription       = "";
                    p.WarningDescription     = "";
                }
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
                    currentPointName    = p.CurrentPointName,
                    currentOffsetX      = p.CurrentOffsetX,
                    currentOffsetY      = p.CurrentOffsetY,
                    currentOffsetZ      = p.CurrentOffsetZ,
                    currentOffsetRX     = p.CurrentOffsetRX,
                    currentOffsetRY     = p.CurrentOffsetRY,
                    currentOffsetRZ     = p.CurrentOffsetRZ,
                    currentToolOffsetX  = p.CurrentToolOffsetX,
                    currentToolOffsetY  = p.CurrentToolOffsetY,
                    currentToolOffsetZ  = p.CurrentToolOffsetZ,
                    currentToolOffsetRX = p.CurrentToolOffsetRX,
                    currentToolOffsetRY = p.CurrentToolOffsetRY,
                    currentToolOffsetRZ = p.CurrentToolOffsetRZ,
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
        /// Returns a slice of a program's logs using a half-open [start, end) range of
        /// ABSOLUTE indices — indices count every entry ever produced, even ones the
        /// ring buffer has since dropped. Omit start to read from the oldest retained
        /// entry; omit end to read to the current tail. Designed for lazy loading:
        /// the client passes back the returned total as the next start to pull only
        /// new entries, and paging keeps working after the buffer wraps past its cap.
        /// </summary>
        /// <returns>(total, start, logs): total = absolute count ever produced,
        /// start = absolute index of the first returned entry.</returns>
        public (int total, int start, List<string> logs) GetProgramLogs(string programName, int? start, int? end)
        {
            lock (_lock)
            {
                if (!_programs.TryGetValue(programName, out var program))
                    return (0, 0, new List<string>());

                var logs    = program.StepLogs;
                int baseIdx = program.LogBaseIndex;
                int total   = baseIdx + logs.Count;

                // Clamp start up to the oldest entry we still retain, so a client that
                // fell behind the ring buffer resumes from the newest available window.
                int s = Math.Clamp(start ?? 0,     baseIdx, total);
                int e = Math.Clamp(end   ?? total, s,       total);
                return (total, s, logs.GetRange(s - baseIdx, e - s));
            }
        }

        /// <summary>Total number of log entries ever produced for a program (absolute).</summary>
        public int GetLogCount(string programName)
        {
            lock (_lock)
            {
                return _programs.TryGetValue(programName, out var p)
                    ? p.LogBaseIndex + p.StepLogs.Count
                    : 0;
            }
        }
    }
}
