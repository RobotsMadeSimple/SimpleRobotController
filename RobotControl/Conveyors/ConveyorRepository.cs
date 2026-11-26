using System.Text.RegularExpressions;
using Controller.RobotControl.Persistence;

namespace Controller.RobotControl.Conveyors
{
    /// <summary>
    /// The configured conveyors (conveyors.json), keyed by stable id. Ids are generated as
    /// <c>c</c> + 12 hex digits — identifier characters only, so an id can be written in an
    /// expression (<c>$conveyor.c3fa9b1e2c4d.travel</c>).
    /// </summary>
    public sealed partial class ConveyorRepository : JsonListRepository<Conveyor>
    {
        public ConveyorRepository(string file = "conveyors.json") : base(file) { }

        [GeneratedRegex("^[A-Za-z0-9_]+$")]
        private static partial Regex IdPattern();

        /// <summary>Can <paramref name="id"/> be used in <c>$conveyor.&lt;id&gt;.…</c>?</summary>
        public static bool IsValidId(string? id) => !string.IsNullOrEmpty(id) && IdPattern().IsMatch(id);

        /// <summary>A new unique id.</summary>
        public string NewId()
        {
            while (true)
            {
                var id = "c" + Guid.NewGuid().ToString("N")[..12];
                if (Get(id) == null) return id;
            }
        }

        /// <summary>By id, exact first, then ignoring case (expressions are case-insensitive).</summary>
        public Conveyor? Find(string? id)
        {
            if (string.IsNullOrEmpty(id)) return null;
            return Get(id) ?? GetAll().FirstOrDefault(c => string.Equals(c.Id, id, StringComparison.OrdinalIgnoreCase));
        }

        /// <summary>Saves <paramref name="c"/>; an empty id gets a new one. Returns the stored item.</summary>
        public Conveyor Save(Conveyor c)
        {
            if (string.IsNullOrWhiteSpace(c.Id)) c.Id = NewId();
            return Upsert(c);
        }
    }
}
