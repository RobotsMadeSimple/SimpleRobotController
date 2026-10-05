using Controller.RobotControl.Plugins;

namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// A plugin function call failed (<c>pluginNotRunning</c>, <c>pluginFunctionTimeout</c>,
    /// <c>pluginFunctionFailed</c>, <c>unknownPluginFunction</c>). Derives from
    /// <see cref="ExpressionParseException"/> so every evaluation path that already turns an
    /// expression error into a program error (step dispatch, while conditions, variable
    /// initialisers, computed variables) reports it too, with the plugin's message.
    /// </summary>
    public sealed class PluginFunctionCallException : ExpressionParseException
    {
        public PluginFunctionCallException(string code, string message) : base(message, -1, code) { }
    }

    /// <summary>
    /// Exposes every installed plugin's expression functions to the evaluator as
    /// <c>id.name(…)</c> (docs/plugins.md §5). Resolution reads the manager's live state on each
    /// call, so installing, reloading or a manifest override in <c>plugin.ready</c> is seen at
    /// once. Calls block the evaluating thread up to the function's <c>timeoutMs</c>.
    /// </summary>
    internal sealed class PluginExpressionFunctions : IDynamicFunctionProvider
    {
        private readonly Func<PluginManager?> _manager;

        public PluginExpressionFunctions(Func<PluginManager?> manager) => _manager = manager;

        public PluginExpressionFunctions(PluginManager manager) : this(() => manager) { }

        public ExpressionFunction? Resolve(string fullName)
        {
            int dot = fullName.IndexOf('.');
            if (dot <= 0 || dot == fullName.Length - 1) return null;
            var host = _manager()?.Get(fullName[..dot]);
            if (host is null || host.Problems.Count > 0 || host.Manifest is not { } m) return null;
            string name = fullName[(dot + 1)..];
            var f = m.Functions.FirstOrDefault(x => string.Equals(x.Name, name, StringComparison.OrdinalIgnoreCase));
            if (f is null) return null;
            string full = $"{host.Id}.{f.Name}";
            return ExpressionFunction.Dynamic(full, f.MinArgs, f.EffectiveMaxArgs,
                string.IsNullOrWhiteSpace(f.Signature) ? $"{full}(…)" : f.Signature!, f.Description ?? "", host.Id);
        }

        public bool IsNamespace(string root) =>
            _manager()?.Get(root) is { Problems.Count: 0, Manifest: not null };

        public double Call(ExpressionFunction fn, ReadOnlySpan<double> args)
        {
            var manager = _manager() ?? throw new PluginFunctionCallException("pluginNotRunning", $"Plugins are not available for {fn.Name}()");
            try
            {
                return manager.CallFunction(fn.Name, args);
            }
            catch (PluginFunctionException ex)
            {
                throw new PluginFunctionCallException(ex.Code, ex.Message);
            }
        }
    }
}
