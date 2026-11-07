namespace Controller.RobotControl.Commands;

/// <summary>
/// The text language (SRS) commands (docs/program-language.md §6.9). Package A registers the
/// names; package D fills them in:
/// <list type="bullet">
/// <item><c>FormatProgramText { program, includeIds? }</c> → <see cref="Language.FormatProgramTextResult"/></item>
/// <item><c>ParseProgramText { text, base?, validate? }</c> → <see cref="Language.ParseProgramTextResult"/></item>
/// </list>
/// Params: <see cref="FormatProgramTextParams"/> / <see cref="ParseProgramTextParams"/>.
/// </summary>
internal sealed class LanguageCommands
{
    public void Register(CommandDispatcher d)
    {
        d.Add("FormatProgramText", CommandNotImplementedException.Handler("FormatProgramText"));
        d.Add("ParseProgramText",  CommandNotImplementedException.Handler("ParseProgramText"));
    }
}
