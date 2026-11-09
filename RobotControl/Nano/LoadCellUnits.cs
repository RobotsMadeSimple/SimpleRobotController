namespace Controller.RobotControl.Io
{
    /// <summary>Convert between grams (the master unit) and a load cell's display/program unit.
    /// Mirrors the app's loadcell.ts. N uses standard gravity (1 kgf = 9.80665 N).</summary>
    public static class LoadCellUnits
    {
        public static double GramsToUnit(double grams, string? unit) => unit switch
        {
            "kg" => grams / 1000.0,
            "oz" => grams / 28.349523125,
            "lb" => grams / 453.59237,
            "N"  => grams * 9.80665 / 1000.0,
            _    => grams, // "g" / default
        };
    }
}
