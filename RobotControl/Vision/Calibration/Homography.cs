namespace Controller.RobotControl.Vision.Calibration
{
    /// <summary>
    /// 3×3 planar homographies as jagged <c>double[3][3]</c> arrays (the on-disk form), in
    /// double math. Results are normalised so that <c>h33 = 1</c> where possible.
    /// </summary>
    public static class Homography
    {
        public static double[][] Identity() => [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

        /// <summary>Maps (x, y) through <paramref name="h"/>, with the perspective divide.</summary>
        public static (double X, double Y) Apply(double[][] h, double x, double y)
        {
            double w  = h[2][0] * x + h[2][1] * y + h[2][2];
            double px = h[0][0] * x + h[0][1] * y + h[0][2];
            double py = h[1][0] * x + h[1][1] * y + h[1][2];
            if (Math.Abs(w) < 1e-15) return (double.NaN, double.NaN);
            return (px / w, py / w);
        }

        public static double[][] Multiply(double[][] a, double[][] b)
        {
            var r = new double[3][];
            for (int i = 0; i < 3; i++)
            {
                r[i] = new double[3];
                for (int j = 0; j < 3; j++)
                    r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
            }
            return r;
        }

        public static double Determinant(double[][] m) =>
              m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

        /// <summary>The inverse homography. Throws when <paramref name="m"/> is singular.</summary>
        public static double[][] Invert(double[][] m)
        {
            double det = Determinant(m);
            if (Math.Abs(det) < 1e-18 || double.IsNaN(det))
                throw new InvalidOperationException("Homography is singular");
            double inv = 1.0 / det;
            var r = new double[][]
            {
                [ (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv,
                  (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv,
                  (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv ],
                [ (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv,
                  (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv,
                  (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv ],
                [ (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv,
                  (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv,
                  (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv ],
            };
            return Normalize(r);
        }

        /// <summary>Scales so that h33 = 1 (left unchanged when h33 is ~0).</summary>
        public static double[][] Normalize(double[][] m)
        {
            double s = m[2][2];
            if (Math.Abs(s) < 1e-15) return m;
            return m.Select(row => row.Select(v => v / s).ToArray()).ToArray();
        }

        /// <summary>Scales the output of <paramref name="h"/> by <paramref name="scale"/> (diag(s, s, 1) · H).</summary>
        public static double[][] ScaleOutput(double[][] h, double scale) =>
            Normalize(Multiply([[scale, 0, 0], [0, scale, 0], [0, 0, 1]], h));

        /// <summary>The 3×3 matrix of a 2-D rigid transform (mirrored when it says so).</summary>
        public static double[][] FromRigid(RigidTransform2D t)
        {
            double m = t.Mirrored ? -1 : 1;
            return [[t.Cos, -t.Sin * m, t.Tx], [t.Sin, t.Cos * m, t.Ty], [0, 0, 1]];
        }

        /// <summary><paramref name="rigid"/> ∘ <paramref name="h"/>: first <paramref name="h"/>, then the rigid transform.</summary>
        public static double[][] ComposeRigid(RigidTransform2D rigid, double[][] h) =>
            Normalize(Multiply(FromRigid(rigid), h));

        /// <summary>Converts an OpenCV 3×3 CV_64F Mat to a jagged array.</summary>
        public static double[][] FromMat(OpenCvSharp.Mat mat)
        {
            var r = new double[3][];
            for (int i = 0; i < 3; i++)
            {
                r[i] = new double[3];
                for (int j = 0; j < 3; j++) r[i][j] = mat.At<double>(i, j);
            }
            return Normalize(r);
        }
    }
}
