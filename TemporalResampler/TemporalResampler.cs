using OpenTabletDriver.Plugin;
using OpenTabletDriver.Plugin.Attributes;
using OpenTabletDriver.Plugin.Output;
using OpenTabletDriver.Plugin.Tablet;
using OpenTabletDriver.Plugin.Timing;
using System.Numerics;

[PluginName("Temporal Resampler"), DeviceHub()]
public class TemporalResampler : AsyncPositionedPipelineElement<IDeviceReport>
{
    public TemporalResampler() : base()
    {
    }

    public override PipelinePosition Position => PipelinePosition.Raw;

    [Property("Prediction Ratio"), DefaultPropertyValue(0.5f), ToolTip
    (
        "Default: 0.5\n" +
        "Range: 0.0 - 1.0\n\n" +

        "Determines the time distance the filter predicts inputs for each tablet update.\n" +
        "Prediction brought to you by Kalman filtering.\n" +
        "0.0 == [slower] 0% predicted, one rps of latency\n" +
        "0.5 == [balanced] 50% predicted, half rps of latency\n" +
        "1.0 == [overkill] 100% predicted, no added latency (works best with some smoothing)"
    )]

    public float frameShift
    {
        set => _frameShift = Math.Clamp(value, 0, 1);
        get => _frameShift;
    }
    public float _frameShift;

    [Property("Follow Radius"), DefaultPropertyValue(0f), Unit("mm"), ToolTip
    (
        "Default: 0.0mm\n\n" +

        "Radius of cursor trailing distance in millimeters."
    )]

    public float followRadius { set; get; }

    [Property("Smoothing Latency"), DefaultPropertyValue(0f), Unit("ms"), ToolTip
    (
        "Default: 0ms\n\n" +

        "The amount of latency added by smoothing, approach is similar to Hawku smoothing."
    )]
    public float latency
    {
        set => _latency = Math.Max(value, 0);
        get => _latency;
    }
    public float _latency;

    [Property("Reverse EMA"), DefaultPropertyValue(1f), ToolTip
    (
        "Default: 1.0\n" +
        "Range: 0.0 - 1.0\n\n" +

        "Removes hardware smoothing, fine-tuned this to your tablet.\n" +
        "1.0 == no effect\n" +
        "lower == removes more hardware smoothing"
    )]
    public float reverseSmoothing
    {
        set => _reverseSmoothing = Math.Clamp(value, 0.001f, 1);
        get => _reverseSmoothing;
    }
    public float _reverseSmoothing;

    [Property("Maximize Frequency"), DefaultPropertyValue(true), ToolTip
    (
        "Default: True\n\n" +

        "Wires ConsumeState to UpdateState for a further increased update frequency with no elevated OTD CPU usage.\n" +
        "Disable if any unexpected behavior is happening or you just don't want it on."
    )]
    public bool extraFrames { set; get; }

    [Property("Log Stats"), DefaultPropertyValue(false), ToolTip
    (
        "Default: False\n\n" +

        "Logs the time and physical latency in OpenTableDriver Console.\n" +
        "Make sure to disable when you are done so you don't waste memory.\n" +
        "[False] == logging disabled\n" +
        "[True] == logging enabled, check the Console tab in OpenTableDriver"
    )]
    public bool loggingEnabled { set; get; }

    protected override void UpdateState()
    {
        if (State is not IAbsolutePositionReport report || !PenIsInRange())
            return;

        // time
        float t = 1 + (float)(runningStopwatch.Elapsed - latestReport).TotalSeconds * rpsAvg;
        t = Math.Clamp(t, 0, 3);

        // interp position
        report.Position = Trajectory
        (
            t,
            (Vector2)stablePoints[2][0],
            (Vector2)stablePoints[1][0],
            (Vector2)stablePoints[0][0]
        );

        // interp pressure
        if (State is ITabletReport tabletReport)
        {
            int tShift = (t < 1) ? 1 : 0;
            double pressure0 = (uint)stablePoints[1 + tShift][1];
            double pressure1 = (uint)stablePoints[0 + tShift][1];
            tabletReport.Pressure = (uint)(pressure0 + (pressure1 - pressure0) * Math.Clamp(t + tShift - 1, 0, 1));
        }

        if (devMode)
            devOutput += $"u{devStopwatch.Elapsed.TotalSeconds},{report.Position.X},{report.Position.Y}";

        State = report;
        OnEmit();
    }

    protected override void ConsumeState()
    {
        if (State is not IAbsolutePositionReport report || TabletReference == null)
        {
            OnEmit();
            return;
        }

        var digitizer = TabletReference.Properties.Specifications.Digitizer;
        float upmm = digitizer.MaxX / digitizer.Width;
        float followUnits = followRadius * upmm;
        float consumeDelta = (float)reportStopwatch.Restart().TotalSeconds;
        float pressure = 0;

        if (State is ITabletReport tabletReport)
            pressure = tabletReport.Pressure;

        if (consumeDelta > 0.03f || consumeDelta < 0.0001f)
        {
            ResetValues(report.Position, (uint)pressure);
            return;
        }

        Vector2 real = report.Position;
        if (devMode)
            devOutput += $"c{devStopwatch.Elapsed.TotalSeconds},{real.X},{real.Y}";

        // rps average
        rpsAvg += (1f / consumeDelta - rpsAvg) * (1f - MathF.Exp(-2f * consumeDelta));
        float secAvg = 1f / rpsAvg;
        float msAvg = 1000f * secAvg;

        // reverse smoothing
        Vector2 smoothed = real;
        if (reverseSmoothing < 1f)
            smoothed = bE + (smoothed - bE) / reverseSmoothing;
        bE = real;
        Vector2 aE = smoothed;

        // smoothing
        float smoothingWeight = MathF.Exp(msAvg / -latency);
        if (latency > 0f) 
        {
            smoothed += (sC - smoothed) * smoothingWeight;
            pressure += (sPressure - pressure) * smoothingWeight;
        }
        sC = smoothed;
        sPressure = pressure;
        uint uPressure = (uint)Math.Max(Math.Round(pressure), 0);
        InsertAtFirst(smoothedPoints, smoothed);

        // prediction
        Vector2 predict = smoothedPoints[0];
        if (frameShift > 0f) 
        {
            predict = kf.Update(smoothedPoints[0], secAvg);
            float predictCoe = (followUnits > 0f) ? Math.Clamp(Vector2.Distance(smoothedPoints[0], smoothedPoints[2]) / followUnits - 1f, 0, 1) : 1f;
            predictCoe += (1f - predictCoe) * smoothingWeight;
            predict += (smoothedPoints[0] - predict) * (1f - predictCoe * frameShift);
        }

        // follow radius
        Vector2 delta = predict - bP;
        float travel = delta.Length();
        if (followUnits > 0f && travel > 0f)
            predict = bP + delta * Math.Clamp(travel / followUnits - 1, 0, 1);

        // variance compensate
        tOffset += secAvg - consumeDelta;
        tOffset *= MathF.Exp(-5f * consumeDelta);
        tOffset = Math.Clamp(tOffset, -secAvg, secAvg);
        latestReport = runningStopwatch.Elapsed + TimeSpan.FromSeconds(tOffset);
        InsertAtFirst(stablePoints, new object[] { predict, uPressure });

        // miscellaneous
        if (loggingEnabled && logCount > 0) // data log
        {
            float measuredSpeed = Vector2.Distance(_aE, aE) * rpsAvg; _aE = aE;
            if (measuredSpeed * 0f != measuredSpeed * 0f)
                measuredSpeed = 0f;

            logReportAvg += rpsAvg;
            logErrorAvg += Math.Abs(tOffset);
            logSpeedAvg += measuredSpeed;
            logDistanceAvg += Vector2.Distance(bP, aE);
            logReportCount++;

            if (logStopwatch.Elapsed.TotalSeconds > 1)
            {
                float aCount = 1f / logReportCount;
                logReportAvg *= aCount;
                logErrorAvg *= aCount;
                logSpeedAvg *= aCount;
                logDistanceAvg *= aCount;

                double baseLatency = 0.5d / (Frequency + (extraFrames ? logReportAvg : 0d)) + logErrorAvg;
                double measuredTimeLatency = Math.Round(((1d - frameShift) / logReportAvg + baseLatency) * 1000d + latency, 2);
                double measuredPhysicalLatency = Math.Round((logDistanceAvg / logSpeedAvg + baseLatency) * 1000d, 2);
                double measuredErrorLatency = Math.Round(logErrorAvg * 1000d, 2);
                Log.Write("TemporalResampler", "(time latency, physical latency, error latency): " + measuredTimeLatency.ToString() + "ms, " + measuredPhysicalLatency.ToString() + "ms, " + measuredErrorLatency + "ms");

                logCount--;
                logReportCount = 0;
                logDistanceAvg = logSpeedAvg = logReportAvg = logErrorAvg = 0d;
                logStopwatch.Restart();
            }
        }
        else
            logStopwatch.Stop();

        if (devMode && devStopwatch.Elapsed.TotalSeconds > 30)
        {
            Log.Write("TemporalResampler", devOutput);
            devStopwatch.Reset();
            devMode = false;
        }

        bP = predict;

        // wire consume state to update state
        if (extraFrames)
            UpdateState();
    }

    void ResetValues(Vector2 p0, uint pressure)
    {
        kf = new KalmanVector2(4, p0);
        smoothedPoints = Enumerable.Repeat(p0, smoothedPoints.Length).ToArray();
        stablePoints = Enumerable.Repeat(new object[] { p0, pressure }, stablePoints.Length).ToArray();
        latestReport = runningStopwatch.Elapsed;
        tOffset = 0;
        bE = bP = sC = _aE = p0;
        sPressure = pressure;
    }

    void InsertAtFirst<T>(T[] arr, T element)
    {
        for (int p = arr.Length - 1; p > 0; p--) arr[p] = arr[p - 1];
        arr[0] = element;
    }

    // trajectory
    private static readonly int steps = 256;
    private static readonly float dt = 1f / steps;
    private float[] arcArr = new float[steps];
    private float arcTar = 0;
    private Vector2 _v1, _v2, _v3;
    private int _floor;
    Vector2 Trajectory(float t, Vector2 v3, Vector2 v2, Vector2 v1)
    {
        var mid = 0.5f * (v1 + v3);
        var accel = 2f * (mid - v2);
        var vel = 2f * v2 - v3 - mid;
        
        // if there is acceleration, then start spacing points evenly using integrals
        if (Vector2.Dot(accel, accel) > 0.001f)
        {
            int floor = (int)Math.Floor(t);
            var _vel = vel + accel * floor;

            // if any of the inputs have changed, recalculate arcArr
            if ((_floor != floor) || (_v1 != v1) || (_v2 != v2) || (_v3 != v3))
            {
                _v1 = v1;
                _v2 = v2;
                _v3 = v3;
                _floor = floor;
                arcTar = 0;

                for (int _t = 0; _t < steps; _t++)
                {
                    arcArr[_t] = arcTar;
                    arcTar += (_vel + _t * dt * accel).Length();
                }
            }

            float _arcTar = arcTar * (t - floor);

            for (int _t = 0; _t < steps; _t++)
            {
                if (arcArr[_t] < _arcTar) continue;
                t = _t * dt + floor;
                break;
            }
        }

        return v3 + t * vel + 0.5f * t * t * accel;
    }

    //values
    KalmanVector2 kf;
    Vector2[] smoothedPoints = new Vector2[3];
    object[][] stablePoints = new object[3][];
    Vector2 bE, bP, sC, _aE;
    TimeSpan latestReport = TimeSpan.Zero;
    float sPressure;
    float rpsAvg = 200f, tOffset;
    HPETDeltaStopwatch reportStopwatch = new HPETDeltaStopwatch(false);
    HPETDeltaStopwatch runningStopwatch = new HPETDeltaStopwatch(true);
    HPETDeltaStopwatch logStopwatch = new HPETDeltaStopwatch(true);
    double logDistanceAvg, logSpeedAvg, logReportAvg, logErrorAvg;
    int logCount = 30, logReportCount;

    bool devMode = false;
    string devOutput;
    HPETDeltaStopwatch devStopwatch = new HPETDeltaStopwatch(true);

    [TabletReference]
    public TabletReference? TabletReference { get; set; }
}

public class KalmanFilter
{
    private readonly double[,] scale_const;
    private readonly int states;
    private double lastMeasuredPos;

    private Matrix x;
    private Matrix P;
    private Matrix Q;
    private Matrix R;
    private Matrix H;

    public KalmanFilter(uint statesNumber, double initialPosition)
    {
        states = (int)statesNumber + 2;

        scale_const = new double[states, states];
        for (int i = 0; i < states; i++)
        {
            int fac_n = 1;
            int fac_i = 0;
            for (int j = i; j < states; j++)
            {
                scale_const[i, j] = 1d / fac_n;
                fac_i++;
                fac_n *= fac_i;
            }
        }

        lastMeasuredPos = initialPosition;
        double[,] xArr = new double[states, 1];
        xArr[0, 0] = initialPosition;

        x = Matrix.Build.DenseOfArray(xArr);
        P = Matrix.Build.DenseIdentity(states);
        Q = Matrix.Build.DenseIdentity(states) * 1;
        R = Matrix.Build.DenseDiagonal(2, 2, 0.0001);
        H = Matrix.Build.DenseDiagonal(2, states, 1);
    }

    public double Update(double measuredPos, double dt)
    {
        double measuredVel = (measuredPos - lastMeasuredPos) / dt;
        lastMeasuredPos = measuredPos;

        var z = Matrix.Build.DenseOfArray(new double[,] { { measuredPos }, { measuredVel } });

        double[,] Aarr = new double[states, states];
        for (int i = 0; i < states; i++) 
        {
            double time_pow = 1;
            for (int j = i; j < states; j++) 
            {
                Aarr[i, j] = time_pow * scale_const[i, j];
                time_pow *= dt;
            } 
        }

        /*
            vvvvvvvvvvv
        4 states should look like this
        double[,] Aarr = new double[,] {
            {          1,          dt^1/1!,    dt^2/2!,    dt^3/3!     },
            {          0,          1,          dt^1/1!,    dt^2/2!     },
            {          0,          0,          1,          dt^1/1!     },
            {          0,          0,          0,          1           }
        }
        */

        var A = Matrix.Build.DenseOfArray(Aarr);

        x = A * x;
        P = A * P * A.Transpose() + Q;

        var S = H * P * H.Transpose() + R;
        var K = P * H.Transpose() * S.Inverse();

        x = x + K * (z - H * x);
        P = (Matrix.Build.DenseIdentity(states) - K * H) * P;

        return (A * x)[0, 0];
    }
}

public class KalmanVector2
{
    private KalmanFilter xFilter;
    private KalmanFilter yFilter;

    public KalmanVector2(uint states, Vector2 initialPosition)
    {
        xFilter = new KalmanFilter(states, initialPosition.X);
        yFilter = new KalmanFilter(states, initialPosition.Y);
    }

    public Vector2 Update(Vector2 measuredPosition, float dt)
    {
        float xState = (float)xFilter.Update(measuredPosition.X, dt);
        float yState = (float)yFilter.Update(measuredPosition.Y, dt);
        return new Vector2(xState, yState);
    }
}

public class Matrix
{
    internal readonly double[,] data;

    public Matrix(double[,] data)
    {
        this.data = data;
    }

    public int Rows => data.GetLength(0);
    public int Cols => data.GetLength(1);

    public double this[int i, int j]
    {
        get => data[i, j];
        set => data[i, j] = value;
    }

    public static Matrix operator +(Matrix a, Matrix b)
    {
        var result = new double[a.Rows, a.Cols];
        for (int i = 0; i < a.Rows; i++)
            for (int j = 0; j < a.Cols; j++)
                result[i, j] = a[i, j] + b[i, j];
        return new Matrix(result);
    }

    public static Matrix operator -(Matrix a, Matrix b)
    {
        var result = new double[a.Rows, a.Cols];
        for (int i = 0; i < a.Rows; i++)
            for (int j = 0; j < a.Cols; j++)
                result[i, j] = a[i, j] - b[i, j];
        return new Matrix(result);
    }

    public static Matrix operator *(Matrix a, Matrix b)
    {
        var result = new double[a.Rows, b.Cols];
        for (int i = 0; i < a.Rows; i++)
            for (int j = 0; j < b.Cols; j++)
                for (int k = 0; k < a.Cols; k++)
                    result[i, j] += a[i, k] * b[k, j];
        return new Matrix(result);
    }

    public static Matrix operator *(Matrix a, double scalar)
    {
        var result = new double[a.Rows, a.Cols];
        for (int i = 0; i < a.Rows; i++)
            for (int j = 0; j < a.Cols; j++)
                result[i, j] = a[i, j] * scalar;
        return new Matrix(result);
    }

    public Matrix Transpose()
    {
        var result = new double[Cols, Rows];
        for (int i = 0; i < Rows; i++)
            for (int j = 0; j < Cols; j++)
                result[j, i] = data[i, j];
        return new Matrix(result);
    }

    public Matrix Inverse()
    {
        if (Rows != Cols) throw new InvalidOperationException("Matrix must be square to invert.");

        int n = Rows;
        var result = new double[n, n];
        var identity = Build.DenseIdentity(n).data;
        var copy = (double[,])data.Clone();

        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                result[i, j] = identity[i, j];

        for (int i = 0; i < n; i++)
        {
            double diag = copy[i, i];
            if (diag == 0) throw new InvalidOperationException("Matrix is singular.");

            for (int j = 0; j < n; j++)
            {
                copy[i, j] /= diag;
                result[i, j] /= diag;
            }

            for (int k = 0; k < n; k++)
            {
                if (k == i) continue;
                double factor = copy[k, i];
                for (int j = 0; j < n; j++)
                {
                    copy[k, j] -= factor * copy[i, j];
                    result[k, j] -= factor * result[i, j];
                }
            }
        }

        return new Matrix(result);
    }

    public static class Build
    {
        public static Matrix DenseOfArray(double[,] data) => new Matrix(data);

        public static Matrix DenseIdentity(int size)
        {
            var result = new double[size, size];
            for (int i = 0; i < size; i++) result[i, i] = 1;
            return new Matrix(result);
        }

        public static Matrix DenseDiagonal(int rows, int cols, Func<int, double> diagFunc)
        {
            var result = new double[rows, cols];
            for (int i = 0; i < Math.Min(rows, cols); i++)
                result[i, i] = diagFunc(i);
            return new Matrix(result);
        }

        public static Matrix DenseDiagonal(int rows, int cols, double value)
        {
            return DenseDiagonal(rows, cols, _ => value);
        }
    }
}
