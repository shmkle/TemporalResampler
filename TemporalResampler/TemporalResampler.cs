using OpenTabletDriver.Plugin;
using OpenTabletDriver.Plugin.Attributes;
using OpenTabletDriver.Plugin.Output;
using OpenTabletDriver.Plugin.Tablet;
using OpenTabletDriver.Plugin.Timing;
using System.Numerics;
using MathNet.Filtering.Kalman;
using MathNet.Numerics.LinearAlgebra;
using System.Diagnostics;

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

        "Determines the time distance the filter predicts inputs for each tablet update.\nPrediction brought to you by Kalman filtering.\n" +
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
        "Default: 0.0\n\n" +

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

        "Removes hardware smoothing, fine-tuned this to your tablet.\nPrediction brought to you by Kalman filtering.\n" +
        "1.0 == no effect\n" +
        "lower == removes more hardware smoothing"
    )]
    public float reverseSmoothing
    {
        set => _reverseSmoothing = Math.Clamp(value, 0, 1);
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
        if (State is not IAbsolutePositionReport report || !PenIsInRange()) return;
        if (State is ITabletReport tabletReport) tabletReport.Pressure = pressure;

        float updateDelta = (float)reportStopwatch.Elapsed.TotalSeconds;
        float _t = t + v * updateDelta + a * 0.5f * updateDelta * updateDelta;
        int iAdd = (_t < -1f) ? 1 : 0;

        report.Position = Trajectory
        (
            Math.Clamp(_t + iAdd, -2f, 1f) + 2f,
            predictPoints[2 + iAdd],
            predictPoints[1 + iAdd],
            predictPoints[0 + iAdd]
        );
         
        if (devMode) devOutput += $"u{devStopwatch.Elapsed.TotalSeconds},{report.Position.X},{report.Position.Y}";

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

        if (State is ITabletReport tabletReport) pressure = tabletReport.Pressure;

        float consumeDelta = (float)reportStopwatch.Restart().TotalSeconds;
        if (consumeDelta > 0.03f || consumeDelta < 0.0001f)
        {
            ResetValues(report.Position);
            return;
        }

        var digitizer = TabletReference.Properties.Specifications.Digitizer;
        float upmm = digitizer.MaxX / digitizer.Width;
        float followUnits = followRadius * upmm;
        Vector2 real = report.Position;

        // rps average
        rpsAvg += (1f / consumeDelta - rpsAvg) * (1f - MathF.Exp(-2f * consumeDelta));
        float msAvg = 1000f / rpsAvg;
        float secAvg = msAvg * 0.001f;

        // reverse smoothing
        Vector2 smoothed = real;
        if (reverseSmoothing < 1f) smoothed = bE + (smoothed - bE) / reverseSmoothing;
        bE = real;
        Vector2 aE = smoothed;
        if (devMode) devOutput += $"c{devStopwatch.Elapsed.TotalSeconds},{smoothed.X},{smoothed.Y}";

        // smoothing
        float smoothingWeight = MathF.Exp(msAvg / -latency);
        if (latency > 0f) smoothed += (sC - smoothed) * smoothingWeight;
        sC = smoothed;
        InsertAtFirst(smoothedPoints, smoothed);

        // prediction
        Vector2 predict = kf.Update(smoothedPoints[0], secAvg);
        float predictCoe = (followUnits > 0f) ? Math.Clamp(Vector2.Distance(smoothedPoints[0], smoothedPoints[2]) / followUnits - 1f, 0, 1) : 1f;
        predictCoe += (1f - predictCoe) * smoothingWeight;
        predict += (smoothedPoints[0] - predict) * (1f - predictCoe * frameShift);

        // follow radius
        Vector2 delta = predict - predictPoints[0];
        float travel = delta.Length();
        predict = (followUnits > 0f && travel > 0f) ? predictPoints[0] + delta * Math.Clamp(travel / followUnits - 1, 0, 1) : predict;
        InsertAtFirst(predictPoints, predict);

        // time
        t--;
        t += v * consumeDelta + a * 0.5f * consumeDelta * consumeDelta;
        v += a * consumeDelta;
        a = -2 * (t + v * secAvg) / (secAvg * secAvg);
        a *= 1f - MathF.Exp(-100f * secAvg);

        // miscellaneous
        if (t < -2f || t > 0f) ResetValues(real);
        if (extraFrames) UpdateState();
        if (loggingEnabled && logCount > 0) // data log
        {
            speedAvg += (Vector2.Distance(_aE, aE) * rpsAvg - speedAvg) * MathF.Exp(-10f * consumeDelta); _aE = aE;
            if (speedAvg * 0f != speedAvg * 0f) speedAvg = 0f;

            logReportAvg += rpsAvg;
            logSpeedAvg += speedAvg;
            logDistanceAvg += Vector2.Distance(Trajectory(t + 3f, predictPoints[3], predictPoints[2], predictPoints[1]), aE);
            logReportCount++;

            if (logStopwatch.Elapsed.TotalSeconds > 1)
            {
                float aCount = 1f / logReportCount;
                logReportAvg *= aCount; logSpeedAvg *= aCount; logDistanceAvg *= aCount;
                double baseLatency = 0.5d / (Frequency + (extraFrames ? logReportAvg : 0d));
                double measuredTimeLatency = Math.Round(((1d - frameShift) / logReportAvg + baseLatency) * 1000d + latency, 2);
                double measuredPhysicalLatency = Math.Round((logDistanceAvg / logSpeedAvg + baseLatency) * 1000d, 2);
                Log.Write("TemporalResampler", "(time latency, physical latency): " + measuredTimeLatency.ToString() + "ms, " + measuredPhysicalLatency.ToString() + "ms");

                logCount--;
                logReportCount = 0;
                logDistanceAvg = logSpeedAvg = logReportAvg = 0d;
                logStopwatch.Restart();
            }
        }
        else
            logStopwatch.Stop();

        if (devMode && devStopwatch.Elapsed.TotalSeconds > 30) {
            Log.Write("TemporalResampler", devOutput);
            devStopwatch.Reset();
            devMode = false;
        }

    }

    public void ResetValues(Vector2 p0)
    {
        kf = new KalmanFilterVector2(p0, 1d, 0.000001d);
        smoothedPoints = Enumerable.Repeat(p0, smoothedPoints.Length).ToArray();
        predictPoints = Enumerable.Repeat(p0, predictPoints.Length).ToArray();
        pressure = 0;
        bE = sC = _aE = p0;
        t = -1f;
        v = rpsAvg;
        a = 0f;
    }
    public void InsertAtFirst<T>(T[] arr, T element)
    {
        for (int p = arr.Length - 1; p > 0; p--) arr[p] = arr[p - 1];
        arr[0] = element;
    }

    public static Vector2 Trajectory(float t, Vector2 v3, Vector2 v2, Vector2 v1)
    {
        var mid = 0.5f * (v1 + v3);
        var accel = 2f * (mid - v2);
        var vel = 2f * v2 - v3 - mid;

        // spacing points evenly using integrals
        if (accel != Vector2.Zero) {
            int steps = 128;
            float dt = 1f / steps;
            float floor = (float)Math.Floor(t);
            var _vel = vel + accel * floor;

            float arcTar = 0;
            float[] arcArr = new float[steps];
            for (int _t = 0; _t < steps; _t++)
            {
                arcArr[_t] = arcTar;
                arcTar += (_vel + _t * dt * accel).Length();
            }
            arcTar *= t - floor;

            for (int _t = 0; _t < steps; _t++)
            {
                if (arcArr[_t] > arcTar)
                {
                    t = _t * dt + floor;
                    break;
                }
            }
        }

        return v3 + t * vel + 0.5f * t * t * accel;
    }

    //values
    public uint pressure;
    public KalmanFilterVector2 kf;
    public Vector2[] smoothedPoints = new Vector2[3], predictPoints = new Vector2[4];
    public Vector2 bE, sC, _aE, tP;
    public float t = -1f, v = 1f, a = 0f, rpsAvg = 200f, speedAvg;
    public HPETDeltaStopwatch reportStopwatch = new HPETDeltaStopwatch(false);
    public HPETDeltaStopwatch logStopwatch = new HPETDeltaStopwatch();
    public double logDistanceAvg, logSpeedAvg, logReportAvg;
    public int logCount = 30, logReportCount;

    public bool devMode = false;
    public string devOutput;
    public HPETDeltaStopwatch devStopwatch = new HPETDeltaStopwatch();

    [TabletReference]
    public TabletReference? TabletReference { get; set; }
}
public class KalmanFilterVector2
{
    public DiscreteKalmanFilter kfx, kfy;
    private MatrixBuilder<double> matrixBuilder = Matrix<double>.Build;
    private Matrix<double> H, R, Q;
    private Vector2 lastPosition;

    public KalmanFilterVector2(Vector2 initial, double proccessNoise, double measurementNoise)
    {
        lastPosition = initial;
        var x0 = matrixBuilder.DenseOfArray(new double[,] { { (double)initial.X }, { 0 }, { 0 }, { 0 } });
        var y0 = matrixBuilder.DenseOfArray(new double[,] { { (double)initial.Y }, { 0 }, { 0 }, { 0 } });

        H = matrixBuilder.DenseDiagonal(2, 4, 1);
        Q = matrixBuilder.DenseDiagonal(4, 4, proccessNoise);
        R = matrixBuilder.DenseDiagonal(2, 2, measurementNoise);

        var p0 = matrixBuilder.DenseDiagonal(4, 4, 10000);
        kfx = new DiscreteKalmanFilter(x0, p0);
        kfy = new DiscreteKalmanFilter(y0, p0);
    }

    public Vector2 Update(Vector2 position, double deltaTime)
    {
        Vector2 velocity = (position - lastPosition) / (float)deltaTime;
        velocity = (velocity * 0 == velocity * 0) ? velocity : Vector2.Zero;
        lastPosition = position;

        var zX = matrixBuilder.DenseOfArray(new double[,] { { (double)position.X }, { (double)velocity.X } });
        var zY = matrixBuilder.DenseOfArray(new double[,] { { (double)position.Y }, { (double)velocity.Y } });

        kfx.Update(zX, H, R);
        kfy.Update(zY, H, R);

        double dt2 = deltaTime * deltaTime / 2.0;
        double dt3 = deltaTime * deltaTime * deltaTime / 6.0;
        var F = matrixBuilder.DenseOfArray(new double[,]
        {
            { 1, deltaTime, dt2, dt3 },
            { 0, 1, deltaTime, dt2 },
            { 0, 0, 1, deltaTime },
            { 0, 0, 0, 1 }
        });

        kfx.Predict(F, Q);
        kfy.Predict(F, Q);

        return new Vector2((float)kfx.State[0, 0], (float)kfy.State[0, 0]);
    }
}
