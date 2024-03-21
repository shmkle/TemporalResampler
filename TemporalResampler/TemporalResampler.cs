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

        "Removes hardware smoothing, fine-tuned this to your tablet. Follow the guide given in the Reconstructor filter wiki.\n" +
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
        if (State is not ITabletReport report || !PenIsInRange()) return;

        int iAdd = t < -1f ? 1 : 0;
        float d = rpsAvg * (float)updateStopwatch.Restart().TotalSeconds * m;
        report.Position = Trajectory(Math.Clamp(iAdd + t + d * 0.5f, iAdd - 2f, iAdd + 1f) + 2f, predictPoints[3 + iAdd], predictPoints[2 + iAdd], predictPoints[1 + iAdd]);
        report.Pressure = pressure;
        t += d;

        State = report;
        OnEmit();
    }

    protected override void ConsumeState()
    {
        if (State is not ITabletReport report || TabletReference == null) return;

        float consumeDelta = (float)reportStopwatch.Restart().TotalSeconds;
        var digitizer = TabletReference.Properties.Specifications.Digitizer;
        float upmm = digitizer.MaxX / digitizer.Width;

        Vector2 real = report.Position;
        pressure = report.Pressure;
        loggingEnabled &= logCount < 30;

        if (!resetDebounce)
            ResetValues(real);
        resetDebounce = true;

        if (consumeDelta < 0.03f && consumeDelta > 0f)
        {
            // rps average
            rpsAvg = (rpsAvg == 0f) ? 1 / consumeDelta : rpsAvg + (1f / consumeDelta - rpsAvg) * (1f - MathF.Exp(-2f * consumeDelta));

            Vector2 smoothed = real;
            // reverse smoothing
            if (reverseSmoothing < 1f)
                smoothed = bE + (smoothed - bE) / reverseSmoothing;
            bE = real; 
            Vector2 aE = smoothed;

            // smoothing
            if (latency > 0f)
                smoothed += (sC - smoothed) * MathF.Exp(1000 / rpsAvg / -latency);
            sC = smoothed;
            InsertPoint(smoothedPoints, smoothed);

            // follow radius
            Vector2 predict = Trajectory(2f + frameShift, smoothedPoints[3], smoothedPoints[2], smoothedPoints[1]);
            if (followRadius > 0f)
            {
                Vector2 delta = predict - predictPoints[1];
                float mag = delta.Length();
                predict = (mag > 0f) ? predictPoints[1] + Math.Clamp((mag - Math.Max(followRadius * upmm, 0)) / mag, 0, 1) * delta : predict;
            }
            InsertPoint(predictPoints, predict);

            if (t > -3f & t < 3f) // fixes tabbing into programs
            {
                t -= 1f;
                
                if (extraFrames)
                    UpdateState();

                m -= (t + m) * 0.1f;

                if (loggingEnabled) // data log
                {
                    speedAvg += (Vector2.Distance(predictPoints[2], predictPoints[1]) * rpsAvg - speedAvg) * (1f - MathF.Exp(-10f * consumeDelta));
                    if (speedAvg * 0f != speedAvg * 0f)
                        speedAvg = 0f;

                    logReportMax = (logReportMax > 0d) ? Math.Min(rpsAvg, logReportMax) : rpsAvg;
                    logSpeedMax = (logSpeedMax > 0d) ? Math.Min(speedAvg, logSpeedMax) : speedAvg;
                    logDistanceMax = Math.Max(Vector2.DistanceSquared(predictPoints[2], aE), logDistanceMax);

                    if (logStopwatch.Elapsed.TotalSeconds > 1)
                    {
                        logDistanceMax = Math.Sqrt(logDistanceMax);
                        double baseLatency = 0.5d / (Frequency + (extraFrames ? logReportMax : 0d));

                        double measuredTimeLatency = Math.Round(((1d - frameShift) / logReportMax + baseLatency) * 1000d + latency, 2);
                        double measuredPhysicalLatency = Math.Round((logDistanceMax / logSpeedMax + baseLatency) * 1000d, 2);
                        Log.Write("TemporalResampler", "(time latency, physical latency): " + measuredTimeLatency.ToString() + "ms, " + measuredPhysicalLatency.ToString() + "ms");
                        
                        logDistanceMax = logSpeedMax = logReportMax = 0d;
                        logCount += 1;
                        logStopwatch.Restart();
                    }
                }
                else
                    logStopwatch.Stop();
            }
            else
                ResetValues(real);
        }
        else
            ResetValues(real);
    }

    private void InsertPoint(Vector2[] arr, Vector2 v)
    {
        arr[4] = arr[3]; arr[3] = arr[2]; arr[2] = arr[1]; arr[1] = v;
    }

    private void ResetValues(Vector2 p0)
    {
        for (int i = 1; i < 4; i++)
        {
            smoothedPoints[i] = predictPoints[i] = p0;
        }
        bE = sC = p0; m = 1f; t = -1f;
    }

    private static Vector2 Trajectory(float t, Vector2 v3, Vector2 v2, Vector2 v1) // cool formula for finding the best-fit quadratic function that passes 3 points
    {
        return v3 + (2f * v2 - v3 - (v1 + v3) * 0.5f) * t + ((v1 + v3) * 0.5f - v2) * t * t;
    }

    // values
    private uint pressure;
    private Vector2[] smoothedPoints = new Vector2[5];
    private Vector2[] predictPoints = new Vector2[5];
    private Vector2 bE, sC;
    private float t = -1f, m = 1f, rpsAvg, speedAvg;
    private bool resetDebounce;
    private HPETDeltaStopwatch reportStopwatch = new HPETDeltaStopwatch();
    private HPETDeltaStopwatch updateStopwatch = new HPETDeltaStopwatch();
    private HPETDeltaStopwatch logStopwatch = new HPETDeltaStopwatch();
    private double logDistanceMax, logSpeedMax, logReportMax;
    private int logCount;

    [TabletReference]
    public TabletReference? TabletReference { get; set; }
}
