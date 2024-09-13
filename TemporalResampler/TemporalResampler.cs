using OpenTabletDriver.Plugin;
using OpenTabletDriver.Plugin.Attributes;
using OpenTabletDriver.Plugin.Output;
using OpenTabletDriver.Plugin.Tablet;
using OpenTabletDriver.Plugin.Timing;
using System;
using System.Numerics;
using System.Runtime.Intrinsics;

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

        float updateDelta = (float)reportStopwatch.Elapsed.TotalSeconds;
        float _t = t + v * updateDelta + a * 0.5f * updateDelta * updateDelta; 
        int iAdd = (_t < -1f) ? 1 : 0;

        report.Pressure = pressure;
        report.Position = Trajectory
        (
            Math.Clamp(_t, -2f, 1f) + 2f + iAdd, 
            predictPoints[2 + iAdd], 
            predictPoints[1 + iAdd], 
            predictPoints[0 + iAdd]
        );
        
        State = report;
        OnEmit();
    }

    protected override void ConsumeState()
    {
        if (State is not ITabletReport report || TabletReference == null)
        {
            OnEmit();
            return;
        }

        float consumeDelta = (float)reportStopwatch.Restart().TotalSeconds;
        var digitizer = TabletReference.Properties.Specifications.Digitizer;
        float upmm = digitizer.MaxX / digitizer.Width;
        float followUnits = followRadius * upmm;

        Vector2 real = report.Position;
        pressure = report.Pressure;
        loggingEnabled &= logCount < 30;

        if (!resetDebounce) ResetValues(real);
        resetDebounce = true;

        if (consumeDelta < 0.03f && consumeDelta > 0f)
        {
            // rps average
            rpsAvg = (rpsAvg == 0f) ? 1 / consumeDelta : rpsAvg + (1f / consumeDelta - rpsAvg) * (1f - MathF.Exp(-2f * consumeDelta));
            float msAvg = 1000f / rpsAvg;
            float secAvg = msAvg * 0.001f;

            // reverse smoothing
            Vector2 smoothed = real;
            if (reverseSmoothing < 1f) smoothed = bE + (smoothed - bE) / reverseSmoothing;
            bE = real;
            Vector2 aE = smoothed;

            // smoothing
            float smoothingWeight = MathF.Exp(msAvg / -latency);
            if (latency > 0f) smoothed += (sC - smoothed) * smoothingWeight;
            sC = smoothed;
            InsertAtFirst(smoothedPoints, smoothed);

            // prediction
            Vector2 predict = new Vector2();
            for (int i = 0; i < weights.Length; i++) predict += weights[i] * smoothedPoints[i];

            float predictCoe = (followUnits > 0f) ? Math.Clamp(Vector2.Distance(smoothedPoints[0], smoothedPoints[2]) / followUnits - 1f, 0, 1) : 1f;
            predictCoe += (1f - predictCoe) * smoothingWeight;
            predict += (smoothedPoints[0] - predict) * (1f - predictCoe * frameShift);

            // follow radius
            Vector2 delta = predict - predictPoints[0];
            float travel = delta.Length();
            predict = (followUnits > 0f && travel > 0f) ? predictPoints[0] + delta * Math.Clamp(travel / followUnits - 1, 0, 1) : predict;
            InsertAtFirst(predictPoints, predict);

            // time
            t -= 1f;
            t += v * consumeDelta + a * 0.5f * consumeDelta * consumeDelta;
            v += a * consumeDelta;
            a = -2 * (t + v * secAvg) / (secAvg * secAvg);
            a *= 1f - MathF.Exp(-100f * secAvg);

            // miscellaneous
            if (t < -2f || t > 0f) ResetValues(real);
            if (extraFrames) UpdateState();
            if (loggingEnabled) // data log
            {
                speedAvg += (Vector2.Distance(_aE, aE) * rpsAvg - speedAvg) * MathF.Exp(-10f * consumeDelta); _aE = aE;
                if (speedAvg * 0f != speedAvg * 0f) speedAvg = 0f;

                logReportAvg += rpsAvg;
                logSpeedAvg += speedAvg;
                logDistanceAvg += Vector2.Distance(predictPoints[1], aE);//Math.Max(Vector2.Distance(predictPoints[1], aE) - followUnits, 0);
                logReportCount++;

                if (logStopwatch.Elapsed.TotalSeconds > 1)
                {
                    float aCount = 1f / logReportCount;
                    logReportAvg *= aCount; logSpeedAvg *= aCount; logDistanceAvg *= aCount;
                    double baseLatency = 0.5d / (Frequency + (extraFrames ? logReportAvg : 0d));
                    double measuredTimeLatency = Math.Round(((1d - frameShift) / logReportAvg + baseLatency) * 1000d + latency, 2);
                    double measuredPhysicalLatency = Math.Round((logDistanceAvg / logSpeedAvg + baseLatency) * 1000d, 2);
                    Log.Write("TemporalResampler", "(time latency, physical latency): " + measuredTimeLatency.ToString() + "ms, " + measuredPhysicalLatency.ToString() + "ms");

                    logCount++;
                    logReportCount = 0;
                    logDistanceAvg = logSpeedAvg = logReportAvg = 0d;
                    logStopwatch.Restart();
                }
            }
            else
                logStopwatch.Stop();
        }
        else
            ResetValues(real);
    }

    public void ResetValues(Vector2 p0)
    {
        smoothedPoints = Enumerable.Repeat(p0, smoothedPoints.Length).ToArray();
        predictPoints = Enumerable.Repeat(p0, predictPoints.Length).ToArray();
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
    public static Vector2 Trajectory(float t, Vector2 v3, Vector2 v2, Vector2 v1) // cool formula for finding the best-fit quadratic function that passes 3 points
    {
        return v3 + (2f * v2 - v3 - (v1 + v3) * 0.5f) * t + ((v1 + v3) * 0.5f - v2) * t * t;
    }

    // values
    public uint pressure;
    public static float[] weights = { 2.4f, -1.2f, -0.8f, 0.6f }; // magical numbers
    public Vector2[] smoothedPoints = new Vector2[weights.Length], predictPoints = new Vector2[4];
    public Vector2 bE, sC, _aE, tP;
    public float t = -1f, v = 1f, a = 0f, rpsAvg = 200f, speedAvg;
    public bool resetDebounce;
    public HPETDeltaStopwatch reportStopwatch = new HPETDeltaStopwatch();
    public HPETDeltaStopwatch logStopwatch = new HPETDeltaStopwatch();
    public double logDistanceAvg, logSpeedAvg, logReportAvg;
    public int logCount, logReportCount;

    [TabletReference]
    public TabletReference? TabletReference { get; set; }
}
