const LeanEstimator =
{
    calibrated: false,
    rollAngleDeg: 0,
    previousTimestampMs: null,

    calibrate(sensorData)
    {
        this.rollAngleDeg = 0;

        this.previousTimestampMs =
            typeof sensorData?.timestamp === "number"
                ? sensorData.timestamp
                : null;

        this.calibrated = true;

        return {
            success: true
        };
    },

    update(sensorData)
    {
        if (!this.calibrated)
        {
            return {
                leanAngle: 0,
                leanRate: 0,
                confidence: 0
            };
        }

        const timestampMs =
            typeof sensorData?.timestamp === "number"
                ? sensorData.timestamp
                : null;

        const rollRateDegPerSecond =
            typeof sensorData?.gyro?.x === "number"
                ? sensorData.gyro.x
                : 0;

        if (
            timestampMs === null ||
            this.previousTimestampMs === null
        )
        {
            this.previousTimestampMs = timestampMs;

            return {
                leanAngle: this.rollAngleDeg,
                leanRate: rollRateDegPerSecond,
                confidence: 0.5
            };
        }

        const deltaSeconds =
            (timestampMs - this.previousTimestampMs) / 1000;

        this.previousTimestampMs = timestampMs;

        if (
            Number.isFinite(deltaSeconds) &&
            deltaSeconds > 0 &&
            deltaSeconds <= 0.25
        )
        {
            this.rollAngleDeg +=
                rollRateDegPerSecond * deltaSeconds;
        }

        return {
            leanAngle: this.rollAngleDeg,
            leanRate: rollRateDegPerSecond,
            confidence: 0.5
        };
    },

    reset()
    {
        this.calibrated = false;
        this.rollAngleDeg = 0;
        this.previousTimestampMs = null;
    }
};

export default LeanEstimator;