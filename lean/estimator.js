const LeanEstimator =
{
    calibrated: false,

    rollAngleDeg: 0,
    previousTimestampMs: null,

    forwardAxis:
    {
        x: 0,
        y: 1,
        z: 0
    },

    calibrate(sensorData)
    {
        const forwardAxis =
            sensorData?.forwardAxis;

        if (
            !forwardAxis ||
            !Number.isFinite(forwardAxis.x) ||
            !Number.isFinite(forwardAxis.y) ||
            !Number.isFinite(forwardAxis.z)
        )
        {
            this.reset();

            return {
                success: false,
                reason: "Invalid forward axis"
            };
        }

        const magnitude =
            Math.sqrt(
                forwardAxis.x * forwardAxis.x +
                forwardAxis.y * forwardAxis.y +
                forwardAxis.z * forwardAxis.z
            );

        if (
            !Number.isFinite(magnitude) ||
            magnitude < 0.000001
        )
        {
            this.reset();

            return {
                success: false,
                reason: "Forward axis has zero length"
            };
        }

        this.forwardAxis =
        {
            x: forwardAxis.x / magnitude,
            y: forwardAxis.y / magnitude,
            z: forwardAxis.z / magnitude
        };

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

        const gyro =
            sensorData?.gyro ?? {};

        const gyroVector =
        {
            x:
                Number.isFinite(gyro.x)
                    ? gyro.x
                    : 0,

            y:
                Number.isFinite(gyro.y)
                    ? gyro.y
                    : 0,

            z:
                Number.isFinite(gyro.z)
                    ? gyro.z
                    : 0
        };

        /*
         * Project all three phone gyro axes onto the calibrated
         * motorcycle-forward axis. Rotation around this axis is roll.
         */
        const rollRateDegPerSecond =
            gyroVector.x * this.forwardAxis.x +
            gyroVector.y * this.forwardAxis.y +
            gyroVector.z * this.forwardAxis.z;

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
            (timestampMs - this.previousTimestampMs) /
            1000;

        this.previousTimestampMs = timestampMs;

        if (
            Number.isFinite(deltaSeconds) &&
            deltaSeconds > 0 &&
            deltaSeconds <= 0.25
        )
        {
            this.rollAngleDeg +=
                rollRateDegPerSecond *
                deltaSeconds;
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

        this.forwardAxis =
        {
            x: 0,
            y: 1,
            z: 0
        };
    }
};

export default LeanEstimator;