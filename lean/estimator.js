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

    steeringAxis: null,

    calibrate(sensorData)
    {
        if (!this.setForwardAxis(sensorData?.forwardAxis))
        {
            this.reset();

            return {
                success: false,
                reason: "Invalid forward axis"
            };
        }

        this.setSteeringAxis(
            sensorData?.steeringAxis
        );

        this.rollAngleDeg = 0;

        this.previousTimestampMs =
            Number.isFinite(sensorData?.timestamp)
                ? sensorData.timestamp
                : null;

        this.calibrated = true;

        return {
            success: true
        };
    },

    setForwardAxis(axis)
    {
        const normalized =
            this.normalizeVector(axis);

        if (!normalized)
        {
            return false;
        }

        this.forwardAxis = normalized;
        return true;
    },

    setSteeringAxis(axis)
    {
        const normalized =
            this.normalizeVector(axis);

        this.steeringAxis =
            normalized;

        return normalized !== null;
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
            Number.isFinite(sensorData?.timestamp)
                ? sensorData.timestamp
                : null;

       const gyroReference =
            this.validVector(
            sensorData?.gyroDevice
        );

        let rollRateDegPerSecond = 0;
        let confidence = 0.35;

        if (gyroReference)
        {
            rollRateDegPerSecond =
                this.solveRollRate(
                    gyroReference
                );

            confidence =
                this.steeringAxis
                    ? 0.70
                    : 0.50;
        }

        if (
            timestampMs !== null &&
            this.previousTimestampMs !== null
        )
        {
            const deltaSeconds =
                (timestampMs - this.previousTimestampMs) /
                1000;

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
        }

        this.previousTimestampMs = timestampMs;

        if (Math.abs(this.rollAngleDeg) < 0.05)
        {
            this.rollAngleDeg = 0;
        }

        return {
            leanAngle: this.rollAngleDeg,
            leanRate: rollRateDegPerSecond,
            confidence
        };
    },

    solveRollRate(angularVelocityReference)
    {
        const forward =
            this.forwardAxis;

        if (!this.steeringAxis)
        {
            return this.dot(
                angularVelocityReference,
                forward
            );
        }

        const steering =
            this.steeringAxis;

        /*
         * Resolve angular velocity as:
         *
         *   omega = rollRate * forwardAxis
         *         + steerRate * steeringAxis
         *
         * This dual-axis solution removes steering even when the
         * steering and roll axes are not perpendicular.
         */
        const coupling =
            this.dot(
                forward,
                steering
            );

        const denominator =
            1 - coupling * coupling;

        if (
            !Number.isFinite(denominator) ||
            denominator < 0.02
        )
        {
            return this.dot(
                angularVelocityReference,
                forward
            );
        }

        const omegaForward =
            this.dot(
                angularVelocityReference,
                forward
            );

        const omegaSteering =
            this.dot(
                angularVelocityReference,
                steering
            );

        return (
            omegaForward -
            coupling * omegaSteering
        ) / denominator;
    },

    validVector(vector)
    {
        if (
            !vector ||
            !Number.isFinite(vector.x) ||
            !Number.isFinite(vector.y) ||
            !Number.isFinite(vector.z)
        )
        {
            return null;
        }

        return {
            x: vector.x,
            y: vector.y,
            z: vector.z
        };
    },

    normalizeVector(vector)
    {
        const valid =
            this.validVector(vector);

        if (!valid)
        {
            return null;
        }

        const magnitude =
            Math.sqrt(
                valid.x * valid.x +
                valid.y * valid.y +
                valid.z * valid.z
            );

        if (
            !Number.isFinite(magnitude) ||
            magnitude < 0.000001
        )
        {
            return null;
        }

        return {
            x: valid.x / magnitude,
            y: valid.y / magnitude,
            z: valid.z / magnitude
        };
    },

    dot(a, b)
    {
        return (
            a.x * b.x +
            a.y * b.y +
            a.z * b.z
        );
    },

    zeroAngle(timestampMs = null)
    {
        this.rollAngleDeg = 0;

        this.previousTimestampMs =
            Number.isFinite(timestampMs)
                ? timestampMs
                : null;

        return {
            success: this.calibrated
        };
    },

    reset()
    {
        this.calibrated = false;
        this.rollAngleDeg = 0;
        this.previousTimestampMs = null;
        this.steeringAxis = null;

        this.forwardAxis =
        {
            x: 0,
            y: 1,
            z: 0
        };
    }
};

export default LeanEstimator;
