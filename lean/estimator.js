const LeanEstimator =
{
    calibrated: false,

    dynamicLeanDeg: 0,
    previousTimestampMs: null,

    forwardAxis:
    {
        x: 0,
        y: 1,
        z: 0
    },

    steeringAxis: null,

    lastSteeringRateDegPerSecond: 0,
    lastRawRollRateDegPerSecond: 0,
    lastRawSteeringRateDegPerSecond: 0,
    lastOmegaForwardDegPerSecond: 0,
    lastOmegaSteeringDegPerSecond: 0,

    lastGyroPredictedAngleDeg: 0,
    lastFusedAngleDeg: 0,

    lastGyroDevice:
    {
        x: 0,
        y: 0,
        z: 0
    },

    lastGyroReference:
    {
        x: 0,
        y: 0,
        z: 0
    },

    lastMode: "REFERENCE",

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

        this.dynamicLeanDeg =
            Number.isFinite(sensorData?.absoluteLeanDeg)
                ? sensorData.absoluteLeanDeg
                : 0;

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
        this.steeringAxis =
            this.normalizeVector(axis);

        return this.steeringAxis !== null;
    },

    zeroAngle(timestampMs = null)
    {
        this.dynamicLeanDeg = 0;

        this.previousTimestampMs =
            Number.isFinite(timestampMs)
                ? timestampMs
                : null;

        return {
            success: this.calibrated
        };
    },

    update(sensorData)
    {
        if (!this.calibrated)
        {
            return this.emptyResult();
        }

        const timestampMs =
            Number.isFinite(sensorData?.timestamp)
                ? sensorData.timestamp
                : null;

        const gyroReference =
            this.validVector(
                sensorData?.gyroReference
            );

        const gyroDevice =
            this.validVector(
                sensorData?.gyroDevice
            );

        this.lastGyroDevice =
            gyroDevice ??
            {
                x: 0,
                y: 0,
                z: 0
            };

        this.lastGyroReference =
            gyroReference ??
            {
                x: 0,
                y: 0,
                z: 0
            };

        let rollRateDegPerSecond = 0;
        let steeringRateDegPerSecond = 0;

        if (gyroReference)
        {
            const rates =
                this.solveAngularRates(
                    gyroReference
                );

            rollRateDegPerSecond =
                rates.rollRate;

            steeringRateDegPerSecond =
                rates.steeringRate;

            this.lastRawRollRateDegPerSecond =
                rates.rollRate;

            this.lastRawSteeringRateDegPerSecond =
                rates.steeringRate;

            this.lastOmegaForwardDegPerSecond =
                rates.omegaForward;

            this.lastOmegaSteeringDegPerSecond =
                rates.omegaSteering;
        }
        else
        {
            this.lastRawRollRateDegPerSecond = 0;
            this.lastRawSteeringRateDegPerSecond = 0;
            this.lastOmegaForwardDegPerSecond = 0;
            this.lastOmegaSteeringDegPerSecond = 0;
        }

        this.lastSteeringRateDegPerSecond =
            steeringRateDegPerSecond;

        const absoluteLeanDeg =
            Number.isFinite(
                sensorData?.absoluteLeanDeg
            )
                ? sensorData.absoluteLeanDeg
                : null;

        let predictedAngleDeg =
            this.dynamicLeanDeg;

        if (
            timestampMs !== null &&
            this.previousTimestampMs !== null
        )
        {
            const deltaSeconds =
                (
                    timestampMs -
                    this.previousTimestampMs
                ) /
                1000;

            if (
                Number.isFinite(deltaSeconds) &&
                deltaSeconds > 0 &&
                deltaSeconds <= 0.25
            )
            {
                predictedAngleDeg =
                    this.dynamicLeanDeg +
                    rollRateDegPerSecond *
                    deltaSeconds;

                if (absoluteLeanDeg !== null)
                {
                    /*
                     * Gyro supplies the immediate response.
                     * The steering-profile result supplies the
                     * long-term absolute reference.
                     *
                     * During stronger steering, correction becomes
                     * faster so steering leakage cannot accumulate
                     * into a persistent offset.
                     */
                    const steeringMagnitude =
                        Math.abs(
                            steeringRateDegPerSecond
                        );

                    const correctionRatePerSecond =
                        steeringMagnitude >= 8
                            ? 8.0
                            : (
                                steeringMagnitude >= 3
                                    ? 4.0
                                    : 1.8
                            );

                    const correctionWeight =
                        Math.min(
                            0.35,
                            deltaSeconds *
                            correctionRatePerSecond
                        );

                    this.dynamicLeanDeg =
                        predictedAngleDeg +
                        correctionWeight *
                        this.angleDifferenceDeg(
                            absoluteLeanDeg,
                            predictedAngleDeg
                        );

                    this.lastMode =
                        steeringMagnitude >= 3
                            ? "STEER REF"
                            : "REFERENCE";
                }
                else
                {
                    this.dynamicLeanDeg =
                        predictedAngleDeg;

                    this.lastMode =
                        "GYRO ONLY";
                }
            }
        }

        this.previousTimestampMs =
            timestampMs;

        if (Math.abs(this.dynamicLeanDeg) < 0.05)
        {
            this.dynamicLeanDeg = 0;
        }

        this.lastGyroPredictedAngleDeg =
            predictedAngleDeg;

        this.lastFusedAngleDeg =
            this.dynamicLeanDeg;

        const confidence =
            absoluteLeanDeg !== null
                ? (
                    Math.abs(
                        steeringRateDegPerSecond
                    ) >= 3
                        ? 0.90
                        : 0.86
                )
                : 0.55;

        /*
         * Accelerometer lean is intentionally not used in v0.9.0.
         * Keep the field for the existing diagnostic UI/CSV schema.
         */
        const accelerometerLeanDeg =
            this.accelerometerLeanDeg(
                sensorData?.accel
            );

        return {
            leanAngle:
                this.dynamicLeanDeg,

            leanRate:
                rollRateDegPerSecond,

            steeringRate:
                steeringRateDegPerSecond,

            rawRollRate:
                this.lastRawRollRateDegPerSecond,

            rawSteeringRate:
                this.lastRawSteeringRateDegPerSecond,

            omegaForward:
                this.lastOmegaForwardDegPerSecond,

            omegaSteering:
                this.lastOmegaSteeringDegPerSecond,

            accelerometerLean:
                accelerometerLeanDeg,

            gyroPredictedAngle:
                this.lastGyroPredictedAngleDeg,

            fusedAngle:
                this.lastFusedAngleDeg,

            gyroDevice:
                this.lastGyroDevice,

            gyroReference:
                this.lastGyroReference,

            confidence,

            mode:
                this.lastMode
        };
    },

    solveAngularRates(angularVelocityReference)
    {
        const forward =
            this.forwardAxis;

        const omegaForward =
            this.dot(
                angularVelocityReference,
                forward
            );

        if (!this.steeringAxis)
        {
            return {
                rollRate:
                    omegaForward,

                steeringRate: 0,

                omegaForward,

                omegaSteering: 0
            };
        }

        const steering =
            this.steeringAxis;

        const coupling =
            this.dot(
                forward,
                steering
            );

        const denominator =
            1 -
            coupling *
            coupling;

        const omegaSteering =
            this.dot(
                angularVelocityReference,
                steering
            );

        if (
            !Number.isFinite(denominator) ||
            denominator < 0.02
        )
        {
            return {
                rollRate:
                    omegaForward,

                steeringRate:
                    omegaSteering,

                omegaForward,

                omegaSteering
            };
        }

        return {
            rollRate:
                (
                    omegaForward -
                    coupling *
                    omegaSteering
                ) /
                denominator,

            steeringRate:
                (
                    omegaSteering -
                    coupling *
                    omegaForward
                ) /
                denominator,

            omegaForward,

            omegaSteering
        };
    },

    accelerometerLeanDeg(acceleration)
    {
        /*
         * Diagnostic only. Never used to correct Dynamic Lean.
         */
        const gravity =
            this.normalizeVector(
                acceleration
            );

        if (!gravity)
        {
            return null;
        }

        const worldUp =
        {
            x: 0,
            y: 0,
            z: 1
        };

        const referencePlane =
            this.projectPerpendicular(
                worldUp,
                this.forwardAxis
            );

        const gravityPlane =
            this.projectPerpendicular(
                gravity,
                this.forwardAxis
            );

        if (
            !referencePlane ||
            !gravityPlane
        )
        {
            return null;
        }

        const sine =
            this.dot(
                this.cross(
                    referencePlane,
                    gravityPlane
                ),
                this.forwardAxis
            );

        const cosine =
            this.dot(
                referencePlane,
                gravityPlane
            );

        return (
            Math.atan2(
                sine,
                cosine
            ) *
            180 /
            Math.PI
        );
    },

    projectPerpendicular(vector, axis)
    {
        const projection =
            this.dot(
                vector,
                axis
            );

        return this.normalizeVector(
            {
                x:
                    vector.x -
                    axis.x *
                    projection,

                y:
                    vector.y -
                    axis.y *
                    projection,

                z:
                    vector.z -
                    axis.z *
                    projection
            }
        );
    },

    angleDifferenceDeg(target, current)
    {
        let difference =
            target -
            current;

        while (difference > 180)
        {
            difference -= 360;
        }

        while (difference < -180)
        {
            difference += 360;
        }

        return difference;
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
                valid.x *
                valid.x +
                valid.y *
                valid.y +
                valid.z *
                valid.z
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

    cross(a, b)
    {
        return {
            x:
                a.y * b.z -
                a.z * b.y,

            y:
                a.z * b.x -
                a.x * b.z,

            z:
                a.x * b.y -
                a.y * b.x
        };
    },

    emptyResult()
    {
        return {
            leanAngle: 0,
            leanRate: 0,
            steeringRate: 0,
            rawRollRate: 0,
            rawSteeringRate: 0,
            omegaForward: 0,
            omegaSteering: 0,
            accelerometerLean: null,
            gyroPredictedAngle: 0,
            fusedAngle: 0,

            gyroDevice:
            {
                x: 0,
                y: 0,
                z: 0
            },

            gyroReference:
            {
                x: 0,
                y: 0,
                z: 0
            },

            confidence: 0,
            mode: "UNCAL"
        };
    },

    reset()
    {
        this.calibrated = false;
        this.dynamicLeanDeg = 0;
        this.previousTimestampMs = null;

        this.forwardAxis =
        {
            x: 0,
            y: 1,
            z: 0
        };

        this.steeringAxis = null;

        this.lastSteeringRateDegPerSecond = 0;
        this.lastRawRollRateDegPerSecond = 0;
        this.lastRawSteeringRateDegPerSecond = 0;
        this.lastOmegaForwardDegPerSecond = 0;
        this.lastOmegaSteeringDegPerSecond = 0;
        this.lastGyroPredictedAngleDeg = 0;
        this.lastFusedAngleDeg = 0;

        this.lastGyroDevice =
        {
            x: 0,
            y: 0,
            z: 0
        };

        this.lastGyroReference =
        {
            x: 0,
            y: 0,
            z: 0
        };

        this.lastMode =
            "REFERENCE";
    }
};

export default LeanEstimator;
