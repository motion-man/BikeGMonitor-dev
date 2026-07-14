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
    uprightGravity: null,

    lastSteeringRateDegPerSecond: 0,
    lastIntegrationWeight: 1,
    lastRawRollRateDegPerSecond: 0,
    lastRawSteeringRateDegPerSecond: 0,
    lastOmegaForwardDegPerSecond: 0,
    lastOmegaSteeringDegPerSecond: 0,
    lastAccelerometerLeanDeg: null,
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
    lastMode: "GYRO",

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

        this.uprightGravity =
            this.normalizeVector(
                sensorData?.uprightGravity
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
        let confidence = 0.35;

        if (gyroReference)
        {
            const rates =
                this.solveAngularRates(
                    gyroReference
                );

            rollRateDegPerSecond =
                rates.rollRate;

            this.lastSteeringRateDegPerSecond =
                rates.steeringRate;

            this.lastRawRollRateDegPerSecond =
                rates.rollRate;

            this.lastRawSteeringRateDegPerSecond =
                rates.steeringRate;

            this.lastOmegaForwardDegPerSecond =
                rates.omegaForward;

            this.lastOmegaSteeringDegPerSecond =
                rates.omegaSteering;

            this.lastIntegrationWeight =
                this.steeringIntegrationWeight(
                    rates.rollRate,
                    rates.steeringRate
                );

            confidence =
                this.steeringAxis
                    ? 0.70
                    : 0.50;
        }
        else
        {
            this.lastSteeringRateDegPerSecond = 0;
            this.lastRawRollRateDegPerSecond = 0;
            this.lastRawSteeringRateDegPerSecond = 0;
            this.lastOmegaForwardDegPerSecond = 0;
            this.lastOmegaSteeringDegPerSecond = 0;
            this.lastIntegrationWeight = 1;
        }

        let accelerometerCorrectionUsed = false;

        this.lastAccelerometerLeanDeg = null;
        this.lastGyroPredictedAngleDeg =
            this.rollAngleDeg;
        this.lastFusedAngleDeg =
            this.rollAngleDeg;

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
                const effectiveRollRateDegPerSecond =
                    rollRateDegPerSecond *
                    this.lastIntegrationWeight;

                const predictedAngleDeg =
                    this.rollAngleDeg +
                    effectiveRollRateDegPerSecond *
                    deltaSeconds;

                const accelerometerLeanDeg =
                    this.accelerometerLeanDeg(
                        sensorData?.accel
                    );

                this.lastAccelerometerLeanDeg =
                    accelerometerLeanDeg;

                this.lastGyroPredictedAngleDeg =
                    predictedAngleDeg;

                const totalG =
                    this.vectorMagnitude(
                        sensorData?.accel
                    ) /
                    9.80665;

                const linearAccelerationG =
                    this.vectorMagnitude(
                        sensorData?.linearAccel
                    ) /
                    9.80665;

                const accelerometerReliable =
                    Number.isFinite(accelerometerLeanDeg) &&
                    Number.isFinite(totalG) &&
                    totalG >= 0.92 &&
                    totalG <= 1.08 &&
                    Number.isFinite(linearAccelerationG) &&
                    linearAccelerationG <= 0.12 &&
                    Math.abs(rollRateDegPerSecond) <= 25 &&
                    Math.abs(
                        this.angleDifferenceDeg(
                            accelerometerLeanDeg,
                            predictedAngleDeg
                        )
                    ) <= 12;

                if (accelerometerReliable)
                {
                    /*
                     * Slow complementary correction:
                     * gyro supplies immediate response; gravity only
                     * removes long-term drift under calm conditions.
                     */
                    const correctionWeight =
                        Math.min(
                            0.012,
                            deltaSeconds * 0.35
                        );

                    this.rollAngleDeg =
                        predictedAngleDeg +
                        correctionWeight *
                        this.angleDifferenceDeg(
                            accelerometerLeanDeg,
                            predictedAngleDeg
                        );

                    accelerometerCorrectionUsed = true;
                }
                else
                {
                    this.rollAngleDeg =
                        predictedAngleDeg;
                }

                this.lastFusedAngleDeg =
                    this.rollAngleDeg;
            }
        }

        this.previousTimestampMs = timestampMs;

        if (Math.abs(this.rollAngleDeg) < 0.05)
        {
            this.rollAngleDeg = 0;
        }

        this.lastFusedAngleDeg =
            this.rollAngleDeg;

        if (accelerometerCorrectionUsed)
        {
            confidence =
                Math.max(
                    confidence,
                    0.82
                );
        }

        if (this.lastIntegrationWeight < 0.95)
        {
            this.lastMode = "STEER";
        }
        else if (accelerometerCorrectionUsed)
        {
            this.lastMode = "FUSED";
        }
        else
        {
            this.lastMode = "GYRO";
        }

        return {
            leanAngle: this.rollAngleDeg,
            leanRate: rollRateDegPerSecond,
            steeringRate:
                this.lastSteeringRateDegPerSecond,
            rawRollRate:
                this.lastRawRollRateDegPerSecond,
            rawSteeringRate:
                this.lastRawSteeringRateDegPerSecond,
            omegaForward:
                this.lastOmegaForwardDegPerSecond,
            omegaSteering:
                this.lastOmegaSteeringDegPerSecond,
            accelerometerLean:
                this.lastAccelerometerLeanDeg,
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

        if (!this.steeringAxis)
        {
            const omegaForward =
                this.dot(
                    angularVelocityReference,
                    forward
                );

            return {
                rollRate:
                    omegaForward,

                steeringRate: 0,

                omegaForward:
                    omegaForward,

                omegaSteering: 0
            };
        }

        const steering =
            this.steeringAxis;

        /*
         * Resolve:
         *
         *   omega = rollRate * forwardAxis
         *         + steeringRate * steeringAxis
         *
         * These values are diagnostic in r10. Integration behaviour
         * is otherwise unchanged from r9.
         */
        const coupling =
            this.dot(
                forward,
                steering
            );

        const denominator =
            1 - coupling * coupling;

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

                omegaForward:
                    omegaForward,

                omegaSteering:
                    omegaSteering
            };
        }

        return {
            rollRate:
                (
                    omegaForward -
                    coupling * omegaSteering
                ) /
                denominator,

            steeringRate:
                (
                    omegaSteering -
                    coupling * omegaForward
                ) /
                denominator,

            omegaForward:
                omegaForward,

            omegaSteering:
                omegaSteering
        };
    },

    steeringIntegrationWeight(
        rollRateDegPerSecond,
        steeringRateDegPerSecond
    )
    {
        const rollMagnitude =
            Math.abs(
                rollRateDegPerSecond
            );

        const steeringMagnitude =
            Math.abs(
                steeringRateDegPerSecond
            );

        /*
         * Do not gate tiny sensor noise.
         */
        if (steeringMagnitude < 2)
        {
            return 1;
        }

        const totalMotion =
            rollMagnitude +
            steeringMagnitude;

        if (totalMotion < 0.000001)
        {
            return 1;
        }

        const steeringShare =
            steeringMagnitude /
            totalMotion;

        /*
         * Roll-dominant motion is integrated normally.
         * As steering becomes dominant, progressively reduce how much
         * of the solved roll rate is allowed into the angle integrator.
         */
        if (steeringShare <= 0.50)
        {
            return 1;
        }

        if (steeringShare >= 0.75)
        {
            return 0.10;
        }

        const fraction =
            (
                steeringShare -
                0.50
            ) /
            0.25;

        return (
            1 -
            fraction * 0.90
        );
    },

    accelerometerLeanDeg(acceleration)
    {
        const currentGravity =
            this.normalizeVector(
                acceleration
            );

        if (
            !currentGravity ||
            !this.uprightGravity
        )
        {
            return null;
        }

        const referencePlane =
            this.projectPerpendicular(
                this.uprightGravity,
                this.forwardAxis
            );

        const currentPlane =
            this.projectPerpendicular(
                currentGravity,
                this.forwardAxis
            );

        if (
            !referencePlane ||
            !currentPlane
        )
        {
            return null;
        }

        const sine =
            this.dot(
                this.cross(
                    referencePlane,
                    currentPlane
                ),
                this.forwardAxis
            );

        const cosine =
            this.dot(
                referencePlane,
                currentPlane
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
                    axis.x * projection,

                y:
                    vector.y -
                    axis.y * projection,

                z:
                    vector.z -
                    axis.z * projection
            }
        );
    },

    vectorMagnitude(vector)
    {
        const valid =
            this.validVector(vector);

        if (!valid)
        {
            return NaN;
        }

        return Math.sqrt(
            valid.x * valid.x +
            valid.y * valid.y +
            valid.z * valid.z
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

    angleDifferenceDeg(target, current)
    {
        let difference =
            target - current;

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
        this.uprightGravity = null;
        this.lastSteeringRateDegPerSecond = 0;
        this.lastIntegrationWeight = 1;
        this.lastRawRollRateDegPerSecond = 0;
        this.lastRawSteeringRateDegPerSecond = 0;
        this.lastOmegaForwardDegPerSecond = 0;
        this.lastOmegaSteeringDegPerSecond = 0;
        this.lastAccelerometerLeanDeg = null;
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
        this.lastMode = "GYRO";

        this.forwardAxis =
        {
            x: 0,
            y: 1,
            z: 0
        };
    }
};

export default LeanEstimator;
