import LeanEstimator from "./lean/estimator.js";const GRAVITY = 9.80665;
const CALIBRATION_SAMPLE_COUNT = 100;
const CALIBRATION_STORAGE_KEY = "bikeGMonitorCalibrationV4";
const TELEMETRY_FORMAT_VERSION = "1.0";
const APP_VERSION = "0.8.0-r13";

const startButton = document.getElementById("startButton");
const calibrateButton = document.getElementById("calibrateButton");

const steeringCalibrateButton =
    document.getElementById("steeringCalibrateButton");

const statusText = document.getElementById("status");
const calibrationStatusText =
    document.getElementById("calibrationStatus");

const steeringCalibrationStatusText =
    document.getElementById("steeringCalibrationStatus");

const xText = document.getElementById("x");
const yText = document.getElementById("y");
const zText = document.getElementById("z");
const totalText = document.getElementById("total");

const linearXText = document.getElementById("linearX");
const linearYText = document.getElementById("linearY");
const linearZText = document.getElementById("linearZ");

const rotationAlphaText =
    document.getElementById("rotationAlpha");

const rotationBetaText =
    document.getElementById("rotationBeta");

const rotationGammaText =
    document.getElementById("rotationGamma");

const orientationAlphaText =
    document.getElementById("orientationAlpha");

const orientationBetaText =
    document.getElementById("orientationBeta");

const orientationGammaText =
    document.getElementById("orientationGamma");

const calibrationXText =
    document.getElementById("calibrationX");

const calibrationYText =
    document.getElementById("calibrationY");

const calibrationZText =
    document.getElementById("calibrationZ");

const calibrationBetaText =
    document.getElementById("calibrationBeta");

const calibrationGammaText =
    document.getElementById("calibrationGamma");

const intervalText = document.getElementById("interval");
const rateText = document.getElementById("rate");

const summaryMotionText =
    document.getElementById("summaryMotion");

const summaryOrientationText =
    document.getElementById("summaryOrientation");

const summaryCalibrationText =
    document.getElementById("summaryCalibration");

const imuLeanText =
    document.getElementById("imuLean");

const imuRateText =
    document.getElementById("imuRate");

const imuConfidenceText =
    document.getElementById("imuConfidence");

const imuSteeringRateText =
    document.getElementById("imuSteeringRate");

const imuModeText =
    document.getElementById("imuMode");

const imuRawRollRateText =
    document.getElementById("imuRawRollRate");

const imuRawSteeringRateText =
    document.getElementById("imuRawSteeringRate");

const imuOmegaForwardText =
    document.getElementById("imuOmegaForward");

const imuOmegaSteeringText =
    document.getElementById("imuOmegaSteering");

const imuAccelRollText =
    document.getElementById("imuAccelRoll");

const imuGyroPredText =
    document.getElementById("imuGyroPred");

const imuFusedOutText =
    document.getElementById("imuFusedOut");

const summaryRateText =
    document.getElementById("summaryRate");

const summaryDeviceText =
    document.getElementById("summaryDevice");

const leanAngleText =
    document.getElementById("leanAngle");

const leanStatusText =
    document.getElementById("leanStatus");

const leanDirectionText =
    document.getElementById("leanDirection");

const maxLeanText =
    document.getElementById("maxLean");

const resetMaxButton =
    document.getElementById("resetMaxButton");

const wakeLockStatusText =
    document.getElementById("wakeLockStatus");

const accelGText =
    document.getElementById("accelG");

const brakeGText =
    document.getElementById("brakeG");

const maxAccelGText =
    document.getElementById("maxAccelG");

const maxBrakeGText =
    document.getElementById("maxBrakeG");

const signedForwardGText =
    document.getElementById("signedForwardG");

const startRideButton =
    document.getElementById("startRideButton");

const stopRideButton =
    document.getElementById("stopRideButton");

const rideStatusText =
    document.getElementById("rideStatus");

const rideTimerText =
    document.getElementById("rideTimer");

const sampleCountText =
    document.getElementById("sampleCount");

const speedText =
    document.getElementById("speed");

const maxSpeedText =
    document.getElementById("maxSpeed");

const gpsStatusText =
    document.getElementById("gpsStatus");

let sensorsStarted = false;
let savedCalibration = null;
let maximumLeanAngle = 0;
let filteredSignedLeanAngle = 0;
let wakeLockSentinel = null;

let filteredForwardG = 0;
let maximumAccelG = 0;
let maximumBrakeG = 0;

let rideRecording = false;
let rideStartEpochMs = 0;
let rideStartPerformanceMs = 0;
let rideSamples = [];
let rideTimerInterval = null;

let latestLeanAngle = 0;
let latestLeanDirection = "UPRIGHT";
let latestSignedForwardG = 0;

let latestAccelerationIncludingGravity =
{
    x: null,
    y: null,
    z: null
};

let latestLinearAcceleration =
{
    x: null,
    y: null,
    z: null
};

let latestRotationRate =
{
    alpha: null,
    beta: null,
    gamma: null
};

let gpsWatchId = null;
let latestLatitude = null;
let latestLongitude = null;
let latestGpsAccuracyM = null;
let latestGpsAltitudeM = null;
let latestGpsAltitudeAccuracyM = null;
let latestGpsHeadingDeg = null;
let latestGpsTimestampMs = null;
let latestSpeedKmh = 0;
let maximumSpeedKmh = 0;
let previousGpsFix = null;

let recordedIntervalTotalMs = 0;
let recordedIntervalCount = 0;

let latestOrientation =
{
    alpha: null,
    beta: null,
    gamma: null
};

let calibrationActive = false;
let calibrationSampleCount = 0;

let steeringCalibrationActive = false;
let steeringCalibrationSampleCount = 0;
let steeringGyroSampleCount = 0;
let steeringCalibrationTimer = null;

let steeringOrientationProfile = [];
let lastSteeringProfileSampleMs = 0;

let steeringCovariance =
{
    xx: 0,
    xy: 0,
    xz: 0,
    yy: 0,
    yz: 0,
    zz: 0
};

let calibrationTotals =
{
    x: 0,
    y: 0,
    z: 0,
    alphaSin: 0,
    alphaCos: 0,
    beta: 0,
    gamma: 0,
    orientationSamples: 0
};

startButton.addEventListener("click", startSensors);
calibrateButton.addEventListener("click", startCalibration);
steeringCalibrateButton.addEventListener(
    "click",
    startSteeringCalibration
);
resetMaxButton.addEventListener("click", resetMaximumLean);
startRideButton.addEventListener("click", startRideRecording);
stopRideButton.addEventListener("click", stopRideRecording);

loadSavedCalibration();
summaryDeviceText.textContent = detectDevice();

document.addEventListener(
    "visibilitychange",
    handleVisibilityChange
);

async function startSensors()
{
    if (sensorsStarted)
    {
        return;
    }

    try
    {
        const motionAllowed =
            await requestMotionPermission();

        if (!motionAllowed)
        {
            statusText.textContent =
                "Motion sensor permission denied";

            return;
        }

        const orientationAllowed =
            await requestOrientationPermission();

        if (!orientationAllowed)
        {
            statusText.textContent =
                "Orientation permission denied";

            return;
        }

        if (typeof DeviceMotionEvent === "undefined")
        {
            statusText.textContent =
                "Device motion is not supported";

            return;
        }

        window.addEventListener(
            "devicemotion",
            handleMotion,
            true
        );

        window.addEventListener(
            "deviceorientation",
            handleOrientation,
            true
        );

        sensorsStarted = true;

        statusText.textContent = "Sensors running";

        startButton.textContent = "Sensors Active";
        startButton.disabled = true;

        calibrateButton.disabled = false;

        steeringCalibrateButton.disabled =
            !savedCalibration;

        if (savedCalibration)
        {
            LeanEstimator.calibrate(
                {
                    timestamp: performance.now(),
                    forwardAxis:
                        savedCalibration.forwardVector,
                    steeringAxis:
                        savedCalibration.steeringAxis,
                    uprightGravity:
                    {
                        x: savedCalibration.x,
                        y: savedCalibration.y,
                        z: savedCalibration.z
                    }
                }
            );
        }

        await requestScreenWakeLock();
        startGpsTracking();
    }
    catch (error)
    {
        statusText.textContent =
            "Sensor error: " + error.message;
    }
}


async function requestScreenWakeLock()
{
    if (!("wakeLock" in navigator))
    {
        wakeLockStatusText.textContent = "Not supported";
        return;
    }

    if (document.visibilityState !== "visible")
    {
        wakeLockStatusText.textContent = "Waiting for active screen";
        return;
    }

    try
    {
        if (wakeLockSentinel && !wakeLockSentinel.released)
        {
            wakeLockStatusText.textContent = "Active";
            return;
        }

        wakeLockSentinel =
            await navigator.wakeLock.request("screen");

        wakeLockStatusText.textContent = "Active";

        wakeLockSentinel.addEventListener(
            "release",
            function ()
            {
                wakeLockStatusText.textContent = "Released";
            }
        );
    }
    catch (error)
    {
        wakeLockSentinel = null;
        wakeLockStatusText.textContent = "Unavailable";
    }
}

async function handleVisibilityChange()
{
    if (
        document.visibilityState === "visible" &&
        sensorsStarted
    )
    {
        await requestScreenWakeLock();
    }
}

async function requestMotionPermission()
{
    if (
        typeof DeviceMotionEvent !== "undefined" &&
        typeof DeviceMotionEvent.requestPermission === "function"
    )
    {
        const result =
            await DeviceMotionEvent.requestPermission();

        return result === "granted";
    }

    return true;
}

async function requestOrientationPermission()
{
    if (
        typeof DeviceOrientationEvent !== "undefined" &&
        typeof DeviceOrientationEvent.requestPermission === "function"
    )
    {
        const result =
            await DeviceOrientationEvent.requestPermission();

        return result === "granted";
    }

    return true;
}

function handleMotion(event)
{
    summaryMotionText.textContent = "Working";

    storeLatestMotionValues(event);

    const angularVelocityDevice =
    {
        x: event.rotationRate?.beta ?? 0,
        y: event.rotationRate?.gamma ?? 0,
        z: event.rotationRate?.alpha ?? 0
    };

    const relativeOrientationMatrix =
        getCurrentRelativeOrientationMatrix();

    const angularVelocityReference =
        relativeOrientationMatrix
            ? multiplyMatrixVector3(
                relativeOrientationMatrix,
                angularVelocityDevice
            )
            : null;

    const imuResult = LeanEstimator.update(
        {
            timestamp:
                typeof event.timeStamp === "number"
                    ? event.timeStamp
                    : performance.now(),

            accel:
            {
                x:
                    event.accelerationIncludingGravity?.x ?? 0,

                y:
                    event.accelerationIncludingGravity?.y ?? 0,

                z:
                    event.accelerationIncludingGravity?.z ?? 0
            },

            linearAccel:
            {
                x:
                    event.acceleration?.x ?? 0,

                y:
                    event.acceleration?.y ?? 0,

                z:
                    event.acceleration?.z ?? 0
            },

            gyroDevice:
                angularVelocityDevice,

            gyroReference:
                angularVelocityReference,

            intervalMs:
                typeof event.interval === "number"
                    ? event.interval
                    : null,

            gps:
            {
                speedKmh:
                    latestSpeedKmh,

                headingDeg:
                    latestGpsHeadingDeg,

                accuracyM:
                    latestGpsAccuracyM
            }
        }
    );

    window.latestImuResult = imuResult;

    imuLeanText.textContent =
        imuResult.leanAngle.toFixed(1) + "°";

    imuRateText.textContent =
        imuResult.leanRate.toFixed(1) + "°/s";

    imuConfidenceText.textContent =
        imuResult.confidence.toFixed(2);

    imuSteeringRateText.textContent =
        imuResult.steeringRate.toFixed(1) + "°/s";

    imuModeText.textContent =
        imuResult.mode;

    imuRawRollRateText.textContent =
        imuResult.rawRollRate.toFixed(1) + "°/s";

    imuRawSteeringRateText.textContent =
        imuResult.rawSteeringRate.toFixed(1) + "°/s";

    imuOmegaForwardText.textContent =
        imuResult.omegaForward.toFixed(1) + "°/s";

    imuOmegaSteeringText.textContent =
        imuResult.omegaSteering.toFixed(1) + "°/s";

    imuAccelRollText.textContent =
        Number.isFinite(imuResult.accelerometerLean)
            ? imuResult.accelerometerLean.toFixed(1) + "°"
            : "N/A";

    imuGyroPredText.textContent =
        imuResult.gyroPredictedAngle.toFixed(1) + "°";

    imuFusedOutText.textContent =
        imuResult.fusedAngle.toFixed(1) + "°";

    updateAccelerationIncludingGravity(
        event.accelerationIncludingGravity
    );

    updateLinearAcceleration(
        event.acceleration
    );

    updateRotationRate(
        event.rotationRate
    );

    updateSamplingInformation(
        event.interval
    );

    updateForwardG(
        event.acceleration
    );

    if (calibrationActive)
    {
        collectCalibrationSample(
            event.accelerationIncludingGravity
        );
    }

    if (steeringCalibrationActive)
    {
        collectSteeringCalibrationSample(
            event.rotationRate
        );
    }

    if (rideRecording)
    {
        recordRideSample(event.interval);
    }
}

function updateAccelerationIncludingGravity(acceleration)
{
    if (!acceleration)
    {
        return;
    }

    const x = acceleration.x ?? 0;
    const y = acceleration.y ?? 0;
    const z = acceleration.z ?? 0;

    const xG = x / GRAVITY;
    const yG = y / GRAVITY;
    const zG = z / GRAVITY;

    const totalG = Math.sqrt(
        xG * xG +
        yG * yG +
        zG * zG
    );

    xText.textContent = xG.toFixed(3);
    yText.textContent = yG.toFixed(3);
    zText.textContent = zG.toFixed(3);
    totalText.textContent = totalG.toFixed(3);
}

function updateLinearAcceleration(acceleration)
{
    if (!acceleration)
    {
        linearXText.textContent = "N/A";
        linearYText.textContent = "N/A";
        linearZText.textContent = "N/A";
        return;
    }

    const x = (acceleration.x ?? 0) / GRAVITY;
    const y = (acceleration.y ?? 0) / GRAVITY;
    const z = (acceleration.z ?? 0) / GRAVITY;

    linearXText.textContent = x.toFixed(3);
    linearYText.textContent = y.toFixed(3);
    linearZText.textContent = z.toFixed(3);
}

function updateRotationRate(rotationRate)
{
    if (!rotationRate)
    {
        rotationAlphaText.textContent = "N/A";
        rotationBetaText.textContent = "N/A";
        rotationGammaText.textContent = "N/A";
        return;
    }

    rotationAlphaText.textContent =
        formatNumber(rotationRate.alpha, 1);

    rotationBetaText.textContent =
        formatNumber(rotationRate.beta, 1);

    rotationGammaText.textContent =
        formatNumber(rotationRate.gamma, 1);
}

function handleOrientation(event)
{
    summaryOrientationText.textContent = "Working";

    latestOrientation.alpha =
        validNumberOrNull(event.alpha);

    latestOrientation.beta =
        validNumberOrNull(event.beta);

    latestOrientation.gamma =
        validNumberOrNull(event.gamma);

    orientationAlphaText.textContent =
        formatNumber(event.alpha, 1);

    orientationBetaText.textContent =
        formatNumber(event.beta, 1);

    orientationGammaText.textContent =
        formatNumber(event.gamma, 1);

    if (steeringCalibrationActive)
    {
        collectSteeringProfileSample();
    }

    updateLeanAngleFromOrientation();
}

function getCurrentRelativeOrientationMatrix()
{
    if (
        !savedCalibration ||
        latestOrientation.alpha === null ||
        latestOrientation.beta === null ||
        latestOrientation.gamma === null
    )
    {
        return null;
    }

    const currentMatrix =
        deviceOrientationMatrix(
            latestOrientation.alpha,
            latestOrientation.beta,
            latestOrientation.gamma
        );

    const referenceMatrix =
        savedCalibration.orientationMatrix;

    if (
        !currentMatrix ||
        !Array.isArray(referenceMatrix) ||
        referenceMatrix.length !== 9
    )
    {
        return null;
    }

    /*
     * Maps vectors from the phone's current moving coordinate frame
     * into the fixed upright-calibration coordinate frame.
     */
    return multiplyMatrices3(
        transposeMatrix3(referenceMatrix),
        currentMatrix
    );
}

function getCurrentRelativeOrientationQuaternion()
{
    const relativeMatrix =
        getCurrentRelativeOrientationMatrix();

    return relativeMatrix
        ? quaternionFromMatrix3(relativeMatrix)
        : null;
}


function updateSamplingInformation(interval)
{
    const validInterval =
        typeof interval === "number" &&
        Number.isFinite(interval) &&
        interval > 0;

    if (!validInterval)
    {
        intervalText.textContent = "N/A";
        rateText.textContent = "N/A";
        summaryRateText.textContent = "N/A";
        return;
    }

    intervalText.textContent = interval.toFixed(1);

    const frequency = 1000 / interval;

    rateText.textContent = frequency.toFixed(1);
    summaryRateText.textContent = frequency.toFixed(1) + " Hz";
}

function startCalibration()
{
    if (!sensorsStarted || calibrationActive)
    {
        return;
    }

    calibrationActive = true;
    calibrationSampleCount = 0;

    calibrationTotals =
    {
        x: 0,
        y: 0,
        z: 0,
        alphaSin: 0,
        alphaCos: 0,
        beta: 0,
        gamma: 0,
        orientationSamples: 0
    };

    calibrateButton.disabled = true;
    calibrateButton.textContent = "Calibrating...";

    calibrationStatusText.textContent =
        "Hold the bike upright and stationary";
}

function collectCalibrationSample(acceleration)
{
    if (!acceleration)
    {
        return;
    }

    const x = validNumberOrNull(acceleration.x);
    const y = validNumberOrNull(acceleration.y);
    const z = validNumberOrNull(acceleration.z);

    if (x === null || y === null || z === null)
    {
        return;
    }

    calibrationTotals.x += x / GRAVITY;
    calibrationTotals.y += y / GRAVITY;
    calibrationTotals.z += z / GRAVITY;

    calibrationSampleCount++;

    if (
        latestOrientation.alpha !== null &&
        latestOrientation.beta !== null &&
        latestOrientation.gamma !== null
    )
    {
        const alphaRadians =
            latestOrientation.alpha * Math.PI / 180;

        calibrationTotals.alphaSin +=
            Math.sin(alphaRadians);

        calibrationTotals.alphaCos +=
            Math.cos(alphaRadians);

        calibrationTotals.beta += latestOrientation.beta;
        calibrationTotals.gamma += latestOrientation.gamma;
        calibrationTotals.orientationSamples++;
    }

    calibrationStatusText.textContent =
        "Hold still: " +
        calibrationSampleCount +
        " / " +
        CALIBRATION_SAMPLE_COUNT;

    if (
        calibrationSampleCount >=
        CALIBRATION_SAMPLE_COUNT
    )
    {
        finishCalibration();
    }
}

function finishCalibration()
{
    const calibration =
    {
        x: calibrationTotals.x /
            calibrationSampleCount,

        y: calibrationTotals.y /
            calibrationSampleCount,

        z: calibrationTotals.z /
            calibrationSampleCount,

        alpha: null,
        beta: null,
        gamma: null,
        orientationMatrix: null,
        orientationUpVector: null,
        forwardVector: null,
        steeringAxis: null,
        steeringProfile: null,
        savedAt: new Date().toISOString()
    };

    if (calibrationTotals.orientationSamples > 0)
    {
        calibration.alpha =
            normalizeDegrees(
                Math.atan2(
                    calibrationTotals.alphaSin,
                    calibrationTotals.alphaCos
                ) * 180 / Math.PI
            );

        calibration.beta =
            calibrationTotals.beta /
            calibrationTotals.orientationSamples;

        calibration.gamma =
            calibrationTotals.gamma /
            calibrationTotals.orientationSamples;

        calibration.orientationMatrix =
            deviceOrientationMatrix(
                calibration.alpha,
                calibration.beta,
                calibration.gamma
            );

        calibration.orientationUpVector =
            worldUpInDeviceCoordinates(
                calibration.beta,
                calibration.gamma
            );
    }

    const uprightGravity = normalizeVector(
        calibration.x,
        calibration.y,
        calibration.z
    );

    if (uprightGravity)
    {
        const screenTop = { x: 0, y: 1, z: 0 };

        calibration.forwardVector = normalizeObject(
            subtractVector(
                screenTop,
                scaleVector(
                    uprightGravity,
                    dotProduct(screenTop, uprightGravity)
                )
            )
        );
    }

    if (
        !calibration.orientationMatrix ||
        !calibration.orientationUpVector ||
        !calibration.forwardVector
    )
    {
        calibrationActive = false;
        calibrateButton.disabled = false;
        calibrateButton.textContent = "Calibrate";
        calibrationStatusText.textContent =
            "Orientation unavailable — calibration failed";
        return;
    }

    try
    {
        localStorage.setItem(
            CALIBRATION_STORAGE_KEY,
            JSON.stringify(calibration)
        );

        savedCalibration = calibration;
        LeanEstimator.calibrate(
            {
                timestamp: performance.now(),
                forwardAxis: calibration.forwardVector,
                steeringAxis: calibration.steeringAxis,
                uprightGravity:
                {
                    x: calibration.x,
                    y: calibration.y,
                    z: calibration.z
                }
            }
        );
        filteredSignedLeanAngle = 0;
        displayCalibration(calibration);
        summaryCalibrationText.textContent = "Saved";
        resetMaximumLean();

        steeringCalibrateButton.disabled = false;

        calibrationStatusText.textContent =
            "Upright calibration saved";

        steeringCalibrationStatusText.textContent =
            "Next: keep bike upright and calibrate steering";
    }
    catch (error)
    {
        calibrationStatusText.textContent =
            "Calibration complete, but could not be saved";
    }

    calibrationActive = false;

    calibrateButton.disabled = false;
    calibrateButton.textContent = "Calibrate";
}

function loadSavedCalibration()
{
    try
    {
        const savedValue =
            localStorage.getItem(
                CALIBRATION_STORAGE_KEY
            );

        if (!savedValue)
        {
            return;
        }

        const calibration = JSON.parse(savedValue);

        if (
            typeof calibration.x !== "number" ||
            typeof calibration.y !== "number" ||
            typeof calibration.z !== "number" ||
            !Array.isArray(calibration.orientationMatrix) ||
            calibration.orientationMatrix.length !== 9 ||
            !calibration.orientationUpVector ||
            !calibration.forwardVector
        )
        {
            calibrationStatusText.textContent =
                "Fresh upright calibration required";
            return;
        }

        savedCalibration = calibration;
        displayCalibration(calibration);
        summaryCalibrationText.textContent = "Saved";

        calibrationStatusText.textContent =
            "Saved upright calibration loaded";

        steeringCalibrateButton.disabled = false;

        steeringCalibrationStatusText.textContent =
            Array.isArray(calibration.steeringProfile)
                ? "Saved steering profile loaded"
                : "Steering profile calibration required";
    }
    catch (error)
    {
        calibrationStatusText.textContent =
            "Saved calibration could not be loaded";
    }
}

function displayCalibration(calibration)
{
    calibrationXText.textContent =
        formatNumber(calibration.x, 3);

    calibrationYText.textContent =
        formatNumber(calibration.y, 3);

    calibrationZText.textContent =
        formatNumber(calibration.z, 3);

    calibrationBetaText.textContent =
        formatNumber(calibration.beta, 1);

    calibrationGammaText.textContent =
        formatNumber(calibration.gamma, 1);
}


function updateLeanAngleFromOrientation()
{
    if (!savedCalibration)
    {
        leanAngleText.textContent = "N/A";
        leanStatusText.textContent = "Calibrate upright first";
        return;
    }

    if (
        !Array.isArray(savedCalibration.steeringProfile) ||
        savedCalibration.steeringProfile.length < 10
    )
    {
        leanAngleText.textContent = "N/A";
        leanStatusText.textContent =
            "Calibrate steering profile";
        return;
    }

    if (
        latestOrientation.alpha === null ||
        latestOrientation.beta === null ||
        latestOrientation.gamma === null
    )
    {
        leanAngleText.textContent = "N/A";
        leanStatusText.textContent = "Waiting for orientation";
        return;
    }

    const currentMatrix =
        deviceOrientationMatrix(
            latestOrientation.alpha,
            latestOrientation.beta,
            latestOrientation.gamma
        );

    const referenceMatrix =
        savedCalibration.orientationMatrix;

    if (
        !currentMatrix ||
        !Array.isArray(referenceMatrix) ||
        referenceMatrix.length !== 9
    )
    {
        leanAngleText.textContent = "N/A";
        leanStatusText.textContent = "Invalid orientation data";
        return;
    }

    const relativeMatrix =
        multiplyMatrices3(
            transposeMatrix3(referenceMatrix),
            currentMatrix
        );

    const currentQuaternion =
        quaternionFromMatrix3(relativeMatrix);

    const forward =
        normalizeObject(
            savedCalibration.forwardVector
        );

    if (!currentQuaternion || !forward)
    {
        leanAngleText.textContent = "N/A";
        leanStatusText.textContent = "Recalibrate upright";
        return;
    }

    const result =
        estimateLeanFromSteeringProfile(
            currentQuaternion,
            savedCalibration.steeringProfile,
            forward
        );

    if (!result)
    {
        leanAngleText.textContent = "N/A";
        leanStatusText.textContent =
            "Unable to match steering profile";
        return;
    }

    const rawSignedAngle =
        result.signedLean;

    const angleDifference =
        normalizeLeanDifference(
            rawSignedAngle,
            filteredSignedLeanAngle
        );

    if (Math.abs(angleDifference) <= 20)
    {
        const smoothing = 0.38;

        filteredSignedLeanAngle +=
            smoothing * angleDifference;
    }

    if (Math.abs(filteredSignedLeanAngle) < 0.35)
    {
        filteredSignedLeanAngle = 0;
    }

    const absoluteAngle =
        Math.abs(filteredSignedLeanAngle);

    latestLeanAngle = absoluteAngle;

    leanAngleText.textContent =
        absoluteAngle.toFixed(1) + "°";

    if (absoluteAngle < 0.5)
    {
        latestLeanDirection = "UPRIGHT";
        leanDirectionText.textContent = "UPRIGHT";
    }
    else if (filteredSignedLeanAngle > 0)
    {
        latestLeanDirection = "RIGHT";
        leanDirectionText.textContent = "RIGHT";
    }
    else
    {
        latestLeanDirection = "LEFT";
        leanDirectionText.textContent = "LEFT";
    }

    if (absoluteAngle > maximumLeanAngle)
    {
        maximumLeanAngle = absoluteAngle;
        maxLeanText.textContent =
            maximumLeanAngle.toFixed(1) + "°";
    }

    leanStatusText.textContent =
        "Steering-profile compensated lean";
}








function startGpsTracking()
{
    if (!("geolocation" in navigator))
    {
        gpsStatusText.textContent = "GPS not supported";
        return;
    }

    if (gpsWatchId !== null)
    {
        return;
    }

    gpsStatusText.textContent = "Requesting location";

    gpsWatchId =
        navigator.geolocation.watchPosition(
            handleGpsPosition,
            handleGpsError,
            {
                enableHighAccuracy: true,
                maximumAge: 1000,
                timeout: 15000
            }
        );
}

function handleGpsPosition(position)
{
    const coordinates = position.coords;

    latestLatitude =
        validNumberOrNull(coordinates.latitude);

    latestLongitude =
        validNumberOrNull(coordinates.longitude);

    latestGpsAccuracyM =
        validNumberOrNull(coordinates.accuracy);

    latestGpsAltitudeM =
        validNumberOrNull(coordinates.altitude);

    latestGpsAltitudeAccuracyM =
        validNumberOrNull(coordinates.altitudeAccuracy);

    latestGpsHeadingDeg =
        validNumberOrNull(coordinates.heading);

    latestGpsTimestampMs =
        validNumberOrNull(position.timestamp);

    let speedKmh = null;

    if (
        typeof coordinates.speed === "number" &&
        Number.isFinite(coordinates.speed) &&
        coordinates.speed >= 0
    )
    {
        speedKmh = coordinates.speed * 3.6;
    }
    else
    {
        speedKmh =
            deriveGpsSpeedKmh(
                latestLatitude,
                latestLongitude,
                latestGpsTimestampMs
            );
    }

    if (
        typeof speedKmh === "number" &&
        Number.isFinite(speedKmh) &&
        speedKmh >= 0
    )
    {
        /*
         * Suppress normal stationary GPS wander.
         */
        latestSpeedKmh =
            speedKmh < 1.5 ? 0 : speedKmh;

        speedText.textContent =
            latestSpeedKmh.toFixed(0) + " km/h";

        if (latestSpeedKmh > maximumSpeedKmh)
        {
            maximumSpeedKmh = latestSpeedKmh;

            maxSpeedText.textContent =
                maximumSpeedKmh.toFixed(0) + " km/h";
        }
    }

    if (
        typeof latestGpsAccuracyM === "number" &&
        Number.isFinite(latestGpsAccuracyM)
    )
    {
        gpsStatusText.textContent =
            "GPS active — accuracy " +
            latestGpsAccuracyM.toFixed(0) +
            " m";
    }
    else
    {
        gpsStatusText.textContent = "GPS active";
    }

    previousGpsFix =
    {
        latitude: latestLatitude,
        longitude: latestLongitude,
        timestampMs: latestGpsTimestampMs
    };
}

function deriveGpsSpeedKmh(
    latitude,
    longitude,
    timestampMs
)
{
    if (
        previousGpsFix === null ||
        latitude === null ||
        longitude === null ||
        timestampMs === null
    )
    {
        return null;
    }

    const elapsedSeconds =
        (timestampMs - previousGpsFix.timestampMs) /
        1000;

    if (
        !Number.isFinite(elapsedSeconds) ||
        elapsedSeconds <= 0
    )
    {
        return null;
    }

    const distanceMetres =
        haversineDistanceMetres(
            previousGpsFix.latitude,
            previousGpsFix.longitude,
            latitude,
            longitude
        );

    if (!Number.isFinite(distanceMetres))
    {
        return null;
    }

    return (distanceMetres / elapsedSeconds) * 3.6;
}

function haversineDistanceMetres(
    latitude1,
    longitude1,
    latitude2,
    longitude2
)
{
    const earthRadiusMetres = 6371000;

    const lat1 = latitude1 * Math.PI / 180;
    const lat2 = latitude2 * Math.PI / 180;

    const deltaLatitude =
        (latitude2 - latitude1) *
        Math.PI /
        180;

    const deltaLongitude =
        (longitude2 - longitude1) *
        Math.PI /
        180;

    const a =
        Math.sin(deltaLatitude / 2) *
        Math.sin(deltaLatitude / 2) +
        Math.cos(lat1) *
        Math.cos(lat2) *
        Math.sin(deltaLongitude / 2) *
        Math.sin(deltaLongitude / 2);

    const c =
        2 *
        Math.atan2(
            Math.sqrt(a),
            Math.sqrt(1 - a)
        );

    return earthRadiusMetres * c;
}

function handleGpsError(error)
{
    latestSpeedKmh = 0;
    speedText.textContent = "--";

    switch (error.code)
    {
        case error.PERMISSION_DENIED:
            gpsStatusText.textContent =
                "Location permission denied";
            break;

        case error.POSITION_UNAVAILABLE:
            gpsStatusText.textContent =
                "GPS position unavailable";
            break;

        case error.TIMEOUT:
            gpsStatusText.textContent =
                "Waiting for GPS";
            break;

        default:
            gpsStatusText.textContent =
                "GPS error";
            break;
    }
}

function storeLatestMotionValues(event)
{
    const gravityAcceleration =
        event.accelerationIncludingGravity;

    if (gravityAcceleration)
    {
        latestAccelerationIncludingGravity =
        {
            x: validNumberOrNull(gravityAcceleration.x),
            y: validNumberOrNull(gravityAcceleration.y),
            z: validNumberOrNull(gravityAcceleration.z)
        };
    }

    const linearAcceleration = event.acceleration;

    if (linearAcceleration)
    {
        latestLinearAcceleration =
        {
            x: validNumberOrNull(linearAcceleration.x),
            y: validNumberOrNull(linearAcceleration.y),
            z: validNumberOrNull(linearAcceleration.z)
        };
    }

    const rotationRate = event.rotationRate;

    if (rotationRate)
    {
        latestRotationRate =
        {
            alpha: validNumberOrNull(rotationRate.alpha),
            beta: validNumberOrNull(rotationRate.beta),
            gamma: validNumberOrNull(rotationRate.gamma)
        };
    }
}

function startRideRecording()
{
    if (!sensorsStarted)
    {
        rideStatusText.textContent =
            "Start sensors before recording";
        return;
    }

    if (!savedCalibration)
    {
        rideStatusText.textContent =
            "Calibrate upright before recording";
        return;
    }

    if (rideRecording)
    {
        return;
    }

    rideSamples = [];
    recordedIntervalTotalMs = 0;
    recordedIntervalCount = 0;

    rideStartEpochMs = Date.now();
    rideStartPerformanceMs = performance.now();
    rideRecording = true;

    startRideButton.disabled = true;
    stopRideButton.disabled = false;

    rideStatusText.textContent = "Recording";
    sampleCountText.textContent = "0";
    rideTimerText.textContent = "00:00:00.0";

    if (rideTimerInterval !== null)
    {
        clearInterval(rideTimerInterval);
    }

    rideTimerInterval = window.setInterval(
        updateRideTimer,
        100
    );
}

function stopRideRecording()
{
    if (!rideRecording)
    {
        return;
    }

    rideRecording = false;

    if (rideTimerInterval !== null)
    {
        clearInterval(rideTimerInterval);
        rideTimerInterval = null;
    }

    startRideButton.disabled = false;
    stopRideButton.disabled = true;

    updateRideTimer();

    if (rideSamples.length === 0)
    {
        rideStatusText.textContent =
            "Stopped — no samples recorded";
        return;
    }

    rideStatusText.textContent =
        "Stopped — preparing CSV";

    downloadRideCsv();

    rideStatusText.textContent =
        "CSV downloaded: " +
        rideSamples.length +
        " samples";
}

function updateRideTimer()
{
    if (rideStartPerformanceMs <= 0)
    {
        rideTimerText.textContent = "00:00:00.0";
        return;
    }

    const elapsedMs =
        Math.max(
            0,
            performance.now() - rideStartPerformanceMs
        );

    rideTimerText.textContent =
        formatElapsedTime(elapsedMs);
}

function formatElapsedTime(elapsedMs)
{
    const totalTenths =
        Math.floor(elapsedMs / 100);

    const tenths = totalTenths % 10;
    const totalSeconds =
        Math.floor(totalTenths / 10);

    const seconds = totalSeconds % 60;
    const totalMinutes =
        Math.floor(totalSeconds / 60);

    const minutes = totalMinutes % 60;
    const hours =
        Math.floor(totalMinutes / 60);

    return (
        String(hours).padStart(2, "0") +
        ":" +
        String(minutes).padStart(2, "0") +
        ":" +
        String(seconds).padStart(2, "0") +
        "." +
        String(tenths)
    );
}

function recordRideSample(sensorIntervalMs)
{
    if (
        typeof sensorIntervalMs === "number" &&
        Number.isFinite(sensorIntervalMs) &&
        sensorIntervalMs > 0
    )
    {
        recordedIntervalTotalMs += sensorIntervalMs;
        recordedIntervalCount++;
    }
    const nowPerformanceMs = performance.now();

    const elapsedMs =
        nowPerformanceMs - rideStartPerformanceMs;

    const timestampMs =
        rideStartEpochMs + elapsedMs;

    const accelG =
        Math.max(0, latestSignedForwardG);

    const brakeG =
        Math.max(0, -latestSignedForwardG);

    rideSamples.push(
        {
            timestamp_iso:
                new Date(timestampMs).toISOString(),

            elapsed_ms:
                Number(elapsedMs.toFixed(3)),

            lean_deg:
                Number(latestLeanAngle.toFixed(3)),

            lean_direction:
                latestLeanDirection,

            signed_forward_g:
                Number(latestSignedForwardG.toFixed(5)),

            accel_g:
                Number(accelG.toFixed(5)),

            brake_g:
                Number(brakeG.toFixed(5)),

            speed_kmh:
                Number(latestSpeedKmh.toFixed(3)),

            latitude:
                latestLatitude,

            longitude:
                latestLongitude,

            gps_accuracy_m:
                latestGpsAccuracyM,

            altitude_m:
                latestGpsAltitudeM,

            altitude_accuracy_m:
                latestGpsAltitudeAccuracyM,

            heading_deg:
                latestGpsHeadingDeg,

            gps_timestamp_ms:
                latestGpsTimestampMs,

            accel_gravity_x_ms2:
                latestAccelerationIncludingGravity.x,

            accel_gravity_y_ms2:
                latestAccelerationIncludingGravity.y,

            accel_gravity_z_ms2:
                latestAccelerationIncludingGravity.z,

            linear_accel_x_ms2:
                latestLinearAcceleration.x,

            linear_accel_y_ms2:
                latestLinearAcceleration.y,

            linear_accel_z_ms2:
                latestLinearAcceleration.z,

            gyro_alpha_dps:
                latestRotationRate.alpha,

            gyro_beta_dps:
                latestRotationRate.beta,

            gyro_gamma_dps:
                latestRotationRate.gamma,

            orientation_alpha_deg:
                latestOrientation.alpha,

            orientation_beta_deg:
                latestOrientation.beta,

            orientation_gamma_deg:
                latestOrientation.gamma
        }
    );

    sampleCountText.textContent =
        String(rideSamples.length);
}

function downloadRideCsv()
{
    const header =
    [
        "timestamp_iso",
        "elapsed_ms",
        "lean_deg",
        "lean_direction",
        "signed_forward_g",
        "accel_g",
        "brake_g",
        "speed_kmh",
        "latitude",
        "longitude",
        "gps_accuracy_m",
        "altitude_m",
        "altitude_accuracy_m",
        "heading_deg",
        "gps_timestamp_ms",
        "accel_gravity_x_ms2",
        "accel_gravity_y_ms2",
        "accel_gravity_z_ms2",
        "linear_accel_x_ms2",
        "linear_accel_y_ms2",
        "linear_accel_z_ms2",
        "gyro_alpha_dps",
        "gyro_beta_dps",
        "gyro_gamma_dps",
        "orientation_alpha_deg",
        "orientation_beta_deg",
        "orientation_gamma_deg"
    ];

    const averageSampleRateHz =
        recordedIntervalCount > 0
            ? 1000 /
              (
                  recordedIntervalTotalMs /
                  recordedIntervalCount
              )
            : null;

    const metadataRows =
    [
        ["metadata", "telemetry_format_version", TELEMETRY_FORMAT_VERSION],
        ["metadata", "app_version", APP_VERSION],
        ["metadata", "device", detectDevice()],
        ["metadata", "user_agent", navigator.userAgent],
        [
            "metadata",
            "ride_start_iso",
            new Date(rideStartEpochMs).toISOString()
        ],
        [
            "metadata",
            "sample_count",
            rideSamples.length
        ],
        [
            "metadata",
            "average_sample_rate_hz",
            averageSampleRateHz === null
                ? ""
                : averageSampleRateHz.toFixed(3)
        ],
        []
    ];

    const rows =
        metadataRows.map(
            function (row)
            {
                return row.map(csvValue).join(",");
            }
        );

    rows.push(header.join(","));

    for (const sample of rideSamples)
    {
        rows.push(
            header
                .map(
                    function (column)
                    {
                        return csvValue(sample[column]);
                    }
                )
                .join(",")
        );
    }

    const csvText = rows.join("\r\n");

    const blob =
        new Blob(
            [csvText],
            {
                type:
                    "text/csv;charset=utf-8"
            }
        );

    const url =
        URL.createObjectURL(blob);

    const downloadLink =
        document.createElement("a");

    downloadLink.href = url;
    downloadLink.download =
        buildRideFilename();

    document.body.appendChild(downloadLink);
    downloadLink.click();
    downloadLink.remove();

    window.setTimeout(
        function ()
        {
            URL.revokeObjectURL(url);
        },
        1000
    );
}

function buildRideFilename()
{
    const date =
        new Date(rideStartEpochMs);

    const year =
        String(date.getFullYear());

    const month =
        String(date.getMonth() + 1).padStart(2, "0");

    const day =
        String(date.getDate()).padStart(2, "0");

    const hours =
        String(date.getHours()).padStart(2, "0");

    const minutes =
        String(date.getMinutes()).padStart(2, "0");

    const seconds =
        String(date.getSeconds()).padStart(2, "0");

    return (
        "BikeRide_" +
        year +
        month +
        day +
        "_" +
        hours +
        minutes +
        seconds +
        ".csv"
    );
}

function csvValue(value)
{
    if (value === null || value === undefined)
    {
        return "";
    }

    const text = String(value);

    if (
        text.includes(",") ||
        text.includes("\"") ||
        text.includes("\n") ||
        text.includes("\r")
    )
    {
        return (
            "\"" +
            text.replaceAll("\"", "\"\"") +
            "\""
        );
    }

    return text;
}

function getBikeForwardVector()
{
    if (!savedCalibration)
    {
        return null;
    }

    const uprightGravity = normalizeVector(
        savedCalibration.x,
        savedCalibration.y,
        savedCalibration.z
    );

    if (!uprightGravity)
    {
        return null;
    }

    /*
     * Mounting rule:
     * Phone is portrait, screen facing the rider, and the top edge
     * points approximately toward the front of the motorcycle.
     */
    const screenTop = { x: 0, y: 1, z: 0 };

    return normalizeObject(
        subtractVector(
            screenTop,
            scaleVector(
                uprightGravity,
                dotProduct(screenTop, uprightGravity)
            )
        )
    );
}

function updateForwardG(acceleration)
{
    if (!savedCalibration)
    {
        accelGText.textContent = "--";
        brakeGText.textContent = "--";
        signedForwardGText.textContent = "Calibrate first";
        return;
    }

    if (!acceleration)
    {
        accelGText.textContent = "N/A";
        brakeGText.textContent = "N/A";
        signedForwardGText.textContent =
            "Linear acceleration unavailable";
        return;
    }

    const forward = getBikeForwardVector();

    if (!forward)
    {
        accelGText.textContent = "N/A";
        brakeGText.textContent = "N/A";
        signedForwardGText.textContent =
            "Unable to determine forward axis";
        return;
    }

    const linearAccelerationG =
    {
        x: (acceleration.x ?? 0) / GRAVITY,
        y: (acceleration.y ?? 0) / GRAVITY,
        z: (acceleration.z ?? 0) / GRAVITY
    };

    const rawForwardG =
        dotProduct(linearAccelerationG, forward);

    /*
     * Light smoothing reduces phone and handlebar vibration while
     * preserving useful acceleration and braking response.
     */
    const smoothing = 0.18;

    filteredForwardG =
        filteredForwardG +
        smoothing * (rawForwardG - filteredForwardG);

    const deadband = 0.015;

    if (Math.abs(filteredForwardG) < deadband)
    {
        filteredForwardG = 0;
    }

    latestSignedForwardG = filteredForwardG;

    const accelerationG =
        Math.max(0, filteredForwardG);

    const brakingG =
        Math.max(0, -filteredForwardG);

    accelGText.textContent =
        accelerationG.toFixed(2) + " G";

    brakeGText.textContent =
        brakingG.toFixed(2) + " G";

    signedForwardGText.textContent =
        "Signed forward G: " +
        (filteredForwardG >= 0 ? "+" : "") +
        filteredForwardG.toFixed(3);

    if (accelerationG > maximumAccelG)
    {
        maximumAccelG = accelerationG;
        maxAccelGText.textContent =
            maximumAccelG.toFixed(2) + " G";
    }

    if (brakingG > maximumBrakeG)
    {
        maximumBrakeG = brakingG;
        maxBrakeGText.textContent =
            maximumBrakeG.toFixed(2) + " G";
    }
}

function resetMaximumLean()
{
    maximumLeanAngle = 0;
    filteredSignedLeanAngle = 0;
    maximumAccelG = 0;
    maximumBrakeG = 0;
    maximumSpeedKmh = 0;

    maxLeanText.textContent = "0.0°";
    maxAccelGText.textContent = "0.00 G";
    maxBrakeGText.textContent = "0.00 G";
    maxSpeedText.textContent = "0 km/h";
}

function startSteeringCalibration()
{
    if (
        !sensorsStarted ||
        !savedCalibration ||
        steeringCalibrationActive
    )
    {
        return;
    }

    steeringCalibrationActive = true;
    steeringCalibrationSampleCount = 0;
    steeringGyroSampleCount = 0;
    steeringOrientationProfile = [];
    lastSteeringProfileSampleMs = 0;

    steeringCovariance =
    {
        xx: 0,
        xy: 0,
        xz: 0,
        yy: 0,
        yz: 0,
        zz: 0
    };

    steeringCalibrateButton.disabled = true;
    calibrateButton.disabled = true;

    steeringCalibrateButton.textContent =
        "Sweeping...";

    steeringCalibrationStatusText.textContent =
        "Keep bike upright; sweep handlebars fully left/right repeatedly";

    steeringCalibrationTimer =
        window.setTimeout(
            finishSteeringCalibration,
            6000
        );
}

function collectSteeringProfileSample()
{
    if (
        !savedCalibration ||
        latestOrientation.alpha === null ||
        latestOrientation.beta === null ||
        latestOrientation.gamma === null
    )
    {
        return;
    }

    const nowMs = performance.now();

    /*
     * About 20 samples per second is enough to describe the complete
     * steering path without creating an unnecessarily large profile.
     */
    if (nowMs - lastSteeringProfileSampleMs < 50)
    {
        return;
    }

    const currentMatrix =
        deviceOrientationMatrix(
            latestOrientation.alpha,
            latestOrientation.beta,
            latestOrientation.gamma
        );

    const referenceMatrix =
        savedCalibration.orientationMatrix;

    if (
        !currentMatrix ||
        !Array.isArray(referenceMatrix) ||
        referenceMatrix.length !== 9
    )
    {
        return;
    }

    const relativeMatrix =
        multiplyMatrices3(
            transposeMatrix3(referenceMatrix),
            currentMatrix
        );

    const relativeQuaternion =
        quaternionFromMatrix3(relativeMatrix);

    if (!relativeQuaternion)
    {
        return;
    }

    steeringOrientationProfile.push(
        relativeQuaternion
    );

    steeringCalibrationSampleCount++;
    lastSteeringProfileSampleMs = nowMs;
}


function collectSteeringCalibrationSample(rotationRate)
{
    if (
        !rotationRate ||
        !savedCalibration ||
        latestOrientation.alpha === null ||
        latestOrientation.beta === null ||
        latestOrientation.gamma === null
    )
    {
        return;
    }

    const angularVelocityDevice =
    {
        x:
            typeof rotationRate.beta === "number"
                ? rotationRate.beta
                : 0,

        y:
            typeof rotationRate.gamma === "number"
                ? rotationRate.gamma
                : 0,

        z:
            typeof rotationRate.alpha === "number"
                ? rotationRate.alpha
                : 0
    };

    const magnitude =
        Math.sqrt(
            angularVelocityDevice.x *
                angularVelocityDevice.x +
            angularVelocityDevice.y *
                angularVelocityDevice.y +
            angularVelocityDevice.z *
                angularVelocityDevice.z
        );

    /*
     * Ignore stationary gyro noise. Only deliberate steering motion
     * contributes to the measured steering axis.
     */
    if (!Number.isFinite(magnitude) || magnitude < 5)
    {
        return;
    }

    const currentMatrix =
        deviceOrientationMatrix(
            latestOrientation.alpha,
            latestOrientation.beta,
            latestOrientation.gamma
        );

    const referenceMatrix =
        savedCalibration.orientationMatrix;

    if (
        !currentMatrix ||
        !Array.isArray(referenceMatrix) ||
        referenceMatrix.length !== 9
    )
    {
        return;
    }

    /*
     * rotationRate is reported in the phone's CURRENT device frame.
     * That frame turns with the handlebars. Convert every sample into
     * the fixed upright calibration frame before accumulating it.
     *
     * relativeMatrix maps current-device coordinates into the
     * calibrated reference-device coordinates.
     */
    const relativeMatrix =
        multiplyMatrices3(
            transposeMatrix3(referenceMatrix),
            currentMatrix
        );

    const angularVelocityReference =
        multiplyMatrixVector3(
            relativeMatrix,
            angularVelocityDevice
        );

    const referenceMagnitude =
        Math.sqrt(
            angularVelocityReference.x *
                angularVelocityReference.x +
            angularVelocityReference.y *
                angularVelocityReference.y +
            angularVelocityReference.z *
                angularVelocityReference.z
        );

    if (
        !Number.isFinite(referenceMagnitude) ||
        referenceMagnitude < 5
    )
    {
        return;
    }

    steeringCovariance.xx +=
        angularVelocityReference.x *
        angularVelocityReference.x;

    steeringCovariance.xy +=
        angularVelocityReference.x *
        angularVelocityReference.y;

    steeringCovariance.xz +=
        angularVelocityReference.x *
        angularVelocityReference.z;

    steeringCovariance.yy +=
        angularVelocityReference.y *
        angularVelocityReference.y;

    steeringCovariance.yz +=
        angularVelocityReference.y *
        angularVelocityReference.z;

    steeringCovariance.zz +=
        angularVelocityReference.z *
        angularVelocityReference.z;

    steeringGyroSampleCount++;
}


function finishSteeringCalibration()
{
    steeringCalibrationActive = false;

    if (steeringCalibrationTimer !== null)
    {
        clearTimeout(steeringCalibrationTimer);
        steeringCalibrationTimer = null;
    }

    steeringCalibrateButton.disabled = false;
    calibrateButton.disabled = false;

    steeringCalibrateButton.textContent =
        "Calibrate Steering";

    if (steeringOrientationProfile.length < 25)
    {
        steeringCalibrationStatusText.textContent =
            "Not enough steering profile samples — retry";
        return;
    }

    if (steeringGyroSampleCount < 10)
    {
        steeringCalibrationStatusText.textContent =
            "Not enough steering gyro motion — retry";
        return;
    }

    const steeringAxis =
        principalAxisFromCovariance(
            steeringCovariance
        );

    if (!steeringAxis)
    {
        steeringCalibrationStatusText.textContent =
            "Could not determine steering axis — retry";
        return;
    }

    /*
     * Axis direction is arbitrary. Choose a repeatable sign so saved
     * calibrations remain stable between runs.
     */
    const dominantComponent =
        Math.abs(steeringAxis.x) >= Math.abs(steeringAxis.y) &&
        Math.abs(steeringAxis.x) >= Math.abs(steeringAxis.z)
            ? steeringAxis.x
            : (
                Math.abs(steeringAxis.y) >= Math.abs(steeringAxis.z)
                    ? steeringAxis.y
                    : steeringAxis.z
            );

    if (dominantComponent < 0)
    {
        steeringAxis.x *= -1;
        steeringAxis.y *= -1;
        steeringAxis.z *= -1;
    }

    savedCalibration.steeringProfile =
        steeringOrientationProfile;

    savedCalibration.steeringAxis =
        steeringAxis;

    savedCalibration.steeringSavedAt =
        new Date().toISOString();

    try
    {
        localStorage.setItem(
            CALIBRATION_STORAGE_KEY,
            JSON.stringify(savedCalibration)
        );

        filteredSignedLeanAngle = 0;
        resetMaximumLean();

        LeanEstimator.setSteeringAxis(
            savedCalibration.steeringAxis
        );

        /*
         * The steering sweep may finish with the bars at either lock.
         * Give the rider time to centre the bars before establishing
         * the final IMU upright zero.
         */
        steeringCalibrateButton.disabled = true;
        calibrateButton.disabled = true;

        steeringCalibrationStatusText.textContent =
            "Centre bars and hold bike upright — zeroing in 4 seconds";

        window.setTimeout(
            function ()
            {
                LeanEstimator.zeroAngle(
                    performance.now()
                );

                steeringCalibrateButton.disabled = false;
                calibrateButton.disabled = false;

                steeringCalibrationStatusText.textContent =
                    "IMU zeroed — begin upright/left/right test";
            },
            4000
        );
    }
    catch (error)
    {
        steeringCalibrationStatusText.textContent =
            "Steering calibration found, but could not be saved";
    }
}


function principalAxisFromCovariance(covariance)
{
    const matrix =
    [
        covariance.xx,
        covariance.xy,
        covariance.xz,

        covariance.xy,
        covariance.yy,
        covariance.yz,

        covariance.xz,
        covariance.yz,
        covariance.zz
    ];

    let vector =
        normalizeObject(
            {
                x: 0.2,
                y: 0.8,
                z: 0.5
            }
        );

    for (let iteration = 0; iteration < 20; iteration++)
    {
        vector =
            normalizeObject(
                multiplyMatrixVector3(
                    matrix,
                    vector
                )
            );

        if (!vector)
        {
            return null;
        }
    }

    return vector;
}

function estimateLeanFromSteeringProfile(
    currentQuaternion,
    steeringProfile,
    forwardAxis
)
{
    let bestResult = null;
    let bestScore = Infinity;

    for (const steeringQuaternion of steeringProfile)
    {
        const steering =
            normalizeQuaternion(steeringQuaternion);

        if (!steering)
        {
            continue;
        }

        /*
         * Remove this candidate steering orientation from the current
         * phone orientation.
         */
        const residual =
            multiplyQuaternions(
                currentQuaternion,
                conjugateQuaternion(steering)
            );

        if (!residual)
        {
            continue;
        }

        /*
         * Motorcycle lean should appear primarily as rotation around
         * the calibrated bike-forward axis.
         */
        const rollTwist =
            extractQuaternionTwist(
                residual,
                forwardAxis
            );

        const nonRollResidual =
            multiplyQuaternions(
                residual,
                conjugateQuaternion(rollTwist)
            );

        const score =
            quaternionRotationMagnitudeDegrees(
                nonRollResidual
            );

        if (score < bestScore)
        {
            bestScore = score;

            bestResult =
            {
                signedLean:
                    signedQuaternionAngleDegrees(
                        rollTwist,
                        forwardAxis
                    ),

                score: score
            };
        }
    }

    return bestResult;
}

function signedQuaternionAngleDegrees(
    quaternion,
    axis
)
{
    const normalized =
        normalizeQuaternion(quaternion);

    if (!normalized)
    {
        return 0;
    }

    const signedVector =
        normalized.x * axis.x +
        normalized.y * axis.y +
        normalized.z * axis.z;

    return (
        2 *
        Math.atan2(
            signedVector,
            normalized.w
        ) *
        180 /
        Math.PI
    );
}

function quaternionRotationMagnitudeDegrees(
    quaternion
)
{
    const normalized =
        normalizeQuaternion(quaternion);

    if (!normalized)
    {
        return Infinity;
    }

    const clampedW =
        clamp(
            Math.abs(normalized.w),
            -1,
            1
        );

    return (
        2 *
        Math.acos(clampedW) *
        180 /
        Math.PI
    );
}


function quaternionFromMatrix3(matrix)
{
    const trace =
        matrix[0] +
        matrix[4] +
        matrix[8];

    let quaternion;

    if (trace > 0)
    {
        const scale =
            Math.sqrt(trace + 1) * 2;

        quaternion =
        {
            w: 0.25 * scale,
            x: (matrix[7] - matrix[5]) / scale,
            y: (matrix[2] - matrix[6]) / scale,
            z: (matrix[3] - matrix[1]) / scale
        };
    }
    else if (
        matrix[0] > matrix[4] &&
        matrix[0] > matrix[8]
    )
    {
        const scale =
            Math.sqrt(
                1 +
                matrix[0] -
                matrix[4] -
                matrix[8]
            ) * 2;

        quaternion =
        {
            w: (matrix[7] - matrix[5]) / scale,
            x: 0.25 * scale,
            y: (matrix[1] + matrix[3]) / scale,
            z: (matrix[2] + matrix[6]) / scale
        };
    }
    else if (matrix[4] > matrix[8])
    {
        const scale =
            Math.sqrt(
                1 +
                matrix[4] -
                matrix[0] -
                matrix[8]
            ) * 2;

        quaternion =
        {
            w: (matrix[2] - matrix[6]) / scale,
            x: (matrix[1] + matrix[3]) / scale,
            y: 0.25 * scale,
            z: (matrix[5] + matrix[7]) / scale
        };
    }
    else
    {
        const scale =
            Math.sqrt(
                1 +
                matrix[8] -
                matrix[0] -
                matrix[4]
            ) * 2;

        quaternion =
        {
            w: (matrix[3] - matrix[1]) / scale,
            x: (matrix[2] + matrix[6]) / scale,
            y: (matrix[5] + matrix[7]) / scale,
            z: 0.25 * scale
        };
    }

    return normalizeQuaternion(quaternion);
}

function normalizeQuaternion(quaternion)
{
    const magnitude =
        Math.sqrt(
            quaternion.w * quaternion.w +
            quaternion.x * quaternion.x +
            quaternion.y * quaternion.y +
            quaternion.z * quaternion.z
        );

    if (!Number.isFinite(magnitude) || magnitude < 0.000001)
    {
        return null;
    }

    return {
        w: quaternion.w / magnitude,
        x: quaternion.x / magnitude,
        y: quaternion.y / magnitude,
        z: quaternion.z / magnitude
    };
}

function extractQuaternionTwist(
    quaternion,
    axis
)
{
    const projection =
        dotProduct(
            {
                x: quaternion.x,
                y: quaternion.y,
                z: quaternion.z
            },
            axis
        );

    const twist =
        normalizeQuaternion(
            {
                w: quaternion.w,
                x: axis.x * projection,
                y: axis.y * projection,
                z: axis.z * projection
            }
        );

    return (
        twist ||
        {
            w: 1,
            x: 0,
            y: 0,
            z: 0
        }
    );
}

function conjugateQuaternion(quaternion)
{
    return {
        w: quaternion.w,
        x: -quaternion.x,
        y: -quaternion.y,
        z: -quaternion.z
    };
}

function multiplyQuaternions(a, b)
{
    return normalizeQuaternion(
        {
            w:
                a.w * b.w -
                a.x * b.x -
                a.y * b.y -
                a.z * b.z,

            x:
                a.w * b.x +
                a.x * b.w +
                a.y * b.z -
                a.z * b.y,

            y:
                a.w * b.y -
                a.x * b.z +
                a.y * b.w +
                a.z * b.x,

            z:
                a.w * b.z +
                a.x * b.y -
                a.y * b.x +
                a.z * b.w
        }
    );
}

function rotateVectorByQuaternion(
    vector,
    quaternion
)
{
    const vectorQuaternion =
    {
        w: 0,
        x: vector.x,
        y: vector.y,
        z: vector.z
    };

    const rotated =
        multiplyQuaternionRaw(
            multiplyQuaternionRaw(
                quaternion,
                vectorQuaternion
            ),
            conjugateQuaternion(quaternion)
        );

    return {
        x: rotated.x,
        y: rotated.y,
        z: rotated.z
    };
}

function multiplyQuaternionRaw(a, b)
{
    return {
        w:
            a.w * b.w -
            a.x * b.x -
            a.y * b.y -
            a.z * b.z,

        x:
            a.w * b.x +
            a.x * b.w +
            a.y * b.z -
            a.z * b.y,

        y:
            a.w * b.y -
            a.x * b.z +
            a.y * b.w +
            a.z * b.x,

        z:
            a.w * b.z +
            a.x * b.y -
            a.y * b.x +
            a.z * b.w
    };
}


function worldUpInDeviceCoordinates(beta, gamma)
{
    if (
        !Number.isFinite(beta) ||
        !Number.isFinite(gamma)
    )
    {
        return null;
    }

    const betaRadians =
        beta * Math.PI / 180;

    const gammaRadians =
        gamma * Math.PI / 180;

    /*
     * This is the third row of the transposed device-orientation
     * matrix applied to world-up. It depends only on beta/gamma,
     * making it invariant to alpha/yaw.
     */
    return normalizeObject(
        {
            x:
                -Math.cos(betaRadians) *
                Math.sin(gammaRadians),

            y:
                Math.sin(betaRadians),

            z:
                Math.cos(betaRadians) *
                Math.cos(gammaRadians)
        }
    );
}


function normalizeLeanDifference(target, current)
{
    let difference = target - current;

    while (difference > 180)
    {
        difference -= 360;
    }

    while (difference < -180)
    {
        difference += 360;
    }

    return difference;
}


function deviceOrientationMatrix(alpha, beta, gamma)
{
    if (
        !Number.isFinite(alpha) ||
        !Number.isFinite(beta) ||
        !Number.isFinite(gamma)
    )
    {
        return null;
    }

    const z = alpha * Math.PI / 180;
    const x = beta * Math.PI / 180;
    const y = gamma * Math.PI / 180;

    const cZ = Math.cos(z);
    const sZ = Math.sin(z);
    const cX = Math.cos(x);
    const sX = Math.sin(x);
    const cY = Math.cos(y);
    const sY = Math.sin(y);

    return [
        cZ * cY - sZ * sX * sY,
        -cX * sZ,
        cY * sZ * sX + cZ * sY,

        cY * sZ + cZ * sX * sY,
        cZ * cX,
        sZ * sY - cZ * cY * sX,

        -cX * sY,
        sX,
        cX * cY
    ];
}

function transposeMatrix3(matrix)
{
    return [
        matrix[0], matrix[3], matrix[6],
        matrix[1], matrix[4], matrix[7],
        matrix[2], matrix[5], matrix[8]
    ];
}

function multiplyMatrices3(a, b)
{
    const result = new Array(9);

    for (let row = 0; row < 3; row++)
    {
        for (let column = 0; column < 3; column++)
        {
            result[row * 3 + column] =
                a[row * 3] * b[column] +
                a[row * 3 + 1] * b[3 + column] +
                a[row * 3 + 2] * b[6 + column];
        }
    }

    return result;
}

function multiplyMatrixVector3(matrix, vector)
{
    return {
        x:
            matrix[0] * vector.x +
            matrix[1] * vector.y +
            matrix[2] * vector.z,

        y:
            matrix[3] * vector.x +
            matrix[4] * vector.y +
            matrix[5] * vector.z,

        z:
            matrix[6] * vector.x +
            matrix[7] * vector.y +
            matrix[8] * vector.z
    };
}

function normalizeDegrees(degrees)
{
    return ((degrees % 360) + 360) % 360;
}

function normalizeVector(x, y, z)
{
    return normalizeObject({ x, y, z });
}

function normalizeObject(vector)
{
    const magnitude = Math.sqrt(
        vector.x * vector.x +
        vector.y * vector.y +
        vector.z * vector.z
    );

    if (!Number.isFinite(magnitude) || magnitude < 0.000001)
    {
        return null;
    }

    return {
        x: vector.x / magnitude,
        y: vector.y / magnitude,
        z: vector.z / magnitude
    };
}

function dotProduct(a, b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

function crossProduct(a, b)
{
    return {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x
    };
}

function subtractVector(a, b)
{
    return {
        x: a.x - b.x,
        y: a.y - b.y,
        z: a.z - b.z
    };
}

function scaleVector(vector, scale)
{
    return {
        x: vector.x * scale,
        y: vector.y * scale,
        z: vector.z * scale
    };
}

function clamp(value, minimum, maximum)
{
    return Math.min(maximum, Math.max(minimum, value));
}

function validNumberOrNull(value)
{
    if (
        typeof value !== "number" ||
        !Number.isFinite(value)
    )
    {
        return null;
    }

    return value;
}

function formatNumber(value, decimals)
{
    if (
        typeof value !== "number" ||
        !Number.isFinite(value)
    )
    {
        return "N/A";
    }

    return value.toFixed(decimals);
}

function detectDevice()
{
    const ua = navigator.userAgent;

    if (/Android/i.test(ua))
    {
        return "Android";
    }

    if (/iPhone/i.test(ua))
    {
        return "iPhone";
    }

    if (/iPad/i.test(ua))
    {
        return "iPad";
    }

    return "Unknown";
}
