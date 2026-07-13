const GRAVITY = 9.80665;
const CALIBRATION_SAMPLE_COUNT = 100;
const CALIBRATION_STORAGE_KEY = "bikeGMonitorCalibrationV1";

const startButton = document.getElementById("startButton");
const calibrateButton = document.getElementById("calibrateButton");

const statusText = document.getElementById("status");
const calibrationStatusText =
    document.getElementById("calibrationStatus");

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

let sensorsStarted = false;
let savedCalibration = null;
let maximumLeanAngle = 0;

let latestOrientation =
{
    alpha: null,
    beta: null,
    gamma: null
};

let calibrationActive = false;
let calibrationSampleCount = 0;

let calibrationTotals =
{
    x: 0,
    y: 0,
    z: 0,
    beta: 0,
    gamma: 0,
    orientationSamples: 0
};

startButton.addEventListener("click", startSensors);
calibrateButton.addEventListener("click", startCalibration);
resetMaxButton.addEventListener("click", resetMaximumLean);

loadSavedCalibration();
summaryDeviceText.textContent = detectDevice();

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
    }
    catch (error)
    {
        statusText.textContent =
            "Sensor error: " + error.message;
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

    updateLeanAngle(
        event.accelerationIncludingGravity
    );

    if (calibrationActive)
    {
        collectCalibrationSample(
            event.accelerationIncludingGravity
        );
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
        latestOrientation.beta !== null &&
        latestOrientation.gamma !== null
    )
    {
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

        beta: null,
        gamma: null,

        savedAt: new Date().toISOString()
    };

    if (calibrationTotals.orientationSamples > 0)
    {
        calibration.beta =
            calibrationTotals.beta /
            calibrationTotals.orientationSamples;

        calibration.gamma =
            calibrationTotals.gamma /
            calibrationTotals.orientationSamples;
    }

    try
    {
        localStorage.setItem(
            CALIBRATION_STORAGE_KEY,
            JSON.stringify(calibration)
        );

        savedCalibration = calibration;
        displayCalibration(calibration);
        summaryCalibrationText.textContent = "Saved";
        resetMaximumLean();

        calibrationStatusText.textContent =
            "Calibration complete and saved";
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
            typeof calibration.z !== "number"
        )
        {
            return;
        }

        savedCalibration = calibration;
        displayCalibration(calibration);
        summaryCalibrationText.textContent = "Saved";

        calibrationStatusText.textContent =
            "Saved calibration loaded";
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


function updateLeanAngle(acceleration)
{
    if (!savedCalibration)
    {
        leanAngleText.textContent = "N/A";
        leanStatusText.textContent = "Calibrate upright first";
        return;
    }

    if (!acceleration)
    {
        leanAngleText.textContent = "N/A";
        leanStatusText.textContent = "No motion data";
        return;
    }

    const currentGravity = normalizeVector(
        acceleration.x / GRAVITY,
        acceleration.y / GRAVITY,
        acceleration.z / GRAVITY
    );

    const uprightGravity = normalizeVector(
        savedCalibration.x,
        savedCalibration.y,
        savedCalibration.z
    );

    if (!currentGravity || !uprightGravity)
    {
        leanAngleText.textContent = "N/A";
        leanStatusText.textContent = "Invalid sensor data";
        return;
    }

    const screenTop = { x: 0, y: 1, z: 0 };

    const forward = normalizeObject(
        subtractVector(
            screenTop,
            scaleVector(
                uprightGravity,
                dotProduct(screenTop, uprightGravity)
            )
        )
    );

    if (!forward)
    {
        leanAngleText.textContent = "N/A";
        leanStatusText.textContent = "Mount angle is unsuitable";
        return;
    }

    const uprightRollPlane = normalizeObject(
        subtractVector(
            uprightGravity,
            scaleVector(
                forward,
                dotProduct(uprightGravity, forward)
            )
        )
    );

    const currentRollPlane = normalizeObject(
        subtractVector(
            currentGravity,
            scaleVector(
                forward,
                dotProduct(currentGravity, forward)
            )
        )
    );

    if (!uprightRollPlane || !currentRollPlane)
    {
        leanAngleText.textContent = "N/A";
        leanStatusText.textContent = "Unable to calculate";
        return;
    }

    const cross = crossProduct(
        uprightRollPlane,
        currentRollPlane
    );

    const sinAngle = dotProduct(forward, cross);
    const cosAngle = clamp(
        dotProduct(uprightRollPlane, currentRollPlane),
        -1,
        1
    );

    const signedAngle =
        Math.atan2(sinAngle, cosAngle) * 180 / Math.PI;

    const absoluteAngle = Math.abs(signedAngle);

    leanAngleText.textContent =
        absoluteAngle.toFixed(1) + "°";

    if (absoluteAngle < 0.8)
    {
        leanDirectionText.textContent = "UPRIGHT";
    }
    else if (signedAngle > 0)
    {
        leanDirectionText.textContent = "LEFT";
    }
    else
    {
        leanDirectionText.textContent = "RIGHT";
    }

    if (absoluteAngle > maximumLeanAngle)
    {
        maximumLeanAngle = absoluteAngle;
        maxLeanText.textContent =
            maximumLeanAngle.toFixed(1) + "°";
    }

    leanStatusText.textContent =
        "Calibrated bike lean";
}

function resetMaximumLean()
{
    maximumLeanAngle = 0;
    maxLeanText.textContent = "0.0°";
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
