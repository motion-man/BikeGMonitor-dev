const GRAVITY = 9.80665;

const startButton = document.getElementById("startButton");
const statusText = document.getElementById("status");

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

const intervalText = document.getElementById("interval");
const rateText = document.getElementById("rate");

let sensorsStarted = false;

startButton.addEventListener("click", startSensors);

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
        return;
    }

    intervalText.textContent = interval.toFixed(1);

    const frequency = 1000 / interval;

    rateText.textContent = frequency.toFixed(1);
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