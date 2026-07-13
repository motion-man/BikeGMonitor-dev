const startButton = document.getElementById("startButton");
const statusText = document.getElementById("status");

const xText = document.getElementById("x");
const yText = document.getElementById("y");
const zText = document.getElementById("z");
const totalText = document.getElementById("total");

const GRAVITY = 9.80665;

let sensorsStarted = false;

startButton.onclick = startSensors;

async function startSensors()
{
    if (sensorsStarted)
    {
        return;
    }

    try
    {
        /*
         * iPhone and iPad require motion permission
         * to be requested from a button press.
         */
        if (
            typeof DeviceMotionEvent !== "undefined" &&
            typeof DeviceMotionEvent.requestPermission === "function"
        )
        {
            const permission =
                await DeviceMotionEvent.requestPermission();

            if (permission !== "granted")
            {
                statusText.innerHTML =
                    "Motion sensor permission denied";

                return;
            }
        }

        if (typeof DeviceMotionEvent === "undefined")
        {
            statusText.innerHTML =
                "Motion sensors are not supported on this device";

            return;
        }

        window.addEventListener(
            "devicemotion",
            handleMotion,
            true
        );

        sensorsStarted = true;

        statusText.innerHTML = "Sensors running";
        startButton.innerHTML = "Sensors Active";
        startButton.disabled = true;
    }
    catch (error)
    {
        statusText.innerHTML =
            "Sensor error: " + error.message;
    }
}

function handleMotion(event)
{
    /*
     * accelerationIncludingGravity includes the
     * Earth's gravity, so a stationary phone should
     * show a total close to 1.000 G.
     */
    const acceleration =
        event.accelerationIncludingGravity;

    if (!acceleration)
    {
        statusText.innerHTML =
            "No accelerometer data received";

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

    xText.innerHTML = xG.toFixed(3);
    yText.innerHTML = yG.toFixed(3);
    zText.innerHTML = zG.toFixed(3);
    totalText.innerHTML = totalG.toFixed(3);
}