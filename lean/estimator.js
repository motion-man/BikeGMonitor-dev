const LeanEstimator =
{
    calibrated: false,

    calibrate(sensorData)
    {
        this.calibrated = true;

        return {
            success: true
        };
    },

    update(sensorData)
    {
        return {
            leanAngle: sensorData.accel.x,
            leanRate: sensorData.gyro.x,
            confidence: this.calibrated ? 1 : 0
        };
    },

    reset()
    {
        this.calibrated = false;
    }
};

export default LeanEstimator;
