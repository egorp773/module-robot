# ESP32 bridge frame conventions

The bridge owns `/dev/module-esp32`; no other ROS node or gateway opens that
device. Its production stream contains only version-1 COBS frames. ROM boot
text is discarded at the first `0x00` boundary and ESP32 library logs are
compiled away from UART0.

## IMU convention during commissioning

`/imu/data_raw` contains BNO085 rotation-vector, calibrated gyro and linear
acceleration values in the sensor's physical axes. The bridge does **not** copy
the legacy compass offsets, rotate axes, or claim that absolute yaw is ENU.

The target REP-103 mounting convention is:

```text
imu_link: +x robot forward, +y robot left, +z up
ENU yaw:  0 along East, positive counter-clockwise toward North
```

Until the mounting transform and a controlled clockwise/counter-clockwise test
are recorded, consumers may use angular velocity and linear acceleration but
must not treat quaternion yaw as proven. The quaternion is still published so
the mounting test can inspect it; its covariance is the larger of the
configured commissioning floor and the BNO085 accuracy estimate. Localization
templates remain disabled until the heading gate passes.

GNSS `NavSatFix` covariance is `hAcc^2, hAcc^2, vAcc^2`. A missing vertical
accuracy uses a deliberately large configured variance. Carrier state remains
on `/rtk/status` because `NavSatFix` cannot represent none/float/fixed fully.
