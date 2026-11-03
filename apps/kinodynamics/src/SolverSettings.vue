<script setup>
import { computed } from 'vue';

const props = defineProps({ record: { type: Object, required: true } });
const settings = computed(() => props.record.settings);
const imu = computed(() => props.record.diagnostics.imu);
const number = value => Number(value).toLocaleString('en-US', { maximumSignificantDigits: 6 });
const degrees = value => value * 180 / Math.PI;
const vector = (values, convert = value => value) => values.map(value => number(convert(value))).join(', ');
const collocationRows = computed(() => {
  const c = settings.value.collocation;
  return [
    ['Root pose rotation / translation', `${number(c.poseRotationSigmaDegrees)}° / ${number(c.poseTranslationSigma)} m`],
    ['Root twist angular / linear', `${number(c.twistAngularSigmaDegrees)} °/s / ${number(c.twistLinearSigma)} m/s`],
    ['Joint angle / rate', `${number(c.jointPositionSigmaDegrees)}° / ${number(c.jointVelocitySigmaDegrees)} °/s`],
    ['Stance sole-origin linear velocity', `${number(c.stanceVelocitySigma)} m/s`],
  ];
});
</script>

<template>
  <section class="recorded-settings" data-math-parameters="kinodynamics">
    <h3>Recorded numerical settings</h3>
    <p>These Gaussian scales are chosen modeling assumptions. Rotation scales are specified in degrees and converted to radians in the native graph. The collocation scales apply to each adjacent-slice residual; rate scales multiply the square root of the actual interval.</p>
    <div class="table-scroll">
      <table class="factor-table">
        <caption>Measurements and regularization · standard deviations per component</caption>
        <thead><tr><th>Residual</th><th>Recorded scale</th></tr></thead>
        <tbody>
          <tr><th>Encoder angle</th><td>{{number(settings.encoderSigmaDegrees)}}°</td></tr>
          <tr><th>Sole pose rotation / translation</th><td>{{number(settings.contactRotationSigmaDegrees)}}° / {{number(settings.contactTranslationSigma)}} m</td></tr>
          <tr v-for="[name,value] in collocationRows" :key="name"><th>{{name}}</th><td>{{value}}</td></tr>
          <tr><th>Actuator torque / force-equivalent sole wrench</th><td>{{number(settings.effortSigmaNm)}} Nm / {{number(settings.wrenchSigmaN)}} N</td></tr>
          <tr><th>Torque rate / force-equivalent wrench rate</th><td>{{number(settings.torqueRateSigmaNmPerSqrtSecond)}} Nm/√s / {{number(settings.wrenchRateSigmaNPerSqrtSecond)}} N/√s</td></tr>
          <tr><th>Root angular / body-linear acceleration</th><td>{{number(settings.baseAngularAccelerationSigmaDegrees)}} °/s² / {{number(settings.baseLinearAccelerationSigma)}} m/s²</td></tr>
          <tr><th>Joint acceleration</th><td>{{number(settings.jointAccelerationSigmaDegrees)}} °/s²</td></tr>
          <tr><th>Translation / yaw gauge costs</th><td>Slice {{record.diagnostics.gaugeSlice}} · {{number(record.diagnostics.gaugeTimeSeconds)}} s · σ {{number(record.diagnostics.gauge.translationSigmaMeters)}} m / {{number(record.diagnostics.gauge.yawSigmaRadians)}} rad; roll and pitch free</td></tr>
          <tr><th>Friction μ / pyramid coefficient μ/√2</th><td>{{number(settings.friction)}} / {{number(settings.friction/Math.sqrt(2))}}</td></tr>
          <tr v-for="(patch,foot) in settings.supportRectangles" :key="foot"><th>{{foot?'Right':'Left'}} sole half-length / half-width</th><td>{{vector(patch)}} m</td></tr>
          <tr><th>Actuator effort limits</th><td>{{settings.torqueLimitsEnforced?'Enforced':'Diagnostic only'}}</td></tr>
        </tbody>
      </table>
    </div>
    <template v-if="imu">
      <div class="table-scroll">
        <table class="factor-table">
          <caption>Gal3 IMU assumptions · continuous-time noise and shared bias</caption>
          <thead><tr><th>Quantity</th><th>Recorded value</th></tr></thead>
          <tbody>
            <tr><th>Accelerometer white-noise density</th><td>{{number(imu.accelerometerNoiseDensity)}} m/s²/√Hz</td></tr>
            <tr><th>Gyroscope white-noise density</th><td>{{number(degrees(imu.gyroscopeNoiseDensity))}} °/s/√Hz</td></tr>
            <tr><th>Integration covariance</th><td>{{imu.integrationCovariance}} I₃</td></tr>
            <tr><th>Accelerometer bias prior mean / σ (XYZ)</th><td>{{vector(imu.biasPriorMean.slice(0,3))}} / {{vector(imu.biasPriorSigmas.slice(0,3))}} m/s²</td></tr>
            <tr><th>Gyroscope bias prior mean / σ (XYZ)</th><td>{{vector(imu.biasPriorMean.slice(3),degrees)}} / {{vector(imu.biasPriorSigmas.slice(3),degrees)}} °/s</td></tr>
            <tr><th>First world COM velocity prior</th><td>Zero mean · σ {{number(imu.initialVelocityPriorSigma)}} m/s</td></tr>
            <tr><th>World gravity XYZ</th><td>{{vector(imu.gravityWorld)}} m/s²</td></tr>
            <tr><th>Preintegration intervals</th><td>{{imu.intervals}} pims · {{number(Math.min(...imu.intervalSeconds)*1000)}}–{{number(Math.max(...imu.intervalSeconds)*1000)}} ms</td></tr>
            <tr v-if="imu.stationaryCalibration"><th>Stationary gyro calibration</th><td>{{imu.stationaryCalibration.sampleCount}} samples · {{imu.stationaryCalibration.sourceWindowSeconds.join('–')}} s</td></tr>
          </tbody>
        </table>
      </div>
      <details><summary>COM-to-IMU transform and estimated shared bias</summary>
        <p>The transform maps sensor coordinates into the root COM. Biases are expressed in pelvis IMU measurement axes; accelerometer components use m/s² and gyro components below use °/s.</p>
        <pre>{{JSON.stringify({com_T_sensor:imu.com_T_sensor,accelerometerBias:record.diagnostics.estimatedBias.slice(0,3),gyroscopeBiasDegreesPerSecond:record.diagnostics.estimatedBias.slice(3).map(degrees)},null,2)}}</pre>
      </details>
    </template>
  </section>
</template>
