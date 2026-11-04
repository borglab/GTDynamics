<script setup>
import MathTypeset from './MathTypeset.vue';
import ContactPoseMath from './ContactPoseMath.vue';
import definitions from './equations/kinematics.math.html?math';
defineProps({ record: { type: Object, required: true } });
const degrees = value => value * 180 / Math.PI;
const number = value => Number(value).toLocaleString(undefined, { maximumSignificantDigits: 6 });
const vector = (values, scale = 1) => values.map(v => number(v * scale)).join(', ');
</script>

<template>
  <section aria-label="Kinematic smoothing factor definitions">
    <ContactPoseMath />
    <MathTypeset :html="definitions" />
    <div class="math-parameters" data-math-parameters="smoothing">
      <table><caption>Selected smoothing graph</caption><thead><tr><th>Quantity</th><th>Value</th></tr></thead><tbody>
        <tr><td>Retained slices / stance landmarks</td><td>{{record.samples}} / {{record.graph.stanceLandmarks}}</td></tr>
        <tr><td>Contact pose standard deviation per axis</td><td>{{record.graph.contactRotationSigmaDegrees}}° / {{record.graph.contactTranslationSigmaMeters}} m</td></tr>
        <tr><td>Encoder angle standard deviation</td><td>{{number(degrees(0.03))}}°</td></tr>
        <tr><td>Upright / height / motion prior</td><td>{{record.graph.uprightPrior?'on':'off'}} / {{record.graph.heightPrior?'on':'off'}} / {{record.graph.motionPrior?'on':'off'}}</td></tr>
        <tr><td>IMU aiding</td><td>{{record.imuAiding?.enabled?'on':'off'}}</td></tr>
        <tr><td>Return-position prior</td><td>{{record.loopClosure?.enabled?'on':'off'}}<template v-if="record.loopClosure?.enabled"> · XYZ σ = {{vector(record.loopClosure.positionSigmasMeters)}} m</template></td></tr>
      </tbody></table>
      <template v-if="record.imuAiding?.enabled">
        <table><caption>Recorded IMU assumptions · provisional manually chosen strengths</caption><thead><tr><th>Quantity</th><th>Value per axis</th></tr></thead><tbody>
          <tr><td>Accelerometer white-noise density</td><td>{{record.imuAiding.accelerometerNoiseDensity}} m/s²/√Hz</td></tr>
          <tr><td>Gyro white-noise density</td><td>{{number(degrees(record.imuAiding.gyroscopeNoiseDensity))}} °/s/√Hz</td></tr>
          <tr><td>Integration covariance</td><td>{{record.imuAiding.integrationCovariance}} I₃</td></tr>
          <tr><td>Accelerometer bias prior mean / σ (XYZ)</td><td>{{vector(record.imuAiding.biasPriorMean.slice(0,3))}} / {{vector(record.imuAiding.biasPriorSigmas.slice(0,3))}} m/s²</td></tr>
          <tr><td>Gyro bias prior mean / σ (XYZ)</td><td>{{vector(record.imuAiding.biasPriorMean.slice(3),180/Math.PI)}} / {{vector(record.imuAiding.biasPriorSigmas.slice(3),180/Math.PI)}} °/s</td></tr>
          <tr><td>Initial velocity prior</td><td>Zero mean; σ = {{record.imuAiding.initialVelocityPriorSigma}} m/s</td></tr>
          <tr><td>World gravity (XYZ)</td><td>{{vector(record.imuAiding.gravityWorld)}} m/s²</td></tr>
          <tr><td>Integration intervals</td><td>{{record.imuAiding.intervals}} pims · {{number(Math.min(...record.imuAiding.intervalSeconds))}}–{{number(Math.max(...record.imuAiding.intervalSeconds))}} s</td></tr>
          <tr v-if="record.imuAiding.stationaryCalibration"><td>Stationary gyro calibration</td><td>{{record.imuAiding.stationaryCalibration.sampleCount}} samples · {{record.imuAiding.stationaryCalibration.sourceWindowSeconds.join('–')}} s</td></tr>
        </tbody></table>
        <table><caption>Fixed COM-to-IMU transform · translations in metres</caption><tbody><tr v-for="(row,i) in record.imuAiding.com_T_sensor" :key="i"><td v-for="(value,j) in row" :key="j">{{number(value)}}</td></tr></tbody></table>
      </template>
    </div>
  </section>
</template>
