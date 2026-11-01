<script setup>
const props = defineProps({bias:Array, aiding:Object, caption:{type:String,default:'Estimated IMU bias'}});
const number = value => Math.abs(value) < 1e-4 && value !== 0 ? value.toExponential(2) : value.toFixed(5);
</script>
<template>
  <section class="plot-block bias-values">
    <h3>{{caption}}</h3>
    <template v-if="bias">
      <table><thead><tr><th scope="col">Sensor</th><th scope="col">x</th><th scope="col">y</th><th scope="col">z</th></tr></thead><tbody>
        <tr><th scope="row">Accelerometer · m/s²</th><td v-for="(value,axis) in bias.slice(0,3)" :key="axis">{{number(value)}}</td></tr>
        <tr><th scope="row">Gyroscope · rad/s</th><td v-for="(value,axis) in bias.slice(3)" :key="axis">{{number(value)}}</td></tr>
        <tr v-if="aiding?.stationaryCalibration"><th scope="row">Gyroscope prior · rad/s</th><td v-for="(value,axis) in aiding.biasPriorMean.slice(3)" :key="axis">{{number(value)}}</td></tr>
      </tbody></table>
      <p class="scope">Pelvis IMU measurement axes. One constant bias shared by all slices.</p>
      <p v-if="aiding?.stationaryCalibration" class="scope">Gyro prior from 0–25 s stationary data; σ={{aiding.biasPriorSigmas[3]}} rad/s per axis.</p>
    </template>
    <p v-else class="scope">Bias is not estimated without IMU aiding.</p>
  </section>
</template>
