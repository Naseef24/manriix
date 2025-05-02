
"use strict";

let accelerometer = require('./accelerometer.js');
let magnetometer = require('./magnetometer.js');
let axis_state = require('./axis_state.js');
let covariance = require('./covariance.js');
let analog_voltage = require('./analog_voltage.js');
let gnss_fix = require('./gnss_fix.js');
let gyroscope = require('./gyroscope.js');
let proximity = require('./proximity.js');
let temperature = require('./temperature.js');
let time_reference = require('./time_reference.js');
let gnss_position = require('./gnss_position.js');
let gnss_track = require('./gnss_track.js');

module.exports = {
  accelerometer: accelerometer,
  magnetometer: magnetometer,
  axis_state: axis_state,
  covariance: covariance,
  analog_voltage: analog_voltage,
  gnss_fix: gnss_fix,
  gyroscope: gyroscope,
  proximity: proximity,
  temperature: temperature,
  time_reference: time_reference,
  gnss_position: gnss_position,
  gnss_track: gnss_track,
};
