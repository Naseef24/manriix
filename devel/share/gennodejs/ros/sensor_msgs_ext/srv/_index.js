
"use strict";

let calibrate_gyroscope = require('./calibrate_gyroscope.js')
let get_proximity_configuration = require('./get_proximity_configuration.js')
let set_axis_home = require('./set_axis_home.js')

module.exports = {
  calibrate_gyroscope: calibrate_gyroscope,
  get_proximity_configuration: get_proximity_configuration,
  set_axis_home: set_axis_home,
};
