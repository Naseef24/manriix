
"use strict";

let DetectedPersons = require('./DetectedPersons.js');
let CompositeDetectedPersons = require('./CompositeDetectedPersons.js');
let TrackedGroup = require('./TrackedGroup.js');
let TrackedPersons = require('./TrackedPersons.js');
let TrackingTimingMetrics = require('./TrackingTimingMetrics.js');
let TrackedGroups = require('./TrackedGroups.js');
let PersonTrajectory = require('./PersonTrajectory.js');
let TrackedPerson2d = require('./TrackedPerson2d.js');
let ImmDebugInfo = require('./ImmDebugInfo.js');
let TrackedPersons2d = require('./TrackedPersons2d.js');
let TrackedPerson = require('./TrackedPerson.js');
let DetectedPerson = require('./DetectedPerson.js');
let PersonTrajectoryEntry = require('./PersonTrajectoryEntry.js');
let ImmDebugInfos = require('./ImmDebugInfos.js');
let CompositeDetectedPerson = require('./CompositeDetectedPerson.js');

module.exports = {
  DetectedPersons: DetectedPersons,
  CompositeDetectedPersons: CompositeDetectedPersons,
  TrackedGroup: TrackedGroup,
  TrackedPersons: TrackedPersons,
  TrackingTimingMetrics: TrackingTimingMetrics,
  TrackedGroups: TrackedGroups,
  PersonTrajectory: PersonTrajectory,
  TrackedPerson2d: TrackedPerson2d,
  ImmDebugInfo: ImmDebugInfo,
  TrackedPersons2d: TrackedPersons2d,
  TrackedPerson: TrackedPerson,
  DetectedPerson: DetectedPerson,
  PersonTrajectoryEntry: PersonTrajectoryEntry,
  ImmDebugInfos: ImmDebugInfos,
  CompositeDetectedPerson: CompositeDetectedPerson,
};
