
"use strict";

let ResetPose = require('./ResetPose.js')
let SetGoal = require('./SetGoal.js')
let RemoveLabel = require('./RemoveLabel.js')
let GetMap2 = require('./GetMap2.js')
let GetNodesInRadius = require('./GetNodesInRadius.js')
let GetNodeData = require('./GetNodeData.js')
let GetMap = require('./GetMap.js')
let GetPlan = require('./GetPlan.js')
let LoadDatabase = require('./LoadDatabase.js')
let GlobalBundleAdjustment = require('./GlobalBundleAdjustment.js')
let ListLabels = require('./ListLabels.js')
let SetLabel = require('./SetLabel.js')
let AddLink = require('./AddLink.js')
let CleanupLocalGrids = require('./CleanupLocalGrids.js')
let DetectMoreLoopClosures = require('./DetectMoreLoopClosures.js')
let PublishMap = require('./PublishMap.js')

module.exports = {
  ResetPose: ResetPose,
  SetGoal: SetGoal,
  RemoveLabel: RemoveLabel,
  GetMap2: GetMap2,
  GetNodesInRadius: GetNodesInRadius,
  GetNodeData: GetNodeData,
  GetMap: GetMap,
  GetPlan: GetPlan,
  LoadDatabase: LoadDatabase,
  GlobalBundleAdjustment: GlobalBundleAdjustment,
  ListLabels: ListLabels,
  SetLabel: SetLabel,
  AddLink: AddLink,
  CleanupLocalGrids: CleanupLocalGrids,
  DetectMoreLoopClosures: DetectMoreLoopClosures,
  PublishMap: PublishMap,
};
