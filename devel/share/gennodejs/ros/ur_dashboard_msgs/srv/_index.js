
"use strict";

let IsProgramRunning = require('./IsProgramRunning.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let Load = require('./Load.js')
let GetProgramState = require('./GetProgramState.js')
let RawRequest = require('./RawRequest.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let AddToLog = require('./AddToLog.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let GetRobotMode = require('./GetRobotMode.js')
let Popup = require('./Popup.js')

module.exports = {
  IsProgramRunning: IsProgramRunning,
  GetLoadedProgram: GetLoadedProgram,
  GetSafetyMode: GetSafetyMode,
  Load: Load,
  GetProgramState: GetProgramState,
  RawRequest: RawRequest,
  IsProgramSaved: IsProgramSaved,
  AddToLog: AddToLog,
  IsInRemoteControl: IsInRemoteControl,
  GetRobotMode: GetRobotMode,
  Popup: Popup,
};
