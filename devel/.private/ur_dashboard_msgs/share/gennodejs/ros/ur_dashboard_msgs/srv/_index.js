
"use strict";

let AddToLog = require('./AddToLog.js')
let Popup = require('./Popup.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let RawRequest = require('./RawRequest.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let Load = require('./Load.js')
let GetProgramState = require('./GetProgramState.js')
let GetRobotMode = require('./GetRobotMode.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let IsProgramRunning = require('./IsProgramRunning.js')

module.exports = {
  AddToLog: AddToLog,
  Popup: Popup,
  IsProgramSaved: IsProgramSaved,
  IsInRemoteControl: IsInRemoteControl,
  RawRequest: RawRequest,
  GetSafetyMode: GetSafetyMode,
  Load: Load,
  GetProgramState: GetProgramState,
  GetRobotMode: GetRobotMode,
  GetLoadedProgram: GetLoadedProgram,
  IsProgramRunning: IsProgramRunning,
};
