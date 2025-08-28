import * as vscode from 'vscode'

import { logger } from '../logging'
import { NovaApi } from '../novaApi'
import type { Pose } from '../types/pose'
import { formatPoseString } from '../utils/pose'
import { insertOrShow } from '../utils/vscode'
import {
  chooseController,
  chooseCoordinateSystem,
  chooseMotionGroup,
} from './selection'

export const ERRORS = {
  NO_CONTROLLERS: 'No controllers found in the cell',
  NO_CONTROLLER_SELECTED: 'No controller selected',
  NO_MOTION_GROUPS: 'No motion groups found for the selected controller',
  NO_MOTION_GROUP_SELECTED: 'No motion group selected',
  NO_COORDINATE_SYSTEM_SELECTED: 'No coordinate system selected',
} as const

/**
 * Wrapper around vscode.window.showQuickPick that displays a dropdown selection menu.
 * Extracted to enable mocking the UI interaction in tests.
 */
export const askQuickPick = (items: string[], placeHolder: string) =>
  vscode.window.showQuickPick(items, { placeHolder })

export async function fetchPose(
  novaApi: NovaApi,
  controller: string,
  motionGroup: string,
  coordinateSystem?: string,
): Promise<Pose> {
  const pose = await novaApi.getRobotPose(
    controller,
    motionGroup,
    coordinateSystem,
  )
  logger.debug('pose', pose)
  return pose
}

export async function readRobotPose(novaApi: NovaApi) {
  const controller = await chooseController(novaApi, askQuickPick)
  if (controller === undefined) {
    vscode.window.showErrorMessage(ERRORS.NO_CONTROLLERS)
    return
  }

  const motionGroup = await chooseMotionGroup(novaApi, controller, askQuickPick)
  if (motionGroup === undefined) {
    // Distinguish "no groups" vs "cancel"
    const groups = await novaApi.getMotionGroups(controller)
    if (groups.length === 0) {
      vscode.window.showErrorMessage(ERRORS.NO_MOTION_GROUPS)
    } else {
      vscode.window.showErrorMessage(ERRORS.NO_MOTION_GROUP_SELECTED)
    }
    return
  }

  const coordinateSystem = await chooseCoordinateSystem(
    novaApi,
    controller,
    askQuickPick,
  )
  if (coordinateSystem === undefined) {
    vscode.window.showErrorMessage(ERRORS.NO_COORDINATE_SYSTEM_SELECTED)
    return
  }

  const pose = await fetchPose(
    novaApi,
    controller,
    motionGroup,
    coordinateSystem,
  )
  const poseString = formatPoseString(pose)

  await insertOrShow(poseString)
}
