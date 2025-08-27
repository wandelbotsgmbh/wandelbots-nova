import * as vscode from 'vscode'

import { logger } from '../logging'
import { NovaApi } from '../novaApi'
import type { Pose } from '../types/pose'
import { formatPoseString } from '../utils/pose'
import { insertOrShow } from '../utils/vscode'
import { chooseController, chooseMotionGroup, chooseTcp } from './selection'

export const ERRORS = {
  NO_CONTROLLERS: 'No controllers found in the cell',
  NO_CONTROLLER_SELECTED: 'No controller selected',
  NO_MOTION_GROUPS: 'No motion groups found for the selected controller',
  NO_MOTION_GROUP_SELECTED: 'No motion group selected',
  NO_TCS: 'No TCPs found for the selected motion group',
  NO_TCP_SELECTED: 'No TCP selected',
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
  tcp: string | undefined,
): Promise<Pose> {
  const pose = await novaApi.getRobotPose(controller, motionGroup, tcp)
  logger.debug('pose', pose)
  return pose
}

export async function readRobotPose(novaApi: NovaApi) {
  // Controller
  const controller = await chooseController(novaApi, askQuickPick)
  if (controller === undefined) {
    vscode.window.showErrorMessage(ERRORS.NO_CONTROLLERS) // either none existed or user cancelled
    return
  }

  // Motion group
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

  // TCP
  const tcp = await chooseTcp(novaApi, controller, motionGroup, askQuickPick)
  if (tcp === undefined) {
    // Distinguish "no tcps" vs "cancel"
    const desc = await novaApi.getMotionGroupDescription(
      controller,
      motionGroup,
    )
    const names = Object.keys(desc.tcps ?? {})
    if (names.length === 0) {
      vscode.window.showErrorMessage(ERRORS.NO_TCS)
    } else {
      vscode.window.showErrorMessage(ERRORS.NO_TCP_SELECTED)
    }
    return
  }

  // Pose → string
  const pose = await fetchPose(novaApi, controller, motionGroup, tcp)
  const poseString = formatPoseString(pose)

  // Output
  await insertOrShow(poseString)
}
