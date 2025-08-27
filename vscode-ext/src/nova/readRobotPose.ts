import * as vscode from 'vscode'

import { logger } from '../logging'
import { NovaApi } from '../novaApi'

export async function readRobotPose(novaApi: NovaApi) {
  const controllers = await novaApi.getControllers()

  if (controllers.length === 0) {
    vscode.window.showErrorMessage('No controllers found in the cell')
    return
  }

  logger.debug('controllers', controllers)

  let selectedController: string

  if (controllers.length === 1) {
    selectedController = controllers[0]
  } else {
    // Show selection list for multiple controllers
    const controllerNames = controllers.map((c) => c.controller)
    const selectedControllerName = await vscode.window.showQuickPick(
      controllerNames,
      {
        placeHolder: 'Select a controller',
      },
    )

    if (!selectedControllerName) {
      return
    }

    selectedController = controllers.find(
      (c) => c.controller === selectedControllerName,
    )
  }

  logger.info('selectedController', selectedController)

  // Get motion groups for the selected controller
  const motionGroups = await novaApi.getMotionGroups(selectedController)

  logger.info('motionGroups', motionGroups)

  if (motionGroups.length === 0) {
    vscode.window.showErrorMessage(
      'No motion groups found for the selected controller',
    )
    return
  }

  let selectedMotionGroup: string

  if (motionGroups.length === 1) {
    selectedMotionGroup = motionGroups[0]
  } else {
    // Show selection list for multiple motion groups
    const selectedMotionGroupName = await vscode.window.showQuickPick(
      motionGroups,
      {
        placeHolder: 'Select a motion group',
      },
    )

    if (!selectedMotionGroupName) {
      return
    }

    selectedMotionGroup = selectedMotionGroupName
  }

  logger.info('selectedMotionGroup', selectedMotionGroup)

  const motionGroupDescription = await novaApi.getMotionGroupDescription(
    selectedController,
    selectedMotionGroup,
  )

  logger.info('motionGroupDescription', motionGroupDescription)

  // Get TCPs for the selected motion group
  const tcps = motionGroupDescription.tcps
  const tcpNames = Object.keys(tcps)

  logger.info('tcps', tcps)

  if (tcpNames.length === 0) {
    vscode.window.showErrorMessage(
      'No TCPs found for the selected motion group',
    )
    return
  }

  let selectedTcp: string | undefined

  if (tcpNames.length === 1) {
    selectedTcp = Object.keys(tcps)[0]
  } else {
    // Show selection list for multiple TCPs
    const selectedTcpName = await vscode.window.showQuickPick(tcpNames, {
      placeHolder: 'Select a TCP',
    })

    if (!selectedTcpName) {
      return
    }

    selectedTcp = tcpNames.find((tcp) => tcp === selectedTcpName)
  }

  logger.info('selectedTcp', selectedTcp)

  // Get the robot pose
  const pose = await novaApi.getRobotPose(
    selectedController,
    selectedMotionGroup,
    selectedTcp,
  )

  console.log('pose', pose)

  // Format the pose string
  const poseString = `Pose((${pose.x.toFixed(3)}, ${pose.y.toFixed(3)}, ${pose.z.toFixed(3)}, ${pose.rx.toFixed(3)}, ${pose.ry.toFixed(3)}, ${pose.rz.toFixed(3)}))`

  // Get the active text editor
  const editor = vscode.window.activeTextEditor
  if (editor) {
    // Insert the pose at the current cursor position
    await editor.edit((editBuilder) => {
      editBuilder.insert(editor.selection.active, poseString)
    })

    vscode.window.showInformationMessage(`Robot pose inserted: ${poseString}`)
  } else {
    // If no active editor, show the pose in a message
    vscode.window.showInformationMessage(`Robot pose: ${poseString}`)
  }
}
