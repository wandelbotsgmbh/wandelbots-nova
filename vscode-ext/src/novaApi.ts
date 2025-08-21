import { NovaClient } from '@wandelbots/nova-js'
import * as vscode from 'vscode'

import { SETTINGS_ACCESS_TOKEN } from './consts'
import { getNovaApiAddress } from './urlResolver'

export interface NovaConfig {
  instanceUrl?: string // Optional since we now use getNovaApiAddress()
  cellId: string
  accessToken: string
}

export interface RobotPose {
  x: number
  y: number
  z: number
  rx: number
  ry: number
  rz: number
}

export class NovaApi {
  private client: NovaClient | null = null
  private config: NovaConfig | null = null

  async connect(config: NovaConfig): Promise<void> {
    try {
      this.config = config
      this.client = new NovaClient({
        instanceUrl: getNovaApiAddress(),
        cellId: config.cellId,
        accessToken: config.accessToken,
      })
    } catch (error) {
      throw new Error(`Failed to connect to Nova API: ${error}`)
    }
  }

  async getControllers(): Promise<any[]> {
    if (!this.client) {
      throw new Error('Not connected to Nova API')
    }

    try {
      const { instances } = await this.client.api.controller.listControllers()
      return instances || []
    } catch (error) {
      throw new Error(`Failed to get controllers: ${error}`)
    }
  }

  async getMotionGroups(controllerName: string): Promise<any[]> {
    if (!this.client) {
      throw new Error('Not connected to Nova API')
    }

    try {
      // Get the controller instance to access motion groups
      const controller = await this.client.api.controller.getControllerInstance(
        this.config!.cellId,
        controllerName,
      )

      if (!controller) {
        throw new Error(`Controller ${controllerName} not found`)
      }

      // Get motion groups for this controller
      const motionGroups = await this.client.api.motionGroup.listMotionGroups(
        this.config!.cellId,
        controller.controller,
      )

      return motionGroups.motion_groups || []
    } catch (error) {
      throw new Error(`Failed to get motion groups: ${error}`)
    }
  }

  async getTcps(motionGroupId: string): Promise<any[]> {
    if (!this.client) {
      throw new Error('Not connected to Nova API')
    }

    try {
      const response = await this.client.api.motionGroup.listTcps(
        this.config!.cellId,
        motionGroupId,
      )
      return response.tcps || []
    } catch (error) {
      throw new Error(`Failed to get TCPs: ${error}`)
    }
  }

  async getRobotPose(
    motionGroupId: string,
    tcpId?: string,
  ): Promise<RobotPose> {
    if (!this.client) {
      throw new Error('Not connected to Nova API')
    }

    try {
      const state = await this.client.api.motionGroup.getMotionGroupState(
        this.config!.cellId,
        motionGroupId,
        tcpId,
      )

      const pose = state.tcp_pose || state.state.tcp_pose
      if (!pose) {
        throw new Error('No TCP pose available')
      }

      return {
        x: pose.position.x,
        y: pose.position.y,
        z: pose.position.z,
        rx: pose.orientation.x,
        ry: pose.orientation.y,
        rz: pose.orientation.z,
      }
    } catch (error) {
      throw new Error(`Failed to get robot pose: ${error}`)
    }
  }

  async getActiveTcp(motionGroupId: string): Promise<any> {
    if (!this.client) {
      throw new Error('Not connected to Nova API')
    }

    try {
      const activeTcp = await this.client.api.motionGroup.getActiveTcp(
        this.config!.cellId,
        motionGroupId,
      )
      return activeTcp
    } catch (error) {
      throw new Error(`Failed to get active TCP: ${error}`)
    }
  }

  dispose(): void {
    this.client = null
    this.config = null
  }
}

export async function readRobotPose(): Promise<void> {
  try {
    // Get configuration from VSCode settings
    const config = vscode.workspace.getConfiguration('wandelbots-nova-viewer')
    const novaApiUrl = config.get<string>('novaApi') // Used for validation
    const accessToken = config.get<string>(SETTINGS_ACCESS_TOKEN)

    // Note: novaApiUrl is still required for validation, even though getNovaApiAddress() is used for the actual connection
    if (!novaApiUrl) {
      vscode.window.showErrorMessage(
        'Nova API URL not configured. Please set "wandelbots-nova-viewer.novaApi" in your VSCode settings.',
      )
      return
    }

    if (!accessToken) {
      vscode.window.showErrorMessage(
        'Nova access token not configured. Please set "wandelbots-nova-viewer.accessToken" in your VSCode settings.',
      )
      return
    }

    // Prompt for cell ID
    const cellId = await vscode.window.showInputBox({
      prompt: 'Enter the cell ID',
      placeHolder: 'e.g., cell',
    })

    if (!cellId) {
      return
    }

    const novaApi = new NovaApi()

    try {
      await novaApi.connect({
        cellId,
        accessToken,
      })

      // Get controllers
      const controllers = await novaApi.getControllers()

      if (controllers.length === 0) {
        vscode.window.showErrorMessage('No controllers found in the cell')
        return
      }

      let selectedController: any

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

      // Get motion groups for the selected controller
      const motionGroups = await novaApi.getMotionGroups(
        selectedController.controller,
      )

      if (motionGroups.length === 0) {
        vscode.window.showErrorMessage(
          'No motion groups found for the selected controller',
        )
        return
      }

      let selectedMotionGroup: any

      if (motionGroups.length === 1) {
        selectedMotionGroup = motionGroups[0]
      } else {
        // Show selection list for multiple motion groups
        const motionGroupNames = motionGroups.map((mg) => mg.id)
        const selectedMotionGroupName = await vscode.window.showQuickPick(
          motionGroupNames,
          {
            placeHolder: 'Select a motion group',
          },
        )

        if (!selectedMotionGroupName) {
          return
        }

        selectedMotionGroup = motionGroups.find(
          (mg) => mg.id === selectedMotionGroupName,
        )
      }

      // Get TCPs for the selected motion group
      const tcps = await novaApi.getTcps(selectedMotionGroup.id)

      if (tcps.length === 0) {
        vscode.window.showErrorMessage(
          'No TCPs found for the selected motion group',
        )
        return
      }

      let selectedTcp: any

      if (tcps.length === 1) {
        selectedTcp = tcps[0]
      } else {
        // Show selection list for multiple TCPs
        const tcpNames = tcps.map((tcp) => tcp.id)
        const selectedTcpName = await vscode.window.showQuickPick(tcpNames, {
          placeHolder: 'Select a TCP',
        })

        if (!selectedTcpName) {
          return
        }

        selectedTcp = tcps.find((tcp) => tcp.id === selectedTcpName)
      }

      // Get the robot pose
      const pose = await novaApi.getRobotPose(
        selectedMotionGroup.id,
        selectedTcp.id,
      )

      // Format the pose string
      const poseString = `Pose((${pose.x.toFixed(3)}, ${pose.y.toFixed(3)}, ${pose.z.toFixed(3)}, ${pose.rx.toFixed(3)}, ${pose.ry.toFixed(3)}, ${pose.rz.toFixed(3)}))`

      // Get the active text editor
      const editor = vscode.window.activeTextEditor
      if (editor) {
        // Insert the pose at the current cursor position
        await editor.edit((editBuilder) => {
          editBuilder.insert(editor.selection.active, poseString)
        })

        vscode.window.showInformationMessage(
          `Robot pose inserted: ${poseString}`,
        )
      } else {
        // If no active editor, show the pose in a message
        vscode.window.showInformationMessage(`Robot pose: ${poseString}`)
      }
    } finally {
      novaApi.dispose()
    }
  } catch (error) {
    vscode.window.showErrorMessage(`Failed to read robot pose: ${error}`)
    console.error('Error reading robot pose:', error)
  }
}
