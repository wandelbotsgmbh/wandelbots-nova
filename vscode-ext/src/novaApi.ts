import { NovaClient } from '@wandelbots/nova-js'

export interface NovaConfig {
  apiUrl: string
  accessToken: string
  cellId: string
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
        instanceUrl: config.apiUrl,
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
