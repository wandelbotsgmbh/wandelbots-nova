import {
  CellApi,
  ControllerApi,
  ControllerInputsOutputsApi,
  MotionGroupApi,
  MotionGroupModelsApi,
  SystemApi,
  TrajectoryCachingApi,
  TrajectoryExecutionApi,
  TrajectoryPlanningApi,
} from '@wandelbots/nova-api/v2/index.js'
import type { AxiosInstance } from 'axios'
import axios from 'axios'
import { logger } from './logging'

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

/**
 * API client providing type-safe access to all the Nova API REST endpoints
 * associated with a specific cell id.
 */
export class NovaCellAPIClient {
  readonly system: any
  readonly cell: any
  readonly motionGroup: any
  readonly motionGroupModels: any
  readonly controller: any
  readonly controllerIOs: any
  readonly trajectoryPlanning: any
  readonly trajectoryExecution: any
  readonly trajectoryCaching: any

  constructor(
    readonly cellId: string,
    readonly opts: any & {
      axiosInstance?: AxiosInstance
      mock?: boolean
    },
  ) {
    const config = {
      ...this.opts,
      isJsonMime: (mime: string) => {
        return mime === 'application/json'
      },
    }
    const basePath = this.opts.basePath ?? ''
    const axiosInstance = this.opts.axiosInstance ?? axios.create()

    this.system = new SystemApi(config, basePath, axiosInstance)
    this.cell = new CellApi(config, basePath, axiosInstance)
    this.motionGroup = new MotionGroupApi(config, basePath, axiosInstance)
    this.motionGroupModels = new MotionGroupModelsApi(config, basePath, axiosInstance)
    this.controller = new ControllerApi(config, basePath, axiosInstance)
    this.controllerIOs = new ControllerInputsOutputsApi(config, basePath, axiosInstance)
    this.trajectoryPlanning = new TrajectoryPlanningApi(config, basePath, axiosInstance)
    this.trajectoryExecution = new TrajectoryExecutionApi(config, basePath, axiosInstance)
    this.trajectoryCaching = new TrajectoryCachingApi(config, basePath, axiosInstance)
  }
}

export class NovaApi {
  private client: any | null = null
  private config: NovaConfig | null = null
  api: NovaCellAPIClient | null = null

  async connect(config: NovaConfig): Promise<void> {
    this.config = config
    const basePath = this.config.apiUrl + '/api/v2'

    const axiosInstance = axios.create({
      baseURL: basePath,
      headers: {
        'X-Wandelbots-Client': 'Wandelbots-Nova-VSCode-Extension',
      },
    })

    logger.info('Connecting to Nova API', basePath)

    axiosInstance.interceptors.request.use(async (request) => {
      if (!request.headers.Authorization) {
        if (this.config?.accessToken) {
          request.headers.Authorization = `Bearer ${this.config.accessToken}`
        }
      }
      return request
    })

    this.api = new NovaCellAPIClient(this.config.cellId, {
      ...config,
      basePath: this.config.apiUrl + '/api/v2',
      isJsonMime: (mime: string) => {
        return mime === 'application/json'
      },
      axiosInstance,
    })
  }

  async renewAuthentication(): Promise<void> {
    console.log('Renewing authentication')
    console.log('Not implemented')
  }

  async getControllers(): Promise<any[]> {
    if (!this.api) {
      throw new Error('Not connected to Nova API')
    }

    try {
      const response = await this.api.controller.listRobotControllers(
        this.config!.cellId
      )
      return response.data || []
    } catch (error) {
      throw new Error(`Failed to get controllers: ${error}`)
    }
  }

  async getMotionGroups(controllerName: string): Promise<string[]> {
    if (!this.api) {
      throw new Error('Not connected to Nova API')
    }

    try {
      // Get the controller instance to access motion groups
      const controllerResponse = await this.api.controller.getControllerDescription(
        this.config!.cellId,
        controllerName,
      )
      const controller = controllerResponse.data
      if (!controller) {
        throw new Error(`Controller ${controllerName} not found`)
      }

      const motionGroups = controller.connected_motion_groups || []
      return motionGroups
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
