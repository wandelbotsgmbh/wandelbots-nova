import {
  CellApi,
  ControllerApi,
  ControllerInputsOutputsApi,
  MotionGroupApi,
  type MotionGroupDescription,
  MotionGroupModelsApi,
  SystemApi,
  TrajectoryCachingApi,
  TrajectoryExecutionApi,
  TrajectoryPlanningApi,
} from '@wandelbots/nova-api/v2/index.js'
import type { AxiosInstance } from 'axios'
import axios from 'axios'

import { logger } from './logging'
import type { Pose } from './types/pose'

export interface NovaConfig {
  apiUrl: string
  accessToken: string
  cellId: string
}

/**
 * API client providing type-safe access to all the Nova API REST endpoints
 * associated with a specific cell id.
 */
export class NovaCellAPIClient {
  readonly system: SystemApi
  readonly cell: CellApi
  readonly motionGroup: MotionGroupApi
  readonly motionGroupModels: MotionGroupModelsApi
  readonly controller: ControllerApi
  readonly controllerIOs: ControllerInputsOutputsApi
  readonly trajectoryPlanning: TrajectoryPlanningApi
  readonly trajectoryExecution: TrajectoryExecutionApi
  readonly trajectoryCaching: TrajectoryCachingApi

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
    this.motionGroupModels = new MotionGroupModelsApi(
      config,
      basePath,
      axiosInstance,
    )
    this.controller = new ControllerApi(config, basePath, axiosInstance)
    this.controllerIOs = new ControllerInputsOutputsApi(
      config,
      basePath,
      axiosInstance,
    )
    this.trajectoryPlanning = new TrajectoryPlanningApi(
      config,
      basePath,
      axiosInstance,
    )
    this.trajectoryExecution = new TrajectoryExecutionApi(
      config,
      basePath,
      axiosInstance,
    )
    this.trajectoryCaching = new TrajectoryCachingApi(
      config,
      basePath,
      axiosInstance,
    )
  }
}

export class NovaApi {
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

    logger.info('Connecting to NOVA API', basePath)

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

  async getControllersNames(): Promise<string[]> {
    if (!this.api) {
      throw new Error('Not connected to NOVA API')
    }

    try {
      const response = await this.api.controller.listRobotControllers(
        this.config!.cellId,
      )
      return response.data
    } catch (error) {
      throw new Error(`Failed to get controllers: ${error}`)
    }
  }

  async getMotionGroups(controllerName: string): Promise<string[]> {
    if (!this.api) {
      throw new Error('Not connected to NOVA API')
    }

    logger.info('Getting motion groups for controller', controllerName)

    try {
      // Get the controller instance to access motion groups
      const controllerResponse =
        await this.api.controller.getControllerDescription(
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

  async getMotionGroupDescription(
    controllerName: string,
    motionGroupId: string,
  ): Promise<MotionGroupDescription> {
    if (!this.api) {
      throw new Error('Not connected to NOVA API')
    }

    try {
      const response = await this.api.motionGroup.getMotionGroupDescription(
        this.config!.cellId,
        controllerName,
        motionGroupId,
      )
      return response.data
    } catch (error) {
      throw new Error(`Failed to get motion group description: ${error}`)
    }
  }

  async listCoordinateSystems(controller: string): Promise<string[]> {
    if (!this.api) {
      throw new Error('Not connected to NOVA API')
    }

    try {
      const response = await this.api.controller.listCoordinateSystems(
        this.config!.cellId,
        controller,
      )
      return response.data.map((cs: any) => cs.coordinate_system)
    } catch (error) {
      throw new Error(`Failed to list coordinate systems: ${error}`)
    }
  }

  async getRobotPose(
    controller: string,
    motionGroup: string,
    coordinateSystem?: string,
  ): Promise<Pose> {
    if (!this.api) {
      throw new Error('Not connected to NOVA API')
    }

    logger.debug(
      'Getting robot pose for controller',
      controller,
      'motion group',
      motionGroup,
      'coordinate system',
      coordinateSystem,
    )

    try {
      const state = await this.api.motionGroup.getCurrentMotionGroupState(
        this.config!.cellId,
        controller,
        motionGroup,
        coordinateSystem,
      )

      const pose = state.data.tcp_pose
      if (!pose || !pose?.position || !pose?.orientation) {
        throw new Error('No TCP pose available')
      }

      return {
        x: pose.position[0],
        y: pose.position[1],
        z: pose.position[2],
        rx: pose.orientation[0],
        ry: pose.orientation[1],
        rz: pose.orientation[2],
      }
    } catch (error) {
      throw new Error(`Failed to get robot pose: ${error}`)
    }
  }

  dispose(): void {
    this.api = null
    this.config = null
  }
}
