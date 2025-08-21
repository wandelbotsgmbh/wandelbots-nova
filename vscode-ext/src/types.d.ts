declare module '@wandelbots/nova-js' {
  export class NovaClient {
    constructor(config: {
      instanceUrl: string
      cellId: string
      accessToken: string
    })

    api: {
      controller: {
        listControllers(): Promise<{ instances: any[] }>
        getControllerInstance(cell: string, name: string): Promise<any>
      }
      motionGroup: {
        listMotionGroups(
          cell: string,
          controller: string,
        ): Promise<{ motion_groups: any[] }>
        listTcps(cell: string, motionGroupId: string): Promise<{ tcps: any[] }>
        getMotionGroupState(
          cell: string,
          motionGroupId: string,
          tcp?: string,
        ): Promise<any>
        getActiveTcp(cell: string, motionGroupId: string): Promise<any>
      }
    }
  }
}
