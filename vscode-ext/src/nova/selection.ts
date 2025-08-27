import { logger } from '../logging'
import { NovaApi } from '../novaApi'
import { singleOrPick } from '../utils/vscode'

export async function chooseController(
  novaApi: NovaApi,
  askToPick: (
    items: string[],
    placeHolder: string,
  ) => Thenable<string | undefined>,
): Promise<string | undefined> {
  const controllers = await novaApi.getControllersNames()
  logger.debug('controllers', controllers)

  const selected = await singleOrPick(
    controllers,
    askToPick,
    'Select a controller',
  )
  if (selected) logger.debug('selectedController', selected)
  return selected
}

export async function chooseMotionGroup(
  novaApi: NovaApi,
  controller: string,
  askToPick: (
    items: string[],
    placeHolder: string,
  ) => Thenable<string | undefined>,
): Promise<string | undefined> {
  const motionGroups = await novaApi.getMotionGroups(controller)
  logger.info('motionGroups', motionGroups)

  const selected = await singleOrPick(
    motionGroups,
    askToPick,
    'Select a motion group',
  )
  if (selected) logger.debug('selectedMotionGroup', selected)
  return selected
}

export async function chooseTcp(
  novaApi: NovaApi,
  controller: string,
  motionGroup: string,
  askToPick: (
    items: string[],
    placeHolder: string,
  ) => Thenable<string | undefined>,
): Promise<string | undefined> {
  const motionGroupDescription = await novaApi.getMotionGroupDescription(
    controller,
    motionGroup,
  )
  logger.debug('motionGroupDescription', motionGroupDescription)

  const tcps = motionGroupDescription.tcps ?? {}
  logger.debug('tcps', tcps)

  const tcpNames = Object.keys(tcps)
  const selectedName = await singleOrPick(tcpNames, askToPick, 'Select a TCP')
  if (selectedName) logger.debug('selectedTcp', selectedName)

  return selectedName
}
