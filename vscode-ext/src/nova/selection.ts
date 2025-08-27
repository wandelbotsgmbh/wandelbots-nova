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
  logger.debug('motionGroups', motionGroups)

  const selected = await singleOrPick(
    motionGroups,
    askToPick,
    'Select a motion group',
  )
  if (selected) logger.debug('selectedMotionGroup', selected)
  return selected
}

export async function chooseCoordinateSystem(
  novaApi: NovaApi,
  controller: string,
  askToPick: (
    items: string[],
    placeHolder: string,
  ) => Thenable<string | undefined>,
): Promise<string | undefined> {
  const worldCoordinateSystem = 'World'
  const coordinateSystems = await novaApi.listCoordinateSystems(controller)
  const formattedCoordinateSystems = coordinateSystems.map((d) =>
    d === '' ? worldCoordinateSystem : d,
  )
  logger.debug('coordinateSystems', formattedCoordinateSystems)
  const pick = await singleOrPick(
    formattedCoordinateSystems,
    askToPick,
    'Select a coordinate system',
  )
  if (pick === worldCoordinateSystem) return ''
  return pick
}
