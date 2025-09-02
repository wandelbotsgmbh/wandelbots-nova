import { Pose } from '../types/pose'

/**
 * Format a Pose into the required string
 */
export function formatPoseString(pose: Pose): string {
  const f = (n: number) => n.toFixed(3)
  return `Pose((${f(pose.x)}, ${f(pose.y)}, ${f(pose.z)}, ${f(pose.rx)}, ${f(pose.ry)}, ${f(pose.rz)}))`
}
