import { describe, expect, it } from 'vitest'

import { formatPoseString } from './pose'

describe('formatPoseString', () => {
  it('should format a pose string', () => {
    expect(formatPoseString({ x: 1, y: 2, z: 3, rx: 4, ry: 5, rz: 6 })).toBe(
      'Pose((1.000, 2.000, 3.000, 4.000, 5.000, 6.000))',
    )
  })
})
