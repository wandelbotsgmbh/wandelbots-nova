import * as assert from 'assert'

import { NovaApi, NovaConfig, RobotPose } from '../novaApi'

suite('NovaApi Tests', () => {
  let novaApi: NovaApi
  let config: NovaConfig

  setup(() => {
    config = {
      apiUrl: 'http://172.31.10.242',
      accessToken: 'test-token-123',
      cellId: 'cell',
    }
    novaApi = new NovaApi()
  })

  teardown(() => {
    novaApi.dispose()
  })

  suite('NovaApi class instantiation', () => {
    test('should create instance correctly', () => {
      const api = new NovaApi()
      assert.ok(api, 'NovaApi instance should be created')
      assert.strictEqual(
        typeof api.connect,
        'function',
        'connect method should exist',
      )
      assert.strictEqual(
        typeof api.getControllers,
        'function',
        'getControllers method should exist',
      )
      assert.strictEqual(
        typeof api.getMotionGroups,
        'function',
        'getMotionGroups method should exist',
      )
      assert.strictEqual(
        typeof api.getTcps,
        'function',
        'getTcps method should exist',
      )
      assert.strictEqual(
        typeof api.getRobotPose,
        'function',
        'getRobotPose method should exist',
      )
      assert.strictEqual(
        typeof api.getActiveTcp,
        'function',
        'getActiveTcp method should exist',
      )
      assert.strictEqual(
        typeof api.dispose,
        'function',
        'dispose method should exist',
      )
    })

    test('should have initial null state', () => {
      const api = new NovaApi()
      assert.strictEqual(
        (api as any).client,
        null,
        'client should be initially null',
      )
      assert.strictEqual(
        (api as any).config,
        null,
        'config should be initially null',
      )
    })
  })

  suite('NovaApi connection state', () => {
    test('should throw error when trying to use methods without connecting', async () => {
      await assert.rejects(
        async () => await novaApi.getControllers(),
        /Not connected to Nova API/,
      )
    })

    test('should throw error when trying to get motion groups without connecting', async () => {
      await assert.rejects(
        async () => await novaApi.getMotionGroups('test-controller'),
        /Not connected to Nova API/,
      )
    })

    test('should throw error when trying to get TCPs without connecting', async () => {
      await assert.rejects(
        async () => await novaApi.getTcps('test-motion-group'),
        /Not connected to Nova API/,
      )
    })

    test('should throw error when trying to get robot pose without connecting', async () => {
      await assert.rejects(
        async () => await novaApi.getRobotPose('test-motion-group'),
        /Not connected to Nova API/,
      )
    })

    test('should throw error when trying to get active TCP without connecting', async () => {
      await assert.rejects(
        async () => await novaApi.getActiveTcp('test-motion-group'),
        /Not connected to Nova API/,
      )
    })
  })

  suite('NovaApi configuration interface', () => {
    test('should accept valid NovaConfig structure', () => {
      assert.strictEqual(
        typeof config.apiUrl,
        'string',
        'apiUrl should be string',
      )
      assert.strictEqual(
        typeof config.accessToken,
        'string',
        'accessToken should be string',
      )
      assert.strictEqual(
        typeof config.cellId,
        'string',
        'cellId should be string',
      )
      assert.ok(config.apiUrl.length > 0, 'apiUrl should not be empty')
      assert.ok(
        config.accessToken.length > 0,
        'accessToken should not be empty',
      )
      assert.ok(config.cellId.length > 0, 'cellId should not be empty')
    })

    test('should validate NovaConfig required properties', () => {
      assert.ok('apiUrl' in config, 'config should have apiUrl property')
      assert.ok(
        'accessToken' in config,
        'config should have accessToken property',
      )
      assert.ok('cellId' in config, 'config should have cellId property')
    })
  })

  suite('NovaApi RobotPose interface', () => {
    test('should accept valid RobotPose structure', () => {
      const pose: RobotPose = {
        x: 100.0,
        y: 200.0,
        z: 300.0,
        rx: 0.1,
        ry: 0.2,
        rz: 0.3,
      }

      assert.strictEqual(typeof pose.x, 'number', 'x should be number')
      assert.strictEqual(typeof pose.y, 'number', 'y should be number')
      assert.strictEqual(typeof pose.z, 'number', 'z should be number')
      assert.strictEqual(typeof pose.rx, 'number', 'rx should be number')
      assert.strictEqual(typeof pose.ry, 'number', 'ry should be number')
      assert.strictEqual(typeof pose.rz, 'number', 'rz should be number')
    })

    test('should validate RobotPose required properties', () => {
      const pose: RobotPose = {
        x: 0,
        y: 0,
        z: 0,
        rx: 0,
        ry: 0,
        rz: 0,
      }

      assert.ok('x' in pose, 'pose should have x property')
      assert.ok('y' in pose, 'pose should have y property')
      assert.ok('z' in pose, 'pose should have z property')
      assert.ok('rx' in pose, 'pose should have rx property')
      assert.ok('ry' in pose, 'pose should have ry property')
      assert.ok('rz' in pose, 'pose should have rz property')
    })
  })

  suite('NovaApi method signatures', () => {
    test('should have correct connect method signature', () => {
      // Test that the method exists and can be called (will fail at runtime due to missing NovaClient)
      assert.strictEqual(
        typeof novaApi.connect,
        'function',
        'connect method should exist',
      )
      assert.strictEqual(
        novaApi.connect.length,
        1,
        'connect method should take 1 parameter',
      )
    })

    test('should have correct getControllers method signature', () => {
      assert.strictEqual(
        typeof novaApi.getControllers,
        'function',
        'getControllers method should exist',
      )
      assert.strictEqual(
        novaApi.getControllers.length,
        0,
        'getControllers method should take 0 parameters',
      )
    })

    test('should have correct getMotionGroups method signature', () => {
      assert.strictEqual(
        typeof novaApi.getMotionGroups,
        'function',
        'getMotionGroups method should exist',
      )
      assert.strictEqual(
        novaApi.getMotionGroups.length,
        1,
        'getMotionGroups method should take 1 parameter',
      )
    })

    test('should have correct getTcps method signature', () => {
      assert.strictEqual(
        typeof novaApi.getTcps,
        'function',
        'getTcps method should exist',
      )
      assert.strictEqual(
        novaApi.getTcps.length,
        1,
        'getTcps method should take 1 parameter',
      )
    })

    test('should have correct getRobotPose method signature', () => {
      assert.strictEqual(
        typeof novaApi.getRobotPose,
        'function',
        'getRobotPose method should exist',
      )
      assert.strictEqual(
        novaApi.getRobotPose.length,
        2,
        'getRobotPose method should take 2 parameters',
      )
    })

    test('should have correct getActiveTcp method signature', () => {
      assert.strictEqual(
        typeof novaApi.getActiveTcp,
        'function',
        'getActiveTcp method should exist',
      )
      assert.strictEqual(
        novaApi.getActiveTcp.length,
        1,
        'getActiveTcp method should take 1 parameter',
      )
    })
  })

  suite('NovaApi error message consistency', () => {
    test('should have consistent error message for unconnected state', async () => {
      const expectedError = 'Not connected to Nova API'

      try {
        await novaApi.getControllers()
        assert.fail('Should have thrown an error')
      } catch (error) {
        assert.strictEqual(
          (error as Error).message,
          expectedError,
          'Error message should match expected',
        )
      }

      try {
        await novaApi.getMotionGroups('test')
        assert.fail('Should have thrown an error')
      } catch (error) {
        assert.strictEqual(
          (error as Error).message,
          expectedError,
          'Error message should match expected',
        )
      }

      try {
        await novaApi.getTcps('test')
        assert.fail('Should have thrown an error')
      } catch (error) {
        assert.strictEqual(
          (error as Error).message,
          expectedError,
          'Error message should match expected',
        )
      }

      try {
        await novaApi.getRobotPose('test')
        assert.fail('Should have thrown an error')
      } catch (error) {
        assert.strictEqual(
          (error as Error).message,
          expectedError,
          'Error message should match expected',
        )
      }

      try {
        await novaApi.getActiveTcp('test')
        assert.fail('Should have thrown an error')
      } catch (error) {
        assert.strictEqual(
          (error as Error).message,
          expectedError,
          'Error message should match expected',
        )
      }
    })
  })
})
