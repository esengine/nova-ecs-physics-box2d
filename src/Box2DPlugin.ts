/* eslint-disable @typescript-eslint/no-unsafe-call */
// Note: BasePhysicsPlugin logError method requires unsafe call due to inheritance
import {
  BasePhysicsPlugin,
  PhysicsPluginConfig,
  IPhysicsEngine,
  IPhysicsEngineFactory
} from '@esengine/nova-ecs-physics-core';
import { Box2DEngine } from './Box2DEngine';

/**
 * Box2D physics engine factory
 * Box2D物理引擎工厂
 */
export class Box2DEngineFactory implements IPhysicsEngineFactory {
  createEngine(): IPhysicsEngine {
    return new Box2DEngine();
  }

  getSupportedFeatures(): string[] {
    return [
      '2d',
      'rigid-bodies',
      'colliders',
      'joints',
      'raycast',
      'aabb-query',
      'collision-events',
      'sensors',
      'continuous-collision-detection',
      'sleeping',
      'debug-draw'
    ];
  }

  isFeatureSupported(feature: string): boolean {
    return this.getSupportedFeatures().includes(feature);
  }
}

/**
 * Box2D physics plugin configuration
 * Box2D物理插件配置
 */
export interface Box2DPluginConfig extends PhysicsPluginConfig {
  /** Enable continuous collision detection | 启用连续碰撞检测 */
  enableCCD?: boolean;
  /** Warm starting for better performance | 热启动以提高性能 */
  enableWarmStarting?: boolean;
  /** Sub-stepping for better stability | 子步进以提高稳定性 */
  enableSubStepping?: boolean;
}

/**
 * Box2D physics plugin for NovaECS
 * NovaECS的Box2D物理插件
 */
export class Box2DPhysicsPlugin extends BasePhysicsPlugin {
  constructor(config: Box2DPluginConfig = {}) {
    super(
      'Box2DPhysics',
      '1.0.0',
      new Box2DEngineFactory(),
      {
        worldConfig: {
          velocityIterations: 8,
          positionIterations: 3,
          ...config.worldConfig
        },
        fixedTimeStep: 1/60,
        maxSubSteps: 10,
        enableDebugRender: false,
        autoCreateSystems: true,
        ...config
      }
    );
  }

  /**
   * Validate Box2D specific configuration
   * 验证Box2D特定配置
   */
  validateConfig(config: Record<string, unknown>): boolean {
    // Validate Box2D specific settings
    if (config.enableCCD !== undefined && typeof config.enableCCD !== 'boolean') {
      this.error('enableCCD must be a boolean');
      return false;
    }

    if (config.enableWarmStarting !== undefined && typeof config.enableWarmStarting !== 'boolean') {
      this.error('enableWarmStarting must be a boolean');
      return false;
    }

    if (config.enableSubStepping !== undefined && typeof config.enableSubStepping !== 'boolean') {
      this.error('enableSubStepping must be a boolean');
      return false;
    }

    return super.validateConfig(config);
  }

  /**
   * Get Box2D specific debug information
   * 获取Box2D特定的调试信息
   */
  getDebugInfo(): Record<string, unknown> {
    const baseInfo = super.getDebugInfo();
    
    return {
      ...baseInfo,
      engineType: 'Box2D',
      wasmBased: true,
      supports2D: true,
      supports3D: false,
      supportsCCD: true,
      supportsJoints: true,
      supportsRaycast: true
    };
  }

  /**
   * Update plugin with Box2D specific logic
   * 使用Box2D特定逻辑更新插件
   */
  update(deltaTime: number): void {
    super.update(deltaTime);
    
    // Box2D specific update logic can be added here
    // 可以在这里添加Box2D特定的更新逻辑
  }
}

/**
 * Utility functions for Box2D integration
 * Box2D集成的实用函数
 */
export class Box2DUtils {
  /**
   * Convert Box2D body type to string
   * 将Box2D刚体类型转换为字符串
   */
  static bodyTypeToString(bodyType: number): string {
    switch (bodyType) {
      case 0: return 'static';
      case 1: return 'kinematic';
      case 2: return 'dynamic';
      default: return 'unknown';
    }
  }

  /**
   * Convert string to Box2D body type
   * 将字符串转换为Box2D刚体类型
   */
  static stringToBodyType(typeString: string): number {
    switch (typeString) {
      case 'static': return 0;
      case 'kinematic': return 1;
      case 'dynamic': return 2;
      default: throw new Error(`Unknown body type: ${typeString}`);
    }
  }

  /**
   * Create default physics material
   * 创建默认物理材质
   */
  static createDefaultMaterial(): { friction: number; restitution: number; density: number } {
    return {
      friction: 0.3,
      restitution: 0.1,
      density: 1.0
    };
  }

  /**
   * Create default collision filter
   * 创建默认碰撞过滤器
   */
  static createDefaultFilter(): { categoryBits: number; maskBits: number; groupIndex: number } {
    return {
      categoryBits: 0x0001,
      maskBits: 0xFFFF,
      groupIndex: 0
    };
  }
}

/**
 * Box2D specific error types
 * Box2D特定的错误类型
 */
export class Box2DError extends Error {
  constructor(message: string, public readonly code?: string) {
    super(`Box2D Error: ${message}`);
    this.name = 'Box2DError';
  }
}

export class Box2DInitializationError extends Box2DError {
  constructor(message: string) {
    super(message, 'INITIALIZATION_ERROR');
  }
}

export class Box2DWorldError extends Box2DError {
  constructor(message: string) {
    super(message, 'WORLD_ERROR');
  }
}

export class Box2DBodyError extends Box2DError {
  constructor(message: string) {
    super(message, 'BODY_ERROR');
  }
}

export class Box2DColliderError extends Box2DError {
  constructor(message: string) {
    super(message, 'COLLIDER_ERROR');
  }
}

export class Box2DJointError extends Box2DError {
  constructor(message: string) {
    super(message, 'JOINT_ERROR');
  }
}
