/**
 * @esengine/nova-ecs-physics-box2d - Box2D physics engine implementation for NovaECS
 * NovaECS的Box2D物理引擎实现
 *
 * @packageDocumentation
 */

// Core Box2D implementations
export {
  Box2DEngine,
  Box2DWorld,
  Box2DRigidBody,
  Box2DCollider,
  Box2DJoint
} from './Box2DEngine';

// Plugin and factory
export {
  Box2DPhysicsPlugin,
  Box2DEngineFactory,
  Box2DPluginConfig,
  Box2DUtils,
  Box2DError,
  Box2DInitializationError,
  Box2DWorldError,
  Box2DBodyError,
  Box2DColliderError,
  Box2DJointError
} from './Box2DPlugin';

// Re-export core types for convenience
export type {
  IPhysicsEngine,
  IPhysicsWorld,
  IRigidBody,
  ICollider,
  IJoint,
  PhysicsWorldConfig,
  RigidBodyConfig,
  ColliderConfig,
  PhysicsMaterial,
  BasePhysicsPlugin,
  PhysicsPluginConfig
} from '@esengine/nova-ecs-physics-core';

// Re-export commonly used types from dependencies
export type { Component, System, Entity, World, BasePlugin } from '@esengine/nova-ecs';
export type { Fixed, FixedVector2, FixedMatrix2x2 } from '@esengine/nova-ecs-math';
