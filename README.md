# @esengine/nova-ecs-physics-box2d

Box2D physics engine implementation for NovaECS physics core. Provides deterministic 2D physics simulation using the industry-standard Box2D engine.

NovaECS物理核心的Box2D物理引擎实现。使用行业标准的Box2D引擎提供确定性2D物理模拟。

[![npm version](https://badge.fury.io/js/%40esengine%2Fnova-ecs-physics-box2d.svg)](https://badge.fury.io/js/%40esengine%2Fnova-ecs-physics-box2d)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.5.4-blue.svg)](https://www.typescriptlang.org/)

## Features | 特性

- **🎯 Deterministic**: Fixed-point integration ensures identical results across platforms | **确定性**：定点集成确保跨平台的相同结果
- **⚡ High Performance**: WebAssembly-based Box2D for optimal performance | **高性能**：基于WebAssembly的Box2D以获得最佳性能
- **🔧 Complete 2D Physics**: Rigid bodies, colliders, joints, raycast, and more | **完整的2D物理**：刚体、碰撞器、关节、射线投射等
- **🧮 Math Integration**: Seamless integration with @esengine/nova-ecs-math | **数学集成**：与@esengine/nova-ecs-math无缝集成
- **🔌 Pluggable**: Drop-in replacement for other physics engines | **可插拔**：其他物理引擎的直接替代品
- **📦 ECS Ready**: Optimized for Entity-Component-System architecture | **ECS就绪**：为实体组件系统架构优化
- **🧪 Battle Tested**: Based on the proven Box2D physics engine | **久经考验**：基于经过验证的Box2D物理引擎

## Installation | 安装

```bash
npm install @esengine/nova-ecs-physics-box2d
```

Required dependencies:
必需的依赖项：

```bash
npm install @esengine/nova-ecs @esengine/nova-ecs-math @esengine/nova-ecs-physics-core
```

## API Documentation | API 文档

For complete API documentation, visit: [https://esengine.github.io/nove-ecs-physics-box2d/](https://esengine.github.io/nova-ecs-physics-box2d/)

完整的API文档请访问：[https://esengine.github.io/nove-ecs-physics-box2d/](https://esengine.github.io/nova-ecs-physics-box2d/)

## Quick Start | 快速开始

```typescript
import { World } from '@esengine/nova-ecs';
import { FixedVector2, Fixed } from '@esengine/nova-ecs-math';
import {
  RigidBodyComponent,
  ColliderComponent,
  PhysicsTransformComponent,
  RigidBodyType,
  ColliderType
} from '@esengine/nova-ecs-physics-core';
import { Box2DPhysicsPlugin } from '@esengine/nova-ecs-physics-box2d';

// Create world and install Box2D physics plugin
const world = new World();
const physicsPlugin = new Box2DPhysicsPlugin({
  worldConfig: {
    gravity: new FixedVector2(0, -9.81),
    allowSleep: true,
    velocityIterations: 8,
    positionIterations: 3
  },
  fixedTimeStep: 1/60,
  enableDebugRender: false
});

await world.plugins.install(physicsPlugin);

// Create a falling box
const box = world.createEntity();

box.addComponent(new PhysicsTransformComponent(
  new FixedVector2(0, 10), // position
  0,                       // rotation
  new FixedVector2(1, 1)   // scale
));

box.addComponent(new RigidBodyComponent(
  RigidBodyType.Dynamic,   // dynamic body
  0.1,                     // linear damping
  0.1,                     // angular damping
  1.0                      // gravity scale
));

box.addComponent(new ColliderComponent({
  type: ColliderType.Box,
  halfWidth: new Fixed(1),
  halfHeight: new Fixed(1)
}, {
  friction: new Fixed(0.3),
  restitution: new Fixed(0.5),
  density: new Fixed(1.0)
}));

// Create static ground
const ground = world.createEntity();

ground.addComponent(new PhysicsTransformComponent(
  new FixedVector2(0, -5)
));

ground.addComponent(new RigidBodyComponent(RigidBodyType.Static));

ground.addComponent(new ColliderComponent({
  type: ColliderType.Box,
  halfWidth: new Fixed(10),
  halfHeight: new Fixed(1)
}));

// Game loop
function gameLoop(deltaTime: number) {
  world.update(deltaTime);
  
  // Get box position for rendering
  const transform = box.getComponent(PhysicsTransformComponent)!;
  console.log(`Box position: ${transform.position.x}, ${transform.position.y}`);
}

setInterval(() => gameLoop(16), 16);
```

## Advanced Usage | 高级用法

### Custom Physics Materials | 自定义物理材质

```typescript
import { PhysicsMaterial } from '@esengine/nova-ecs-physics-core';

// Create bouncy material
const bouncyMaterial: PhysicsMaterial = {
  friction: new Fixed(0.1),
  restitution: new Fixed(0.9),  // Very bouncy
  density: new Fixed(0.5)
};

// Create ice material
const iceMaterial: PhysicsMaterial = {
  friction: new Fixed(0.02),    // Very slippery
  restitution: new Fixed(0.1),
  density: new Fixed(0.8)
};

entity.addComponent(new ColliderComponent(
  { type: ColliderType.Circle, radius: new Fixed(1) },
  bouncyMaterial
));
```

### Collision Detection | 碰撞检测

```typescript
import { CollisionEventComponent } from '@esengine/nova-ecs-physics-core';

const entity = world.createEntity();
// ... add other components

const collisionEvents = new CollisionEventComponent();

// Handle collision begin
collisionEvents.addCollisionBeginCallback((other) => {
  console.log('Collision started with:', other);
});

// Handle collision end
collisionEvents.addCollisionEndCallback((other) => {
  console.log('Collision ended with:', other);
});

entity.addComponent(collisionEvents);
```

### Raycast | 射线投射

```typescript
import { RaycastInput } from '@esengine/nova-ecs-physics-core';

const physicsWorld = physicsPlugin.getWorldSystem()?.getPhysicsWorld();

if (physicsWorld) {
  const raycast: RaycastInput = {
    origin: new FixedVector2(0, 10),
    direction: new FixedVector2(0, -1), // Downward
    maxDistance: new Fixed(20)
  };
  
  const results = physicsWorld.raycast(raycast);
  
  for (const result of results) {
    if (result.hit) {
      console.log(`Hit at: ${result.point?.x}, ${result.point?.y}`);
      console.log(`Distance: ${result.distance?.toNumber()}`);
    }
  }
}
```

### Force and Impulse | 力和冲量

```typescript
// Apply continuous force (like wind)
const rigidBody = entity.getComponent(RigidBodyComponent)!;
rigidBody.applyForce(new FixedVector2(10, 0)); // Rightward force

// Apply instant impulse (like explosion)
rigidBody.applyImpulse(new FixedVector2(0, 50)); // Upward impulse

// Apply force at specific point (creates rotation)
rigidBody.applyForceAtPoint(
  new FixedVector2(10, 0),     // force
  new FixedVector2(0, 1)       // point offset from center
);
```

### Collision Filtering | 碰撞过滤

```typescript
import { CollisionFilter } from '@esengine/nova-ecs-physics-core';

// Define collision categories
const CATEGORY_PLAYER = 0x0001;
const CATEGORY_ENEMY = 0x0002;
const CATEGORY_WALL = 0x0004;
const CATEGORY_PICKUP = 0x0008;

// Player collides with enemies, walls, and pickups
const playerFilter: CollisionFilter = {
  categoryBits: CATEGORY_PLAYER,
  maskBits: CATEGORY_ENEMY | CATEGORY_WALL | CATEGORY_PICKUP
};

// Enemy collides with player and walls only
const enemyFilter: CollisionFilter = {
  categoryBits: CATEGORY_ENEMY,
  maskBits: CATEGORY_PLAYER | CATEGORY_WALL
};

entity.addComponent(new ColliderComponent(
  { type: ColliderType.Box, halfWidth: new Fixed(1), halfHeight: new Fixed(1) },
  undefined, // Use default material
  playerFilter
));
```

## Configuration | 配置

### Plugin Configuration | 插件配置

```typescript
const physicsPlugin = new Box2DPhysicsPlugin({
  worldConfig: {
    gravity: new FixedVector2(0, -9.81),
    allowSleep: true,
    velocityIterations: 8,      // Higher = more accurate but slower
    positionIterations: 3,      // Higher = more accurate but slower
    timeStep: new Fixed(1/60)
  },
  fixedTimeStep: 1/60,          // Physics time step
  maxSubSteps: 10,              // Maximum physics sub-steps per frame
  enableDebugRender: false,     // Enable debug visualization
  autoCreateSystems: true,      // Automatically create physics systems
  enableCCD: true,              // Continuous collision detection
  enableWarmStarting: true,     // Warm starting for better performance
  enableSubStepping: false      // Sub-stepping for better stability
});
```

## Performance Tips | 性能提示

1. **Use appropriate iteration counts** | **使用适当的迭代次数**
   - Lower velocityIterations and positionIterations for better performance
   - 降低velocityIterations和positionIterations以获得更好的性能

2. **Enable sleeping** | **启用休眠**
   - Allow bodies to sleep when not moving to save CPU
   - 允许不移动的物体休眠以节省CPU

3. **Use collision filtering** | **使用碰撞过滤**
   - Reduce unnecessary collision checks with proper filtering
   - 通过适当的过滤减少不必要的碰撞检查

4. **Optimize collider shapes** | **优化碰撞器形状**
   - Use simple shapes (box, circle) when possible
   - 尽可能使用简单形状（盒子、圆形）

## Supported Features | 支持的功能

- ✅ Rigid Bodies (Static, Kinematic, Dynamic) | 刚体（静态、运动学、动态）
- ✅ Colliders (Box, Circle) | 碰撞器（盒子、圆形）
- ✅ Physics Materials | 物理材质
- ✅ Collision Events | 碰撞事件
- ✅ Raycast | 射线投射
- ✅ AABB Queries | AABB查询
- ✅ Collision Filtering | 碰撞过滤
- ✅ Sensors | 传感器
- ✅ Continuous Collision Detection | 连续碰撞检测
- ✅ Sleeping | 休眠
- 🚧 Joints (Planned) | 关节（计划中）
- 🚧 Polygon Colliders (Planned) | 多边形碰撞器（计划中）
- 🚧 Debug Rendering (Planned) | 调试渲染（计划中）

## Related Projects | 相关项目

- [NovaECS](https://github.com/esengine/NovaECS) - Next-generation Entity Component System framework
- [nova-ecs-math](https://github.com/esengine/nova-ecs-math) - Fixed-point mathematics library
- [nova-ecs-physics-core](https://github.com/esengine/nova-ecs-physics-core) - Physics engine abstraction layer
- [Box2D](https://box2d.org/) - The original Box2D physics engine

## License | 许可证

MIT
