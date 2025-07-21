/* eslint-disable @typescript-eslint/no-unsafe-assignment, @typescript-eslint/no-unsafe-call, @typescript-eslint/no-unsafe-member-access, @typescript-eslint/no-unsafe-argument, @typescript-eslint/no-explicit-any, @typescript-eslint/no-unsafe-return, @typescript-eslint/restrict-template-expressions, @typescript-eslint/no-unsafe-enum-comparison */
// Note: Box2D WASM bindings require unsafe operations due to WASM interface limitations
import { Fixed, FixedVector2 } from '@esengine/nova-ecs-math';
import {
  IPhysicsEngine,
  IPhysicsWorld,
  IRigidBody,
  ICollider,
  IJoint,
  PhysicsWorldConfig,
  RigidBodyConfig,
  ColliderConfig,
  BaseJointConfig,
  PhysicsMaterial,
  RaycastInput,
  RaycastResult,
  CollisionEventData,
  JointType,
  PhysicsLogger,
  CollisionFilter
} from '@esengine/nova-ecs-physics-core';
import {
  DistanceJointConfig,
  RevoluteJointConfig,
  PrismaticJointConfig,
  WeldJointConfig,
  RopeJointConfig,
  MouseJointConfig,
  PulleyJointConfig,
  GearJointConfig,
  MotorJointConfig,
  WheelJointConfig,
  JointConfig
} from '@esengine/nova-ecs-physics-core';
import Box2DFactory from 'box2d-wasm';

// 定义Box2D实例类型
type Box2DInstance = typeof Box2D & EmscriptenModule;

/**
 * Box2D physics world implementation
 * Box2D物理世界实现
 */
export class Box2DWorld implements IPhysicsWorld {
  private world: Box2D.b2World;
  private box2d: Box2DInstance;
  private collisionCallbacks: Array<(data: CollisionEventData) => void> = [];
  private stepBeginCallbacks: Array<() => void> = [];
  private stepEndCallbacks: Array<() => void> = [];
  private activeContacts: Map<string, { startTime: number; contact: any }> = new Map();

  constructor(world: Box2D.b2World, box2d: Box2DInstance) {
    this.world = world;
    this.box2d = box2d;
    this.setupContactListener();
  }

  // Get the underlying Box2D world for advanced usage
  getBox2DWorld(): Box2D.b2World {
    return this.world;
  }

  step(deltaTime: Fixed): void {
    const timeStep = deltaTime.toNumber();
    const velocityIterations = 8;
    const positionIterations = 3;
    
    // Trigger step begin events
    this.stepBeginCallbacks.forEach(callback => {
      try {
        callback();
      } catch (error) {
        PhysicsLogger.warn(`Error in step begin callback: ${error}`);
      }
    });
    
    // Perform physics step
    this.world.Step(timeStep, velocityIterations, positionIterations);
    
    // Trigger step end events
    this.stepEndCallbacks.forEach(callback => {
      try {
        callback();
      } catch (error) {
        PhysicsLogger.warn(`Error in step end callback: ${error}`);
      }
    });
  }

  setGravity(gravity: FixedVector2): void {
    const b2Gravity = new this.box2d.b2Vec2(gravity.x.toNumber(), gravity.y.toNumber());
    this.world.SetGravity(b2Gravity);
  }

  getGravity(): FixedVector2 {
    const gravity = this.world.GetGravity();
    return new FixedVector2(gravity.x, gravity.y);
  }

  raycast(input: RaycastInput): RaycastResult[] {
    const results: RaycastResult[] = [];

    const callback = new this.box2d.JSRayCastCallback();
    callback.ReportFixture = (fixture: any, point: any, normal: any, fraction: number): number => {
      const hitPoint = new FixedVector2(point.x, point.y);
      const hitNormal = new FixedVector2(normal.x, normal.y);
      const distance = input.maxDistance.multiply(new Fixed(fraction));

      results.push({
        hit: true,
        point: hitPoint,
        normal: hitNormal,
        distance: distance,
        body: fixture.GetBody()
      });

      return fraction; // Continue raycast
    };

    const start = new this.box2d.b2Vec2(input.origin.x.toNumber(), input.origin.y.toNumber());
    const end = new this.box2d.b2Vec2(
      input.origin.x.add(input.direction.x.multiply(input.maxDistance)).toNumber(),
      input.origin.y.add(input.direction.y.multiply(input.maxDistance)).toNumber()
    );

    this.world.RayCast(callback, start, end);

    return results;
  }

  queryAABB(lowerBound: FixedVector2, upperBound: FixedVector2): unknown[] {
    const bodies: unknown[] = [];
    
    const callback = new this.box2d.JSQueryCallback();
    callback.ReportFixture = (fixture: any): boolean => {
      bodies.push(fixture.GetBody());
      return true; // Continue query
    };

    const aabb = new this.box2d.b2AABB();
    aabb.lowerBound.Set(lowerBound.x.toNumber(), lowerBound.y.toNumber());
    aabb.upperBound.Set(upperBound.x.toNumber(), upperBound.y.toNumber());

    this.world.QueryAABB(callback, aabb);
    
    return bodies;
  }

  onCollision(callback: (data: CollisionEventData) => void): void {
    this.collisionCallbacks.push(callback);
  }

  offCollision(callback: (data: CollisionEventData) => void): void {
    const index = this.collisionCallbacks.indexOf(callback);
    if (index !== -1) {
      this.collisionCallbacks.splice(index, 1);
    }
  }

  /**
   * Register callback for physics step begin events
   * 注册物理步骤开始事件的回调
   */
  onStepBegin(callback: () => void): void {
    this.stepBeginCallbacks.push(callback);
  }

  /**
   * Unregister callback for physics step begin events
   * 取消注册物理步骤开始事件的回调
   */
  offStepBegin(callback: () => void): void {
    const index = this.stepBeginCallbacks.indexOf(callback);
    if (index !== -1) {
      this.stepBeginCallbacks.splice(index, 1);
    }
  }

  /**
   * Register callback for physics step end events
   * 注册物理步骤结束事件的回调
   */
  onStepEnd(callback: () => void): void {
    this.stepEndCallbacks.push(callback);
  }

  /**
   * Unregister callback for physics step end events
   * 取消注册物理步骤结束事件的回调
   */
  offStepEnd(callback: () => void): void {
    const index = this.stepEndCallbacks.indexOf(callback);
    if (index !== -1) {
      this.stepEndCallbacks.splice(index, 1);
    }
  }

  private setupContactListener(): void {
    const contactListener = new this.box2d.JSContactListener();

    contactListener.BeginContact = (contact: any): void => {
      this.handleContact(contact, true);
    };

    contactListener.EndContact = (contact: any): void => {
      this.handleContact(contact, false);
    };

    this.world.SetContactListener(contactListener);
  }

  private handleContact(contact: any, isBeginContact: boolean): void {
    const fixtureA = contact.GetFixtureA();
    const fixtureB = contact.GetFixtureB();
    const bodyA = fixtureA.GetBody();
    const bodyB = fixtureB.GetBody();

    // Create unique contact ID
    const contactId = this.getContactId(bodyA, bodyB);
    
    if (isBeginContact) {
      // Track contact start time for duration calculation
      this.activeContacts.set(contactId, {
        startTime: Date.now(),
        contact
      });
    }

    // Extract contact points and impulses
    const contactPoints = this.extractContactPoints(contact);
    
    // Handle contact tracking for end events
    if (!isBeginContact) {
      const contactData = this.activeContacts.get(contactId);
      if (contactData) {
        this.activeContacts.delete(contactId);
      }
    }

    const eventData: CollisionEventData = {
      bodyA,
      bodyB,
      contacts: contactPoints,
      isBeginContact,
      isEndContact: !isBeginContact
    };

    this.collisionCallbacks.forEach(callback => callback(eventData));
  }

  /**
   * Generate unique ID for contact pair
   * 为接触对生成唯一ID
   */
  private getContactId(bodyA: any, bodyB: any): string {
    const idA = bodyA.GetUserData()?.id || bodyA.ptr || 'unknown';
    const idB = bodyB.GetUserData()?.id || bodyB.ptr || 'unknown';
    
    // Ensure consistent ordering for the same pair
    return idA < idB ? `${idA}-${idB}` : `${idB}-${idA}`;
  }

  /**
   * Extract contact points from Box2D contact
   * 从Box2D接触中提取接触点
   */
  private extractContactPoints(contact: any): Array<{
    position: FixedVector2;
    normal: FixedVector2;
    separation: Fixed;
    normalImpulse: Fixed;
    tangentImpulse: Fixed;
  }> {
    const contactPoints: Array<{
      position: FixedVector2;
      normal: FixedVector2;
      separation: Fixed;
      normalImpulse: Fixed;
      tangentImpulse: Fixed;
    }> = [];

    try {
      const manifold = contact.GetManifold();
      const worldManifold = new (this.world.constructor as any).b2WorldManifold();
      contact.GetWorldManifold(worldManifold);

      const pointCount = manifold.pointCount;
      
      for (let i = 0; i < pointCount; i++) {
        const manifoldPoint = manifold.points[i];
        const worldPoint = worldManifold.points[i];
        
        contactPoints.push({
          position: new FixedVector2(worldPoint.x, worldPoint.y),
          normal: new FixedVector2(worldManifold.normal.x, worldManifold.normal.y),
          separation: new Fixed(worldManifold.separations[i]),
          normalImpulse: new Fixed(manifoldPoint.normalImpulse),
          tangentImpulse: new Fixed(manifoldPoint.tangentImpulse)
        });
      }
    } catch (error) {
      // If contact point extraction fails, return empty array
      PhysicsLogger.warn(`Failed to extract contact points: ${error}`);
    }

    return contactPoints;
  }

  destroy(): void {
    try {
      // Clear all callbacks to prevent memory leaks
      this.collisionCallbacks.length = 0;
      this.stepBeginCallbacks.length = 0;
      this.stepEndCallbacks.length = 0;
      this.activeContacts.clear();

      // Remove contact listener
      if (this.world) {
        // Create an empty contact listener to replace the current one
        const emptyListener = new this.box2d.JSContactListener();
        this.world.SetContactListener(emptyListener);
      }
    } catch (error) {
      PhysicsLogger.warn(`Error during Box2DWorld cleanup: ${error}`);
    }
  }

  getDebugDrawData(): unknown {
    if (!this.world) {
      return null;
    }

    const debugData = {
      bodies: this.extractBodyDebugData(),
      joints: this.extractJointDebugData(),
      contacts: this.extractContactDebugData(),
      statistics: this.getWorldStatistics()
    };

    return debugData;
  }

  /**
   * Extract debug data for all bodies in the world
   * 为世界中的所有物体提取调试数据
   */
  private extractBodyDebugData(): Array<{
    id: string;
    type: string;
    position: FixedVector2;
    rotation: Fixed;
    linearVelocity: FixedVector2;
    angularVelocity: Fixed;
    isAwake: boolean;
    isActive: boolean;
    fixtures: Array<{
      shape: string;
      vertices?: FixedVector2[];
      radius?: Fixed;
      isSensor: boolean;
      density: Fixed;
      friction: Fixed;
      restitution: Fixed;
    }>;
  }> {
    const bodyData: Array<any> = [];
    
    try {
      for (let body = this.world.GetBodyList(); body; body = body.GetNext()) {
        const position = body.GetPosition();
        const velocity = body.GetLinearVelocity();
        
        const fixtures: Array<any> = [];
        for (let fixture = body.GetFixtureList(); fixture; fixture = fixture.GetNext()) {
          const shape = fixture.GetShape();
          const fixtureData: any = {
            isSensor: fixture.IsSensor(),
            density: new Fixed(fixture.GetDensity()),
            friction: new Fixed(fixture.GetFriction()),
            restitution: new Fixed(fixture.GetRestitution())
          };

          // Extract shape-specific data
          const shapeType = shape.GetType();
          switch (shapeType) {
            case 0: // b2Shape::e_circle
              fixtureData.shape = 'circle';
              fixtureData.radius = new Fixed((shape as any).m_radius);
              break;
            case 1: // b2Shape::e_edge
              fixtureData.shape = 'edge';
              fixtureData.vertices = [
                new FixedVector2((shape as any).m_vertex1.x, (shape as any).m_vertex1.y),
                new FixedVector2((shape as any).m_vertex2.x, (shape as any).m_vertex2.y)
              ];
              break;
            case 2: // b2Shape::e_polygon
              fixtureData.shape = 'polygon';
              const vertices: FixedVector2[] = [];
              const polygonShape = shape as any;
              for (let i = 0; i < polygonShape.m_count; i++) {
                const vertex = polygonShape.m_vertices[i];
                vertices.push(new FixedVector2(vertex.x, vertex.y));
              }
              fixtureData.vertices = vertices;
              break;
            case 3: // b2Shape::e_chain
              fixtureData.shape = 'chain';
              const chainVertices: FixedVector2[] = [];
              const chainShape = shape as any;
              for (let i = 0; i < chainShape.m_count; i++) {
                const vertex = chainShape.m_vertices[i];
                chainVertices.push(new FixedVector2(vertex.x, vertex.y));
              }
              fixtureData.vertices = chainVertices;
              break;
            default:
              fixtureData.shape = 'unknown';
          }

          fixtures.push(fixtureData);
        }

        bodyData.push({
          id: body.GetUserData() || 'unknown',
          type: this.getBodyTypeString(body.GetType()),
          position: new FixedVector2(position.x, position.y),
          rotation: new Fixed(body.GetAngle()),
          linearVelocity: new FixedVector2(velocity.x, velocity.y),
          angularVelocity: new Fixed(body.GetAngularVelocity()),
          isAwake: body.IsAwake(),
          isActive: body.IsEnabled(),
          fixtures
        });
      }
    } catch (error) {
      PhysicsLogger.warn(`Failed to extract body debug data: ${error}`);
    }

    return bodyData;
  }

  /**
   * Extract debug data for all joints in the world
   * 为世界中的所有关节提取调试数据
   */
  private extractJointDebugData(): Array<{
    id: string;
    type: string;
    bodyA: string;
    bodyB: string;
    anchorA: FixedVector2;
    anchorB: FixedVector2;
    reactionForce: FixedVector2;
    reactionTorque: Fixed;
    isActive: boolean;
  }> {
    const jointData: Array<any> = [];
    
    try {
      for (let joint = this.world.GetJointList(); joint; joint = joint.GetNext()) {
        const bodyA = joint.GetBodyA();
        const bodyB = joint.GetBodyB();
        const anchorA = joint.GetAnchorA();
        const anchorB = joint.GetAnchorB();
        const reactionForce = joint.GetReactionForce(1.0);

        jointData.push({
          id: joint.GetUserData() || 'unknown',
          type: this.getJointTypeString(joint.GetType()),
          bodyA: bodyA.GetUserData() || 'unknown',
          bodyB: bodyB.GetUserData() || 'unknown',
          anchorA: new FixedVector2(anchorA.x, anchorA.y),
          anchorB: new FixedVector2(anchorB.x, anchorB.y),
          reactionForce: new FixedVector2(reactionForce.x, reactionForce.y),
          reactionTorque: new Fixed(joint.GetReactionTorque(1.0)),
          isActive: !!(joint && joint.GetBodyA() && joint.GetBodyB())
        });
      }
    } catch (error) {
      PhysicsLogger.warn(`Failed to extract joint debug data: ${error}`);
    }

    return jointData;
  }

  /**
   * Extract debug data for all contacts in the world
   * 为世界中的所有接触提取调试数据
   */
  private extractContactDebugData(): Array<{
    bodyA: string;
    bodyB: string;
    points: Array<{
      point: FixedVector2;
      normal: FixedVector2;
      separation: Fixed;
    }>;
    isEnabled: boolean;
    isTouching: boolean;
  }> {
    const contactData: Array<any> = [];
    
    try {
      for (let contact = this.world.GetContactList(); contact; contact = contact.GetNext()) {
        if (!contact.IsTouching()) continue;

        const bodyA = contact.GetFixtureA().GetBody();
        const bodyB = contact.GetFixtureB().GetBody();
        
        const manifold = contact.GetManifold();
        const worldManifold = new (this.world.constructor as any).b2WorldManifold();
        contact.GetWorldManifold(worldManifold);

        const points: Array<any> = [];
        const pointCount = manifold.pointCount;
        
        for (let i = 0; i < pointCount; i++) {
          const worldPoint = worldManifold.points[i];
          points.push({
            point: new FixedVector2(worldPoint.x, worldPoint.y),
            normal: new FixedVector2(worldManifold.normal.x, worldManifold.normal.y),
            separation: new Fixed(worldManifold.separations[i])
          });
        }

        contactData.push({
          bodyA: bodyA.GetUserData() || 'unknown',
          bodyB: bodyB.GetUserData() || 'unknown',
          points,
          isEnabled: contact.IsEnabled(),
          isTouching: contact.IsTouching()
        });
      }
    } catch (error) {
      PhysicsLogger.warn(`Failed to extract contact debug data: ${error}`);
    }

    return contactData;
  }

  /**
   * Get world statistics for debug purposes
   * 获取用于调试的世界统计信息
   */
  private getWorldStatistics(): {
    bodyCount: number;
    activeBodies: number;
    sleepingBodies: number;
    jointCount: number;
    contactCount: number;
    proxyCount: number;
    treeHeight: number;
  } {
    let bodyCount = 0;
    let activeBodies = 0;
    let sleepingBodies = 0;
    let jointCount = 0;
    let contactCount = 0;

    // Count bodies
    for (let body = this.world.GetBodyList(); body; body = body.GetNext()) {
      bodyCount++;
      if (body.IsAwake()) {
        activeBodies++;
      } else {
        sleepingBodies++;
      }
    }

    // Count joints
    for (let joint = this.world.GetJointList(); joint; joint = joint.GetNext()) {
      jointCount++;
    }

    // Count contacts
    for (let contact = this.world.GetContactList(); contact; contact = contact.GetNext()) {
      contactCount++;
    }

    return {
      bodyCount,
      activeBodies,
      sleepingBodies,
      jointCount,
      contactCount,
      proxyCount: this.world.GetProxyCount(),
      treeHeight: this.world.GetTreeHeight()
    };
  }

  /**
   * Convert Box2D body type to string
   * 将Box2D物体类型转换为字符串
   */
  private getBodyTypeString(bodyType: number): string {
    switch (bodyType) {
      case 0: return 'static';    // b2_staticBody
      case 1: return 'kinematic'; // b2_kinematicBody
      case 2: return 'dynamic';   // b2_dynamicBody
      default: return 'unknown';
    }
  }

  /**
   * Convert Box2D joint type to string
   * 将Box2D关节类型转换为字符串
   */
  private getJointTypeString(jointType: number): string {
    switch (jointType) {
      case 1: return 'revolute';  // e_revoluteJoint
      case 2: return 'prismatic'; // e_prismaticJoint
      case 3: return 'distance';  // e_distanceJoint
      case 4: return 'pulley';    // e_pulleyJoint
      case 5: return 'mouse';     // e_mouseJoint
      case 6: return 'gear';      // e_gearJoint
      case 7: return 'wheel';     // e_wheelJoint
      case 8: return 'weld';      // e_weldJoint
      case 9: return 'friction';  // e_frictionJoint
      case 10: return 'rope';     // e_ropeJoint
      case 11: return 'motor';    // e_motorJoint
      default: return 'unknown';
    }
  }
}

/**
 * Box2D rigid body implementation
 * Box2D刚体实现
 */
export class Box2DRigidBody implements IRigidBody {
  private body: Box2D.b2Body;
  private box2d: Box2DInstance;

  constructor(body: Box2D.b2Body, box2d: Box2DInstance) {
    this.body = body;
    this.box2d = box2d;
  }

  getPosition(): FixedVector2 {
    try {
      if (!this.body) {
        throw new Error('Body has been destroyed');
      }
      const pos = this.body.GetPosition();
      return new FixedVector2(pos.x, pos.y);
    } catch (error) {
      PhysicsLogger.warn(`Error getting body position: ${error}`);
      return new FixedVector2();
    }
  }

  setPosition(position: FixedVector2): void {
    try {
      if (!this.body) {
        throw new Error('Body has been destroyed');
      }
      const angle = this.body.GetAngle();
      const pos = new this.box2d.b2Vec2(position.x.toNumber(), position.y.toNumber());
      this.body.SetTransform(pos, angle);
    } catch (error) {
      PhysicsLogger.warn(`Error setting body position: ${error}`);
    }
  }

  getRotation(): Fixed {
    return new Fixed(this.body.GetAngle());
  }

  setRotation(rotation: Fixed): void {
    const pos = this.body.GetPosition();
    this.body.SetTransform(pos, rotation.toNumber());
  }

  getLinearVelocity(): FixedVector2 {
    const vel = this.body.GetLinearVelocity();
    return new FixedVector2(vel.x, vel.y);
  }

  setLinearVelocity(velocity: FixedVector2): void {
    this.body.SetLinearVelocity(
      new this.box2d.b2Vec2(velocity.x.toNumber(), velocity.y.toNumber())
    );
  }

  getAngularVelocity(): Fixed {
    return new Fixed(this.body.GetAngularVelocity());
  }

  setAngularVelocity(velocity: Fixed): void {
    this.body.SetAngularVelocity(velocity.toNumber());
  }

  applyForce(force: FixedVector2): void {
    const center = this.body.GetWorldCenter();
    this.body.ApplyForce(
      new this.box2d.b2Vec2(force.x.toNumber(), force.y.toNumber()),
      center,
      true
    );
  }

  applyForceAtPoint(force: FixedVector2, point: FixedVector2): void {
    this.body.ApplyForce(
      new this.box2d.b2Vec2(force.x.toNumber(), force.y.toNumber()),
      new this.box2d.b2Vec2(point.x.toNumber(), point.y.toNumber()),
      true
    );
  }

  applyImpulse(impulse: FixedVector2): void {
    const center = this.body.GetWorldCenter();
    this.body.ApplyLinearImpulse(
      new this.box2d.b2Vec2(impulse.x.toNumber(), impulse.y.toNumber()),
      center,
      true
    );
  }

  applyImpulseAtPoint(impulse: FixedVector2, point: FixedVector2): void {
    this.body.ApplyLinearImpulse(
      new this.box2d.b2Vec2(impulse.x.toNumber(), impulse.y.toNumber()),
      new this.box2d.b2Vec2(point.x.toNumber(), point.y.toNumber()),
      true
    );
  }

  applyTorque(torque: Fixed): void {
    this.body.ApplyTorque(torque.toNumber(), true);
  }

  getMass(): Fixed {
    return new Fixed(this.body.GetMass());
  }

  setMass(mass: Fixed): void {
    const massData = new this.box2d.b2MassData();
    massData.mass = mass.toNumber();
    massData.center = this.body.GetLocalCenter();
    massData.I = this.body.GetInertia();
    this.body.SetMassData(massData);
  }

  isAwake(): boolean {
    return this.body.IsAwake();
  }

  setAwake(awake: boolean): void {
    this.body.SetAwake(awake);
  }

  isActive(): boolean {
    return this.body.IsEnabled();
  }

  setActive(active: boolean): void {
    this.body.SetEnabled(active);
  }

  getUserData(): unknown {
    try {
      const userData = this.body.GetUserData();
      return userData.pointer;
    } catch (error) {
      PhysicsLogger.warn(`Failed to get body user data: ${error}`);
      return undefined;
    }
  }

  setUserData(data: unknown): void {
    try {
      const userData = this.body.GetUserData();
      userData.pointer = data as number;
    } catch (error) {
      PhysicsLogger.warn(`Failed to set body user data: ${error}`);
    }
  }

  destroy(): void {
    try {
      if (this.body) {
        // Clear user data to prevent potential memory leaks
        try {
          const userData = this.body.GetUserData();
          userData.pointer = 0;
        } catch (error) {
          PhysicsLogger.warn(`Failed to clear body user data: ${error}`);
        }

        // Note: Body destruction is handled by the world
        // We just mark it as destroyed for safety
        this.body = null as any;
      }
    } catch (error) {
      PhysicsLogger.warn(`Error during body cleanup: ${error}`);
    }
  }

  /**
   * Check if the body is still valid
   * 检查物体是否仍然有效
   */
  isValid(): boolean {
    return this.body !== null && this.body !== undefined;
  }

  // Get the underlying Box2D body for advanced usage
  getBox2DBody(): Box2D.b2Body {
    return this.body;
  }
}

/**
 * Box2D collider implementation
 * Box2D碰撞器实现
 */
export class Box2DCollider implements ICollider {
  private fixture: Box2D.b2Fixture;
  private rigidBody: Box2DRigidBody;
  private box2d: Box2DInstance;

  constructor(fixture: Box2D.b2Fixture, rigidBody: Box2DRigidBody, box2d: Box2DInstance) {
    this.fixture = fixture;
    this.rigidBody = rigidBody;
    this.box2d = box2d;
  }

  getBody(): IRigidBody {
    return this.rigidBody;
  }

  setMaterial(material: PhysicsMaterial): void {
    this.fixture.SetFriction(material.friction.toNumber());
    this.fixture.SetRestitution(material.restitution.toNumber());
    this.fixture.SetDensity(material.density.toNumber());
    this.fixture.GetBody().ResetMassData();
  }

  getMaterial(): PhysicsMaterial {
    return {
      friction: new Fixed(this.fixture.GetFriction()),
      restitution: new Fixed(this.fixture.GetRestitution()),
      density: new Fixed(this.fixture.GetDensity())
    };
  }

  setFilter(filter: CollisionFilter): void {
    const filterData = new this.box2d.b2Filter();
    
    // Set collision category (what this fixture is)
    filterData.categoryBits = filter.categoryBits || 0x0001;
    
    // Set collision mask (what this fixture collides with)
    filterData.maskBits = filter.maskBits !== undefined ? filter.maskBits : 0xFFFF;
    
    // Set group index (positive = always collide, negative = never collide, 0 = normal)
    filterData.groupIndex = filter.groupIndex || 0;
    
    this.fixture.SetFilterData(filterData);
    
    // Trigger contact filter recalculation
    const body = this.fixture.GetBody();
    
    // Flag contacts for filtering recalculation
    for (let edge = body.GetContactList(); edge; edge = edge.next) {
      const contact = edge.contact;
      const fixtureA = contact.GetFixtureA();
      const fixtureB = contact.GetFixtureB();
      
      if (fixtureA === this.fixture || fixtureB === this.fixture) {
        // Note: FlagForFiltering may not be available in all Box2D versions
        // contact.FlagForFiltering();
        PhysicsLogger.warn('Contact filtering requested but not implemented');
      }
    }
  }

  getFilter(): CollisionFilter {
    const filterData = this.fixture.GetFilterData();
    return {
      categoryBits: filterData.categoryBits,
      maskBits: filterData.maskBits,
      groupIndex: filterData.groupIndex
    };
  }

  setSensor(isSensor: boolean): void {
    this.fixture.SetSensor(isSensor);
  }

  isSensor(): boolean {
    return this.fixture.IsSensor();
  }

  getUserData(): unknown {
    try {
      const userData = this.fixture.GetUserData();
      return userData.pointer;
    } catch (error) {
      PhysicsLogger.warn(`Failed to get fixture user data: ${error}`);
      return undefined;
    }
  }

  setUserData(data: unknown): void {
    try {
      const userData = this.fixture.GetUserData();
      userData.pointer = data as number;
    } catch (error) {
      PhysicsLogger.warn(`Failed to set fixture user data: ${error}`);
    }
  }

  destroy(): void {
    // Fixture destruction will be handled by the body
  }

  // Get the underlying Box2D fixture for advanced usage
  getBox2DFixture(): Box2D.b2Fixture {
    return this.fixture;
  }
}

/**
 * Box2D joint implementation
 * Box2D关节实现
 */
export class Box2DJoint implements IJoint {
  private joint: Box2D.b2Joint;
  private bodyA: Box2DRigidBody;
  private bodyB: Box2DRigidBody;
  private box2d: Box2DInstance;

  constructor(joint: Box2D.b2Joint, bodyA: Box2DRigidBody, bodyB: Box2DRigidBody, box2d: Box2DInstance) {
    this.joint = joint;
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    this.box2d = box2d;
  }

  getBodyA(): IRigidBody {
    return this.bodyA;
  }

  getBodyB(): IRigidBody {
    return this.bodyB;
  }

  getReactionForce(): FixedVector2 {
    const force = this.joint.GetReactionForce(1.0);
    return new FixedVector2(force.x, force.y);
  }

  getReactionTorque(): Fixed {
    return new Fixed(this.joint.GetReactionTorque(1.0));
  }

  isActive(): boolean {
    // Box2D doesn't have IsActive method for joints
    // We consider a joint active if it exists and has valid bodies
    try {
      return !!(this.joint && this.joint.GetBodyA() && this.joint.GetBodyB());
    } catch (error) {
      PhysicsLogger.warn(`Failed to check joint active state: ${error}`);
      return false;
    }
  }

  getUserData(): unknown {
    try {
      const userData = this.joint.GetUserData();
      return userData.pointer;
    } catch (error) {
      PhysicsLogger.warn(`Failed to get joint user data: ${error}`);
      return undefined;
    }
  }

  setUserData(data: unknown): void {
    try {
      const userData = this.joint.GetUserData();
      userData.pointer = data as number;
    } catch (error) {
      PhysicsLogger.warn(`Failed to set joint user data: ${error}`);
    }
  }

  destroy(): void {
    // Joint destruction will be handled by the world
  }

  // Get the underlying Box2D joint for advanced usage
  getBox2DJoint(): Box2D.b2Joint {
    return this.joint;
  }
}

/**
 * Box2D physics engine implementation
 * Box2D物理引擎实现
 */
export class Box2DEngine implements IPhysicsEngine {
  readonly name = 'Box2D';
  readonly version = '7.0.0';

  private box2d: Box2DInstance | null = null;
  private initialized = false;

  async initialize(): Promise<void> {
    if (this.initialized) return;

    try {
      this.box2d = await Box2DFactory();
      if (!this.box2d) {
        throw new Error('Failed to load Box2D WASM module');
      }
      this.initialized = true;
    } catch (error) {
      this.initialized = false;
      throw new Error(`Box2D initialization failed: ${error}`);
    }
  }

  createWorld(config: PhysicsWorldConfig): IPhysicsWorld {
    if (!this.box2d) {
      throw new Error('Box2D engine not initialized');
    }

    try {
      // Validate configuration
      if (!config.gravity) {
        throw new Error('World configuration missing gravity');
      }

      const gravity = new this.box2d.b2Vec2(
        config.gravity.x.toNumber(),
        config.gravity.y.toNumber()
      );

      const world = new this.box2d.b2World(gravity);
      
      // Configure world settings
      world.SetAllowSleeping(config.allowSleep ?? true);
      
      // Set iterations if provided
      if (config.velocityIterations !== undefined && config.velocityIterations > 0) {
        // Note: iterations are set during step, stored for validation
      }
      if (config.positionIterations !== undefined && config.positionIterations > 0) {
        // Note: iterations are set during step, stored for validation
      }

      return new Box2DWorld(world, this.box2d);
    } catch (error) {
      throw new Error(`Failed to create Box2D world: ${error}`);
    }
  }

  createRigidBody(world: IPhysicsWorld, config: RigidBodyConfig): IRigidBody {
    if (!this.box2d) {
      throw new Error('Box2D engine not initialized');
    }

    try {
      // Validate inputs
      if (!world) {
        throw new Error('World is required');
      }
      if (!config) {
        throw new Error('RigidBody configuration is required');
      }
      if (!config.position) {
        throw new Error('RigidBody position is required');
      }

      const box2dWorld = (world as Box2DWorld).getBox2DWorld();
      if (!box2dWorld) {
        throw new Error('Invalid Box2D world');
      }

    const bodyDef = new this.box2d.b2BodyDef();

    // Set body type
    switch (config.type as string) {
      case 'static':
        bodyDef.type = this.box2d.b2_staticBody;
        break;
      case 'kinematic':
        bodyDef.type = this.box2d.b2_kinematicBody;
        break;
      case 'dynamic':
        bodyDef.type = this.box2d.b2_dynamicBody;
        break;
    }

    // Set position and rotation
    bodyDef.position.Set(
      config.position.x.toNumber(),
      config.position.y.toNumber()
    );
    bodyDef.angle = config.rotation?.toNumber() ?? 0;

    // Set velocities
    if (config.linearVelocity) {
      bodyDef.linearVelocity.Set(
        config.linearVelocity.x.toNumber(),
        config.linearVelocity.y.toNumber()
      );
    }
    bodyDef.angularVelocity = config.angularVelocity?.toNumber() ?? 0;

    // Set damping
    bodyDef.linearDamping = config.linearDamping?.toNumber() ?? 0;
    bodyDef.angularDamping = config.angularDamping?.toNumber() ?? 0;

    // Set other properties
    bodyDef.gravityScale = config.gravityScale?.toNumber() ?? 1;
    bodyDef.allowSleep = config.allowSleep ?? true;
    bodyDef.awake = config.awake ?? true;
    bodyDef.fixedRotation = config.fixedRotation ?? false;
    bodyDef.bullet = config.bullet ?? false;

      const body = box2dWorld.CreateBody(bodyDef);
      if (!body) {
        throw new Error('Failed to create Box2D body');
      }
      
      if (config.userData !== undefined) {
        try {
          const userData = body.GetUserData();
          userData.pointer = config.userData as number;
        } catch (error) {
          PhysicsLogger.warn(`Failed to set body user data: ${error}`);
        }
      }

      return new Box2DRigidBody(body, this.box2d);
    } catch (error) {
      throw new Error(`Failed to create rigid body: ${error}`);
    }
  }

  createCollider(body: IRigidBody, config: ColliderConfig, material?: PhysicsMaterial): ICollider {
    if (!this.box2d) {
      throw new Error('Box2D engine not initialized');
    }

    const box2dBody = (body as Box2DRigidBody).getBox2DBody();
    const fixtureDef = new this.box2d.b2FixtureDef();

    // Set material properties
    if (material) {
      fixtureDef.friction = material.friction.toNumber();
      fixtureDef.restitution = material.restitution.toNumber();
      fixtureDef.density = material.density.toNumber();
    }

    // Create shape based on config type
    let shape: any;

    switch (config.type as string) {
      case 'box': {
        const boxConfig = config as any; // BoxColliderConfig
        shape = new this.box2d.b2PolygonShape();
        shape.SetAsBox(
          boxConfig.halfWidth.toNumber(),
          boxConfig.halfHeight.toNumber(),
          boxConfig.offset ? new this.box2d.b2Vec2(
            boxConfig.offset.x.toNumber(),
            boxConfig.offset.y.toNumber()
          ) : new this.box2d.b2Vec2(0, 0),
          boxConfig.rotation?.toNumber() ?? 0
        );
        break;
      }
      case 'circle': {
        const circleConfig = config as any; // CircleColliderConfig
        shape = new this.box2d.b2CircleShape();
        shape.m_radius = circleConfig.radius.toNumber();
        if (circleConfig.offset) {
          shape.m_p.Set(
            circleConfig.offset.x.toNumber(),
            circleConfig.offset.y.toNumber()
          );
        }
        break;
      }
      case 'polygon': {
        const polygonConfig = config as any; // PolygonColliderConfig
        shape = new this.box2d.b2PolygonShape();
        
        // Create vertices array
        const vertices: Box2D.b2Vec2[] = [];
        for (const vertex of polygonConfig.vertices) {
          vertices.push(new this.box2d.b2Vec2(vertex.x.toNumber(), vertex.y.toNumber()));
        }
        
        // Set vertices (Box2D expects vertices in counter-clockwise order)
        shape.Set(vertices, vertices.length);
        break;
      }
      case 'edge': {
        const edgeConfig = config as any; // EdgeColliderConfig
        shape = new this.box2d.b2EdgeShape();
        shape.SetTwoSided(
          new this.box2d.b2Vec2(edgeConfig.vertex1.x.toNumber(), edgeConfig.vertex1.y.toNumber()),
          new this.box2d.b2Vec2(edgeConfig.vertex2.x.toNumber(), edgeConfig.vertex2.y.toNumber())
        );
        break;
      }
      case 'chain': {
        const chainConfig = config as any; // ChainColliderConfig
        shape = new this.box2d.b2ChainShape();
        
        // Create vertices array
        const vertices: Box2D.b2Vec2[] = [];
        for (const vertex of chainConfig.vertices) {
          vertices.push(new this.box2d.b2Vec2(vertex.x.toNumber(), vertex.y.toNumber()));
        }
        
        if (chainConfig.loop) {
          shape.CreateLoop(vertices, vertices.length);
        } else {
          shape.CreateChain(vertices, vertices.length);
        }
        break;
      }
      default:
        throw new Error(`Unsupported collider type: ${config.type}`);
    }

    fixtureDef.shape = shape;
    const fixture = box2dBody.CreateFixture(fixtureDef);

    return new Box2DCollider(fixture, body as Box2DRigidBody, this.box2d);
  }

  createJoint(world: IPhysicsWorld, config: BaseJointConfig): IJoint {
    if (!this.box2d) {
      throw new Error('Box2D engine not initialized');
    }

    const box2dWorld = (world as Box2DWorld).getBox2DWorld();
    const jointConfig = config as JointConfig;

    // Get the Box2D bodies from the joint configuration
    const bodyA = (jointConfig.bodyA as Box2DRigidBody).getBox2DBody();
    const bodyB = (jointConfig.bodyB as Box2DRigidBody).getBox2DBody();

    let joint: Box2D.b2Joint;

    switch (jointConfig.type as string) {
      case JointType.Distance:
        joint = this.createDistanceJoint(box2dWorld, bodyA, bodyB, jointConfig as DistanceJointConfig);
        break;
      case JointType.Revolute:
        joint = this.createRevoluteJoint(box2dWorld, bodyA, bodyB, jointConfig as RevoluteJointConfig);
        break;
      case JointType.Prismatic:
        joint = this.createPrismaticJoint(box2dWorld, bodyA, bodyB, jointConfig as PrismaticJointConfig);
        break;
      case JointType.Weld:
        joint = this.createWeldJoint(box2dWorld, bodyA, bodyB, jointConfig as WeldJointConfig);
        break;
      case JointType.Rope:
        joint = this.createRopeJoint(box2dWorld, bodyA, bodyB, jointConfig as RopeJointConfig);
        break;
      case JointType.Mouse:
        joint = this.createMouseJoint(box2dWorld, bodyA, bodyB, jointConfig as MouseJointConfig);
        break;
      case JointType.Pulley:
        joint = this.createPulleyJoint(box2dWorld, bodyA, bodyB, jointConfig as PulleyJointConfig);
        break;
      case JointType.Gear:
        joint = this.createGearJoint(box2dWorld, bodyA, bodyB, jointConfig as GearJointConfig);
        break;
      case JointType.Motor:
        joint = this.createMotorJoint(box2dWorld, bodyA, bodyB, jointConfig as MotorJointConfig);
        break;
      case JointType.Wheel:
        joint = this.createWheelJoint(box2dWorld, bodyA, bodyB, jointConfig as WheelJointConfig);
        break;
      default:
        throw new Error(`Unsupported joint type: ${String(jointConfig.type)}`);
    }

    return new Box2DJoint(joint, jointConfig.bodyA as Box2DRigidBody, jointConfig.bodyB as Box2DRigidBody, this.box2d);
  }

  /**
   * Create distance joint with spring and damping properties
   * 创建具有弹簧和阻尼属性的距离关节
   */
  private createDistanceJoint(world: Box2D.b2World, bodyA: Box2D.b2Body, bodyB: Box2D.b2Body, config: DistanceJointConfig): Box2D.b2Joint {
    if (!this.box2d) {
      throw new Error('Box2D engine not initialized');
    }

    const jointDef = new this.box2d.b2DistanceJointDef();
    
    // Set anchor points
    const anchorA = new this.box2d.b2Vec2(config.localAnchorA.x.toNumber(), config.localAnchorA.y.toNumber());
    const anchorB = new this.box2d.b2Vec2(config.localAnchorB.x.toNumber(), config.localAnchorB.y.toNumber());
    
    jointDef.Initialize(bodyA, bodyB, 
      bodyA.GetWorldPoint(anchorA),
      bodyB.GetWorldPoint(anchorB)
    );
    
    // Set length
    jointDef.length = config.length.toNumber();
    
    // Set optional properties
    if (config.minLength !== undefined) {
      jointDef.minLength = config.minLength.toNumber();
    }
    if (config.maxLength !== undefined) {
      jointDef.maxLength = config.maxLength.toNumber();
    }
    if (config.stiffness !== undefined) {
      jointDef.stiffness = config.stiffness.toNumber();
    }
    if (config.damping !== undefined) {
      jointDef.damping = config.damping.toNumber();
    }

    // Set common joint properties
    jointDef.collideConnected = config.collideConnected ?? false;

    return world.CreateJoint(jointDef);
  }

  /**
   * Create revolute joint with motor and limit support
   * 创建具有电机和限制支持的旋转关节
   */
  private createRevoluteJoint(world: Box2D.b2World, bodyA: Box2D.b2Body, bodyB: Box2D.b2Body, config: RevoluteJointConfig): Box2D.b2Joint {
    if (!this.box2d) {
      throw new Error('Box2D engine not initialized');
    }

    const jointDef = new this.box2d.b2RevoluteJointDef();
    
    // Set anchor points
    const anchorA = new this.box2d.b2Vec2(config.localAnchorA.x.toNumber(), config.localAnchorA.y.toNumber());
    
    jointDef.Initialize(bodyA, bodyB, bodyA.GetWorldPoint(anchorA));
    
    // Set reference angle
    if (config.referenceAngle !== undefined) {
      jointDef.referenceAngle = config.referenceAngle.toNumber();
    }
    
    // Set limits
    if (config.enableLimit !== undefined) {
      jointDef.enableLimit = config.enableLimit;
      if (config.lowerAngle !== undefined) {
        jointDef.lowerAngle = config.lowerAngle.toNumber();
      }
      if (config.upperAngle !== undefined) {
        jointDef.upperAngle = config.upperAngle.toNumber();
      }
    }
    
    // Set motor
    if (config.enableMotor !== undefined) {
      jointDef.enableMotor = config.enableMotor;
      if (config.motorSpeed !== undefined) {
        jointDef.motorSpeed = config.motorSpeed.toNumber();
      }
      if (config.maxMotorTorque !== undefined) {
        jointDef.maxMotorTorque = config.maxMotorTorque.toNumber();
      }
    }

    // Set common joint properties
    jointDef.collideConnected = config.collideConnected ?? false;

    return world.CreateJoint(jointDef);
  }

  /**
   * Create prismatic joint with translation limits and motors
   * 创建具有平移限制和电机的棱柱关节
   */
  private createPrismaticJoint(world: Box2D.b2World, bodyA: Box2D.b2Body, bodyB: Box2D.b2Body, config: PrismaticJointConfig): Box2D.b2Joint {
    if (!this.box2d) {
      throw new Error('Box2D engine not initialized');
    }

    const jointDef = new this.box2d.b2PrismaticJointDef();
    
    // Set anchor points and axis
    const anchorA = new this.box2d.b2Vec2(config.localAnchorA.x.toNumber(), config.localAnchorA.y.toNumber());
    const axis = new this.box2d.b2Vec2(config.localAxisA.x.toNumber(), config.localAxisA.y.toNumber());
    
    jointDef.Initialize(bodyA, bodyB, bodyA.GetWorldPoint(anchorA), axis);
    
    // Set reference angle
    if (config.referenceAngle !== undefined) {
      jointDef.referenceAngle = config.referenceAngle.toNumber();
    }
    
    // Set limits
    if (config.enableLimit !== undefined) {
      jointDef.enableLimit = config.enableLimit;
      if (config.lowerTranslation !== undefined) {
        jointDef.lowerTranslation = config.lowerTranslation.toNumber();
      }
      if (config.upperTranslation !== undefined) {
        jointDef.upperTranslation = config.upperTranslation.toNumber();
      }
    }
    
    // Set motor
    if (config.enableMotor !== undefined) {
      jointDef.enableMotor = config.enableMotor;
      if (config.motorSpeed !== undefined) {
        jointDef.motorSpeed = config.motorSpeed.toNumber();
      }
      if (config.maxMotorForce !== undefined) {
        jointDef.maxMotorForce = config.maxMotorForce.toNumber();
      }
    }

    // Set common joint properties
    jointDef.collideConnected = config.collideConnected ?? false;

    return world.CreateJoint(jointDef);
  }

  /**
   * Create weld joint for rigid connections
   * 创建用于刚性连接的焊接关节
   */
  private createWeldJoint(world: Box2D.b2World, bodyA: Box2D.b2Body, bodyB: Box2D.b2Body, config: WeldJointConfig): Box2D.b2Joint {
    if (!this.box2d) {
      throw new Error('Box2D engine not initialized');
    }

    const jointDef = new this.box2d.b2WeldJointDef();
    
    // Set anchor points
    const anchorA = new this.box2d.b2Vec2(config.localAnchorA.x.toNumber(), config.localAnchorA.y.toNumber());
    
    jointDef.Initialize(bodyA, bodyB, bodyA.GetWorldPoint(anchorA));
    
    // Set reference angle
    if (config.referenceAngle !== undefined) {
      jointDef.referenceAngle = config.referenceAngle.toNumber();
    }
    
    // Set stiffness and damping
    if (config.stiffness !== undefined) {
      jointDef.stiffness = config.stiffness.toNumber();
    }
    if (config.damping !== undefined) {
      jointDef.damping = config.damping.toNumber();
    }

    // Set common joint properties
    jointDef.collideConnected = config.collideConnected ?? false;

    return world.CreateJoint(jointDef);
  }

  /**
   * Create rope joint for maximum distance constraints
   * 创建用于最大距离约束的绳索关节
   */
  private createRopeJoint(world: Box2D.b2World, bodyA: Box2D.b2Body, bodyB: Box2D.b2Body, config: RopeJointConfig): Box2D.b2Joint {
    if (!this.box2d) {
      throw new Error('Box2D engine not initialized');
    }

    // Note: Box2D doesn't have b2RopeJointDef, using distance joint instead
    const jointDef = new this.box2d.b2DistanceJointDef();
    
    // Set anchor points
    jointDef.localAnchorA.Set(config.localAnchorA.x.toNumber(), config.localAnchorA.y.toNumber());
    jointDef.localAnchorB.Set(config.localAnchorB.x.toNumber(), config.localAnchorB.y.toNumber());
    jointDef.bodyA = bodyA;
    jointDef.bodyB = bodyB;
    
    // Set maximum length
    jointDef.maxLength = config.maxLength.toNumber();

    // Set common joint properties
    jointDef.collideConnected = config.collideConnected ?? false;

    return world.CreateJoint(jointDef);
  }

  /**
   * Create mouse joint for user interaction
   * 创建用于用户交互的鼠标关节
   */
  private createMouseJoint(world: Box2D.b2World, bodyA: Box2D.b2Body, bodyB: Box2D.b2Body, config: MouseJointConfig): Box2D.b2Joint {
    if (!this.box2d) {
      throw new Error('Box2D engine not initialized');
    }

    const jointDef = new this.box2d.b2MouseJointDef();
    
    jointDef.bodyA = bodyA;
    jointDef.bodyB = bodyB;
    
    // Set target point
    jointDef.target.Set(config.target.x.toNumber(), config.target.y.toNumber());
    
    // Set maximum force
    jointDef.maxForce = config.maxForce.toNumber();
    
    // Set stiffness and damping
    if (config.stiffness !== undefined) {
      jointDef.stiffness = config.stiffness.toNumber();
    }
    if (config.damping !== undefined) {
      jointDef.damping = config.damping.toNumber();
    }

    // Set common joint properties
    jointDef.collideConnected = config.collideConnected ?? false;

    return world.CreateJoint(jointDef);
  }

  /**
   * Create pulley joint for mechanical advantage systems
   * 创建用于机械优势系统的滑轮关节
   */
  private createPulleyJoint(world: Box2D.b2World, bodyA: Box2D.b2Body, bodyB: Box2D.b2Body, config: PulleyJointConfig): Box2D.b2Joint {
    if (!this.box2d) {
      throw new Error('Box2D engine not initialized');
    }

    const jointDef = new this.box2d.b2PulleyJointDef();
    
    // Set anchor points
    const groundAnchorA = new this.box2d.b2Vec2(config.groundAnchorA.x.toNumber(), config.groundAnchorA.y.toNumber());
    const groundAnchorB = new this.box2d.b2Vec2(config.groundAnchorB.x.toNumber(), config.groundAnchorB.y.toNumber());
    const anchorA = new this.box2d.b2Vec2(config.localAnchorA.x.toNumber(), config.localAnchorA.y.toNumber());
    const anchorB = new this.box2d.b2Vec2(config.localAnchorB.x.toNumber(), config.localAnchorB.y.toNumber());
    
    jointDef.Initialize(
      bodyA, bodyB,
      groundAnchorA, groundAnchorB,
      bodyA.GetWorldPoint(anchorA),
      bodyB.GetWorldPoint(anchorB),
      config.ratio?.toNumber() ?? 1.0
    );
    
    // Set rope lengths
    jointDef.lengthA = config.lengthA.toNumber();
    jointDef.lengthB = config.lengthB.toNumber();

    // Set common joint properties
    jointDef.collideConnected = config.collideConnected ?? false;

    return world.CreateJoint(jointDef);
  }

  /**
   * Create gear joint connecting two other joints
   * 创建连接两个其他关节的齿轮关节
   */
  private createGearJoint(world: Box2D.b2World, bodyA: Box2D.b2Body, bodyB: Box2D.b2Body, config: GearJointConfig): Box2D.b2Joint {
    if (!this.box2d) {
      throw new Error('Box2D engine not initialized');
    }

    const jointDef = new this.box2d.b2GearJointDef();
    
    jointDef.bodyA = bodyA;
    jointDef.bodyB = bodyB;
    
    // Set the two joints to connect
    jointDef.joint1 = config.joint1 as Box2D.b2Joint;
    jointDef.joint2 = config.joint2 as Box2D.b2Joint;
    
    // Set gear ratio
    jointDef.ratio = config.ratio.toNumber();

    // Set common joint properties
    jointDef.collideConnected = config.collideConnected ?? false;

    return world.CreateJoint(jointDef);
  }

  /**
   * Create motor joint for direct force/torque application
   * 创建用于直接力/扭矩应用的电机关节
   */
  private createMotorJoint(world: Box2D.b2World, bodyA: Box2D.b2Body, bodyB: Box2D.b2Body, config: MotorJointConfig): Box2D.b2Joint {
    if (!this.box2d) {
      throw new Error('Box2D engine not initialized');
    }

    const jointDef = new this.box2d.b2MotorJointDef();
    
    jointDef.Initialize(bodyA, bodyB);
    
    // Set linear and angular offsets
    jointDef.linearOffset.Set(config.linearOffset.x.toNumber(), config.linearOffset.y.toNumber());
    jointDef.angularOffset = config.angularOffset.toNumber();
    
    // Set maximum force and torque
    jointDef.maxForce = config.maxForce.toNumber();
    jointDef.maxTorque = config.maxTorque.toNumber();
    
    // Set correction factor
    if (config.correctionFactor !== undefined) {
      jointDef.correctionFactor = config.correctionFactor.toNumber();
    }

    // Set common joint properties
    jointDef.collideConnected = config.collideConnected ?? false;

    return world.CreateJoint(jointDef);
  }

  /**
   * Create wheel joint for vehicle suspension systems
   * 创建用于车辆悬挂系统的车轮关节
   */
  private createWheelJoint(world: Box2D.b2World, bodyA: Box2D.b2Body, bodyB: Box2D.b2Body, config: WheelJointConfig): Box2D.b2Joint {
    if (!this.box2d) {
      throw new Error('Box2D engine not initialized');
    }

    const jointDef = new this.box2d.b2WheelJointDef();
    
    // Set anchor points and axis
    const anchorA = new this.box2d.b2Vec2(config.localAnchorA.x.toNumber(), config.localAnchorA.y.toNumber());
    const axis = new this.box2d.b2Vec2(config.localAxisA.x.toNumber(), config.localAxisA.y.toNumber());
    
    jointDef.Initialize(bodyA, bodyB, bodyA.GetWorldPoint(anchorA), axis);
    
    // Set motor
    if (config.enableMotor !== undefined) {
      jointDef.enableMotor = config.enableMotor;
      if (config.motorSpeed !== undefined) {
        jointDef.motorSpeed = config.motorSpeed.toNumber();
      }
      if (config.maxMotorTorque !== undefined) {
        jointDef.maxMotorTorque = config.maxMotorTorque.toNumber();
      }
    }
    
    // Set spring properties
    if (config.stiffness !== undefined) {
      jointDef.stiffness = config.stiffness.toNumber();
    }
    if (config.damping !== undefined) {
      jointDef.damping = config.damping.toNumber();
    }

    // Set common joint properties
    jointDef.collideConnected = config.collideConnected ?? false;

    return world.CreateJoint(jointDef);
  }

  destroy(): void {
    this.box2d = null;
    this.initialized = false;
  }

  getDebugInfo(): unknown {
    return {
      name: this.name,
      version: this.version,
      initialized: this.initialized
    };
  }
}
