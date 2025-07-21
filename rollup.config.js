import typescript from '@rollup/plugin-typescript';
import { dts } from 'rollup-plugin-dts';

const external = [
  '@esengine/nova-ecs',
  '@esengine/nova-ecs-math',
  '@esengine/nova-ecs-physics-core',
  'box2d-wasm'
];

export default [
  // ES Module build
  {
    input: 'src/index.ts',
    output: {
      file: 'dist/nova-ecs-physics-box2d.esm.js',
      format: 'es',
      sourcemap: true
    },
    external,
    plugins: [
      typescript({
        tsconfig: './tsconfig.json',
        declaration: false,
        declarationMap: false
      })
    ]
  },
  
  // CommonJS build
  {
    input: 'src/index.ts',
    output: {
      file: 'dist/nova-ecs-physics-box2d.cjs.js',
      format: 'cjs',
      sourcemap: true
    },
    external,
    plugins: [
      typescript({
        tsconfig: './tsconfig.json',
        declaration: false,
        declarationMap: false
      })
    ]
  },
  
  // UMD build
  {
    input: 'src/index.ts',
    output: {
      file: 'dist/nova-ecs-physics-box2d.umd.js',
      format: 'umd',
      name: 'NovaECSPhysicsBox2D',
      sourcemap: true,
      globals: {
        '@esengine/nova-ecs': 'NovaECS',
        '@esengine/nova-ecs-math': 'NovaECSMath',
        '@esengine/nova-ecs-physics-core': 'NovaECSPhysicsCore',
        'box2d-wasm': 'Box2D'
      }
    },
    external,
    plugins: [
      typescript({
        tsconfig: './tsconfig.json',
        declaration: false,
        declarationMap: false
      })
    ]
  },
  
  // Type definitions
  {
    input: 'src/index.ts',
    output: {
      file: 'dist/nova-ecs-physics-box2d.d.ts',
      format: 'es'
    },
    external,
    plugins: [dts()]
  }
];
