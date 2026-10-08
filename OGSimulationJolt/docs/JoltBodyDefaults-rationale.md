<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltBodyDefaults.h` — rationale

The named constants the Jolt factory gives every slot body, each equal to the value Unreal's Chaos
gives the same body today, and the Chaos mass formula. The point is that no slot body ever runs on a
Jolt default by accident: the two engines' defaults differ in every row below.

**If this file and `JoltBodyDefaults.h` disagree, the header is authoritative and this file is stale.**

Engine line references are to Unreal Engine 5.6 (`C:/dev/UnrealEngine` at the time of writing) and to
the vendored Jolt v5.6.0. The game file cited is og-brawler-unreal's
Source/OGBrawlerUnreal/OGBrawlerUECharacter.cpp.

---

## §1 Why constants and not descriptor fields

og-simulation's body descriptor (`BodyDescriptor` in `QueryGeometry.h`) carries flags only: simulate,
gravity, root, rotation lock, resim policy. It has no material, damping or mass fields, and M1 does not
add any (a user ruling: no new core fields in M1). So the backend supplies them. Descriptor-driven
materials and the richer body fields are a later step; when they land, these constants become the
defaults those fields start from.

## §2 The values and where each comes from

| Constant | Value | Chaos source |
|---|---|---|
| `kCreatedBodyMaterial` | friction 0.7, restitution 0.3 | Every body the Chaos factory **creates** (the weapon, guard and projectile spheres) has no material override, so UE uses the engine default physical material, /Engine/EngineMaterials/DefaultPhysicalMaterial. That asset overrides no property (its name table holds no property names), so it carries the UPhysicalMaterial constructor values: Friction 0.7, Restitution 0.3 (PhysicsCore/Private/PhysicalMaterial.cpp:44-46). Measured at run time: `getall PhysicalMaterial Friction` / `Restitution` print 0.7 / 0.3 |
| `kAdoptedRootMaterial` | friction 0, restitution 0 | The **adopted root** is the character's own capsule. The game gives it a material override with Friction 0 and Restitution 0 (OGBrawlerUECharacter.cpp:137-140, applied in PostInitializeComponents at :197). Measured: BrawlerCapsulePhysMat Friction 0, Restitution 0 |
| `kLinearDamping` | 0.01 | FBodyInstance constructor, LinearDamping(0.01) (Engine/Private/PhysicsEngine/BodyInstance.cpp:399). Measured: get_linear_damping() = 0.01 on all seven simulated brawler bodies |
| `kAngularDamping` | 0.0 | FBodyInstance constructor, AngularDamping(0.0) (BodyInstance.cpp:400). Measured: 0.0 |
| `kMaxAngularVelocityDegreesPerSecond` / `kMaxAngularVelocityRadiansPerSecond` | 3600 °/s = 62.8319 rad/s | UPhysicsSettingsCore::MaxAngularVelocity(3600) (PhysicsCore/Private/PhysicsSettingsCore.cpp:43); the project's DefaultEngine.ini does not override it. FBodyInstance reads it at body creation (BodyInstance.cpp:415, :3583-3586, applied at :4536) and Chaos clamps the angular speed to it during integration (Chaos/Private/Chaos/PBDRigidsEvolutionGBF.cpp:873-877; the clamp itself is :874-876) |
| `kMaxLinearVelocityMetresPerSecond` | 500 m/s | **Deviation.** Chaos has no linear speed cap for these bodies (the particle's MaxLinearSpeedSq starts at the numeric maximum, Chaos/Public/Chaos/ParticleHandle.h:116). Jolt needs a finite cap; 500 m/s (50,000 cm/s) is Jolt's own value, set explicitly, and is about 25 times the game's fastest body speed (the 2,000 cm/s knockback) |
| `kDensityGramsPerCubicCentimetre` / `kDensityKilogramsPerCubicCentimetre` | 1 g/cm³ = 0.001 kg/cm³ | UPhysicalMaterial Density = 1.0 for both materials above (PhysicalMaterial.cpp:48; measured 1.0 on both); converted g→kg as UE does (Engine/Private/PhysicsEngine/BodyUtils.cpp:29-45) |
| `kRaiseMassToPower` | 0.75 | UPhysicalMaterial RaiseMassToPower = 0.75 (PhysicalMaterial.cpp:47), applied by BodyUtils.cpp:48-114 (the power at :63-64) |
| `kMassScale` | 1 | FBodyInstance MassScale(1.f) (BodyInstance.cpp:403) |
| `kMinimumMassKilograms` | 0.001 | The 1 g floor in BodyUtils.cpp:67 |

**Combine rules.** `combineFrictionAverage` and `combineRestitutionAverage` return (a + b) / 2. Chaos
combines two materials with the mode ChooseCombineMode picks (the larger enum value of the two
materials' modes) and CombineHelper (Chaos/Public/Chaos/Defines.h:146-164;
Chaos/Private/Chaos/PBDCollisionConstraints.cpp:469-473). Each material's mode is copied from the
UPhysicalMaterial's own FrictionCombineMode / RestitutionCombineMode
(PhysicsCore/Private/ChaosEngineInterface.cpp:242-244). Neither property is set by the constructor, so
both are 0 = Average (PhysicsSettingsEnums.h:19); measured: `getall PhysicalMaterial
FrictionCombineMode` prints Average for both materials. In UE 5.6 nothing reads the project-wide
UPhysicsSettingsCore combine modes or the bOverride flags on this path (a search of the engine runtime
finds no reader), so the effective rule is Average for every pair. Jolt's defaults are sqrt(a·b) for
friction and max(a, b) for restitution (ContactConstraintManager.h:554-555).

Consequence worth knowing: under Average the capsule (0 / 0) against a default-material static (0.7 /
0.3) gets friction 0.35 and restitution 0.15, not 0 / 0. That is what Chaos does today, so it is what
Jolt must do.

## §3 Mass and inertia: `chaosParityMassPropertiesOf`

UE computes a body's mass from its collision shape at uniform density, then bends it
(Engine/Private/PhysicsEngine/BodyUtils.cpp:48-114):

1. raw mass = density × volume (kg, from cm³),
2. mass = max(MassScale × raw^RaiseMassToPower, 0.001),
3. the inertia tensor is scaled by mass / raw.

The shape formulas are Chaos's: sphere volume 4/3·π·r³ and inertia 2/5·m·r² (Chaos/Public/Chaos/Sphere.h:255-262,
357-363); capsule volume π·r²·(H + 4/3·r) and inertia diag(I₁, I₁, I₃) with
I₁ = m·(5H³ + 20H²r + 45Hr² + 32r³)/(60H + 80r), I₃ = m·r²·(15H + 16r)/(30H + 40r)
(Chaos/Public/Chaos/Capsule.h:491-509), where H is the cylinder length. UE builds a capsule
component's shape with H = 2·(halfHeight − radius) (Engine/Private/Components/CapsuleComponent.cpp:199),
which is how `CapsuleGeometry::halfHeight` is read here. Box: volume x·y·z, inertia m/12·(y²+z²), … with
full extents (not used by any brawler body).

The function computes in double and returns float kg and kg·cm². For the brawler (measured at run time
on today's Chaos build with UPrimitiveComponent GetMass / GetInertiaTensor, and equal to this function
to 7 significant digits):

| Body | Mass (kg) | Inertia (kg·cm²) |
|---|---|---|
| sphere r = 30 (weapon axis, projectiles) | 34.6808 | 12,485.09 on each axis |
| sphere r = 40 (guard axis) | 66.2524 | 42,401.52 on each axis |
| capsule 42 / 96 (character) | 165.527 | (454,866.2, 454,866.2, 136,024.6) |

Jolt's own default for the same shapes is density 1000 kg/m³ with no power law: the r = 30 sphere
would weigh 113.1 kg instead of 34.68. The capsule formula above is the exact solid-capsule inertia and
agrees with Jolt's own capsule formula (checked numerically: 2,747.985 and 821.766 cm² per kg on the
two axes).

## §4 Guards

**None.** Every value is pinned by a test (`JoltPhysicsFactory.EveryBrawlerDeclarationBindsAChaosParityBody`).
