// SPDX-License-Identifier: MPL-2.0
// docs/JoltStaticWorldBuilder-rationale.md · docs/JoltStaticWorldBuilder-guards.md

#include "OGSimulationJolt/JoltStaticWorldBuilder.h"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <tuple>
#include <type_traits>
#include <variant>

#include <Jolt/Core/Reference.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>

#include "glm/common.hpp"
#include "glm/geometric.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/mat3x3.hpp"
#include "OGSimulation/OGAssert.h"
#include "OGSimulationJolt/JoltUnits.h"

namespace
{
	static_assert(kStaticShapeTypeCount == 5,
		"JoltStaticWorldBuilder: StaticShape gained or lost an alternative; give it a Jolt shape in ElementShapeBuilder and a name in the build summary line");

	constexpr float kRigidScaleTolerance = 1e-4f;
	constexpr uint32_t kMaxPerElementWarnings = 16;

	class ContentHash
	{
	public:
		void u32(uint32_t value)
		{
			for (int i = 0; i < 4; ++i)
			{
				m_value ^= static_cast<uint8_t>(value >> (8 * i));
				m_value *= 1099511628211ull;
			}
		}

		void f32(float value) { u32(std::bit_cast<uint32_t>(value)); }
		void vec3(const glm::vec3& value) { f32(value.x); f32(value.y); f32(value.z); }
		[[nodiscard]] uint64_t value() const { return m_value; }

	private:
		uint64_t m_value = 14695981039346656037ull;
	};

	struct ContentHasher
	{
		ContentHash& hash;

		void operator()(const StaticBox& box) const { hash.vec3(box.halfExtents); }
		void operator()(const StaticSphere& sphere) const { hash.f32(sphere.radius); }
		void operator()(const StaticCapsuleZ& capsule) const { hash.f32(capsule.radius); hash.f32(capsule.totalHalfHeight); }
		void operator()(const StaticConvexHull& hull) const
		{
			hash.u32(static_cast<uint32_t>(hull.points.size()));
			for (const glm::vec3& point : hull.points)
			{
				hash.vec3(point);
			}
		}
		void operator()(const StaticTriangleMesh& mesh) const
		{
			hash.u32(static_cast<uint32_t>(mesh.vertices.size()));
			for (const glm::vec3& vertex : mesh.vertices)
			{
				hash.vec3(vertex);
			}
			hash.u32(static_cast<uint32_t>(mesh.indices.size()));
			for (const uint32_t index : mesh.indices)
			{
				hash.u32(index);
			}
		}
	};

	uint64_t contentHashOf(const StaticShapeDescriptor& descriptor)
	{
		ContentHash hash;
		hash.u32(static_cast<uint32_t>(descriptor.shape.index()));
		std::visit(ContentHasher{ hash }, descriptor.shape);
		for (int column = 0; column < 4; ++column)
		{
			for (int row = 0; row < 4; ++row)
			{
				hash.f32(descriptor.localToWorld[column][row]);
			}
		}
		hash.u32(descriptor.categories.bits);
		hash.u32(descriptor.blockingCategories.bits);
		hash.f32(descriptor.friction);
		hash.f32(descriptor.restitution);
		return hash.value();
	}

	struct Placement
	{
		JPH::Vec3 positionMetres = JPH::Vec3::sZero();
		JPH::Quat rotation = JPH::Quat::sIdentity();
		JPH::Vec3 scale = JPH::Vec3::sReplicate(1.f);
		bool rigid = true;
		bool valid = true;
	};

	Placement placementOf(const glm::mat4& localToWorld)
	{
		Placement placement;
		const glm::vec3 axisX(localToWorld[0]);
		const glm::vec3 axisY(localToWorld[1]);
		const glm::vec3 axisZ(localToWorld[2]);
		glm::vec3 scale(glm::length(axisX), glm::length(axisY), glm::length(axisZ));
		if (!(scale.x > 0.f && scale.y > 0.f && scale.z > 0.f) || !std::isfinite(scale.x + scale.y + scale.z))
		{
			placement.valid = false;
			return placement;
		}
		if (glm::dot(axisX, glm::cross(axisY, axisZ)) < 0.f)
		{
			scale.x = -scale.x;
		}
		const glm::mat3 rotation(axisX / scale.x, axisY / scale.y, axisZ / scale.z);
		const glm::quat quaternion = glm::normalize(glm::quat_cast(rotation));
		placement.rotation = JPH::Quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w).Normalized();
		placement.positionMetres = joltUnits::centimetresToMetres(glm::vec3(localToWorld[3]));
		placement.scale = JPH::Vec3(scale.x, scale.y, scale.z);
		placement.rigid = std::abs(scale.x - 1.f) <= kRigidScaleTolerance && std::abs(scale.y - 1.f) <= kRigidScaleTolerance
			&& std::abs(scale.z - 1.f) <= kRigidScaleTolerance;
		return placement;
	}

	struct ElementShapeBuilder
	{
		const JoltStaticWorldBuilderOptions& options;

		float convexRadiusForMinExtentCm(float minFullExtentCm) const
		{
			const float radiusCm = std::min(options.convexRadiusFractionOfMinExtent * minFullExtentCm, options.maxConvexRadiusCm);
			return joltUnits::centimetresToMetres(std::max(radiusCm, 0.f));
		}

		JPH::Shape::ShapeResult operator()(const StaticBox& box) const
		{
			const float minFullExtentCm = 2.f * std::min({ box.halfExtents.x, box.halfExtents.y, box.halfExtents.z });
			return JPH::BoxShapeSettings(joltUnits::centimetresToMetres(box.halfExtents), convexRadiusForMinExtentCm(minFullExtentCm)).Create();
		}

		JPH::Shape::ShapeResult operator()(const StaticSphere& sphere) const
		{
			return JPH::SphereShapeSettings(joltUnits::centimetresToMetres(sphere.radius)).Create();
		}

		JPH::Shape::ShapeResult operator()(const StaticCapsuleZ& capsule) const
		{
			if (capsule.totalHalfHeight < capsule.radius)
			{
				JPH::Shape::ShapeResult result;
				result.SetError("StaticCapsuleZ::totalHalfHeight is less than its radius");
				return result;
			}
			const float cylinderHalfHeight = joltUnits::centimetresToMetres(capsule.totalHalfHeight - capsule.radius);
			const float radius = joltUnits::centimetresToMetres(capsule.radius);
			if (cylinderHalfHeight <= 0.f)
			{
				return JPH::SphereShapeSettings(radius).Create();
			}
			const JPH::Quat yAxisToZAxis = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.5f * JPH::JPH_PI);
			return JPH::RotatedTranslatedShapeSettings(JPH::Vec3::sZero(), yAxisToZAxis, new JPH::CapsuleShapeSettings(cylinderHalfHeight, radius)).Create();
		}

		JPH::Shape::ShapeResult operator()(const StaticConvexHull& hull) const
		{
			if (hull.points.empty())
			{
				JPH::Shape::ShapeResult result;
				result.SetError("StaticConvexHull has no points");
				return result;
			}
			JPH::Array<JPH::Vec3> points;
			points.reserve(hull.points.size());
			glm::vec3 lower = hull.points.front();
			glm::vec3 upper = hull.points.front();
			for (const glm::vec3& point : hull.points)
			{
				points.push_back(joltUnits::centimetresToMetres(point));
				lower = glm::min(lower, point);
				upper = glm::max(upper, point);
			}
			const glm::vec3 extent = upper - lower;
			const float minFullExtentCm = std::min({ extent.x, extent.y, extent.z });
			return JPH::ConvexHullShapeSettings(points, convexRadiusForMinExtentCm(minFullExtentCm)).Create();
		}

		JPH::Shape::ShapeResult operator()(const StaticTriangleMesh& mesh) const
		{
			JPH::Shape::ShapeResult result;
			if (mesh.indices.empty() || mesh.indices.size() % 3 != 0)
			{
				result.SetError("StaticTriangleMesh::indices is empty or not a multiple of 3");
				return result;
			}
			for (const uint32_t index : mesh.indices)
			{
				if (index >= mesh.vertices.size())
				{
					result.SetError("StaticTriangleMesh::indices references a vertex out of range");
					return result;
				}
			}
			JPH::VertexList vertices;
			vertices.reserve(mesh.vertices.size());
			for (const glm::vec3& vertex : mesh.vertices)
			{
				JPH::Float3 stored;
				joltUnits::centimetresToMetres(vertex).StoreFloat3(&stored);
				vertices.push_back(stored);
			}
			JPH::IndexedTriangleList triangles;
			triangles.reserve(mesh.indices.size() / 3);
			for (size_t first = 0; first < mesh.indices.size(); first += 3)
			{
				// ⛔G-02  docs/JoltStaticWorldBuilder-guards.md
				triangles.push_back(JPH::IndexedTriangle(mesh.indices[first], mesh.indices[first + 1], mesh.indices[first + 2], 0));
			}
			return JPH::MeshShapeSettings(std::move(vertices), std::move(triangles)).Create();
		}
	};

	struct SortedElement
	{
		const StaticShapeDescriptor* descriptor = nullptr;
		JoltLayerKey layerKey;
		JoltStaticSurface surface;
		uint64_t contentHash = 0;
	};

	auto surfaceOrderOf(const JoltStaticSurface& surface)
	{
		return std::make_tuple(std::bit_cast<uint32_t>(surface.friction), std::bit_cast<uint32_t>(surface.restitution));
	}

	bool sameBody(const SortedElement& first, const SortedElement& second)
	{
		return first.layerKey == second.layerKey && surfaceOrderOf(first.surface) == surfaceOrderOf(second.surface);
	}

	struct PreparedElement
	{
		uint64_t stableKey = 0;
		JPH::RefConst<JPH::Shape> shape;
		Placement placement;
	};

	double steadySeconds()
	{
		return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
	}
}

JoltStaticWorldBuilder::JoltStaticWorldBuilder(JoltWorld& world, JoltWorldLoggerFn logger, JoltStaticWorldBuilderOptions options)
	: m_world(world)
	, m_logger(std::move(logger))
	, m_options(options)
{
	OG_CHECK(m_options.maxShapesPerChunk >= 1, "JoltStaticWorldBuilder: maxShapesPerChunk must be at least 1");
	m_options.maxShapesPerChunk = std::max(m_options.maxShapesPerChunk, 1u);
}

StaticWorldBuildReport JoltStaticWorldBuilder::build(const StaticWorldDescription& description)
{
	StaticWorldBuildReport report;
	OG_CHECK(!m_built, "JoltStaticWorldBuilder::build - a builder builds its world's statics once");
	if (m_built)
	{
		log("[Warning] JoltStaticWorldBuilder: build called twice; the second call added nothing");
		return report;
	}
	m_built = true;
	const double startSeconds = steadySeconds();

	std::vector<SortedElement> sorted;
	sorted.reserve(description.shapes.size());
	for (const StaticShapeDescriptor& descriptor : description.shapes)
	{
		SortedElement element;
		element.descriptor = &descriptor;
		element.layerKey = JoltLayerKey{ JoltBroadPhaseClass::Static, descriptor.categories.bits, descriptor.blockingCategories.bits };
		element.surface = JoltStaticSurface{ descriptor.friction, descriptor.restitution };
		if (descriptor.friction == 0.f && descriptor.restitution == 0.f)
		{
			++m_stats.zeroSurfaceElements;
			if (m_options.surfaceForZeroFrictionAndRestitution.has_value())
			{
				element.surface = *m_options.surfaceForZeroFrictionAndRestitution;
			}
		}
		element.contentHash = contentHashOf(descriptor);
		sorted.push_back(element);
	}
	// ⛔G-01  docs/JoltStaticWorldBuilder-guards.md
	std::sort(sorted.begin(), sorted.end(), [](const SortedElement& first, const SortedElement& second)
	{
		return std::make_tuple(first.layerKey, surfaceOrderOf(first.surface), first.descriptor->stableKey, first.contentHash)
			< std::make_tuple(second.layerKey, surfaceOrderOf(second.surface), second.descriptor->stableKey, second.contentHash);
	});
	for (size_t index = 1; index < sorted.size(); ++index)
	{
		if (sorted[index].descriptor->stableKey == sorted[index - 1].descriptor->stableKey)
		{
			++m_stats.duplicateStableKeys;
		}
	}

	if (m_stats.zeroSurfaceElements > 0)
	{
		char line[256];
		std::snprintf(line, sizeof(line),
			"[Warning] JoltStaticWorldBuilder: %u static element(s) arrived with friction = restitution = 0 (a host with no physical material); %s",
			m_stats.zeroSurfaceElements,
			m_options.surfaceForZeroFrictionAndRestitution.has_value() ? "they use the configured fallback surface" : "they keep 0/0 (no fallback configured)");
		log(line);
	}

	ElementShapeBuilder shapeBuilder{ m_options };
	uint32_t elementWarnings = 0;
	const auto warnElement = [this, &elementWarnings](const char* what, uint64_t stableKey, const char* detail)
	{
		if (elementWarnings++ >= kMaxPerElementWarnings)
		{
			return;
		}
		char line[320];
		std::snprintf(line, sizeof(line), "[Warning] JoltStaticWorldBuilder: %s (stableKey 0x%016llx): %s", what,
			static_cast<unsigned long long>(stableKey), detail);
		log(line);
	};

	std::vector<JPH::BodyID> created;
	const auto createBody = [this, &created](const JPH::Shape* shape, JPH::ObjectLayer layer, const JoltStaticSurface& surface)
	{
		JPH::BodyCreationSettings settings(shape, JPH::RVec3::sZero(), JPH::Quat::sIdentity(), JPH::EMotionType::Static, layer);
		settings.mFriction = surface.friction;
		settings.mRestitution = surface.restitution;
		const JPH::BodyID id = m_world.createStaticBody(settings);
		if (!id.IsInvalid())
		{
			created.push_back(id);
		}
	};
	const auto compoundOf = [](const PreparedElement* first, size_t count)
	{
		JPH::StaticCompoundShapeSettings compound;
		for (size_t index = 0; index < count; ++index)
		{
			compound.AddShape(first[index].placement.positionMetres, first[index].placement.rotation, first[index].shape);
		}
		return compound.Create();
	};

	size_t groupBegin = 0;
	while (groupBegin < sorted.size())
	{
		size_t groupEnd = groupBegin + 1;
		while (groupEnd < sorted.size() && sameBody(sorted[groupBegin], sorted[groupEnd]))
		{
			++groupEnd;
		}
		++m_stats.groups;

		const std::optional<JPH::ObjectLayer> layer = m_world.layers().findLayer(sorted[groupBegin].layerKey);
		if (!layer.has_value())
		{
			m_stats.skippedNoLayer += static_cast<uint32_t>(groupEnd - groupBegin);
			char line[256];
			std::snprintf(line, sizeof(line),
				"[Warning] JoltStaticWorldBuilder: no STATIC layer for categories 0x%08x / blocking 0x%08x in the world's table; %u element(s) skipped",
				sorted[groupBegin].layerKey.categories, sorted[groupBegin].layerKey.blockingCategories, static_cast<unsigned>(groupEnd - groupBegin));
			log(line);
			OG_CHECK(false, "JoltStaticWorldBuilder: the world was built without the description's static layer keys (JoltWorldConfig::staticLayers = staticLayerKeysOf(description))");
			groupBegin = groupEnd;
			continue;
		}

		std::vector<PreparedElement> prepared;
		prepared.reserve(groupEnd - groupBegin);
		for (size_t index = groupBegin; index < groupEnd; ++index)
		{
			const StaticShapeDescriptor& descriptor = *sorted[index].descriptor;
			PreparedElement element;
			element.stableKey = descriptor.stableKey;
			element.placement = placementOf(descriptor.localToWorld);
			if (!element.placement.valid)
			{
				++m_stats.skippedInvalidShape;
				warnElement("skipped an element", descriptor.stableKey, "localToWorld has a zero or non-finite axis");
				continue;
			}
			JPH::Shape::ShapeResult shape = std::visit(shapeBuilder, descriptor.shape);
			if (shape.IsValid() && !element.placement.rigid)
			{
				const JPH::Vec3 validScale = shape.Get()->MakeScaleValid(element.placement.scale);
				if (!validScale.IsClose(element.placement.scale, 1e-12f))
				{
					++m_stats.approximatedScales;
					warnElement("approximated a scale", descriptor.stableKey, "the shape cannot take this localToWorld scale exactly; Jolt's nearest valid scale is used");
				}
				shape = shape.Get()->ScaleShape(element.placement.scale);
			}
			if (!shape.IsValid())
			{
				++m_stats.skippedInvalidShape;
				warnElement("skipped an element", descriptor.stableKey, shape.HasError() ? shape.GetError().c_str() : "Jolt returned no shape");
				continue;
			}
			element.shape = shape.Get();
			++report.shapeCountByType[descriptor.shape.index()];
			++m_stats.shapesBuilt;
			prepared.push_back(std::move(element));
		}

		for (size_t chunkBegin = 0; chunkBegin < prepared.size(); chunkBegin += m_options.maxShapesPerChunk)
		{
			const size_t chunkCount = std::min<size_t>(m_options.maxShapesPerChunk, prepared.size() - chunkBegin);
			++m_stats.chunks;
			JPH::Shape::ShapeResult chunk = compoundOf(&prepared[chunkBegin], chunkCount);
			if (chunk.IsValid())
			{
				createBody(chunk.Get(), *layer, sorted[groupBegin].surface);
				continue;
			}
			++m_stats.chunksSplitIntoSingles;
			warnElement("split a chunk into one body per element", prepared[chunkBegin].stableKey,
				chunk.HasError() ? chunk.GetError().c_str() : "Jolt returned no compound");
			for (size_t index = chunkBegin; index < chunkBegin + chunkCount; ++index)
			{
				JPH::Shape::ShapeResult single = compoundOf(&prepared[index], 1);
				if (single.IsValid())
				{
					createBody(single.Get(), *layer, sorted[groupBegin].surface);
				}
				else
				{
					++m_stats.skippedInvalidShape;
					warnElement("skipped an element", prepared[index].stableKey, single.HasError() ? single.GetError().c_str() : "Jolt returned no shape");
				}
			}
		}
		groupBegin = groupEnd;
	}

	m_stats.bodies = created;
	if (!created.empty())
	{
		JPH::BodyInterface& bodyInterface = m_world.bodies();
		JPH::BodyInterface::AddState addState = bodyInterface.AddBodiesPrepare(created.data(), static_cast<int>(created.size()));
		bodyInterface.AddBodiesFinalize(created.data(), static_cast<int>(created.size()), addState, JPH::EActivation::DontActivate);
	}
	m_world.physics().OptimizeBroadPhase();

	report.buildSeconds = steadySeconds() - startSeconds;

	char summary[400];
	std::snprintf(summary, sizeof(summary),
		"JoltStaticWorldBuilder: built %u of %u shapes (box=%u sphere=%u capsule=%u convex=%u trimesh=%u) into %u static bodies (%u groups, %u chunks, %u split) in %.4f s; skipped invalid=%u noLayer=%u; zeroSurface=%u duplicateKeys=%u approximatedScale=%u",
		m_stats.shapesBuilt, static_cast<unsigned>(description.shapes.size()),
		report.countOf<StaticBox>(), report.countOf<StaticSphere>(), report.countOf<StaticCapsuleZ>(), report.countOf<StaticConvexHull>(),
		report.countOf<StaticTriangleMesh>(), static_cast<unsigned>(m_stats.bodies.size()), m_stats.groups, m_stats.chunks,
		m_stats.chunksSplitIntoSingles, report.buildSeconds, m_stats.skippedInvalidShape, m_stats.skippedNoLayer, m_stats.zeroSurfaceElements,
		m_stats.duplicateStableKeys, m_stats.approximatedScales);
	log(summary);
	return report;
}

void JoltStaticWorldBuilder::log(const char* message) const
{
	if (m_logger)
	{
		m_logger(message);
	}
}
