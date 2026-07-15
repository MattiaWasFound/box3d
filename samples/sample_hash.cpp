// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "sample.h"
#include "mesh_loader.h"
#include "gfx/keycodes.h"
#include "utils.h"

#include "box3d/box3d.h"
#include "box3d/box3d_test.h"
#include "box3d/math_functions.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static constexpr float RayLength = 250.0f;

struct Rng
{
	uint32_t s;

	explicit Rng( uint32_t seed )
		: s( seed == 0 ? 0x12345678u : seed )
	{
	}

	uint32_t Next()
	{
		uint32_t x = s;
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		s = x;
		return x;
	}

	float Unit()
	{
		return ( Next() >> 8 ) * ( 1.0f / 16777215.0f );
	}

	float Range( float lo, float hi )
	{
		return lo + ( hi - lo ) * Unit();
	}

	bool OneIn( int n )
	{
		return Next() % (uint32_t)n == 0;
	}

	float NonZeroAxis()
	{
		return ( Next() & 1u ) == 0 ? -1.0f : 1.0f;
	}
};

struct DriverState
{
	int nextInputTick = 0;
	float moveForward = 1.0f;
	float moveRight = 1.0f;
	bool run = false;
	bool grabbed = false;
	int drags = 0;
	b3Pos target = b3Vec3_zero;
};

struct Result
{
	int bodies = 0;
	int dynamicBodies = 0;
	int drags = 0;
	uint64_t hash = 0;
	bool finite = true;
	bool exercisedInput = true;
};

static int CompareSamples( const void* a, const void* b )
{
	const SampleEntry* entryA = (const SampleEntry*)a;
	const SampleEntry* entryB = (const SampleEntry*)b;

	int result = strcmp( entryA->Category, entryB->Category );
	return result == 0 ? strcmp( entryA->Name, entryB->Name ) : result;
}

static uint32_t HashSeed( uint32_t seed, const char* category, const char* name )
{
	uint32_t h = seed == 0 ? 2166136261u : seed;
	auto feed = [&h]( const char* s ) {
		for ( ; *s != 0; ++s )
		{
			h ^= (uint8_t)*s;
			h *= 16777619u;
		}
	};
	feed( category );
	h ^= (uint8_t)'/';
	h *= 16777619u;
	feed( name );
	return h;
}

static void FeedFloat( uint64_t* h, float f )
{
	uint32_t bits;
	memcpy( &bits, &f, sizeof( bits ) );
	for ( int i = 0; i < 4; ++i )
	{
		*h ^= ( bits >> ( 8 * i ) ) & 0xFFu;
		*h *= 1099511628211ull;
	}
}

static bool IsFiniteTransform( b3WorldTransform t )
{
	return isfinite( t.p.x ) && isfinite( t.p.y ) && isfinite( t.p.z ) &&
		   isfinite( t.q.v.x ) && isfinite( t.q.v.y ) && isfinite( t.q.v.z ) && isfinite( t.q.s );
}

// --hash-velocities widens the hash to linear and angular velocities, catching states that
// agree on pose at the sampled tick but would diverge later. Off by default so existing
// recorded hashes stay comparable. The managed checker implements the identical extension.
static bool s_hashVelocities = false;

static uint64_t StateHash( b3WorldId worldId, bool* finite )
{
	uint64_t h = 14695981039346656037ull;
	int capacity = b3World_GetBodyCapacity( worldId );
	for ( int i = 0; i < capacity; ++i )
	{
		b3BodyId bodyId = b3World_GetBodyByIndex( worldId, i );
		if ( B3_IS_NULL( bodyId ) || b3Body_IsValid( bodyId ) == false || b3Body_GetType( bodyId ) != b3_dynamicBody )
		{
			continue;
		}

		b3WorldTransform t = b3Body_GetTransform( bodyId );
		if ( IsFiniteTransform( t ) == false )
		{
			*finite = false;
		}
		FeedFloat( &h, t.p.x );
		FeedFloat( &h, t.p.y );
		FeedFloat( &h, t.p.z );
		FeedFloat( &h, t.q.v.x );
		FeedFloat( &h, t.q.v.y );
		FeedFloat( &h, t.q.v.z );
		FeedFloat( &h, t.q.s );

		if ( s_hashVelocities )
		{
			b3Vec3 v = b3Body_GetLinearVelocity( bodyId );
			b3Vec3 w = b3Body_GetAngularVelocity( bodyId );
			FeedFloat( &h, v.x );
			FeedFloat( &h, v.y );
			FeedFloat( &h, v.z );
			FeedFloat( &h, w.x );
			FeedFloat( &h, w.y );
			FeedFloat( &h, w.z );
		}
	}
	return h;
}

static int DynamicBodyCount( b3WorldId worldId )
{
	int count = 0;
	int capacity = b3World_GetBodyCapacity( worldId );
	for ( int i = 0; i < capacity; ++i )
	{
		b3BodyId bodyId = b3World_GetBodyByIndex( worldId, i );
		if ( B3_IS_NON_NULL( bodyId ) && b3Body_IsValid( bodyId ) && b3Body_GetType( bodyId ) == b3_dynamicBody )
		{
			count += 1;
		}
	}
	return count;
}

static bool TryGetDynamicBodyTransform( b3WorldId worldId, int dynamicIndex, b3BodyId* body, b3WorldTransform* transform )
{
	int seen = 0;
	int capacity = b3World_GetBodyCapacity( worldId );
	for ( int i = 0; i < capacity; ++i )
	{
		b3BodyId bodyId = b3World_GetBodyByIndex( worldId, i );
		if ( B3_IS_NULL( bodyId ) || b3Body_IsValid( bodyId ) == false || b3Body_GetType( bodyId ) != b3_dynamicBody )
		{
			continue;
		}

		if ( seen++ == dynamicIndex )
		{
			*body = bodyId;
			*transform = b3Body_GetTransform( bodyId );
			return true;
		}
	}
	return false;
}

static b3Vec3 Sub( b3Vec3 a, b3Vec3 b )
{
	return { a.x - b.x, a.y - b.y, a.z - b.z };
}

static void MakeRay( b3Pos origin, b3Pos target, b3Pos* rayOrigin, b3Vec3* rayDir )
{
	b3Vec3 d = Sub( target, origin );
	float len = b3Length( d );
	*rayOrigin = origin;
	*rayDir = len > 1.0e-6f ? ( 1.0f / len ) * d : b3Vec3{ 0.0f, -0.2f, -1.0f };
}

static void RandomRay( Rng& rng, b3Pos* rayOrigin, b3Vec3* rayDir )
{
	b3Pos origin = { rng.Range( -12.0f, 12.0f ), rng.Range( 4.0f, 30.0f ), rng.Range( 18.0f, 45.0f ) };
	b3Pos target = { rng.Range( -8.0f, 8.0f ), rng.Range( 0.0f, 10.0f ), rng.Range( -8.0f, 8.0f ) };
	MakeRay( origin, target, rayOrigin, rayDir );
}

static void RayThrough( b3Pos target, Rng& rng, b3Pos* rayOrigin, b3Vec3* rayDir )
{
	b3Pos origin = { target.x + rng.Range( -1.5f, 1.5f ), target.y + 6.0f + rng.Range( 0.0f, 4.0f ),
					 target.z + 18.0f + rng.Range( -2.0f, 2.0f ) };
	MakeRay( origin, target, rayOrigin, rayDir );
}

static void ReleaseGrab( Sample* sample )
{
	if ( b3Joint_IsValid( sample->m_mouseJointId ) )
	{
		b3DestroyJoint( sample->m_mouseJointId, true );
	}
	if ( b3Body_IsValid( sample->m_mouseBodyId ) )
	{
		b3DestroyBody( sample->m_mouseBodyId );
	}
	sample->m_mouseJointId = b3_nullJointId;
	sample->m_mouseBodyId = b3_nullBodyId;
	sample->m_mouseFraction = 0.0f;
}

static bool GrabRay( Sample* sample, b3Pos rayOrigin, b3Vec3 rayDir, float rayLength )
{
	ReleaseGrab( sample );

	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.name = "sample_hash_grab";
	b3RayResult result = b3World_CastRayClosest( sample->m_worldId, rayOrigin, rayLength * rayDir, filter );
	b3BodyId bodyId = result.hit ? b3Shape_GetBody( result.shapeId ) : b3_nullBodyId;
	if ( result.hit == false || b3Body_GetType( bodyId ) != b3_dynamicBody )
	{
		return false;
	}

	sample->m_mousePoint = result.point;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_kinematicBody;
	bodyDef.position = sample->m_mousePoint;
	bodyDef.enableSleep = false;
	sample->m_mouseBodyId = b3CreateBody( sample->m_worldId, &bodyDef );

	b3MotorJointDef jointDef = b3DefaultMotorJointDef();
	jointDef.base.bodyIdA = sample->m_mouseBodyId;
	jointDef.base.bodyIdB = bodyId;
	jointDef.base.localFrameB.p = b3Body_GetLocalPoint( bodyId, result.point );
	jointDef.linearHertz = 7.5f;
	jointDef.linearDampingRatio = 1.0f;
	jointDef.maxSpringForce = 100.0f * b3Body_GetMassData( bodyId ).mass * b3Length( b3World_GetGravity( sample->m_worldId ) );
	sample->m_mouseJointId = b3CreateMotorJoint( sample->m_worldId, &jointDef );
	b3Body_SetAwake( bodyId, true );
	sample->m_mouseFraction = result.fraction;
	return true;
}

static bool GrabBody( Sample* sample, b3BodyId bodyId, b3Pos point )
{
	ReleaseGrab( sample );

	if ( b3Body_IsValid( bodyId ) == false || b3Body_GetType( bodyId ) != b3_dynamicBody )
	{
		return false;
	}

	sample->m_mousePoint = point;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_kinematicBody;
	bodyDef.position = sample->m_mousePoint;
	bodyDef.enableSleep = false;
	sample->m_mouseBodyId = b3CreateBody( sample->m_worldId, &bodyDef );

	b3MotorJointDef jointDef = b3DefaultMotorJointDef();
	jointDef.base.bodyIdA = sample->m_mouseBodyId;
	jointDef.base.bodyIdB = bodyId;
	jointDef.base.localFrameB.p = b3Body_GetLocalPoint( bodyId, point );
	jointDef.linearHertz = 7.5f;
	jointDef.linearDampingRatio = 1.0f;
	jointDef.maxSpringForce = 100.0f * b3Body_GetMassData( bodyId ).mass * b3Length( b3World_GetGravity( sample->m_worldId ) );
	sample->m_mouseJointId = b3CreateMotorJoint( sample->m_worldId, &jointDef );
	b3Body_SetAwake( bodyId, true );
	sample->m_mouseFraction = 0.0f;
	return true;
}

static void MoveGrab( Sample* sample, b3Pos rayOrigin, b3Vec3 rayDir, float rayLength )
{
	if ( B3_IS_NULL( sample->m_mouseJointId ) )
	{
		return;
	}

	sample->m_mousePoint = rayOrigin + sample->m_mouseFraction * rayLength * rayDir;
}

static void Shoot( Sample* sample, b3Pos rayOrigin, b3Vec3 rayDir )
{
	b3Vec3 direction = b3Normalize( rayDir );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = rayOrigin + 2.0f * direction;
	bodyDef.linearVelocity = 20.0f * direction;
	bodyDef.isBullet = true;
	b3BodyId bodyId = b3CreateBody( sample->m_worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { b3Vec3_zero, 0.25f };
	shapeDef.density *= 4.0f;
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );
}

static void SetInputs( DriverState& driver, Rng& rng, int tick )
{
	if ( tick >= driver.nextInputTick )
	{
		driver.moveForward = rng.NonZeroAxis();
		driver.moveRight = rng.NonZeroAxis();
		driver.run = rng.OneIn( 2 );
		driver.nextInputTick = tick + 12 + (int)( rng.Next() % 24u );
	}

	SetKeyDown( KEY_W, driver.moveForward > 0.0f );
	SetKeyDown( KEY_S, driver.moveForward < 0.0f );
	SetKeyDown( KEY_D, driver.moveRight > 0.0f );
	SetKeyDown( KEY_A, driver.moveRight < 0.0f );
	SetKeyDown( KEY_SPACE, tick % 23 == 0 || rng.OneIn( 53 ) );
	SetKeyDown( KEY_B, IsKeyDown( KEY_SPACE ) );
	SetKeyDown( KEY_L, IsKeyDown( KEY_SPACE ) );
	SetKeyDown( KEY_LEFT_SHIFT, driver.run );
}

static void DriveInput( Sample* sample, Rng& rng, DriverState& driver, int tick )
{
	SetInputs( driver, rng, tick );

	if ( tick % 23 == 0 )
	{
		sample->Keyboard( KEY_SPACE, ACTION_PRESS, 0 );
		sample->Keyboard( KEY_L, ACTION_PRESS, 0 );
		sample->Keyboard( KEY_B, ACTION_PRESS, 0 );
	}

	if ( tick % 41 == 0 )
	{
		ReleaseGrab( sample );
		driver.grabbed = false;

		int dynamicCount = DynamicBodyCount( sample->m_worldId );
		b3BodyId bodyId = b3_nullBodyId;
		b3WorldTransform transform = {};
		if ( dynamicCount > 0 && TryGetDynamicBodyTransform( sample->m_worldId, (int)( rng.Next() % (uint32_t)dynamicCount ), &bodyId, &transform ) )
		{
			driver.target = transform.p;
			driver.grabbed = GrabBody( sample, bodyId, driver.target );
			if ( driver.grabbed )
			{
				driver.drags += 1;
			}
		}
	}
	else if ( tick % 41 < 14 && driver.grabbed )
	{
		driver.target = { driver.target.x + rng.Range( -0.35f, 0.35f ), driver.target.y + rng.Range( -0.10f, 0.40f ),
						  driver.target.z + rng.Range( -0.35f, 0.35f ) };
		b3Pos rayOrigin;
		b3Vec3 rayDir;
		RayThrough( driver.target, rng, &rayOrigin, &rayDir );
		MoveGrab( sample, rayOrigin, rayDir, RayLength );
	}
	else if ( tick % 41 == 14 && driver.grabbed )
	{
		ReleaseGrab( sample );
		driver.grabbed = false;
	}
}

static Result RunSample( const SampleEntry& entry, int ticks, uint32_t seed, bool randomInput, bool trace = false, bool dumpState = false )
{
	SampleContext context = {};
	context.workerCount = 1;
	context.hertz = 60.0f;
	context.subStepCount = 4;
	context.enableSleep = true;
	context.enableWarmStarting = true;
	context.enableContinuous = true;
	context.headless = true;
	context.hasInputBasis = true;
	context.showUI = false;
	context.windowWidth = 1920;
	context.windowHeight = 1080;

	// Sample construction always runs from RAND_SEED: the Sample base constructor resets
	// g_randomSeed before any derived constructor body, and the managed port mirrors that.
	// Only the grab driver below varies with --seed.
	Sample* sample = entry.CreateFcn( &context );
	context.sample = sample;

	Rng rng( HashSeed( seed, entry.Category, entry.Name ) );
	DriverState driver;

	for ( int i = 0; i < ticks; ++i )
	{
		if ( randomInput )
		{
			DriveInput( sample, rng, driver, i );
		}
		if ( trace )
		{
			printf( "DRIVER\t%d\t%.0f\t%.0f\t%d\t%d\n", i + 1, driver.moveForward, driver.moveRight, driver.run ? 1 : 0,
					IsKeyDown( KEY_SPACE ) ? 1 : 0 );
		}
		sample->Step();
		if ( trace )
		{
			bool finite = true;
			printf( "TRACE\t%d\t%016" PRIx64 "\n", i + 1, StateHash( sample->m_worldId, &finite ) );
		}
	}

	Result result = {};
	if ( dumpState )
	{
		int dynamicIndex = 0;
		int capacity = b3World_GetBodyCapacity( sample->m_worldId );
		for ( int i = 0; i < capacity; ++i )
		{
			b3BodyId id = b3World_GetBodyByIndex( sample->m_worldId, i );
			if ( B3_IS_NULL( id ) || b3Body_GetType( id ) != b3_dynamicBody ) continue;
			b3WorldTransform t = b3Body_GetTransform( id );
			uint32_t bits[7];
			float values[7] = { t.p.x, t.p.y, t.p.z, t.q.v.x, t.q.v.y, t.q.v.z, t.q.s };
			memcpy( bits, values, sizeof( bits ) );
			printf( "STATE\t%d\t%08x\t%08x\t%08x\t%08x\t%08x\t%08x\t%08x\n", dynamicIndex++, bits[0], bits[1], bits[2],
					bits[3], bits[4], bits[5], bits[6] );
		}
	}
	result.bodies = b3World_GetCounters( sample->m_worldId ).bodyCount;
	result.dynamicBodies = DynamicBodyCount( sample->m_worldId );
	result.drags = driver.drags;
	result.finite = true;
	result.hash = StateHash( sample->m_worldId, &result.finite );
	result.exercisedInput = randomInput == false || result.dynamicBodies == 0 || result.drags > 0;
	ReleaseGrab( sample );
	delete sample;
	return result;
}

static int ArgInt( int argc, char** argv, const char* name, int fallback )
{
	for ( int i = 1; i + 1 < argc; ++i )
	{
		if ( strcmp( argv[i], name ) == 0 )
		{
			return atoi( argv[i + 1] );
		}
	}
	return fallback;
}

static bool HasArg( int argc, char** argv, const char* name )
{
	for ( int i = 1; i < argc; ++i )
	{
		if ( strcmp( argv[i], name ) == 0 )
		{
			return true;
		}
	}
	return false;
}

static const char* ArgString( int argc, char** argv, const char* name )
{
	for ( int i = 1; i + 1 < argc; ++i )
	{
		if ( strcmp( argv[i], name ) == 0 )
		{
			return argv[i + 1];
		}
	}
	return nullptr;
}

static void NullLog( const char* )
{
}

int main( int argc, char** argv )
{
	const char* objPath = ArgString( argc, argv, "--obj" );
	if ( objPath != nullptr )
	{
		TempMesh mesh;
		LoadTempMesh( objPath, &mesh, 1.0f, false );
		uint64_t h = 14695981039346656037ULL;
		auto feed = [&h]( const void* data, size_t size ) {
			const uint8_t* bytes = static_cast<const uint8_t*>( data );
			for ( size_t i = 0; i < size; ++i ) { h ^= bytes[i]; h *= 1099511628211ULL; }
		};
		feed( mesh.vertices.data(), mesh.vertices.size() * sizeof( b3Vec3 ) );
		feed( mesh.indices.data(), mesh.indices.size() * sizeof( int ) );
		feed( mesh.materialIndices.data(), mesh.materialIndices.size() );
		printf( "OBJ\t%zu\t%zu\t%016" PRIx64 "\n", mesh.vertices.size(), mesh.indices.size(), h );
		return 0;
	}
	int ticks = ArgInt( argc, argv, "--ticks", 120 );
	uint32_t seed = (uint32_t)ArgInt( argc, argv, "--seed", 0x5eed );
	bool randomInput = HasArg( argc, argv, "--no-random" ) == false;
	s_hashVelocities = HasArg( argc, argv, "--hash-velocities" );
	bool tsv = HasArg( argc, argv, "--tsv" );
	bool trace = HasArg( argc, argv, "--trace" );
	bool dumpState = HasArg( argc, argv, "--dump-state" );
	const char* category = ArgString( argc, argv, "--category" );
	const char* sampleName = ArgString( argc, argv, "--sample" );

	b3SetLogFcn( NullLog );

	qsort( g_sampleEntries, g_sampleCount, sizeof( SampleEntry ), CompareSamples );

	int failures = 0;
	int matched = 0;
	if ( tsv == false )
	{
		printf( "%-28s %7s %5s %5s  %16s  status\n", "category/name", "bodies", "dyn", "drags", "hash" );
	}
	for ( int i = 0; i < g_sampleCount; ++i )
	{
		SampleEntry& entry = g_sampleEntries[i];
		if ( category != nullptr && strcmp( entry.Category, category ) != 0 )
		{
			continue;
		}
		if ( sampleName != nullptr && strcmp( entry.Name, sampleName ) != 0 )
		{
			continue;
		}
		matched += 1;
		Result a = RunSample( entry, ticks, seed, randomInput, trace, dumpState );
		Result b = RunSample( entry, ticks, seed, randomInput );
		bool ok = a.finite && b.finite && a.hash == b.hash && a.exercisedInput;
		if ( ok == false )
		{
			failures += 1;
		}

		const char* status = a.finite == false || b.finite == false ? "NON-FINITE" :
							 a.hash != b.hash						 ? "DIVERGED" :
							 a.exercisedInput == false				 ? "NO-INPUT" :
																	   "ok";
		if ( tsv )
		{
			printf( "%s\t%s\t%d\t%d\t%d\t%016" PRIx64 "\t%s\n", entry.Category, entry.Name, a.bodies, a.dynamicBodies,
					a.drags, a.hash, status );
		}
		else
		{
			printf( "%s/%-24s %7d %5d %5d  %016" PRIx64 "  %s\n", entry.Category, entry.Name, a.bodies, a.dynamicBodies,
					a.drags, a.hash, status );
		}
	}

	// A filter that matches nothing must fail loudly: a renamed category or a typo'd shard
	// name would otherwise report PASS while running zero samples.
	if ( matched == 0 )
	{
		fprintf( stderr, "no samples matched --category/--sample filter\n" );
		return 1;
	}

	if ( tsv == false )
	{
		printf( failures == 0 ? "\nPASS (native samples)\n" : "\nFAIL (%d native sample(s))\n", failures );
	}
	return failures == 0 ? 0 : 1;
}
