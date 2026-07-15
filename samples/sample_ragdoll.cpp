// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "human.h"
#include "gfx/debug_adapter.h"
#include "imgui.h"
#include "sample.h"
#include "gfx/keycodes.h"
#include "gfx/draw.h"

#include "box3d/box3d.h"

#include <string.h>

class RagdollOnBox : public Sample
{
public:
	explicit RagdollOnBox( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 6.0f, b3Pos_zero );
		}

		AddGroundBox( 20.0f );

		m_jointFrictionTorque = 5.0f;
		m_jointHertz = 1.0f;
		m_jointDampingRatio = 0.7f;

		m_human = {};

		Spawn();
	}

	void Spawn()
	{
		CreateHuman( &m_human, m_worldId, { 0.0f, 2.0f, 0.0f }, m_jointFrictionTorque, m_jointHertz, m_jointDampingRatio, 1,
					 nullptr, false );
		// Human_ApplyRandomAngularImpulse( &m_human, 10.0f );
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		if ( ImGui::SliderFloat( "Joint Friction", &m_jointFrictionTorque, 0.0f, 20.0f, "%3.0f" ) )
		{
			Human_SetJointFrictionTorque( &m_human, m_jointFrictionTorque );
		}

		if ( ImGui::SliderFloat( "Hertz", &m_jointHertz, 0.0f, 20.0f, "%3.1f" ) )
		{
			Human_SetJointSpringHertz( &m_human, m_jointHertz );
		}

		if ( ImGui::SliderFloat( "Damping", &m_jointDampingRatio, 0.0f, 4.0f, "%3.1f" ) )
		{
			Human_SetJointDampingRatio( &m_human, m_jointDampingRatio );
		}

		if ( ImGui::Button( "Respawn" ) )
		{
			DestroyHuman( &m_human );
			Spawn();
		}
		ImGui::PopItemWidth();
		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new RagdollOnBox( context );
	}

	Human m_human;
	float m_jointFrictionTorque;
	float m_jointHertz;
	float m_jointDampingRatio;
};

static int sampleRagdollOnBox = RegisterSample( "Ragdoll", "Box", RagdollOnBox::Create );

// A playable rewind prototype built on the recording player's exact world snapshots. The replay
// world becomes the new live world when playback resumes, so contacts, warm-start impulses, islands,
// broad-phase ordering, and id pools all continue from the selected frame.
class RagdollRewind : public Sample
{
public:
	explicit RagdollRewind( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 25.0f, 7.0f, { 0.0f, 1.0f, 0.0f } );
		}

		AddGroundBox( 20.0f );
		m_human = {};
		CreateHuman( &m_human, m_worldId, { 0.0f, 2.0f, 0.0f }, 5.0f, 1.0f, 0.7f, 1, nullptr, true );
		BeginTimeline();
	}

	~RagdollRewind() override
	{
		DiscardRecording();
		if ( m_previousRecording != nullptr )
		{
			b3DestroyRecording( m_previousRecording );
			m_previousRecording = nullptr;
		}
		if ( m_previousPlayer != nullptr )
		{
			b3RecPlayer_Destroy( m_previousPlayer );
			m_previousPlayer = nullptr;
		}
		if ( m_worldOwner != nullptr )
		{
			// The player owns the current world. Prevent Sample::~Sample from destroying it twice.
			m_worldId = b3_nullWorldId;
			b3RecPlayer_Destroy( m_worldOwner );
			m_worldOwner = nullptr;
		}
	}

	void Step() override
	{
		if ( m_scrubbing )
		{
			m_didStep = false;
			return;
		}

		Sample::Step();
		if ( m_didStep )
		{
			m_timelineFrames += 1;
			// Two half-window recordings give a bounded rolling window: between one half and one
			// full requested duration is always available, without editing the append-only tape.
			int frameLimit = b3MaxInt( 1, (int)( 0.5f * m_historySeconds * m_context->hertz ) );
			int halfTapeBudget = b3MaxInt( 256 * 1024, m_tapeBudgetMB * 1024 * 1024 / 2 );
			bool tapeFull = b3Recording_GetSize( m_recording ) >= halfTapeBudget;
			if ( m_timelineFrames >= frameLimit || tapeFull )
			{
				RotateTimeline();
				m_rotations += 1;
			}
		}
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		(void)modifiers;
		if ( key == KEY_SPACE && action == ACTION_PRESS )
		{
			if ( m_scrubbing )
			{
				ResumeFromHere();
			}
			else
			{
				PauseAndBuildHistory();
			}
		}
	}

	bool DrawControls() override
	{
		ImGui::TextWrapped( "Ctrl-drag a body. Space pauses; scrub anywhere; Space branches and plays from there." );
		ImGui::PushItemWidth( 8.0f * ImGui::GetFontSize() );

		if ( m_scrubbing )
		{
			int frame = m_scrubFrame;
			if ( ImGui::SliderInt( "Timeline", &frame, 0, m_frameCount ) )
			{
				uint64_t ticks = b3GetTicks();
				SeekTimeline( frame );
				m_lastSeekMs = b3GetMilliseconds( ticks );
			}
			double seconds = m_context->hertz > 0.0f ? frame / m_context->hertz : 0.0;
			ImGui::Text( "paused at %.2f s  (%d / %d)", seconds, frame, m_frameCount );
			size_t keyframeBytes = b3RecPlayer_GetKeyframeBytes( m_worldOwner );
			size_t logicalBytes = b3RecPlayer_GetKeyframeLogicalBytes( m_worldOwner );
			if ( m_previousPlayer != nullptr )
			{
				keyframeBytes += b3RecPlayer_GetKeyframeBytes( m_previousPlayer );
				logicalBytes += b3RecPlayer_GetKeyframeLogicalBytes( m_previousPlayer );
			}
			int skipped = b3RecPlayer_GetSkippedKeyframeCount( m_worldOwner );
			if ( m_previousPlayer != nullptr )
			{
				skipped += b3RecPlayer_GetSkippedKeyframeCount( m_previousPlayer );
			}
			ImGui::Text( "recordings %.2f MB", m_recordingBytes / ( 1024.0 * 1024.0 ) );
			float savedPercent = logicalBytes > 0 ? b3MaxFloat( 0.0f, 100.0f * ( 1.0f - (float)keyframeBytes / (float)logicalBytes ) ) : 0.0f;
			ImGui::Text( "keyframes %.2f / %d MB, %.0f%% COW saved", keyframeBytes / ( 1024.0 * 1024.0 ), m_keyframeBudgetMB,
						 savedPercent );
			ImGui::Text( "%d idle keyframes skipped", skipped );
			ImGui::Text( "pause build %.2f ms, last seek %.2f ms", m_buildHistoryMs, m_lastSeekMs );
		}
		else
		{
			ImGui::SliderFloat( "History seconds", &m_historySeconds, 2.0f, 60.0f, "%.0f s" );
			ImGui::SliderInt( "Tape budget", &m_tapeBudgetMB, 1, 256, "%d MB" );
			ImGui::SliderInt( "Keyframe budget", &m_keyframeBudgetMB, 8, 512, "%d MB" );
			ImGui::SliderInt( "Keyframe interval", &m_keyframeInterval, 4, 64, "%d frames" );
			int bytes = m_recording != nullptr ? b3Recording_GetSize( m_recording ) : 0;
			int previousBytes = m_previousRecording != nullptr ? b3Recording_GetSize( m_previousRecording ) : 0;
			int retainedFrames = m_previousFrames + m_timelineFrames;
			ImGui::Text( "capturing: %.2f MB, %.1f s retained", ( bytes + previousBytes ) / ( 1024.0 * 1024.0 ),
						m_context->hertz > 0.0f ? retainedFrames / m_context->hertz : 0.0f );
			if ( m_rotations > 0 )
			{
				ImGui::TextDisabled( "history window rotated %d time%s", m_rotations, m_rotations == 1 ? "" : "s" );
			}
		}

		ImGui::PopItemWidth();
		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new RagdollRewind( context );
	}

private:
	void BeginTimeline()
	{
		DiscardRecording();
		if ( m_previousRecording != nullptr )
		{
			b3DestroyRecording( m_previousRecording );
			m_previousRecording = nullptr;
		}
		m_previousFrames = 0;
		m_recording = b3CreateRecording( 256 * 1024 );
		b3World_StartRecording( m_worldId, m_recording );
		m_timelineFrames = 0;
	}

	void RotateTimeline()
	{
		b3World_StopRecording( m_worldId );
		if ( m_previousRecording != nullptr )
		{
			b3DestroyRecording( m_previousRecording );
		}
		m_previousRecording = m_recording;
		m_previousFrames = m_timelineFrames;
		m_recording = b3CreateRecording( 256 * 1024 );
		b3World_StartRecording( m_worldId, m_recording );
		m_timelineFrames = 0;
	}

	void DiscardRecording()
	{
		if ( m_recording == nullptr )
		{
			return;
		}
		b3World_StopRecording( m_worldId );
		b3DestroyRecording( m_recording );
		m_recording = nullptr;
	}

	void PauseAndBuildHistory()
	{
		uint64_t buildTicks = b3GetTicks();
		// Do not carry the host-side grab helper into scrub mode.
		MouseUp( {}, 0 );
		b3World_StopRecording( m_worldId );
		int currentBytes = b3Recording_GetSize( m_recording );
		int previousBytes = m_previousRecording != nullptr ? b3Recording_GetSize( m_previousRecording ) : 0;
		m_recordingBytes = currentBytes + previousBytes;

		int playerCount = m_previousRecording != nullptr ? 2 : 1;
		size_t playerBudget = (size_t)m_keyframeBudgetMB * 1024u * 1024u / (size_t)playerCount;
		b3RecPlayer* nextOwner = CreatePlayer( m_recording, playerBudget );
		if ( nextOwner == nullptr )
		{
			BeginTimeline();
			return;
		}

		b3RecPlayer* previousPlayer = m_previousRecording != nullptr ? CreatePlayer( m_previousRecording, playerBudget ) : nullptr;
		if ( m_previousRecording != nullptr && previousPlayer == nullptr )
		{
			b3RecPlayer_Destroy( nextOwner );
			BeginTimeline();
			return;
		}

		m_previousFrames = previousPlayer != nullptr ? b3RecPlayer_GetFrameCount( previousPlayer ) : 0;
		int currentFrames = b3RecPlayer_GetFrameCount( nextOwner );
		m_frameCount = m_previousFrames + currentFrames;
		b3RecPlayer_SeekFrame( nextOwner, currentFrames );
		if ( previousPlayer != nullptr )
		{
			b3RecPlayer_SeekFrame( previousPlayer, m_previousFrames );
		}

		b3WorldId previousWorld = m_worldId;
		b3RecPlayer* previousOwner = m_worldOwner;
		m_worldOwner = nextOwner;
		m_previousPlayer = previousPlayer;
		m_worldId = b3RecPlayer_GetWorldId( nextOwner );

		// The new replay world is now authoritative, so the old timeline can be released.
		if ( previousOwner != nullptr )
		{
			b3RecPlayer_Destroy( previousOwner );
		}
		else
		{
			b3DestroyWorld( previousWorld );
		}

		b3DestroyRecording( m_recording );
		m_recording = nullptr;
		if ( m_previousRecording != nullptr )
		{
			b3DestroyRecording( m_previousRecording );
			m_previousRecording = nullptr;
		}
		m_scrubFrame = m_frameCount;
		m_scrubbing = true;
		m_buildHistoryMs = b3GetMilliseconds( buildTicks );
	}

	b3RecPlayer* CreatePlayer( b3Recording* recording, size_t budget )
	{
		b3RecPlayer* player = b3RecPlayer_Create( b3Recording_GetData( recording ), b3Recording_GetSize( recording ),
											  m_context->workerCount );
		if ( player == nullptr )
		{
			return nullptr;
		}
		b3RecPlayer_SetKeyframePolicy( player, budget, m_keyframeInterval );
		b3WorldDef defTemplate = b3DefaultWorldDef();
		AttachToWorldDef( &defTemplate );
		b3RecPlayer_SetDebugShapeCallbacks( player, defTemplate.createDebugShape, defTemplate.destroyDebugShape,
										 defTemplate.userDebugShapeContext );
		return player;
	}

	void SeekTimeline( int frame )
	{
		m_scrubFrame = b3ClampInt( frame, 0, m_frameCount );
		if ( m_previousPlayer != nullptr && m_scrubFrame <= m_previousFrames )
		{
			b3RecPlayer_SeekFrame( m_previousPlayer, m_scrubFrame );
			m_worldId = b3RecPlayer_GetWorldId( m_previousPlayer );
		}
		else
		{
			int localFrame = m_scrubFrame - m_previousFrames;
			b3RecPlayer_SeekFrame( m_worldOwner, localFrame );
			m_worldId = b3RecPlayer_GetWorldId( m_worldOwner );
		}
	}

	void ResumeFromHere()
	{
		ClearSelection();
		if ( m_previousPlayer != nullptr && m_scrubFrame <= m_previousFrames )
		{
			b3RecPlayer_Destroy( m_worldOwner );
			m_worldOwner = m_previousPlayer;
			m_previousPlayer = nullptr;
			m_worldId = b3RecPlayer_GetWorldId( m_worldOwner );
		}
		else if ( m_previousPlayer != nullptr )
		{
			b3RecPlayer_Destroy( m_previousPlayer );
			m_previousPlayer = nullptr;
		}
		b3RecPlayer_TrimHistory( m_worldOwner );
		DestroyResurrectedGrabRig();
		m_scrubbing = false;
		BeginTimeline();
	}

	void DestroyResurrectedGrabRig()
	{
		// Branching from a frame where a Ctrl-drag was active resurrects the recorded
		// kinematic mouse body and its motor joint with the throw velocity still baked in.
		// Nothing owns that rig in the branched timeline (the live grab helper's ids died
		// with the pre-pause world), so the kinematic body would fly forever and haul the
		// grabbed body with it. Destroy it exactly as MouseUp would have; destroying the
		// body also destroys the attached joint.
		int bodyCount = b3RecPlayer_GetBodyCount( m_worldOwner );
		for ( int i = 0; i < bodyCount; ++i )
		{
			b3BodyId bodyId = b3RecPlayer_GetBodyId( m_worldOwner, i );
			if ( b3Body_IsValid( bodyId ) && strcmp( b3Body_GetName( bodyId ), "mouse" ) == 0 )
			{
				b3DestroyBody( bodyId );
			}
		}
	}

	Human m_human = {};
	b3RecPlayer* m_worldOwner = nullptr;
	b3RecPlayer* m_previousPlayer = nullptr;
	b3Recording* m_previousRecording = nullptr;
	int m_timelineFrames = 0;
	int m_previousFrames = 0;
	int m_frameCount = 0;
	int m_scrubFrame = 0;
	int m_recordingBytes = 0;
	int m_tapeBudgetMB = 16;
	int m_keyframeBudgetMB = 64;
	int m_keyframeInterval = 8;
	int m_rotations = 0;
	float m_buildHistoryMs = 0.0f;
	float m_lastSeekMs = 0.0f;
	float m_historySeconds = 15.0f;
	bool m_scrubbing = false;
};

static int sampleRagdollRewind = RegisterSample( "Ragdoll", "Rewind", RagdollRewind::Create );

class RagdollOnMesh : public Sample
{
public:
	explicit RagdollOnMesh( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 6.0f, b3Pos_zero );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			m_groundId = b3CreateBody( m_worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			m_groundMesh = b3CreateGridMesh( 20, 20, 2.0f, 2, true );
			b3CreateMeshShape( m_groundId, &shapeDef, m_groundMesh, b3Vec3_one );
		}

		{
			b3Transform transform;
			transform.p = { 0.0f, 5.0f, -20.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 20.0f, 5.0f, 0.1f, transform );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( m_groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { 0.0f, 5.0f, 20.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 20.0f, 5.0f, 0.1f, transform );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( m_groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { -20.0f, 5.0f, 0.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, 5.0f, 20.0f, transform );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( m_groundId, &shapeDef, &wallBox.base );
		}

		{
			b3Transform transform;
			transform.p = { 20.0f, 5.0f, 0.0f };
			transform.q = b3Quat_identity;
			b3BoxHull wallBox = b3MakeTransformedBoxHull( 0.1f, 5.0f, 20.0f, transform );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( m_groundId, &shapeDef, &wallBox.base );
		}

		m_jointFrictionTorque = 5.0f;
		m_jointHertz = 2.0f;
		m_jointDampingRatio = 0.7f;

		m_human = {};

		Spawn();
	}

	~RagdollOnMesh() override
	{
		b3DestroyMesh( m_groundMesh );
	}

	void Spawn()
	{
		CreateHuman( &m_human, m_worldId, { 0.0f, 1.0f, 0.0f }, m_jointFrictionTorque, m_jointHertz, m_jointDampingRatio, 1,
					 nullptr, false );
		// Human_AlignSpring( &m_human, m_worldId, m_groundId, 25.0f, 1.0f );
		//  Human_ApplyRandomAngularImpulse( &m_human, 10.0f );
		// b3Body_SetType( m_human.bones[bone_thigh_l].bodyId, b3_kinematicBody );
		// b3Body_SetType( m_human.bones[bone_thigh_r].bodyId, b3_kinematicBody );
		// b3Body_SetType( m_human.bones[bone_pelvis].bodyId, b3_kinematicBody );
		// Human_CreateMotorAnchors( &m_human, m_worldId );
		Human_CreateParallelAnchors( &m_human, m_worldId );
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		if ( ImGui::SliderFloat( "Joint Friction", &m_jointFrictionTorque, 0.0f, 20.0f, "%3.0f" ) )
		{
			Human_SetJointFrictionTorque( &m_human, m_jointFrictionTorque );
		}

		if ( ImGui::SliderFloat( "Hertz", &m_jointHertz, 0.0f, 20.0f, "%3.1f" ) )
		{
			Human_SetJointSpringHertz( &m_human, m_jointHertz );
		}

		if ( ImGui::SliderFloat( "Damping", &m_jointDampingRatio, 0.0f, 4.0f, "%3.1f" ) )
		{
			Human_SetJointDampingRatio( &m_human, m_jointDampingRatio );
		}

		if ( ImGui::Button( "Respawn" ) )
		{
			DestroyHuman( &m_human );
			Spawn();
		}
		ImGui::PopItemWidth();
		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new RagdollOnMesh( context );
	}

	b3MeshData* m_groundMesh;
	b3BodyId m_groundId;
	Human m_human;
	float m_jointFrictionTorque;
	float m_jointHertz;
	float m_jointDampingRatio;
};

static int sampleRagdollMesh = RegisterSample( "Ragdoll", "Mesh", RagdollOnMesh::Create );

class RagdollPile : public Sample
{
public:
	enum
	{
#ifdef NDEBUG
		e_count = 20
#else
		e_count = 8
#endif
	};

	explicit RagdollPile( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 180.0f, 30.0f, 20.0f, b3Pos_zero );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		m_groundMesh = b3CreateGridMesh( 20, 20, 1.0f, 1, true );
		b3CreateMeshShape( groundId, &shapeDef, m_groundMesh, b3Vec3_one );

		for ( int i = 0; i < e_count; ++i )
		{
			b3Pos position = { 0.1f * i, 2.0f + 0.5f * i, -0.1f * i };
			float torque = 10.0f;
			float hertz = 0.5f;
			float damping = 0.7f;
			int groupIndex = i;
			void* userData = nullptr;
			bool colorize = false;
			CreateHuman( m_humans + i, m_worldId, position, torque, hertz, damping, groupIndex, userData, colorize );
		}
	}

	~RagdollPile() override
	{
		b3DestroyMesh( m_groundMesh );
	}

	static Sample* Create( SampleContext* context )
	{
		return new RagdollPile( context );
	}

	b3MeshData* m_groundMesh;
	Human m_humans[e_count] = {};
};

static int sampleRagdollPile = RegisterSample( "Ragdoll", "Pile", RagdollPile::Create );

class RagdollIncline : public Sample
{
public:
	explicit RagdollIncline( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( -20.0f, 30.0f, 25.0f, b3Pos_zero );
		}

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		m_groundMesh = b3CreateGridMesh( 4, 4, 2.0f, 1, true );

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { -10.0f, 2.0f, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, -0.2f * B3_PI );
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateMeshShape( groundId, &shapeDef, m_groundMesh, b3Vec3_one );
		}

		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position = { 0.0f, 0.0f, 0.0f };
			b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );
			b3Vec3 scale = { 4.0f, 4.0f, 4.0f };
			b3CreateMeshShape( groundId, &shapeDef, m_groundMesh, scale );
		}

		m_human = {};
		b3Pos position = { -12.0f, 6.0f, 0.0f };
		float torque = 10.0f;
		float hertz = 2.0f;
		float damping = 0.7f;
		int groupIndex = 1;
		void* userData = nullptr;
		bool colorize = false;
		CreateHuman( &m_human, m_worldId, position, torque, hertz, damping, groupIndex, userData, colorize );
		m_time = 0.0f;
		m_motorized = true;
	}

	~RagdollIncline() override
	{
		b3DestroyMesh( m_groundMesh );
	}

	void Step() override
	{
		if ( m_time > 2.0f && m_motorized == true )
		{
			Human_SetJointFrictionTorque( &m_human, 0.5f );
			Human_SetJointSpringHertz( &m_human, 0.5f );
			m_motorized = false;
		}

		m_time += m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new RagdollIncline( context );
	}

	b3MeshData* m_groundMesh;
	Human m_human;
	float m_time;
	bool m_motorized;
};

static int sampleRagdollIncline = RegisterSample( "Ragdoll", "Incline", RagdollIncline::Create );

#if 0
class RagdollPose : public Sample
{
public:
	static Sample* Create( SampleContext* context )
	{
		return new RagdollPose( context );
	}

	explicit RagdollPose( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_camera->SetView( 45.0f, 30.0f, 6.0f, b3Vec3_zero );
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { 0.0f, 0.0f, 0.0f };
		// bodyDef.rotation = b3Rotation(B3_VEC3_AXIS_Y, 0.25f * B3_PI);
		b3BodyId groundId = m_worldId->CreateBody( &bodyDef );
		m_ground = b3CreateGrid( 4, 2.0f, 0.0f );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		groundId->AddMesh( &shapeDef, m_ground );

		b3HullData* Hull = b3CreateOffsetBox( { { -3.0f, 0.5f, 0.0f }, b3Quat_identity }, { 0.25f, 0.5f, 3.0f } );
		groundId->AddHull( &shapeDef, Hull );
		b3DestroyHull( Hull );

		m_motorized = false;
		m_motorHertz = 1.0f;
		m_human.Spawn( m_worldId, { 0.0f, 0.1f, 0.0f }, m_motorHertz, m_motorized );

		m_poseControl = true;
		m_poseHertz = 2.0f;
		m_human.EnablePoseControl( m_worldId, m_poseHertz, m_poseControl );

		m_time = 0.0f;

		{
			b3HullData* Cylinder = b3CreateCylinder( 0.5f, 0.5f, 0.0f, 16 );
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 3.0f, 0.5f, 0.0f };
			bodyDef.rotation = b3MakeQuatFromAxisAngle( B3_VEC3_AXIS_Z, 0.5f * B3_PI );
			b3BodyId Body = m_worldId->CreateBody( &bodyDef );
			Body->AddHull( &shapeDef, Cylinder );
			b3DestroyHull( Cylinder );
		}

		m_angle = 0.0f;
		m_angularVelocity = 0.0f;
	}

	~RagdollPose() override
	{
		b3DestroyMesh( m_ground );
	}

	void OnRenderUI( GLFWwindow* ) override
	{
		ImGui::SetNextWindowPos( ImVec2( 10.0f, 600.0f ) );
		ImGui::SetNextWindowSize( ImVec2( 260.0f, 160.0f ) );
		ImGui::Begin( "Pose", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		if ( ImGui::Checkbox( "motors", &m_motorized ) )
		{
			m_human.EnableMotors( m_motorized );
			m_human.AdjustMotors( m_motorHertz );
		}

		if ( ImGui::SliderFloat( "motor hertz", &m_motorHertz, 0.01f, 20.0f, "%.2f" ) )
		{
			m_human.AdjustMotors( m_motorHertz );
		}

		if ( ImGui::Checkbox( "pose", &m_poseControl ) )
		{
			m_human.EnablePoseControl( m_worldId, m_poseHertz, m_poseControl );
		}

		if ( ImGui::SliderFloat( "pose hertz", &m_poseHertz, 0.01f, 8.0f, "%.2f" ) )
		{
			m_human.AdjustPoseControl( m_poseHertz );
		}

		ImGui::SliderFloat( "omega", &m_angularVelocity, 0.0f, 8.0f, "%.1f" );

		ImGui::End();
	}

	void OnUpdate() override
	{
		float timeStep = 1.0f / m_context->Settings.Frequency;
		m_time += timeStep;

		if ( m_poseControl )
		{
			b3Transform transform = m_human.m_rootBody->GetTransform();
			transform.p.x = 4.0f * sinf( 0.5f * m_time );
			transform.p.y = 0.5f * ( cosf( 1.0f * m_time + B3_PI ) + 1.0f );
			m_angle += m_angularVelocity * timeStep;
			transform.q = b3MakeQuatFromAxisAngle( B3_VEC3_AXIS_Y, m_angle );
			m_human.DriveBase( transform, timeStep );
		}

		// transform = mHuman.m_bones[0].poseJoint->GetRelativeTransform();
		// transform.Translation.Y = 0.5f * ( cosf( 1.0f * mTime + B3_PI ) + 1.0f );
		// mHuman.m_bones[0].poseJoint->SetRelativeTransform( transform );

		Sample::OnUpdate();
	}

	b3MeshData* m_ground;
	Human m_human;
	float m_time;
	float m_motorHertz;
	float m_poseHertz;
	float m_angle;
	float m_angularVelocity;
	bool m_motorized;
	bool m_poseControl;
};

static int sampleRagodllPose = RegisterSample( "Ragdoll", "Pose", RagdollPose::Create );
#endif
