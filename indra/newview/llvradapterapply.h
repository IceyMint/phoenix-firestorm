/**
 * @file llvradapterapply.h
 * @brief P2-07B C1: Stage-2 adapter apply point (implementation spec
 *        section 1/C1 and section 7/B3; brief
 *        P2-07B-C1-IMPLEMENTATION-BRIEF, SHIP round 3).
 *
 * C1 is the single post-blend write site for C6/C7 solver outputs.
 * A guarded hook inside LLMotionController::updateMotions,
 * immediately after mPoseBlender::blendAndApply(), calls
 * LLCharacter::applyVRAdapterPostBlend() (DP-1 [R3]: an
 * LLCharacter virtual with an empty default, overridden by
 * LLVOAvatarSelf). The override forwards to the glue at the bottom
 * of llvradapterapply.cpp, which assembles plan inputs from the
 * live coordinator decisions, settings snapshot and skeleton, runs
 * the plan, and performs the committed writes.
 *
 * Factoring (brief section 4): the gating and write-plan
 * computation is a pure function over plain inputs
 * (FSVRAdapterApply::computePlan), factored exactly as the Stage-1
 * arbitrator is pure, so the standalone H1 harness can exercise it
 * without an avatar. The glue is excluded from harness builds by
 * defining FSVR_ADAPTER_PLAN_ONLY.
 *
 * Package A state (brief OQ-6/OQ-7): C4 has not landed the Stage-2
 * snapshot fields or the runtime-effective decision form, and C3
 * has not registered the spine/arm bones, so the glue is inert by
 * construction: gate 1 rejects the shadow-only Stage-1 decisions,
 * and every registry entry's constraint_adapter_id is empty, which
 * maps to no solver domain. Non-VR and runtime behavior are
 * unchanged (INV-12).
 */

#ifndef LL_LLVRADAPTERAPPLY_H
#define LL_LLVRADAPTERAPPLY_H

#include "llmath.h"

#include <map>
#include <string>
#include <vector>

// P2-07B C6-E (CP-3/CP-4): C5's session-neutral view, defined in
// llvrcalibration.h. Forward-declared here so the CP-4
// composition helper's declaration names it without coupling
// this header to the calibration TU; the definition site and
// every caller see the complete type.
struct FSVRSessionNeutralView;

// ---------------------------------------------------------------------------
// P2-07B Fix 3 (brief sections 2.1/2.4): the posture and
// movement-scaling vocabulary. Declared here, on the
// adapter side of the include edge, because both the
// plan (below) and C5's session-neutral view (in
// llvrcalibration.h, which includes this header) name
// these types.
// ---------------------------------------------------------------------------

// The posture flavour a sync established (brief section
// 2.1), stamped on the neutral view beside scale_source
// under the same CP-S1-1 producer rule: every producer
// of an established view stamps it, and the
// default-constructed state is the fail-closed one -- a
// producer that stamps nothing yields UNDETERMINED, and
// the system treats that as the brief's section 2.5
// state (posture behaviours off, shipped profile
// continues, announced), never as a guessed flavour.
// Posture is a property of the neutral, not of the live
// pose: it changes only when a sync establishes a new
// neutral, and it is invalidated with the neutral. It is
// NEVER inferred from any measured quantity (brief
// section 2.5; F3-P3's negative control).
enum class FSVRPosture : U8
{
    UNDETERMINED = 0,   // default-constructed; fail-closed
    SEATED       = 1,   // established by a seated sync
    STANDING     = 2,   // established by a standing sync
};

// The effective behaviour a cycle runs (brief section
// 2.1): a pure function of the neutral's stamped
// posture and FSVRPostureSwap -- SEATED runs the sitting
// behaviour, STANDING the standing behaviour, the swap
// exchanges them, and UNDETERMINED runs neither posture
// behaviour whatever the swap says.
enum class FSVRPostureBehaviour : U8
{
    NONE     = 0,   // undetermined: no posture behaviour
    SITTING  = 1,   // spine lean as shipped; root still
    STANDING = 2,   // root follows; no translational lean
};

FSVRPostureBehaviour fsvrEffectivePostureBehaviour(
    FSVRPosture posture, bool posture_swap);

// Record/display names for the Fix 3 facts (the
// gateName idiom): stable strings the bonelog records
// and the sync's chat feedback use, produced in one
// place so every surface names a state identically.
const char* fsvrPostureName(FSVRPosture posture);
const char* fsvrPostureBehaviourName(FSVRPostureBehaviour behaviour);

// ---------------------------------------------------------------------------
// P2-07B Fix 3 R5 (brief sections 2.1-2.3): the avatar-state
// vocabulary. R5 supersedes the sync-declared posture as the
// behaviour authority: the avatar's CURRENT in-world state,
// sampled by the glue from live avatar facts, classifies into
// one of these logical states, and behaviour selection (P2)
// consumes the classification -- never the sync's flavour
// stamp (which remains a calibration record only) and never
// the user's physical posture.
// ---------------------------------------------------------------------------

// The avatar's ground-sit motion evidence for one sample, as
// the classifier needs it (brief section 2.2): the stock
// ground-sit loading-motion guard makes an ACTIVE ground
// state incompatible with a settled ground sit; a loading or
// otherwise unavailable motion read is its own value, never
// folded into ACTIVE or INACTIVE.
enum class FSVRGroundMotion : U8
{
    INACTIVE = 0,   // ANIM_AGENT_SIT_GROUND_CONSTRAINED inactive
    ACTIVE   = 1,   // ANIM_AGENT_SIT_GROUND_CONSTRAINED active
    UNKNOWN  = 2,   // loading / motion state unavailable
};

// One raw sample of the avatar facts the classifier reads,
// assembled by the glue from the live avatar at the apply
// point (after updateCharacter's parent reconciliation for
// the same frame). Pure data; the classifier below is the
// only interpretation of these facts.
struct FSVRAvatarStateSample
{
    bool avatar_valid = false;      // the self avatar exists
    bool sitting_flag = false;      // LLVOAvatar::isSitting()
    bool parent_present = false;    // getParent() non-null
    // The parent is the avatar's root prim (an object sit).
    // False with parent_present true means the parent could
    // not be resolved as an object sit this sample.
    bool parent_is_object = false;
    FSVRGroundMotion ground_motion = FSVRGroundMotion::UNKNOWN;
};

// The classified logical avatar state for one sample (brief
// section 2.1). SITTING_SUBTYPE_UNKNOWN is a SITTING state
// for every behaviour purpose (brief section 2.3: sitting is
// sitting); the subtype is a diagnostic label only.
enum class FSVRAvatarPostureState : U8
{
    INVALID                 = 0,   // no self avatar to sample
    UNKNOWN                 = 1,   // facts contradictory/unreadable
    STANDING                = 2,
    SITTING_OBJECT          = 3,
    SITTING_GROUND          = 4,
    SITTING_SUBTYPE_UNKNOWN = 5,
};

// The classification truth table (brief section 2.2, with
// the plan's extension rows), a pure function of the sample:
//   avatar invalid                              -> INVALID
//   sitting, parent present, parent is object   -> SITTING_OBJECT
//   sitting, no parent, ground ACTIVE           -> SITTING_GROUND
//     (the ground-sit motion IS the ground sit: its
//     start/stop drives sitDown(true/false) in the stock
//     animation-state path, and updateCharacter's
//     reconciliation preserves an unparented sit exactly
//     while that motion is active)
//   sitting, no parent, ground INACTIVE         -> UNKNOWN
//     (the flag says sitting but the motion that would
//     hold the sit is neither active nor loading: the
//     stock reconciliation retires such a sit the same
//     frame -- contradictory, never guessed)
//   sitting, no parent, ground UNKNOWN          -> SITTING_SUBTYPE_UNKNOWN
//   sitting, parent present, parent unresolved  -> SITTING_SUBTYPE_UNKNOWN
//   not sitting, no parent, ground INACTIVE     -> STANDING
//   not sitting, no parent, ground ACTIVE       -> UNKNOWN
//   not sitting, no parent, ground UNKNOWN      -> UNKNOWN
//   not sitting, parent present (any ground)    -> UNKNOWN
//     (a parented avatar whose sitting flag is clear
//     contradicts the stock setParent contract)
// No row consults head height, HMD pose, eye height,
// calibration state or history: the classification is a
// function of this sample alone (brief sections 2.2, 7).
FSVRAvatarPostureState fsvrClassifyAvatarState(
    const FSVRAvatarStateSample& sample);

// Record/display names for the R5 states (the gateName
// idiom): stable strings the bonelog and the in-world
// diagnostic use, produced in one place.
const char* fsvrAvatarPostureStateName(FSVRAvatarPostureState state);
const char* fsvrGroundMotionName(FSVRGroundMotion motion);

// True for every state whose behaviour is the sitting
// behaviour (brief section 2.3): the two resolved subtypes
// and the subtype-unknown state alike.
bool fsvrAvatarStateIsSitting(FSVRAvatarPostureState state);

// [FIX3-R5-CODE-03] The glue's lifecycle identity: the
// character object, its skeleton-rebuild generation and
// the posture install generation. A home base, a watch
// expectation or a tracked posture belongs to ONE
// lifecycle; any change of identity retires all three
// before another root-follow write, so a reconstructed
// or replaced avatar can never inherit the previous
// avatar's home base through a STANDING -> STANDING
// non-transition.
struct FSVRGlueLifecycleKey
{
    const void* character = nullptr;
    U64 skeleton_generation = 0;
    U32 install_generation = 0;

    bool operator==(const FSVRGlueLifecycleKey& other) const
    {
        return character == other.character &&
               skeleton_generation == other.skeleton_generation &&
               install_generation == other.install_generation;
    }
    bool operator!=(const FSVRGlueLifecycleKey& other) const
    {
        return !(*this == other);
    }
};

// The persistent in-world indicator's three lines
// (brief section 8.2), a pure function of the facts the
// cycle actually consumed -- the indicator never infers
// posture on its own. Line 1 names the sampled state,
// line 2 the effective behaviour, line 3 the root
// channel's status. Root precedence: suspension first;
// a non-standing behaviour is never root-eligible; an
// unproven detector or the Head Position toggle
// suppresses; an out-of-force boundary reports
// BOUNDARY; writes flowing report ACTIVE; anything
// else is NOT ELIGIBLE.
struct FSVRIndicatorFacts
{
    FSVRAvatarPostureState avatar_state = FSVRAvatarPostureState::UNKNOWN;
    FSVRPostureBehaviour behaviour = FSVRPostureBehaviour::NONE;
    bool session_established = false;
    bool root_follow_suspended = false;
    bool detector_proven = false;
    bool head_position_on = true;
    bool root_follow_bound_valid = false;
    bool root_writes_eligible = false;
};

struct FSVRIndicatorLines
{
    std::string avatar_line;
    std::string behaviour_line;
    std::string root_line;
};

FSVRIndicatorLines fsvrFormatIndicatorLines(
    const FSVRIndicatorFacts& facts);

// The R5 behaviour selector (brief section 2.3): a pure
// function of the classified avatar state. STANDING runs
// the standing behaviour (root-follow, no lean); every
// sitting state runs the sitting behaviour (lean, never
// root-moved); UNKNOWN and INVALID run NEITHER posture
// behaviour (brief section 2.4). The Head Position toggle
// is deliberately NOT an input: its shipped gate sites
// (plan gate 4's spine authority, the root step's zero,
// the glue's write eligibility) already implement the
// toggle matrix's OFF column exactly, and the F3-G5 golden
// pins that extraction.
FSVRPostureBehaviour fsvrSelectPostureBehaviour(
    FSVRAvatarPostureState state);

// Per-cycle avatar-state tracking for the glue: the current
// logical state, the frame it was sampled in, and whether
// the last update changed it. Transitions (sitting->standing
// above all) drive the home-base lifecycle (brief section
// 3.1); the frame stamp is the P1 freshness contract -- a
// sample is usable only in the frame it was taken.
struct FSVRAvatarStateTracker
{
    FSVRAvatarPostureState current = FSVRAvatarPostureState::UNKNOWN;
    bool has_sample = false;
    U64 sample_frame = 0;
    // Record a fresh sample; returns the previous state.
    FSVRAvatarPostureState update(FSVRAvatarPostureState state, U64 frame)
    {
        const FSVRAvatarPostureState prev = current;
        current = state;
        has_sample = true;
        sample_frame = frame;
        return prev;
    }
    bool fresh(U64 frame) const
    {
        return has_sample && sample_frame == frame;
    }
    void reset()
    {
        current = FSVRAvatarPostureState::UNKNOWN;
        has_sample = false;
        sample_frame = 0;
    }
};

// ---------------------------------------------------------------------------
// P2-07B Fix 3 R5 (brief section 4.3; implementation plan P3
// rev. 5): the positional-discontinuity (teleport) watch.
// Pure over the positions the glue feeds it; the glue owns
// the sampling, the home-base store and the suspension's
// effect on the root write. Contract (plan P3, [FIX3-PLAN-
// R2/R3/R4]): comparison at cycle start against P_expected;
// own writes update P_expected atomically; P_expected_prev
// is eligible for the single comparison window immediately
// following the write that created it and is consumed by
// that window whether or not it matches (R4-01); a region
// change is a discontinuity by definition; on discontinuity
// the watch latches suspended and only reset() (a fresh
// sync / session end, at the glue) clears it.
// ---------------------------------------------------------------------------

enum class FSVRWatchVerdict : U8
{
    CONTINUITY    = 0,
    DISCONTINUITY = 1,
};

// The provisional discontinuity threshold, in metres of
// per-cycle agent displacement. PROVISIONAL (plan R3-01):
// the measured legitimate/teleport bands have not yet been
// recorded, so this value is a conservative placeholder,
// never to be tuned silently; the detector's enablement
// for live root-follow is gated on the recorded bands
// being shown separable (plan P3 criterion 4).
constexpr F32 FSVR_TELEPORT_PROVISIONAL_EPSILON_M = 10.f;

// Why the watch latched [FIX3-R5-CODE-R3-2B]: a
// positional discontinuity and a region change both
// suspend root-follow and invalidate the home base,
// but they are different events and the record must
// not confuse them -- a region crossing on foot is not
// a suspected teleport.
enum class FSVRWatchCause : U8
{
    NONE       = 0,
    POSITIONAL = 1,
    REGION     = 2,
};

struct FSVRTeleportWatch
{
    bool expected_valid = false;
    LLVector3 p_expected = LLVector3::zero;
    bool prev_valid = false;
    LLVector3 p_expected_prev = LLVector3::zero;
    bool region_valid = false;
    U64 region_id = 0;
    bool suspended = false;
    FSVRWatchCause last_cause = FSVRWatchCause::NONE;
    // Band evidence for the R3-01 measurement
    // [FIX3-R5-CODE-02 / CODE-R2-02]: the populations are
    // labeled by an INDEPENDENT source -- the viewer's
    // own teleport state, passed in as known_teleport --
    // never by the watch's own verdict, so the evidence
    // is not circular. max_continuity_m is the largest
    // step observed with no teleport in progress (the
    // legitimate band); teleport_step_max_m the largest
    // step observed during a known teleport;
    // teleport_min_m / teleport_count the labeled
    // teleport discontinuities (a region change with a
    // beyond-epsilon step is one by construction);
    // unexplained_* the discontinuities no label covers
    // (the ambiguous population); last_discontinuity_m
    // the most recent discontinuity of any kind (-1
    // until one occurs).
    F32 max_continuity_m = 0.f;
    F32 teleport_step_max_m = 0.f;
    F32 teleport_min_m = -1.f;
    U32 teleport_count = 0;
    U32 unexplained_count = 0;
    F32 unexplained_max_m = 0.f;
    F32 last_discontinuity_m = -1.f;

    // Cycle start: compare the freshly sampled agent
    // position against the expected state. Consumes the
    // P_expected_prev window (R4-01) on every call.
    // known_teleport labels the cycle's evidence only;
    // it never changes a verdict.
    FSVRWatchVerdict observe(const LLVector3& p_now, U64 region,
                           F32 epsilon_m,
                           bool known_teleport = false)
    {
        if (suspended)
        {
            return FSVRWatchVerdict::DISCONTINUITY;
        }
        if (region_valid && region != region_id)
        {
            // A region change is a discontinuity by
            // definition (plan P3 contract point 3). For
            // the evidence: a within-epsilon step across
            // the border is a crossing on foot (legitimate
            // band, or the teleport-context band when a
            // teleport is in progress); a beyond-epsilon
            // step is a labeled teleport.
            if (expected_valid)
            {
                const F32 step_m = (p_now - p_expected).length();
                if (step_m <= epsilon_m)
                {
                    recordContinuityStep(step_m, known_teleport);
                }
                else
                {
                    recordTeleport(step_m);
                }
                last_discontinuity_m = step_m;
            }
            latchSuspended(FSVRWatchCause::REGION);
            return FSVRWatchVerdict::DISCONTINUITY;
        }
        region_id = region;
        region_valid = true;
        if (!expected_valid)
        {
            // First sample of a watch: establishes the
            // expectation, judges nothing.
            p_expected = p_now;
            expected_valid = true;
            prev_valid = false;
            return FSVRWatchVerdict::CONTINUITY;
        }
        const bool prev_window = prev_valid;
        const LLVector3 prev = p_expected_prev;
        prev_valid = false;   // consumed by this window (R4-01)
        const F32 step_m = (p_now - p_expected).length();
        if (step_m <= epsilon_m)
        {
            recordContinuityStep(step_m, known_teleport);
            return FSVRWatchVerdict::CONTINUITY;
        }
        if (prev_window && (p_now - prev).length() <= epsilon_m)
        {
            // A not-yet-reflected own write (read-back lag),
            // inside its single permitted window. The
            // evidence records the step from the pre-write
            // position -- the avatar's actual motion --
            // not the inflated distance from the written
            // expectation.
            recordContinuityStep((p_now - prev).length(),
                                 known_teleport);
            return FSVRWatchVerdict::CONTINUITY;
        }
        last_discontinuity_m = step_m;
        if (known_teleport)
        {
            recordTeleport(step_m);
        }
        else
        {
            ++unexplained_count;
            if (step_m > unexplained_max_m)
            {
                unexplained_max_m = step_m;
            }
        }
        latchSuspended();
        return FSVRWatchVerdict::DISCONTINUITY;
    }

    void recordContinuityStep(F32 step_m, bool known_teleport)
    {
        if (known_teleport)
        {
            if (step_m > teleport_step_max_m)
            {
                teleport_step_max_m = step_m;
            }
        }
        else if (step_m > max_continuity_m)
        {
            max_continuity_m = step_m;
        }
    }

    void recordTeleport(F32 step_m)
    {
        ++teleport_count;
        if (teleport_min_m < 0.f || step_m < teleport_min_m)
        {
            teleport_min_m = step_m;
        }
        last_discontinuity_m = step_m;
    }

    // The glue's own root write: updates P_expected
    // atomically with the write and retains the displaced
    // value as P_expected_prev for one window (R4-01).
    void noteWrite(const LLVector3& target)
    {
        if (expected_valid)
        {
            p_expected_prev = p_expected;
            prev_valid = true;
        }
        p_expected = target;
        expected_valid = true;
    }

    // End of a cycle in which the adapter wrote nothing:
    // the expectation becomes the position sampled at the
    // end of the previous cycle (plan P3 contract point 1).
    // A no-write cycle also retires any unconsumed prev
    // window: prev never survives an intervening sampled
    // cycle (R4-01).
    void noteNoWrite(const LLVector3& p_end)
    {
        if (!suspended)
        {
            p_expected = p_end;
            expected_valid = true;
            prev_valid = false;
        }
    }

    void latchSuspended(FSVRWatchCause cause =
                            FSVRWatchCause::POSITIONAL)
    {
        suspended = true;
        expected_valid = false;
        prev_valid = false;
        last_cause = cause;
    }

    // Fresh sync / session end (glue's call): the watch
    // starts over; the next observe() re-establishes.
    void reset()
    {
        expected_valid = false;
        prev_valid = false;
        region_valid = false;
        suspended = false;
        max_continuity_m = 0.f;
        teleport_step_max_m = 0.f;
        teleport_min_m = -1.f;
        teleport_count = 0;
        unexplained_count = 0;
        unexplained_max_m = 0.f;
        last_discontinuity_m = -1.f;
        last_cause = FSVRWatchCause::NONE;
    }
};

// Complete teleport events [FIX3-R5-CODE-R3-2A]: the
// per-frame statistics cannot express a teleport that
// arrives as several small updates, nor one that never
// exceeds the per-frame epsilon. This tracker measures
// whole events from the independent teleport-state
// transitions the glue feeds it: the position at the
// last quiet cycle before the event, and the position
// once the event has ended and the avatar has settled
// (a cycle whose step is within the settle threshold,
// or the settle bound, whichever comes first). An
// event still open when the tracker is reset counts
// as incomplete and contributes no displacement.
struct FSVRTeleportEventTracker
{
    static constexpr F32 SETTLE_STEP_M = 0.02f;
    static constexpr U32 SETTLE_MAX_CYCLES = 90;

    bool in_event = false;
    bool settling = false;
    U32 settle_cycles = 0;
    LLVector3 start_position = LLVector3::zero;
    // Whether start_position is a real pre-event
    // baseline [FIX3-R5-CODE-R4-1A]: an event first
    // observed already in progress has none, and no
    // displacement may be published for it -- a partial
    // measurement presented as whole is worse than an
    // explicit unmeasurable.
    bool start_valid = false;
    LLVector3 last_position = LLVector3::zero;
    bool last_position_valid = false;

    F32 event_min_m = -1.f;
    F32 event_max_m = 0.f;
    U32 events_completed = 0;
    // Events retired without a measurement (unanchored).
    U32 events_incomplete = 0;

    // An event still open when its generation ends is
    // also an incomplete observation of that generation.
    U32 incompleteAtEnd() const { return in_event ? 1 : 0; }

    // Returns true (and sets net_out) on the cycle an
    // event completes.
    bool observe(bool teleport_active, const LLVector3& p_now,
                 F32& net_out)
    {
        net_out = 0.f;
        if (teleport_active)
        {
            if (!in_event)
            {
                in_event = true;
                settling = false;
                settle_cycles = 0;
                // The baseline is the last quiet position
                // -- when one was ever observed. A teleport
                // already in progress at the first
                // observation has no baseline: the event
                // is tracked but unanchored, and will be
                // retired as incomplete, never measured.
                start_valid = last_position_valid;
                if (start_valid)
                {
                    start_position = last_position;
                }
            }
            else if (settling)
            {
                // A new event began while the previous
                // one was settling: the previous event's
                // displacement is final at the position
                // where this one starts. (complete()
                // reports whether it was measurable.)
                const bool measured = complete(p_now, net_out);
                in_event = true;
                settling = false;
                settle_cycles = 0;
                // The handover position was observed
                // during the inactive settling cycles:
                // a real baseline for the new event.
                start_position = p_now;
                start_valid = true;
                last_position = p_now;
                last_position_valid = true;
                return measured;
            }
            last_position = p_now;
            last_position_valid = true;
            return false;
        }
        if (in_event && !settling)
        {
            settling = true;
            settle_cycles = 0;
        }
        if (settling)
        {
            ++settle_cycles;
            const F32 step = last_position_valid
                ? (p_now - last_position).length() : 0.f;
            last_position = p_now;
            last_position_valid = true;
            if (step <= SETTLE_STEP_M ||
                settle_cycles >= SETTLE_MAX_CYCLES)
            {
                const bool measured = complete(p_now, net_out);
                in_event = false;
                settling = false;
                return measured;
            }
            return false;
        }
        last_position = p_now;
        last_position_valid = true;
        return false;
    }

    // Returns true when a displacement was measured.
    // An unanchored event retires as incomplete and
    // publishes nothing.
    bool complete(const LLVector3& end_position, F32& net_out)
    {
        if (!start_valid)
        {
            ++events_incomplete;
            net_out = 0.f;
            return false;
        }
        net_out = (end_position - start_position).length();
        ++events_completed;
        if (event_min_m < 0.f || net_out < event_min_m)
        {
            event_min_m = net_out;
        }
        if (net_out > event_max_m)
        {
            event_max_m = net_out;
        }
        return true;
    }

    // Fresh sync / session end (the glue reads the
    // aggregates, including incompleteAtEnd(), before
    // calling this).
    void reset()
    {
        in_event = false;
        settling = false;
        settle_cycles = 0;
        start_valid = false;
        last_position_valid = false;
        event_min_m = -1.f;
        event_max_m = 0.f;
        events_completed = 0;
        events_incomplete = 0;
    }
};

// The two scale contracts, per channel (brief section
// 2.4). PROPORTIONAL is in force only when the channel's
// factor rests on landmark-matched endpoints (the
// measured establishment's matched pair) or on a valid
// manual ratio (proportional by the user's own
// declaration). PROVISIONAL is everything else (baseline
// x gain -- an honest sensitivity, never described as
// avatar-size scaling). OFF is the failed-closed state:
// an invalid scale parameter took the channel out, with
// its event; it is labeled neither, because it is not a
// contract at all.
enum class FSVRScaleContract : U8
{
    PROVISIONAL  = 0,
    PROPORTIONAL = 1,
    OFF          = 2,
};

const char* fsvrScaleContractName(FSVRScaleContract contract);

// A channel's manual limb-ratio state (brief section
// 2.4), as consumed through the settings snapshot's
// shipped global form. ABSENT: the derived ratio (or the
// provisional baseline) rules. VALID: the user's stated
// ratio replaces the derived one and the channel is
// proportional by declaration. INVALID: present but
// outside the valid domain or non-finite -- the channel
// fails closed, never clamped, never quietly replaced
// by the derived value. Production derives the state
// per channel from the global settings
// FSVRManualLeanRatio / FSVRManualRootRatio through
// the snapshot [FIX3-CODE-R1-01]: the declared 0 is
// ABSENT (unset), a finite value within 0.05..3.0 is
// VALID, any other present value is INVALID.
enum class FSVRManualRatioState : U8
{
    ABSENT  = 0,
    VALID   = 1,
    INVALID = 2,
};

// The one mapping from a snapshot's per-channel manual
// derivation -- its (set, invalid) pair over the
// channel's global ratio setting -- to the seam state
// above [FIX3-CODE-R1-01]. Every production consumer of
// the fact (the adapter glue's cycle inputs, the
// calibration shell's establishment-record labels)
// maps through here, never re-deriving it at a call
// site.
FSVRManualRatioState fsvrManualRatioState(bool is_set,
                                          bool is_invalid);

// Which solver domain a joint belongs to. NONE joints are never
// written by the C1 solvers: the head and neck (brief section 3.5
// [R1]: C1 never writes mHead or mNeck in Package A -- the one
// narrow exception is Fix 1's head-group reconciliation, which
// conditionally writes mNeck/mHead outside the solver domains),
// the collars (never written by the positional solve, spec
// section 6.2), the pelvis root, and structural ancestors pulled
// into the plan tree.
enum class FSVRAdapterDomain
{
    NONE,
    SPINE,
    ARM_LEFT,
    ARM_RIGHT
};

enum class FSVRAdapterSide
{
    LEFT,
    RIGHT
};

// Motion-cycle classification, derived by the glue from production
// state (motion-controller paused flag, Animation Preview render
// mode). The [R1] H1 cases feed exactly these values: the gate set
// consumes the classification, it does not assume it.
enum class FSVRAdapterCycleKind
{
    NORMAL,         // ordinary motion cycle; the blend ran
    PAUSED_NORMAL,  // paused controller, ordinary update (no blend;
                    // structurally unreachable from the hook)
    PAUSED_FORCE,   // paused controller, FORCE_UPDATE (blend ran)
    PREVIEW,        // Animation Preview cycle (blend ran)
    HIDDEN          // hidden update (no blend; never reaches the hook)
};

// First failing gate, in brief section 3.3 order (the cycle and
// context checks precede gate 1). C9 diagnostics trace this value.
enum class FSVRAdapterGate
{
    NONE = 0,
    CYCLE,              // no blend this cycle, or a paused / preview /
                        // hidden cycle kind
    CONTEXT,            // bridge not armed (non-self avatar)
    FRAME_IDENTITY,     // gate 1: decisions absent, stale, or not
                        // runtime-effective (shadow-only)
    VR_SESSION,         // gate 2: vr_active false (headset off)
    POSITIONAL_MASTER,  // gate 3: FSVRDiagPositionalSolve false
    NO_SOLVER,          // gate 4 outcome: neither solver holds
                        // constraint authority this cycle
    TOKEN               // apply-once wrapper: frame token already
                        // processed
};

// One joint of the plan tree. The list is topologically ordered
// (parents before children) and forms a single tree under the plan
// root frame, so composition always resolves a parent before its
// children (brief section 3.4 steps 3/6).
struct FSVRAdapterJointInput
{
    std::string name;
    // Index into the plan input's joint list; -1 means the joint's
    // parent is the plan root frame.
    S32 parent_index = -1;
    // Local position in the parent frame (the skeleton XML pos).
    // Composition goes through these real offsets, never a
    // stacked-lever approximation (C0 section 2 pivot zigzag).
    LLVector3 rest_offset = LLVector3::zero;
    // The post-blend local rotation: the arbitrated pose for this
    // cycle (brief section 3.4 step 1).
    LLQuaternion base_local;
    FSVRAdapterDomain domain = FSVRAdapterDomain::NONE;
    bool in_registry = false;
    // Arbitration grants this joint a tracking base (brief gate 5).
    bool decision_grants_base = false;
    // The joint is in the snapshot's disabled-bone set.
    bool disabled = false;
};

// P2-07B C6-E (CP-2; Package B spec section 7.2):
// spine_sample_valid is exactly the conjunction
// E5 ^ E6 ^ E7 ^ E8 ^ !E9 over these inputs. E1-E4 ride their
// own gates in computePlan and are NOT inputs here. Pure data;
// the glue supplies the cycle's values from the session
// machinery (CP-3), the manager (E7), and C5's transaction
// state (E9).
struct FSVRSpineSampleInputs
{
    bool scale_input_valid = false;      // E5: scale-input
        // validity (P2-07B Stage 1, CP-S1-4): the C5 view's
        // scale source is not NONE (a MEASURED_RECORD view's
        // record was validated USABLE_* at its producers; a
        // PROVISIONAL_SYNC view's scale is the baseline
        // constant) AND the settings snapshot's lean gain is
        // valid (bounded domain, CP-S1-3). Renamed from
        // record_usable_for_head: the redesign falsified the
        // old name -- a sync-established neutral carries no
        // usable record at all.
    bool neutral_established = false;    // E6: the C5 view's
        // established flag, post any C8 invalidation
    bool hmd_position_valid = false;     // E7: the manager
        // accessor's three-term predicate, this cycle (the
        // bool getHmdPosition returns)
    bool sitting_signals_clear = false;  // E8: the section
        // 4.2 step-3 signals read not-sitting, this cycle
        // (the C5 view's per-cycle fact, ICR-2)
    bool calibration_in_progress = false; // E9:
        // FSVRCalibration::transactionActive(), read at
        // population (negated in the conjunction)
};

struct FSVRAdapterPlanInput
{
    U64 frame_token = 0;

    FSVRAdapterCycleKind cycle = FSVRAdapterCycleKind::NORMAL;
    bool blend_ran = false;
    // Self avatar with the bridge armed (DP-1). Non-self characters
    // never reach the glue; the plan still checks this so the
    // harness can exercise the non-self case.
    bool context_armed = false;

    // Gate 1 (frame identity).
    bool decisions_published = false;
    bool decisions_current = false;
    // The legacy all-bones runtime-effective form (C4's
    // derivation over the published set: non-empty and no bone
    // shadow-only). RETAINED AS A RECORDED FACT ONLY (P2-07B
    // C6-E, CP-5 part 3): traces and H1 fixtures read it, but
    // it no longer gates -- gate 1 and gate 4 consume the
    // per-domain scoped fields below, because a domain's
    // shadow state must never block the other domain.
    bool decisions_runtime_effective = false;
    // P2-07B C6-E (CP-5 part 3): the scoped gate-1 facts, one
    // per solver domain, derived by the glue from the
    // arbitrator's emitted shadow_only form (the pure
    // derivations in llvrtrackingarbitrator.h). Gate 1 passes
    // when either domain's fact holds; gate 4 additionally
    // requires each domain's own fact for that domain's
    // authority, so no authority ever forms on shadow
    // decisions for its domain.
    bool spine_decisions_runtime_effective = false;
    bool arm_decisions_runtime_effective = false;

    // Gate 2 (session gate only [R1]): the headset/session state.
    // Never derived from any bone's source decision (amendment 2).
    bool vr_active = false;

    // Gate 3: FSVRDiagPositionalSolve (spec section 3.4). False
    // suspends both positional solves at the apply point; the
    // orientation path is not C1's and is unaffected.
    bool positional_solve_enabled = false;

    // Gate 4 (per-solver [R1]): enablement and positional sample
    // validity are evaluated separately for each solver. Constraint
    // authority never depends on a bone holding a TRACKED source
    // decision (spec section 1, R2-01).
    bool head_position_enabled = false; // FSVRTrackHeadPosition
    bool hand_position_enabled = false; // FSVRTrackHandPosition
    bool spine_sample_valid = false;
    bool arm_sample_valid = false;

    // L4 (FSVRSolverBendDisabledBones, spec section 6.1): the
    // solver may bend disabled SPINE bones. Never extends to arms
    // (D4).
    bool bend_disabled_bones = false;

    // Diagnostic pass-throughs the solvers read from the context.
    // C1 never interprets them: FSVRDiagBendLimits=false suspends
    // the section 6.3 clamp INSIDE the solvers, and C1 must not
    // treat an unclamped result as limit-satisfying (spec section
    // 3.4 safety boundary).
    bool diag_bend_limits = true;
    bool diag_full_chain_solve = true;

    // The transaction root frame: the world rotation/position of
    // the parent of every parent_index == -1 joint (the pelvis
    // frame for the torso/arm chains). Solver target worlds are
    // expressed in this frame's world space.
    //
    // PRE-RUNTIME-ENABLE VERIFICATION OBLIGATION (recorded, code
    // cross-check round 1): the glue sources this frame from the
    // root joint's cached world values at apply time. Before any
    // runtime writes are enabled (C6/C7 integration), the root
    // frame must be demonstrated invariant or current for the
    // cycle it anchors -- the adapter's own composition never
    // reads a cached parent world for an adapter-owned joint
    // (brief section 2.6), and the root frame may not become the
    // one stale read that survives. Package A produces no writes,
    // so the obligation is discharged by recording it here and at
    // the glue's sourcing site; it rides with C6/C7 integration.
    LLQuaternion root_world_rotation;
    LLVector3 root_world_position = LLVector3::zero;

    std::vector<FSVRAdapterJointInput> joints;

    // The previous valid solved targets, an explicit input to the
    // solvers' continuity term (brief section 3.6). The plan holds
    // no cross-cycle pose state of its own; this map is data the
    // caller supplies, never a C1-accumulated baseline (H1
    // no-drift check).
    std::map<S32, LLQuaternion> previous_targets;

    // C6/C7 positional targets and solver inputs (brief CP-2;
    // additions only -- no existing field changes meaning).
    // P2-07B C6-E (CP-2/CP-4): the production glue now
    // populates spine_sample_valid from the E5-E9 conjunction
    // and composes the head-target block per cycle; the wrist
    // block stays written-false (Package B spec section 7.4).
    // Production remains inert until an authorization build
    // (CP-5) lets gate 1 pass; harnesses populate all of it.

    // FSVRDiagSoftPreference pass-through (D3): the C6 soft term
    // is read from here (the snapshot via the glue, CP-7).
    bool diag_soft_preference = true;
    // Continuity identity (CP-7/DP-4): the settings snapshot
    // generation these inputs were assembled under. A change
    // retires the caller's continuity history (previous targets
    // and pole state) before the next apply.
    U64 settings_generation = 0;

    // P2-07B C8 (CP-3): the session continuity epoch these
    // inputs were assembled under -- C8's dedicated counter,
    // advanced by a finalized LOCAL transition or a skeleton
    // reinitialization since the previous apply. A change
    // retires the caller's continuity history exactly as a
    // settings-generation change does (Package B spec section
    // 6.3): those two events produce neither a frame-token
    // gap nor a generation change, so without this stamp both
    // histories would carry across them. Supplied by the glue
    // from FSVRContinuity at assembly; a stamp, not a callback.
    U64 continuity_epoch = 0;

    // C6's target: the cornea-midpoint world position (already
    // composed and ratio-scaled by the CP-3 seam's caller;
    // Packages B/C supply it in production), with the
    // skeleton-derived cornea offset in the mHead local frame.
    // head_target_valid == false is "no constraint": C6 returns
    // an empty map.
    bool head_target_valid = false;
    LLVector3 head_target_world = LLVector3::zero;
    LLVector3 cornea_offset_head_local = LLVector3::zero;

    // C7's targets, per side (index 0 = LEFT, 1 = RIGHT):
    // wrist world positions, already composed (CP-3's
    // provisional seam in Package A). wrist_target_valid[side]
    // == false is "no constraint" for that side.
    bool wrist_target_valid[2] = {false, false};
    LLVector3 wrist_target_world[2] = {LLVector3::zero,
                                       LLVector3::zero};

    // Caller-owned pole history, per side [R2 -- C6C7-03
    // transport]: the pole direction (world frame) the caller
    // adopted from the last committed apply, carried beside
    // previous_targets under the same lifecycle (retired on
    // authority lapse, frame-token discontinuity, or settings
    // generation change; LOCAL-transition and skeleton
    // invalidation are C8's signals, brief OQ-3). C7 reads pole
    // history only from the solver context's copies of these
    // fields; it stores none itself.
    bool previous_pole_valid[2] = {false, false};
    LLVector3 previous_pole_world[2] = {LLVector3::zero,
                                        LLVector3::zero};

    // P2-07B C5: an active calibration transaction (Package B
    // spec section 3.1). Set only by the glue from C5's
    // transaction state; the apply's calibration-hold branch
    // keys on this field alone. Independent of gates 1-5 by
    // design (spec section 3.1 authorization contract).
    bool calibration_hold_active = false;

    // P2-07B Fix 3 (brief sections 2.2/4): the posture
    // behaviour in force for this cycle -- the pure
    // function of the neutral's stamped posture and the
    // snapshot's swap, computed by the glue (or by a
    // harness mirroring it) -- and the standing channel's
    // computed root offset for the cycle. The offset is a
    // VALUE the plan carries to the result for the record;
    // the plan itself moves nothing (its application is
    // the glue's, through the agent movement channel
    // Leah's OQ-5 ruling settled). posture_behaviour_
    // present is false when no behaviour was determined
    // for the cycle (the record then omits the fact).
    FSVRPostureBehaviour posture_behaviour = FSVRPostureBehaviour::NONE;
    bool posture_behaviour_present = false;
    bool root_offset_present = false;
    LLVector3 root_offset = LLVector3::zero;
    // P2-07B Fix 3 R5 (brief section 8.1): the cycle's
    // recorder facts for the sampled avatar state, the
    // root-follow suspension and the home base. Like the
    // posture behaviour above, they are carried to the
    // result for the record; the plan reads none of them.
    FSVRAvatarPostureState avatar_state = FSVRAvatarPostureState::UNKNOWN;
    bool avatar_state_present = false;
    bool root_follow_suspended = false;
    bool home_base_valid = false;
    U32 home_base_generation = 0;
    // FSVRPostureSwap's configured value, recorded so the
    // bonelog distinguishes it from its (inert, under R5)
    // effect: posture_swap_inert is true exactly when the
    // cycle's behaviour came from the sampled avatar state.
    bool posture_swap_configured = false;
    bool posture_swap_inert = false;
};

// Read-only view handed to the solvers: the plan input plus the
// base (arbitrated) pose composed over the root frame.
struct FSVRAdapterSolverContext
{
    const FSVRAdapterPlanInput* plan = nullptr;
    std::vector<LLQuaternion> base_world_rotations;
    std::vector<LLVector3> base_world_positions;
    std::map<S32, LLQuaternion> previous_targets;
    // The plan input's caller-owned pole history, copied in by
    // the transaction beside previous_targets [R2 -- C6C7-03
    // transport]. Indexed 0 = LEFT, 1 = RIGHT as on the plan.
    bool previous_pole_valid[2] = {false, false};
    LLVector3 previous_pole_world[2] = {LLVector3::zero,
                                        LLVector3::zero};
};

// The solver boundary is the target-world list (brief section 5):
// solvers return desired WORLD orientations keyed by joint index;
// an absent index is "no constraint" for that joint. Solvers write
// nothing themselves; C1 owns the final committed output (OQ-2
// [R1] terminology). Implementations apply the C0 limit rules to
// their own outputs (brief section 3.4): C6 uses C0 section 5.1;
// C7 uses C0 sections 5.2/5.3 after its RP3 pass, with RP3's
// internal writes provisional under the OQ-2 [R1] containment
// conditions.
class FSVRAdapterSpineSolver
{
public:
    virtual ~FSVRAdapterSpineSolver() = default;
    virtual void solveSpine(const FSVRAdapterSolverContext& context,
                            std::map<S32, LLQuaternion>& out_target_worlds) = 0;
};

class FSVRAdapterArmSolver
{
public:
    virtual ~FSVRAdapterArmSolver() = default;
    // shoulder_world_* are resolved from the solved spine before
    // this call (brief section 3.4 step 4; D6: hand targets never
    // re-open the spine solve).
    virtual void solveArm(const FSVRAdapterSolverContext& context,
                          FSVRAdapterSide side,
                          const LLQuaternion& shoulder_world_rotation,
                          const LLVector3& shoulder_world_position,
                          std::map<S32, LLQuaternion>& out_target_worlds) = 0;
};

// Package A runtime solvers: no constraint, ever. The real C6/C7
// solvers land with their own components; until then even a fully
// gated apply computes an empty write list.
class FSVRAdapterNullSpineSolver final : public FSVRAdapterSpineSolver
{
public:
    void solveSpine(const FSVRAdapterSolverContext& /*context*/,
                    std::map<S32, LLQuaternion>& /*out_target_worlds*/) override {}
};

class FSVRAdapterNullArmSolver final : public FSVRAdapterArmSolver
{
public:
    void solveArm(const FSVRAdapterSolverContext& /*context*/,
                  FSVRAdapterSide /*side*/,
                  const LLQuaternion& /*shoulder_world_rotation*/,
                  const LLVector3& /*shoulder_world_position*/,
                  std::map<S32, LLQuaternion>& /*out_target_worlds*/) override {}
};

// ---------------------------------------------------------------------------
// The C7/RP3 write-through boundary (brief section 5, OQ-2 [R1];
// code cross-check round-1 amendment).
//
// RP3 does not fit the target-world boundary above: LLJointSolverRP3
// returns nothing -- it WRITES rotations (lljointsolverrp3.cpp:390-391),
// and its setWorldRotation path derives locals from the parent's
// CACHED world matrix (lljoint.cpp:834), which inside the apply is
// the previous cycle's (brief section 2.6). The brief resolved OQ-2
// by containment, not by a compute-only redesign of RP3: RP3's
// writes are provisional solver operations inside C1's synchronous
// transaction (conditions at brief section 3.4 step 5 [R1]). This
// boundary is the mechanism for that resolution; a future C7 built
// on RP3 implements THIS interface for its RP3 pass, and may not
// claim the brief by implementing the plain target-world interface
// alone.
//
// The contract, as obligations of the boundary:
// (a) PROVISIONAL ONLY. A solver operating through the host writes
//     provisional locals. Nothing it writes is a committed output;
//     C1 owns the final committed output (OQ-2 [R1] terminology).
// (b) ALWAYS-CURRENT PARENT WORLDS (containment condition (i)).
//     The host keeps no cached worlds across a provisional write:
//     every world read is composed from the current provisional
//     locals, so a parent-world read after a provisional
//     parent-changing write necessarily reflects it. This is the
//     refresh condition discharged by construction rather than by
//     refresh bookkeeping -- there is no cache that could go stale.
// (c) NON-OBSERVABILITY / ATOMICITY (containment condition (ii)).
//     Provisional state lives only inside the synchronous
//     transaction. If the solver does not complete, the transaction
//     discards the provisional state for that side entire: the
//     side's joints keep the values the transaction had already
//     resolved for them, and no provisional local reaches the
//     commit. In the viewer this is structural -- the glue performs
//     committed writes only from the finished plan result, after
//     updateMotions' blend, in the same synchronous call -- and at
//     plan level it is the discard rule above.
// (d) READBACK, POST-CLAMP, COMMIT. On completion the transaction
//     reads the provisional locals back, applies the arm post-clamp
//     (C0 sections 5.2/5.3, via FSVRAdapterArmPostClamp) to the
//     readback, recomputes the wrist-position error from the FINAL
//     clamped rotations (the values C1 commits; brief section 3.4,
//     H7 extension), and commits through the ordinary gate-5
//     eligibility filter. A readback local for an ineligible joint
//     is dropped, exactly as a rogue target is.
//     The post-clamp is MANDATORY for a write-through completion
//     (code cross-check round 2, R2-01): a write-through solver
//     supplied without a post-clamp fails closed -- that side's
//     write-through result is aborted before the solver runs, the
//     side keeps the pose the transaction already resolved for it,
//     and the side's outcome names the missing clamp
//     (FSVRAdapterArmAbortReason::MISSING_POST_CLAMP). There is no
//     unclamped write-through transaction.
//     Gate-5 rejection CASCADES over the readback (round 2,
//     R2-02): a provisionally written joint's readback local was
//     computed against its provisional parents, so if a written
//     joint is rejected at gate 5, every provisionally written
//     descendant of it in the same pass is rejected too -- its
//     readback depended on a parent the commit did not take.
//     Rejected joints keep the locals the transaction resolved
//     for them before the arm phase.
// ---------------------------------------------------------------------------
class FSVRAdapterWriteThroughHost
{
public:
    virtual ~FSVRAdapterWriteThroughHost() = default;

    // Provisional write of one joint local. Only the joints of the
    // side's arm chain (the domain the host was opened for) may be
    // written; any other joint -- another chain's, a collar's, the
    // head's or neck's -- is rejected (returns false) and nothing
    // is recorded. A provisional write is never a committed output
    // (contract point (a)).
    virtual bool writeProvisionalLocal(S32 joint_index,
                                       const LLQuaternion& local) = 0;
    // The current provisional local of a joint (its transaction
    // value if written, the value the transaction resolved for it
    // at host opening otherwise).
    virtual bool readProvisionalLocal(S32 joint_index,
                                      LLQuaternion& out_local) const = 0;
    // World rotation/position composed from the CURRENT provisional
    // state (contract point (b)).
    virtual bool readWorldRotation(S32 joint_index,
                                   LLQuaternion& out_rotation) const = 0;
    virtual bool readWorldPosition(S32 joint_index,
                                   LLVector3& out_position) const = 0;
};

// What a write-through solve reports back to the transaction.
struct FSVRAdapterWriteThroughOutcome
{
    // False aborts the transaction for this side: the provisional
    // state is discarded entire (contract point (c)).
    bool completed = false;
    // The wrist world position the solve aimed at, for the
    // post-clamp wrist-error recompute (contract point (d)).
    bool has_wrist_target = false;
    LLVector3 wrist_target_world = LLVector3::zero;
    // The pole the solver actually selected this call [R2 --
    // C6C7-03 transport]: has_pole is set ONLY when the RP3
    // procedure ran and selected a pole; the aim/flex modes and
    // the no-target return leave it false. pole_world is a world
    // direction. The transaction forwards it to the side's arm
    // outcome only after the side has been accepted (post-clamp
    // and commit processing complete), so a discarded side
    // exposes no provisional pole.
    bool has_pole = false;
    LLVector3 pole_world = LLVector3::zero;
};

// A write-through arm solver: C7's RP3 pass, when C7 lands. Runs
// INSIDE the transaction (boundary contract above), after the
// shoulders have been resolved from the solved spine; it operates
// on the side's chain through the host and reports an outcome.
// It never re-opens the spine (D6).
class FSVRAdapterWriteThroughArmSolver
{
public:
    virtual ~FSVRAdapterWriteThroughArmSolver() = default;
    virtual FSVRAdapterWriteThroughOutcome solveArmWriteThrough(
        const FSVRAdapterSolverContext& context,
        FSVRAdapterSide side,
        FSVRAdapterWriteThroughHost& host) = 0;
};

// The arm post-clamp (C0 sections 5.2/5.3), invoked BY the
// transaction on the write-through readback, before commit (brief
// section 3.4: C1's ordering duty is that the post-RP3 clamp runs
// before the arm locals commit). The clamp implementation itself
// is C7's (the brief: C6 and C7 call the C0 limiter inside their
// own compute); this interface is where C7's clamp plugs into the
// transaction so the ordering is the transaction's guarantee, not
// the solver's promise. It receives the readback locals and the
// worlds composed from them, and edits the readback locals in
// place; the transaction recomposes from the edited locals.
// A write-through transaction without a clamp does not run at
// all: the clamp is mandatory, not optional (contract point (d),
// R2-01 fail-closed rule).
class FSVRAdapterArmPostClamp
{
public:
    virtual ~FSVRAdapterArmPostClamp() = default;
    virtual void clampArm(
        const FSVRAdapterSolverContext& context,
        FSVRAdapterSide side,
        const std::map<S32, LLQuaternion>& readback_world_rotations,
        const std::map<S32, LLVector3>& readback_world_positions,
        std::map<S32, LLQuaternion>& in_out_readback_locals) = 0;
};

struct FSVRAdapterWrite
{
    S32 joint_index = -1;
    LLQuaternion local; // the committed local rotation
};

// Why a side's write-through result was aborted (boundary
// contract points (c)/(d)). Surfaced on the plan result -- the
// plan layer's diagnostic surface, traced the same way
// FSVRAdapterGate is (C9) -- and named by
// FSVRAdapterApply::armAbortReasonName.
enum class FSVRAdapterArmAbortReason
{
    NONE = 0,
    SOLVER_INCOMPLETE,  // the solver did not complete; its
                        // provisional state was discarded entire
    MISSING_POST_CLAMP  // a write-through solver was supplied
                        // without the mandatory post-clamp; the
                        // side failed closed before the solver ran
                        // (R2-01)
};

// Per-side write-through outcome (boundary contract point (d)).
// wrist_error is the distance between the wrist world position
// composed from the final clamped rotations C1 committed and the
// target the solver reported -- the accepted error of the clamp
// (spec section 6.2/6.3: constraints win). The wrist is the
// structural joint whose parent is the side's elbow; when the
// chain carries no wrist joint, or the solver reported no target,
// wrist_error_valid is false.
struct FSVRAdapterArmOutcome
{
    bool write_through_ran = false;
    bool write_through_aborted = false;
    FSVRAdapterArmAbortReason abort_reason = FSVRAdapterArmAbortReason::NONE;
    bool wrist_error_valid = false;
    F32 wrist_error = 0.f;
    // The pole the caller adopts as this side's history for the
    // next cycle [R2 -- C6C7-03 transport]: pole_valid is set
    // ONLY when the side ran, was not aborted, its post-clamp
    // ran, and its result was committed -- the assignment
    // happens at the end of the side's transaction processing,
    // never mechanically beside the solver's return. An aborted
    // or discarded side exposes no provisional pole.
    bool pole_valid = false;
    LLVector3 pole_world = LLVector3::zero;
};

struct FSVRAdapterPlanResult
{
    // True when the transaction ran (every gate passed), even if
    // the solvers returned no constraints and nothing was written.
    bool applied = false;
    // P2-07B Fix 1 (CP-F1-1 step 5 [R4]): the head-group
    // combined-bound selection refused this cycle's candidate
    // (the base head frame beyond the two head-group locals'
    // combined reach from the committed parent), and the
    // transaction was rejected entire: writes is empty, the
    // resolved arrays report the base composition, and the arm
    // outcomes' committed claims (pole_valid, wrist_error_valid)
    // are retracted. applied stays true and first_failing_gate
    // stays NONE -- the transaction ran; this field names why
    // it did not commit, in the arm outcomes' diagnostic idiom
    // (a ran transaction's discarded outcome is named on the
    // result, not on the gate enum, which names why a
    // transaction did not run).
    bool head_group_refusal = false;
    FSVRAdapterGate first_failing_gate = FSVRAdapterGate::NONE;
    bool spine_solver_invoked = false;
    bool arm_solver_invoked_left = false;
    bool arm_solver_invoked_right = false;
    // The committed writes, in D6 order (spine pass, then arms).
    std::vector<FSVRAdapterWrite> writes;
    // Per input joint: the final local rotation (the write local
    // where written, the base local otherwise) and the composed
    // world rotation/position after the transaction.
    std::vector<LLQuaternion> resolved_locals;
    std::vector<LLQuaternion> resolved_world_rotations;
    std::vector<LLVector3> resolved_world_positions;
    // P2-07B DIAG-1 (D-1, Route A -- settled): the plan's
    // base composition, retained verbatim on the result.
    // computePlan composes the base worlds in every outcome
    // (before any gate) and copies them here before any
    // committed-state modification; they remain unchanged
    // on gate failures and on whole-transaction refusal.
    // Plan-internal outputs, not inputs; the bonelog
    // recorder reads them from the result it already
    // receives.
    std::vector<LLQuaternion> base_world_rotations;
    std::vector<LLVector3> base_world_positions;
    // P2-07B Fix 4 (CP-F4-3, instrumentation only): the
    // spine solver's returned target worlds, retained
    // verbatim -- assigned in computePlan immediately after
    // solveSpine returns, before any clamping or
    // commit-local conversion -- so the recorder can carry
    // them (the bonelog's optional solver_targets array).
    // Empty whenever the solver was not invoked (gate
    // failures, the hold branch, no spine authority).
    // Nothing in the plan reads this field; retaining it
    // changes no behaviour (F4-Z1 A/B).
    std::map<S32, LLQuaternion> solver_target_worlds;
    // P2-07B Fix 3 (brief section 4): the plan input's
    // posture behaviour and computed root offset, retained
    // verbatim for the recorder (the bonelog plan block's
    // optional keys). Copied in computePlan beside the
    // base composition, before any gate: they describe
    // the cycle's composition facts, not its outcome, and
    // remain unchanged on gate failures.
    FSVRPostureBehaviour posture_behaviour = FSVRPostureBehaviour::NONE;
    bool posture_behaviour_present = false;
    bool root_offset_present = false;
    LLVector3 root_offset = LLVector3::zero;
    // P2-07B Fix 3 R5 (brief section 8.1): the plan input's
    // R5 recorder facts, retained verbatim (see the input).
    FSVRAvatarPostureState avatar_state = FSVRAvatarPostureState::UNKNOWN;
    bool avatar_state_present = false;
    bool root_follow_suspended = false;
    bool home_base_valid = false;
    U32 home_base_generation = 0;
    bool posture_swap_configured = false;
    bool posture_swap_inert = false;
    // Write-through boundary outcomes, per side (zero when the
    // boundary did not run for that side).
    FSVRAdapterArmOutcome arm_outcome_left;
    FSVRAdapterArmOutcome arm_outcome_right;
};

// P2-07B C6-E (CP-2): the E5-E9 conjunction as a named pure
// function, so H10 tests the production composition, never a
// reimplementation (Package B spec section 9). Defined above
// the plan-only line in llvradapterapply.cpp with the other
// extracted plan helpers.
// == in.scale_input_valid && in.neutral_established
//  && in.hmd_position_valid && in.sitting_signals_clear
//  && !in.calibration_in_progress
bool decideSpineSampleValid(const FSVRSpineSampleInputs& in);

// ---------------------------------------------------------------------------
// P2-07B Fix 3 (brief sections 2.2/2.4/2.6): the posture
// cycle's inputs and outputs, and the pure channel seams
// the composition and the glue share. One implementation
// of each fact, consumed by the composition, by the
// glue's root application, and by the harnesses -- never
// reimplemented at a call site.
// ---------------------------------------------------------------------------

// The cycle's Fix 3 facts beyond the shipped
// composition's own parameters: the swap and the Head
// position toggle from the settings snapshot, both
// channels' gains with their validities (the shipped
// gain contract: invalid is rejected, never clamped,
// and the channel fails closed), and the two channels'
// manual limb-ratio states (section 2.4: a VALID
// manual ratio replaces the channel's derived ratio
// and the gain still multiplies it; an INVALID one
// fails the channel closed). The lean gain's VALUE
// reaches the composition through
// composeHeadTargetBlock's lean_gain parameter (the
// shipped Stage 1 surface); its VALIDITY rides here
// [FIX3-CODE-R1-02] so the lean factor decision itself
// fails closed on an invalid gain instead of consuming
// the snapshot's pinned 1.0 as a valid value.
// Production populates the manual states from the
// snapshot's per-channel derivations over the global
// settings FSVRManualLeanRatio / FSVRManualRootRatio
// [FIX3-CODE-R1-01] (per-avatar profile storage
// remains redesign Stage 2 machinery, outside this
// brief; the seam is where a future source lands).
struct FSVRFix3CycleInputs
{
    // P2-07B Fix 3 R5 (brief sections 2.1-2.3): the cycle's
    // sampled avatar state, classified by the glue from the
    // live avatar at the apply point. avatar_state_present
    // is the staged-contract flag (the posture_behaviour_
    // present idiom): production glue always sets it; when
    // it is false the cycle's behaviour derives from the
    // session's sync stamp and the swap exactly as R3
    // shipped (the Package B / Fix 4 contract, untouched).
    // The default-constructed state is UNKNOWN, the
    // fail-closed one.
    FSVRAvatarPostureState avatar_state =
        FSVRAvatarPostureState::UNKNOWN;
    bool avatar_state_present = false;
    bool posture_swap = false;          // FSVRPostureSwap
    bool head_position_enabled = false; // FSVRTrackHeadPosition
    bool lean_gain_valid = true;        // FSVRLeanResponseGain
    F32  root_gain = 1.f;               // FSVRRootResponseGain
    bool root_gain_valid = true;
    FSVRManualRatioState manual_lean = FSVRManualRatioState::ABSENT;
    F32  manual_lean_ratio = 0.f;
    FSVRManualRatioState manual_root = FSVRManualRatioState::ABSENT;
    F32  manual_root_ratio = 0.f;
    // OQ-2's ruled boundary policy [FIX3-CODE-R3]: the
    // root-follow bound from the snapshot
    // (FSVRRootFollowBoundMeters), in PHYSICAL metres on
    // the pre-scale displacement from the sync anchor,
    // with its validity. The policy is in force only
    // when root_follow_bound_valid: the standing step
    // then radially clamps the consumed displacement to
    // root_follow_bound_m before scaling, the
    // composition suppresses translational solver input,
    // and the glue may write the agent position. When
    // the bound is invalid the policy is NOT in force:
    // no write, no suppression -- the standing
    // composition keeps the shipped lean behaviour
    // [FIX3-CODE-R2-01]. The default-constructed fact
    // set (no policy) is the fail-closed one.
    F32  root_follow_bound_m = 0.f;
    bool root_follow_bound_valid = false;
    // P2-07B Fix 3 R5 (brief section 4.3): root-follow is
    // suspended (a teleport discontinuity latched the
    // watch). Suspension withholds the WRITE and the
    // composition's standing suppression, exactly as the
    // boundary policy not being in force does
    // [FIX3-CODE-R2-01]: the offset is still computed and
    // recorded, and the shipped lean continues -- a gate
    // withholds the write; it does not authorize
    // suppressing the existing positional response while
    // the avatar cannot move.
    bool root_follow_suspended = false;
    // P2-07B Fix 3 R5 [FIX3-R5-CODE-02]: the teleport
    // detector's measured-band readiness. Consulted ONLY
    // when avatar_state_present (the R5 path): while
    // false, live root-follow stays off -- the step is
    // not live and the composition falls through exactly
    // as bound-gated -- because the legitimate/teleport
    // displacement bands have not yet been measured and
    // proven separable. The legacy path never consults
    // it and keeps the R3 contract bit-for-bit.
    bool detector_proven = false;
};

// What the Fix 3 composition determined for the cycle,
// beyond the head target itself. behaviour is the
// effective posture behaviour (the pure function of the
// view's posture and the swap). root_offset is the
// standing channel's computed offset (ScaleRoot of the
// anchor-relative displacement state -- of the
// OQ-2-clamped displacement when the boundary policy
// is in force [FIX3-CODE-R3]), meaningful only
// when root_offset_valid: standing behaviour in force,
// Head position ON, and the root channel's factor
// applied. It is computed and recorded in every such
// state, including while the live write is gated off
// (the policy not in force): the record describes the
// channel's determination, not the write's outcome.
// In every other state root_offset is exactly
// zero and root_offset_valid is false (sitting: F3-R2;
// undetermined: section 2.5; Head position OFF: section
// 0.4 of the brief). lean_scale_invalid /
// root_scale_invalid name the channels an invalid scale
// parameter failed closed this cycle (section 2.4): the
// glue turns each into its scale_parameter_invalid
// bonelog event, edge-latched.
struct FSVRFix3CompositionResult
{
    FSVRPostureBehaviour behaviour = FSVRPostureBehaviour::NONE;
    bool root_offset_valid = false;
    LLVector3 root_offset = LLVector3::zero;
    bool lean_scale_invalid = false;
    bool root_scale_invalid = false;
};

// One channel's factor decision (brief section 2.4):
// factor is the channel's total F(c) = ratio x gain,
// composed in F32, ready for the single F32
// multiplication of the mapped displacement. applied is
// false when the channel is off -- invalid is then true
// when an invalid scale parameter (not a mere absence)
// is what took it out.
struct FSVRChannelFactor
{
    bool applied = false;
    bool invalid = false;
    F32  factor = 1.f;
};

// The lean channel's factor for an establishment: the
// measured establishment's ratio is its record's
// head_ratio (the matched pair's own value), the
// provisional baseline otherwise; a valid manual ratio
// replaces either; FSVRDiagForceUnitScale forces the
// derived ratio to 1.0 and never touches a manual ratio
// or a gain (the shipped diagnostic meaning). The gain
// multiplies on EVERY path now (brief section 2.4: the
// Stage 1 asymmetry -- gain on the provisional path
// only -- is absorbed into the contract). An invalid
// lean gain (inputs.lean_gain_valid false) fails the
// channel closed ahead of every ratio source: the
// snapshot's pinned substitute value is never consumed
// as a valid gain [FIX3-CODE-R1-02].
FSVRChannelFactor fsvrLeanChannelFactor(
    const FSVRSessionNeutralView& session, F32 lean_gain,
    bool diag_force_unit_scale,
    const FSVRFix3CycleInputs& inputs);

// The root channel's factor: no proportional contract
// is specified for it in this brief (OQ-7), so its ratio
// is the provisional baseline 1.0, or a valid manual
// ratio (proportional by declaration). The gain is the
// snapshot's root gain under its validity flag.
FSVRChannelFactor fsvrRootChannelFactor(
    const FSVRFix3CycleInputs& inputs);

// The shipped displacement mapping, factored out of the
// composition so the glue's root step and the
// composition read the identical value: the
// anchor-relative LOCAL displacement mapped through the
// rotation path's vector form (P, the session's yaw-only
// q_yaw, the root frame's rotation). Pure over its
// arguments; no factor is applied here.
LLVector3 fsvrMapAnchorDisplacement(
    const FSVRSessionNeutralView& session,
    const LLVector3& hmd_position_local,
    F32 session_base_yaw_rad,
    const LLQuaternion& root_world_rotation);

// The standing root step for one cycle, as one pure
// decision: the behaviour in force, and -- only when
// the standing behaviour is in force and Head position
// is ON -- the root channel's offset for the cycle
// (ScaleRoot of the current anchor-relative
// displacement; OQ-1's follow form, Leah's 2026-11-07
// ruling). Since [FIX3-CODE-R3] the displacement the
// offset is computed from is the OQ-2-clamped one when
// the boundary policy is in force (radial clamp to the
// bound, applied before scaling; bound_clamped names a
// cycle the clamp actually reduced). A failed-closed
// root channel yields offset_valid false with
// root_scale_invalid set: the cycle's displacement is
// then consumed NOWHERE, never handed to the solver as
// a remainder (brief section 2.2 item 1's stated
// invariant). live is the step's write authorization:
// offset_valid AND the boundary policy in force -- the
// one fact the glue's write decision and the harness
// rig's application both read. The glue applies a live
// offset through the agent movement channel before
// base composition; the composition recomputes the
// identical value for the record.
struct FSVRStandingRootStep
{
    FSVRPostureBehaviour behaviour = FSVRPostureBehaviour::NONE;
    bool offset_valid = false;
    LLVector3 root_offset = LLVector3::zero;
    bool root_scale_invalid = false;
    bool bound_clamped = false;
    bool live = false;
};

// The one derivation of a cycle's effective posture
// behaviour, consumed by both Fix 3 seams (the standing
// root step and the head-target composition) so they can
// never disagree: with the R5 sampled state present, the
// R5 selector over that state; otherwise the R3 derivation
// from the session's sync stamp and the swap, verbatim.
// FSVRPostureSwap is recorded-but-inert on the R5 path
// (implementation plan P4): it is read for the record and
// changes no behaviour.
FSVRPostureBehaviour fsvrCycleBehaviour(
    const FSVRSessionNeutralView& session,
    const FSVRFix3CycleInputs& inputs);

FSVRStandingRootStep fsvrComputeStandingRootStep(
    const FSVRSessionNeutralView& session,
    const LLVector3& hmd_position_local,
    F32 session_base_yaw_rad,
    const LLQuaternion& root_world_rotation,
    const FSVRFix3CycleInputs& inputs);

// P2-07B C6-E (CP-4): the per-cycle head-target composition
// (Package B spec section 7.3), pure over its named inputs.
// Returns false -- and composes nothing -- unless every input
// the composition needs is present for this cycle (the
// session neutral established). On true, out_head_target_world
// is C6's shipped composeHeadTarget applied to the neutral
// cornea re-expressed in the CURRENT root frame plus the HMD
// displacement from the session anchor mapped through the
// rotation path's vector form,
//   d_world = (P(d_local) * q_yaw) * root_world_rotation,
// P(dx, dy, dz) = (-dz, -dx, dy) (brief section 2.13), and
// out_cornea_offset_head_local is the session's section 4.6
// value verbatim. Defined beside decideSpineSampleValid.
// P2-07B Stage 1 (CP-S1-3): lean_gain is the settings
// snapshot's provisional lean gain, consumed only when the
// session's scale source is PROVISIONAL_SYNC -- it
// multiplies the mapped displacement before the solver,
// and the composition's ratio argument is selected by the
// scale source (the record's head_ratio for MEASURED_RECORD,
// the provisional baseline constant otherwise).
// FSVRDiagForceUnitScale keeps its shipped meaning (it
// forces the *calibration* ratio to 1.0); the gain is a
// user preference and survives the diagnostic.
// P2-07B Fix 3 (brief sections 2.2/2.4): the composition
// is extended, not replaced. The lean channel's factor
// (fsvrLeanChannelFactor: ratio x gain, composed in F32)
// is applied as ONE F32 multiplication of the mapped
// displacement, in the shipped lean_gain position, on
// every path -- the measured path's gain-blindness is
// superseded by the brief's contract (its gain now
// applies as the contract's k; at factor exactly 1.0 the
// composed target is bit-identical to the shipped
// composition, F3-S2). Under the standing behaviour WITH
// OQ-2's boundary policy in force [FIX3-CODE-R3] the
// root channel consumes the (clamped) displacement
// entire and the solver's translational input is ZERO
// by stated invariant (never a computed residual);
// under standing WITHOUT the policy in force the
// composition is the shipped lean one, exactly as
// under sitting or undetermined [FIX3-CODE-R2-01].
// fix3_inputs == nullptr runs
// the cycle with the brief's default fact set (no swap,
// Head position OFF for the root step, gains 1.0 valid,
// no manual ratios); fix3_result, when non-null,
// receives the cycle's Fix 3 determinations.
bool composeHeadTargetBlock(
    const FSVRSessionNeutralView& session,
    const LLVector3& hmd_position_local,
    F32 session_base_yaw_rad,
    const LLQuaternion& root_world_rotation,
    const LLVector3& root_world_position,
    bool diag_force_unit_scale,
    F32 lean_gain,
    LLVector3& out_head_target_world,
    LLVector3& out_cornea_offset_head_local,
    const FSVRFix3CycleInputs* fix3_inputs = nullptr,
    FSVRFix3CompositionResult* fix3_result = nullptr);

// ---------------------------------------------------------------------------
// P2-07B Fix 4 (brief sections 2.2/2.3/2.5): base
// provenance -- the pure seam the glue drives so the plan
// is presented a base that excludes the adapter's own
// prior commits (INV-F4-01). Per tracked joint the store
// keeps three distinct facts, never conflated:
//   anim_local:          the joint's animation-attributed
//                        local -- the value presented as
//                        the base. Initially the joint's
//                        rest/current local (seeded before
//                        its first adapter write, F4-S6).
//   commit_local:        the adapter's most recent write.
//                        Authorship history only: read by
//                        nothing but diagnostics.
//   expected_live_local: the live skeleton's expected
//                        local as of the last classified
//                        blend result or applied write --
//                        the LINEAGE live locals are
//                        compared against. It is the only
//                        comparison reference; it is never
//                        the last commit after a blend has
//                        intervened [R2].
// Classification of a joint's cycle comes from the blend
// path's own observation (the generalized DIAG-1 sink),
// never from comparing quaternions for equality: equality
// cannot distinguish "the animation stack produced the
// commit's value afresh" from "nothing overwrote the
// commit" [R1].
// ---------------------------------------------------------------------------
enum class FSVRBaseBlendClass
{
    NO_WRITE,       // the blend did not write the joint
    SEED_DISCARDED, // a seed-discarding write: the live
                    // local is animation output entire
    SEED_PRESERVED  // an additive-only write: the live
                    // local factors as added_rot * seed
};

// One joint's blend outcome for one cycle, as the glue
// reads it from the generation-validated blend record.
// present is false when the record carries no current
// entry for the joint (not written this pass -- which the
// caller must not confuse with a classified NO_WRITE for
// a joint the store already tracks; see
// provenanceAssembly's fail-closed rule).
struct FSVRBaseBlendFact
{
    bool present = false;
    FSVRBaseBlendClass blend_class = FSVRBaseBlendClass::NO_WRITE;
    LLQuaternion seed_local;
    LLQuaternion added_rot;
    LLQuaternion result_local;
};

struct FSVRBaseProvenanceEntry
{
    bool has_commit = false;
    LLQuaternion anim_local;
    LLQuaternion commit_local;
    LLQuaternion expected_live_local;
};

// Keyed by joint name (as the bonelog header's plan-joint
// list is keyed): a plan-list index shift between cycles
// can never misattribute provenance.
struct FSVRBaseProvenanceStore
{
    std::map<std::string, FSVRBaseProvenanceEntry> entries;
    // Provenance of joints temporarily absent from the
    // presented set, preserved under the same character /
    // skeleton ownership [FIX4-CODE-R1-02]: excluded from
    // active processing while absent, restored verbatim
    // on re-entry, and cleared wholesale with the store
    // on an ownership reset (provenanceReset). Bounded
    // by the set of names ever presented under this
    // ownership.
    std::map<std::string, FSVRBaseProvenanceEntry> dormant;
};

enum class FSVRBaseProvenanceEventKind
{
    RESET,
    UNCLASSIFIED
};

// A provenance event the glue surfaces on the bonelog.
// joint_name empty names the whole store (a wholesale
// reset). reason carries the lifecycle cause for RESET
// ("character_change", "skeleton_reinit", "membership",
// "record_unavailable", "record_inconsistent");
// divergence_deg carries the seed-gate divergence for
// UNCLASSIFIED.
struct FSVRBaseProvenanceEvent
{
    FSVRBaseProvenanceEventKind kind =
        FSVRBaseProvenanceEventKind::RESET;
    std::string joint_name;
    std::string reason;
    F32 divergence_deg = 0.f;
};

class FSVRAdapterApply
{
public:
    // The production cycle classification, factored out of the
    // glue so the H1 harness covers the mapping itself and not
    // only the classification values the gates consume: the
    // Animation Preview flag dominates; a paused controller
    // reaching the post-blend hook got there through FORCE_UPDATE.
    // (A paused controller's ordinary update, and hidden updates,
    // never reach the hook at all; their kinds exist for the
    // gate's defense in depth and are fed by the harness only.)
    static FSVRAdapterCycleKind classifyCycle(bool preview_cycle,
                                              bool controller_paused);

    // P2-07B C5 (CP-5): the calibration hold set -- spec
    // section 3.1's in-scope joints by skeleton joint name
    // (the D7 spine chain, mNeck, mHead, and per side
    // collar/shoulder/elbow/wrist; fingers are not held).
    // The canonical list lives in this TU so the plan's
    // hold branch and C5's transaction machinery
    // (FSVRCalibration::holdJointNames forwards here) can
    // never drift, and so the plan keeps its one-directional
    // dependency shape (C5 depends on the plan, never the
    // reverse). Any addition is a spec amendment.
    static const std::vector<std::string>& calibrationHoldJointNames();

    // P2-07B C8 (CP-3): the glue's continuity predicate,
    // extracted verbatim from FSVRAdapterApplyForCharacter so
    // H8 fixtures the production expression (the house pattern:
    // C4's couplingWouldFire, C5's exposed helpers). Continuity
    // carries iff a previous apply exists, the frame token is
    // its immediate successor, and BOTH the settings generation
    // and the continuity epoch are unchanged since it. The
    // epoch term is C8's addition: a mismatch retires both
    // histories pre-transaction, through the glue's existing
    // whole-history clear, whether or not a solver is invoked.
    // P2-07B Stage 1 (CP-S1-2, S1-BRIEF-R1-01): the
    // installation-generation term is Stage 1's addition,
    // on the same terms -- the carried history's install
    // generation (the session view's, recorded when the
    // history was stored) must equal the current view's
    // install generation. A resync replaces the neutral the
    // history was produced against, so unchanged token,
    // settings generation, and epoch do NOT make old-anchor
    // history a valid continuity witness under a new one.
    // C8's epochs remain independently authoritative; this
    // is an additional necessary condition, never a
    // replacement.
    static bool continuityCarries(bool have_previous,
                                  U64 frame_token, U64 previous_token,
                                  U64 settings_generation,
                                  U64 previous_settings_generation,
                                  U64 continuity_epoch,
                                  U64 previous_continuity_epoch,
                                  U64 install_generation,
                                  U64 previous_install_generation);

    // P2-07B Fix 4 (CP-F4-1): the pure base-provenance
    // seam. Defined above the plan-only line beside
    // continuityCarries; the glue owns the store and the
    // lifecycle detection, these functions own the rules.
    //
    // provenanceReset clears the store (active and
    // dormant entries alike) and seeds every
    // listed joint from its base_local (at reset time the
    // joint's current local, read by the caller before any
    // adapter write of the new ownership), emitting ONE
    // wholesale RESET event (empty joint name).
    static void provenanceReset(
        FSVRBaseProvenanceStore& store,
        const std::vector<FSVRAdapterJointInput>& joints,
        const char* reason,
        std::vector<FSVRBaseProvenanceEvent>& events);
    //
    // provenanceAssembly runs one cycle's classification.
    // On entry each joint's base_local is its LIVE local
    // (the glue's population read); on exit, a tracked
    // joint's base_local is its animation-attributed local
    // (the presented base) and an untracked joint's is its
    // live local, adopted as its tracked value from this
    // cycle. facts is parallel to joints. Membership is
    // reconciled first: a stored joint absent from the
    // list leaves the active set with a per-joint RESET
    // event ("membership") and its provenance moves to
    // the store's dormant map, preserved under the same
    // ownership [FIX4-CODE-R1-02]; a listed joint with a
    // dormant entry is restored from it, never re-seeded
    // from a live local that may still be an old commit;
    // a listed joint with no entry at all is seeded
    // silently from its live local (sound only because
    // first sight under an ownership precedes any adapter
    // write to it, F4-S6). A TRACKED joint
    // whose fact is unavailable (record_valid false, or
    // the entry for it absent) fails closed WITHOUT
    // losing provenance [FIX4-CODE-R1-01]: the RESET
    // event ("record_unavailable" /
    // "record_inconsistent") is emitted, the trusted
    // animation local is presented, and the commit
    // history and live lineage are retained untouched --
    // a stale or absent observation is never silently
    // read as NO_WRITE [R3 verdict safeguard], and it
    // never re-seeds from a live local that may be the
    // adapter's own commit.
    static void provenanceAssembly(
        FSVRBaseProvenanceStore& store,
        std::vector<FSVRAdapterJointInput>& joints,
        const std::vector<FSVRBaseBlendFact>& facts,
        bool record_valid,
        std::vector<FSVRBaseProvenanceEvent>& events);
    //
    // provenanceCommitWrites refreshes the store for the
    // writes a plan result actually applied (the caller
    // passes result.writes verbatim, after applying them):
    // per written joint, has_commit is set and both
    // commit_local and expected_live_local take the write
    // value, in write order (last wins, as the skeleton
    // applies them). Gate-failed, token-suppressed and
    // refused results carry empty writes and refresh
    // nothing -- exactly the brief's rule.
    static void provenanceCommitWrites(
        FSVRBaseProvenanceStore& store,
        const std::vector<FSVRAdapterJointInput>& joints,
        const std::vector<FSVRAdapterWrite>& writes);

    // The pure plan (brief section 4). No globals, no cross-cycle
    // pose state: identical inputs give identical outputs.
    static FSVRAdapterPlanResult computePlan(
        const FSVRAdapterPlanInput& input,
        FSVRAdapterSpineSolver& spine_solver,
        FSVRAdapterArmSolver& arm_solver);

    // The plan with the C7/RP3 write-through boundary (above).
    // When write_through_solver is non-null it takes the arm phase
    // for both sides (the plain arm solver is not invoked), and
    // post_clamp is then MANDATORY: a null post_clamp fails each
    // side closed (contract point (d), R2-01) -- the solver is not
    // run, nothing from the side reaches the commit, and the
    // side's outcome names MISSING_POST_CLAMP. There is no
    // unclamped write-through path, in fixtures or otherwise.
    // With a null write_through_solver this is exactly the
    // three-argument plan.
    static FSVRAdapterPlanResult computePlan(
        const FSVRAdapterPlanInput& input,
        FSVRAdapterSpineSolver& spine_solver,
        FSVRAdapterArmSolver& arm_solver,
        FSVRAdapterWriteThroughArmSolver* write_through_solver,
        FSVRAdapterArmPostClamp* post_clamp);

    // Brief section 3.3 gate 5, exposed for the harness. A joint is
    // eligible when it is registered, belongs to a solver domain,
    // and either arbitration grants it a tracking base or the
    // domain solver's gate-4 constraint authority covers it; a
    // disabled joint is eligible only under the spine-only L4
    // exception.
    static bool isJointEligible(const FSVRAdapterPlanInput& input,
                                const FSVRAdapterJointInput& joint,
                                bool spine_authority,
                                bool arm_authority);

    static const char* gateName(FSVRAdapterGate gate);
    static const char* armAbortReasonName(FSVRAdapterArmAbortReason reason);
};

// Token-once wrapper used by the viewer glue: each frame token is
// processed at most once (the "exactly once per motion cycle"
// mechanism, brief section 4), and a repeat returns a TOKEN no-op
// without invoking the solvers. Holds frame identity only, never
// pose state (brief section 3.6).
//
// INTEGRATION CHECK RECORDED (code cross-check round 1): the token
// is the coordinator's app_frame_token, not an independently
// generated motion-cycle identifier. Before runtime writes are
// enabled (C6/C7 integration), verify that an app frame corresponds
// 1:1 to self-avatar motion cycles on this path, or supply an
// independent motion-cycle id as the token.
class FSVRAdapterApplyOnce
{
public:
    FSVRAdapterPlanResult apply(const FSVRAdapterPlanInput& input,
                                FSVRAdapterSpineSolver& spine_solver,
                                FSVRAdapterArmSolver& arm_solver);
    // As apply(), with the write-through boundary (see the
    // five-argument computePlan). Token-once suppresses the
    // write-through solver exactly as it suppresses the others.
    FSVRAdapterPlanResult apply(const FSVRAdapterPlanInput& input,
                                FSVRAdapterSpineSolver& spine_solver,
                                FSVRAdapterArmSolver& arm_solver,
                                FSVRAdapterWriteThroughArmSolver* write_through_solver,
                                FSVRAdapterArmPostClamp* post_clamp);
    // Number of transactions that ran (diagnostics).
    U32 applyCount() const { return mApplyCount; }

private:
    bool mHasToken = false;
    U64 mLastToken = 0;
    U32 mApplyCount = 0;
};

class LLCharacter;

// Viewer glue (implementation at the bottom of
// llvradapterapply.cpp; excluded from FSVR_ADAPTER_PLAN_ONLY
// harness builds). Called by LLVOAvatarSelf::applyVRAdapterPostBlend
// exactly once per post-blend motion cycle. preview_cycle is the
// Animation Preview state the override reads from the avatar.
void FSVRAdapterApplyForCharacter(LLCharacter* character, bool preview_cycle);

#endif // LL_LLVRADAPTERAPPLY_H
