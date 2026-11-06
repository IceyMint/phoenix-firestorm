/**
 * @file llvrbonelog_state_test.cpp
 * @brief P2-07B bonelog standalone STATE test (BONELOG-R2-01
 *        / R2-02 evidence).
 *
 * Drives the recorder's REAL state machine -- the
 * production llvrbonelog.cpp compiled with
 * FSVR_BONELOG_TEST_RIG + FSVR_BONELOG_TEST_HOOKS against
 * the minimal viewer stand-ins in
 * llvrbonelog_test_stubs.h -- in two modes, each a
 * separate process invocation:
 *
 *   header     Forces a full ring across a simulated
 *              skeleton change and proves: (a) no sample
 *              under the new skeleton is written before
 *              its header; (b) recovery -- once capacity
 *              frees, the pending header lands and samples
 *              resume under the new schema; (c) the drop
 *              accounting is exact.
 *   writerfail Arms the injected writer exception and
 *              proves: no std::terminate (the process lives
 *              to report), writer_failed is set, the
 *              recorder reaches BS_DEAD, producers cease,
 *              and shutdown() joins the dead-but-joinable
 *              thread.
 *   writerfailrace
 *              BONELOG-R3-01 startup race: the writer is
 *              forced to fail immediately upon entry and a
 *              test-only barrier holds the startup caller
 *              inside ensureStarted until the failure has
 *              fully completed, so the failure lands
 *              entirely before the caller finishes. Proves
 *              the terminal BS_DEAD is not overwritten by
 *              a startup-side BS_RUNNING store, no producer
 *              restarts the writer (entry count stays 1),
 *              and shutdown() joins leaving BS_DEAD intact.
 *
 * Build/run: run_bonelog_state_test.sh (FS_VR_BONELOG=1 is
 * set by the runner). Exit code 0 iff every check in the
 * mode passes.
 */

#include "llvrbonelog.h"
#include "llvrbonelog_test_stubs.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace
{

int g_checks = 0;
int g_failures = 0;

void check(bool ok, const std::string& what)
{
    ++g_checks;
    if (!ok)
    {
        ++g_failures;
        std::cout << "FAIL: " << what << std::endl;
    }
}

bool waitFor(const std::function<bool()>& pred, int timeout_ms)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (pred())
        {
            return true;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(5));
    }
    return pred();
}

const char* const kBonelogPath = "/tmp/firestorm_bonelog.jsonl";

// BS_DEAD in llvrbonelog.cpp's BoneLogRunState (TU-local
// enum: NOT_STARTED=0, RUNNING=1, STOPPING=2, DEAD=3).
const int kRunStateDead = 3;

// A fake avatar skeleton on one LLCharacter; the skeleton
// "changes" by bumping the serial and rebuilding the joint
// tree under the same character object.
struct FakeSkeleton
{
    LLJoint root;
    LLJoint mid;
    LLJoint head;
    LLCharacter character;

    // Empty mid_name builds a two-joint skeleton.
    void build(const std::string& root_name,
               const std::string& mid_name,
               const std::string& head_name,
               U32 serial)
    {
        root.mName = root_name;
        root.mWorldPosition = LLVector3(1.f, 2.f, 3.f);
        root.mWorldRotation =
            LLQuaternion(0.f, 0.f, 0.f, 1.f);
        root.mChildren.clear();
        head.mName = head_name;
        head.mWorldPosition = LLVector3(1.f, 2.f, 4.f);
        head.mWorldRotation =
            LLQuaternion(0.f, 0.f, 0.f, 1.f);
        head.mChildren.clear();
        if (!mid_name.empty())
        {
            mid.mName = mid_name;
            mid.mWorldPosition = LLVector3(1.f, 2.f, 3.5f);
            mid.mWorldRotation =
                LLQuaternion(0.f, 0.f, 0.f, 1.f);
            mid.mChildren.clear();
            mid.mChildren.push_back(&head);
            root.mChildren.push_back(&mid);
        }
        else
        {
            root.mChildren.push_back(&head);
        }
        character.mRootJoint = &root;
        character.mSkeletonSerial = serial;
    }
};

void sample(FakeSkeleton& skel, U64 token)
{
    FSVRAdapterPlanInput input;
    input.frame_token = token;
    input.head_target_valid = false;
    input.root_world_position = LLVector3::zero;
    input.root_world_rotation =
        LLQuaternion(0.f, 0.f, 0.f, 1.f);
    FSVRAdapterPlanResult result;
    FSVRBoneLog::sampleCycle(&skel.character, input, result,
                             false, LLVector3::zero);
}

int summary(const char* mode)
{
    if (g_failures == 0)
    {
        std::cout << "BONELOG_STATE_TEST[" << mode
                  << "]: ALL " << g_checks << " CHECKS PASSED"
                  << std::endl;
        return 0;
    }
    std::cout << "BONELOG_STATE_TEST[" << mode << "]: "
              << g_failures << " FAILURE(S) of " << g_checks
              << " checks" << std::endl;
    return 1;
}

int runHeaderMode()
{
    check(FSVRBoneLog::enabled(),
          "recorder enabled (FS_VR_BONELOG set by runner)");

    FakeSkeleton skel;
    skel.build("mRoot", "", "mHead", 1);

    // Stall the writer BEFORE the first enqueue, so the
    // ring can be forced full deterministically.
    FSVRBoneLog::testSetWriterStall(true);

    // Produce 300 samples under skeleton A (2 joints). The
    // ring holds 256 records: header + 255 samples queue;
    // the remaining 45 samples drop.
    for (U64 token = 1; token <= 300; ++token)
    {
        sample(skel, token);
    }
    check(FSVRBoneLog::testQueuedCount() == 256,
          "ring full at 256 records while writer stalled");
    check(FSVRBoneLog::testDroppedCount() == 45,
          "exactly 45 samples dropped filling the ring");

    // Skeleton change under the same character: serial
    // bump, new joint set (3 joints, one new name).
    skel.build("mRoot", "mSpine", "mHead", 2);
    for (U64 token = 301; token <= 305; ++token)
    {
        sample(skel, token);
    }
    check(FSVRBoneLog::testHeaderPending(),
          "header stays pending while the ring is full");
    check(FSVRBoneLog::testQueuedCount() == 256,
          "no new-schema sample queued ahead of its header");
    check(FSVRBoneLog::testDroppedCount() == 50,
          "5 failed header enqueues counted as drops; the "
          "skipped samples were never produced");

    // Recovery: release the writer, let it drain, then
    // produce again -- the pending header must land before
    // the next sample.
    FSVRBoneLog::testSetWriterStall(false);
    check(waitFor(
              [] { return FSVRBoneLog::testQueuedCount() == 0; },
              10000),
          "writer drained the ring after stall release");
    for (U64 token = 306; token <= 308; ++token)
    {
        sample(skel, token);
    }
    check(!FSVRBoneLog::testHeaderPending(),
          "header published once capacity freed");

    FSVRBoneLog::shutdown();

    // ---- Verify the file: the schema-order invariant. ----
    std::ifstream in(kBonelogPath);
    check(in.good(), "bonelog file exists after shutdown");
    namespace pt = boost::property_tree;
    long long current_schema = -1;
    U32 samples_before_header_b = 0;
    U32 samples_after_header_b = 0;
    bool header_a_seen = false;
    bool header_b_seen = false;
    bool header_b_has_spine = false;
    bool drops_seen = false;
    U64 final_dropped = 0;
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty())
        {
            continue;
        }
        pt::ptree tree;
        bool parsed = true;
        try
        {
            std::istringstream stream(line);
            pt::read_json(stream, tree);
        }
        catch (...)
        {
            parsed = false;
        }
        check(parsed, "every written line parses as JSON");
        if (!parsed)
        {
            continue;
        }
        if (tree.count("header"))
        {
            const pt::ptree& joints = tree.get_child("joints");
            current_schema =
                static_cast<long long>(joints.size());
            if (current_schema == 2)
            {
                header_a_seen = true;
            }
            if (current_schema == 3)
            {
                header_b_seen = true;
                for (const pt::ptree::value_type& entry : joints)
                {
                    if (entry.second.get_value<std::string>() ==
                        "mSpine")
                    {
                        header_b_has_spine = true;
                    }
                }
            }
        }
        else if (tree.count("bones"))
        {
            const pt::ptree& bones = tree.get_child("bones");
            check(static_cast<long long>(bones.size()) ==
                      current_schema,
                  "sample bone count matches the schema of "
                  "the most recent header");
            if (!header_b_seen)
            {
                ++samples_before_header_b;
            }
            else
            {
                ++samples_after_header_b;
            }
        }
        else if (tree.get<std::string>("event", "") ==
                 "bonelog_drops")
        {
            drops_seen = true;
            final_dropped = tree.get<U64>("dropped", 0);
        }
    }
    check(header_a_seen, "header A (2 joints) present");
    check(header_b_seen, "header B (3 joints) present");
    check(header_b_has_spine,
          "header B carries the new joint name");
    check(samples_before_header_b == 255,
          "exactly the 255 queued A samples precede header B");
    check(samples_after_header_b == 3,
          "samples resumed under schema B after recovery");
    check(drops_seen && final_dropped == 50,
          "final drops record reports exactly 50");
    return summary("header");
}

int runWriterFailMode()
{
    check(FSVRBoneLog::enabled(),
          "recorder enabled (FS_VR_BONELOG set by runner)");

    FakeSkeleton skel;
    skel.build("mRoot", "", "mHead", 1);

    FSVRBoneLog::testArmWriterThrow(true);
    // Trigger the lazy start through an event; the writer
    // throws at the top of its body and the throw must be
    // contained by writerMain's catch-all. (Reaching the
    // checks below at all is the no-terminate evidence.)
    FSVRBoneLog::eventCalibrateRequested();

    check(waitFor([] { return FSVRBoneLog::testWriterFailed(); },
                  10000),
          "writer_failed set after the injected exception");
    check(waitFor(
              [] {
                  return FSVRBoneLog::testRunState() ==
                         kRunStateDead;
              },
              10000),
          "recorder transitioned to BS_DEAD");

    // Producers have ceased: further calls return quietly
    // and change nothing.
    for (U64 token = 1; token <= 5; ++token)
    {
        sample(skel, token);
    }
    FSVRBoneLog::eventCalibrateRequested();
    FSVRBoneLog::eventSyncExecuted(7, LLVector3::zero, 0.f,
                                    LLVector3::zero,
                                    LLVector3::zero);
    check(FSVRBoneLog::testDroppedCount() == 0,
          "producers ceased: no drops after writer death");
    check(!FSVRBoneLog::testHeaderPending(),
          "producers ceased: no header was ever pended");

    // The dead thread must remain joinable: shutdown joins
    // it and returns, leaving the recorder DEAD.
    FSVRBoneLog::shutdown();
    check(FSVRBoneLog::testRunState() == kRunStateDead,
          "shutdown() joined the failed writer and returned");

    // The writer threw before opening the file: no output.
    std::ifstream in(kBonelogPath, std::ios::ate);
    check(!in.good() || in.tellg() == std::streampos(0),
          "no bonelog file output from the failed writer");
    return summary("writerfail");
}

int runWriterFailRaceMode()
{
    check(FSVRBoneLog::enabled(),
          "recorder enabled (FS_VR_BONELOG set by runner)");

    FakeSkeleton skel;
    skel.build("mRoot", "", "mHead", 1);

    // Force the BONELOG-R3-01 interleaving: the writer
    // throws at the top of its body, and the startup
    // barrier holds ensureStarted (inside the trigger call
    // below) until the writer's terminal stores have fully
    // completed -- the failure lands entirely BEFORE the
    // startup caller finishes, which is the one ordering
    // the pre-fix code could not survive.
    FSVRBoneLog::testArmWriterThrow(true);
    FSVRBoneLog::testArmStartupBarrier(true);

    // Trigger the lazy start through an event. This call
    // does not return until startup completes, and the
    // barrier guarantees the writer has already failed by
    // then -- so every check below is deterministic and
    // needs no polling.
    FSVRBoneLog::eventCalibrateRequested();

    check(FSVRBoneLog::testFailureComplete(),
          "barrier engaged: writer failure completed before "
          "the startup caller returned");
    // Verdict check (1).
    check(FSVRBoneLog::testWriterFailed(),
          "R3-01 (1): writer_failed == true");
    // Verdict check (2), at the strongest point: after the
    // startup caller has returned.
    check(FSVRBoneLog::testRunState() == kRunStateDead,
          "R3-01 (2): run_state == BS_DEAD after the startup "
          "caller returned (not resurrected to BS_RUNNING)");
    check(FSVRBoneLog::testWriterEntryCount() == 1,
          "writer thread entered exactly once");

    // Verdict checks (3) and (6): producers must neither
    // restart the dead writer nor produce anything.
    for (U64 token = 1; token <= 5; ++token)
    {
        sample(skel, token);
    }
    FSVRBoneLog::eventCalibrateRequested();
    FSVRBoneLog::eventSyncExecuted(7, LLVector3::zero, 0.f,
                                    LLVector3::zero,
                                    LLVector3::zero);
    check(FSVRBoneLog::testWriterEntryCount() == 1,
          "R3-01 (3): no subsequent producer restarted the "
          "writer");
    check(FSVRBoneLog::testDroppedCount() == 0,
          "R3-01 (6): producers stayed ceased: no drops "
          "after writer death");
    check(!FSVRBoneLog::testHeaderPending(),
          "R3-01 (6): producers stayed ceased: no header "
          "was pended");
    check(FSVRBoneLog::testRunState() == kRunStateDead,
          "R3-01 (2): run_state still BS_DEAD after "
          "producer calls");

    // Verdict checks (4) and (5): shutdown() joins the
    // dead-but-joinable thread and returns (reaching the
    // next check at all is the join evidence), and it
    // leaves the terminal state untouched.
    FSVRBoneLog::shutdown();
    check(FSVRBoneLog::testRunState() == kRunStateDead,
          "R3-01 (4)+(5): shutdown() joined the failed "
          "writer and left BS_DEAD unchanged");
    check(FSVRBoneLog::testWriterEntryCount() == 1,
          "R3-01 (3): shutdown did not restart the writer");

    // The writer threw before opening the file: no output.
    std::ifstream in(kBonelogPath, std::ios::ate);
    check(!in.good() || in.tellg() == std::streampos(0),
          "no bonelog file output from the failed writer");
    return summary("writerfailrace");
}

} // namespace

int main(int argc, char** argv)
{
    const std::string mode = argc > 1 ? argv[1] : "";
    if (mode == "header")
    {
        return runHeaderMode();
    }
    if (mode == "writerfail")
    {
        return runWriterFailMode();
    }
    if (mode == "writerfailrace")
    {
        return runWriterFailRaceMode();
    }
    std::cout << "usage: bonelog_state_test "
                 "header|writerfail|writerfailrace"
              << std::endl;
    return 2;
}
