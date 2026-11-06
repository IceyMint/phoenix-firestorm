/**
 * @file llvrbonelog_format_test.cpp
 * @brief P2-07B bonelog format-validation test (BONELOG-R1-03).
 *
 * Standalone test, deliberately NOT another broad H-series
 * harness: it exercises the recorder's pure serialization
 * builders (llvrbonelog.cpp compiled with
 * FSVR_BONELOG_SERIALIZE_ONLY, so no viewer state is linked)
 * for a header, both event kinds, the drops record, and
 * samples; writes every produced line to a file; then parses
 * EVERY line back -- from the file, not from memory -- with a
 * real JSON parser (boost::property_tree::json_parser) and
 * asserts structural and value-level validity.
 *
 * The header case carries a hostile joint name containing a
 * quote, a backslash, a newline, a tab, and raw control
 * bytes (0x01, 0x1f); the test asserts the name round-trips
 * byte-exact through escaping, disk, and parsing.
 *
 * Build/run: run_bonelog_format_test.sh (the H-harness
 * pattern: the viewer's own build-tree flags, chroot).
 * Exit code 0 iff every check passes.
 */

#include "llvrbonelog.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

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

bool near(F32 a, F32 b)
{
    return std::fabs(a - b) < 1e-6f;
}

// Parse one line with the real JSON parser; false on any
// parse error.
bool parseLine(const std::string& line,
               boost::property_tree::ptree& out_tree)
{
    try
    {
        std::istringstream stream(line);
        boost::property_tree::read_json(stream, out_tree);
        return true;
    }
    catch (const std::exception& e)
    {
        std::cout << "PARSE ERROR: " << e.what()
                  << " on line: " << line << std::endl;
        return false;
    }
}

F32 ptreeFloat(const boost::property_tree::ptree& tree)
{
    return static_cast<F32>(
        std::stod(tree.get_value<std::string>()));
}

} // namespace

int main(int argc, char** argv)
{
    const char* out_path =
        argc > 1 ? argv[1] : "/tmp/bonelog_format_test.jsonl";

    // The hostile name: quote, backslash, newline, tab, and
    // raw control bytes 0x01 / 0x1f.
    const std::string hostile =
        std::string("evil\"quo\\te\nnl\ttab") + '\x01' + '\x1f';
    const std::vector<std::string> joint_names = {
        "mRoot", "mHead", hostile
    };

    std::vector<std::string> lines;
    std::string line;

    // 1. Header.
    FSVRBoneLog::buildHeaderRecord(line, joint_names,
                                   123456789LL);
    lines.push_back(line);

    // 2. Events.
    FSVRBoneLog::buildCalibrateRequestedRecord(line, 123456790LL);
    lines.push_back(line);
    FSVRBoneLog::buildSyncExecutedRecord(
        line, 123456791LL, 4242,
        LLVector3(0.25f, 1.5f, -0.75f), 0.5f,
        LLVector3(0.01f, 0.02f, 0.03f),
        LLVector3(-0.1f, 0.2f, -0.3f));
    lines.push_back(line);
    FSVRBoneLog::buildDropsRecord(line, 123456792LL, 7);
    lines.push_back(line);

    // 3. Full sample (valid hmd quaternion, target block).
    const std::vector<LLVector3> bone_pos = {
        LLVector3(0.f, 0.f, 0.f),
        LLVector3(1.25f, -2.5f, 3.75f),
        LLVector3(-0.5f, 0.625f, -1.125f)
    };
    const std::vector<LLQuaternion> bone_rot = {
        LLQuaternion(0.f, 0.f, 0.f, 1.f),
        LLQuaternion(0.5f, 0.5f, 0.5f, 0.5f),
        LLQuaternion(0.f, 0.f, 0.f, 1.f)
    };
    FSVRBoneLog::buildSampleRecord(
        line, 123456793LL, 4243,
        true, LLVector3(0.125f, 1.625f, -0.375f),
        true, LLQuaternion(0.f, 0.f, 0.f, 1.f),
        LLVector3(10.f, 20.f, 30.f),
        LLQuaternion(0.f, 0.f, 0.f, 1.f),
        true, LLVector3(10.5f, 21.5f, 29.5f),
        LLQuaternion(0.f, 0.f, 0.f, 1.f),
        bone_pos, bone_rot,
        true, LLVector3(-0.1f, 0.2f, -0.3f), 0.0125f);
    lines.push_back(line);

    // 4. Sample with invalid hmd orientation: "q":null,
    // position kept.
    FSVRBoneLog::buildSampleRecord(
        line, 123456794LL, 4244,
        true, LLVector3(0.125f, 1.625f, -0.375f),
        false, LLQuaternion(),
        LLVector3::zero, LLQuaternion(),
        false, LLVector3::zero, LLQuaternion(),
        bone_pos, bone_rot,
        false, LLVector3::zero, 0.f);
    const std::string null_q_line = line;
    lines.push_back(line);

    // 5. Sample with no hmd at all: "hmd":null, and a
    // non-finite bone coordinate (sanitizer -> null).
    std::vector<LLVector3> bone_pos_nan = bone_pos;
    bone_pos_nan[1].mV[VX] =
        std::numeric_limits<F32>::quiet_NaN();
    FSVRBoneLog::buildSampleRecord(
        line, 123456795LL, 4245,
        false, LLVector3::zero,
        false, LLQuaternion(),
        LLVector3::zero, LLQuaternion(),
        false, LLVector3::zero, LLQuaternion(),
        bone_pos_nan, bone_rot,
        false, LLVector3::zero, 0.f);
    const std::string null_hmd_line = line;
    lines.push_back(line);

    // 6. DIAG-1 v2 header (D1-F2): schema + plan joints.
    const std::vector<std::string> plan_names = {
        "mPelvis", "mSpine1", "mNeck", "mHead"
    };
    FSVRBoneLog::buildHeaderRecord(line, joint_names,
                                   plan_names, 123456796LL);
    lines.push_back(line);

    // 7. DIAG-1 v2 sample: plan + headrot + blend blocks.
    FSVRBoneLogPlanData plan;
    const LLQuaternion qid(0.f, 0.f, 0.f, 1.f);
    const LLQuaternion qx(0.5f, 0.5f, 0.5f, 0.5f);
    for (S32 i = 0; i < 4; ++i)
    {
        plan.base_locals.push_back(qid);
        plan.base_world_positions.push_back(
            LLVector3(0.f, 0.f, static_cast<F32>(i)));
        plan.base_world_rotations.push_back(qid);
        plan.resolved_locals.push_back(qx);
        plan.resolved_world_positions.push_back(
            LLVector3(1.f, 0.f, static_cast<F32>(i)));
        plan.resolved_world_rotations.push_back(qx);
        FSVRBoneLogPlanDomain dom;
        dom.domain = i == 1 ? 1 : 0; // spine for joint 1
        dom.decision_grants_base = i != 3;
        dom.disabled = i == 2;
        plan.domains.push_back(dom);
    }
    plan.applied = true;
    plan.gate_name = nullptr;
    plan.spine_solver_invoked = true;
    plan.head_group_refusal = false;
    plan.writes_count = 5;
    FSVRBoneLogHeadRotSample headrot;
    headrot.neck_state = qx;
    headrot.head_state = qx;
    headrot.last_head_rot_before = qid;
    headrot.last_head_rot_after = qx;
    headrot.instant = false;
    headrot.generation = 17;
    FSVRBoneLogBlendSample blend;
    blend.neck_present = true;
    blend.neck.generation = 9;
    blend.neck.seed_local = qid;
    {
        FSVRBoneLogBlendSlot slot;
        slot.additive = true;
        slot.priority = 7;
        slot.weight = 0.5f;
        slot.rotation = qx;
        slot.has_rot_sum = false;
        slot.delta = 0.5f;
        slot.blended_after = qid;
        slot.added_after = qx;
        slot.source_headrot = true;
        blend.neck.slots.push_back(slot);
        FSVRBoneLogBlendSlot slot2;
        slot2.additive = false;
        slot2.priority = 3;
        slot2.weight = 1.f;
        slot2.rotation = qx;
        slot2.has_rot_sum = true;
        slot2.rot_sum_after = 1.f;
        slot2.blended_after = qx;
        slot2.added_after = qx;
        slot2.source_headrot = false;
        blend.neck.slots.push_back(slot2);
    }
    blend.neck.rot_weight_sum = 1.f;
    blend.neck.blended_rot = qx;
    blend.neck.added_rot = qx;
    blend.neck.result_local = qx;
    // CODE-R1-01: this entry's consistency validation
    // failed; the marker must serialize.
    blend.neck.source_integrity_failure = true;
    blend.head_present = false;
    FSVRBoneLog::buildSampleRecordV2(
        line, 123456797LL, 4246,
        true, LLVector3(0.125f, 1.625f, -0.375f),
        true, LLQuaternion(0.f, 0.f, 0.f, 1.f),
        LLVector3(10.f, 20.f, 30.f),
        LLQuaternion(0.f, 0.f, 0.f, 1.f),
        true, LLVector3(10.5f, 21.5f, 29.5f),
        LLQuaternion(0.f, 0.f, 0.f, 1.f),
        bone_pos, bone_rot,
        true, LLVector3(-0.1f, 0.2f, -0.3f), 0.0125f,
        &plan, &headrot, &blend);
    const std::string v2_sample_line = line;
    lines.push_back(line);

    // 8. DIAG-1 v2 sample with NO headrot/blend captures:
    // the blocks are absent (never fabricated); the plan
    // block is present. Also: gate name serializes, and a
    // non-finite plan value serializes as null.
    plan.gate_name = "no_solver";
    plan.base_locals[1] = LLQuaternion(
        std::numeric_limits<F32>::quiet_NaN(), 0.f, 0.f, 1.f);
    FSVRBoneLog::buildSampleRecordV2(
        line, 123456798LL, 4247,
        false, LLVector3::zero,
        false, LLQuaternion(),
        LLVector3::zero, LLQuaternion(),
        false, LLVector3::zero, LLQuaternion(),
        bone_pos, bone_rot,
        false, LLVector3::zero, 0.f,
        &plan, nullptr, nullptr);
    const std::string v2_bare_line = line;
    lines.push_back(line);

    check(v2_sample_line.find("\"source\":\"headrot\"") !=
              std::string::npos,
          "v2 sample carries the semantic headrot source marker "
          "(and no pointer anywhere)");
    check(v2_sample_line.find(
              "\"source_integrity\":\"failed\"") !=
              std::string::npos,
          "v2 blend entry with a failed consistency validation "
          "carries the source_integrity failure marker "
          "(CODE-R1-01)");
    // The same entry with validation passed (or no slot
    // attributed) omits the marker entirely.
    blend.neck.source_integrity_failure = false;
    FSVRBoneLog::buildSampleRecordV2(
        line, 123456797LL, 4246,
        true, LLVector3(0.125f, 1.625f, -0.375f),
        true, LLQuaternion(0.f, 0.f, 0.f, 1.f),
        LLVector3(10.f, 20.f, 30.f),
        LLQuaternion(0.f, 0.f, 0.f, 1.f),
        true, LLVector3(10.5f, 21.5f, 29.5f),
        LLQuaternion(0.f, 0.f, 0.f, 1.f),
        bone_pos, bone_rot,
        true, LLVector3(-0.1f, 0.2f, -0.3f), 0.0125f,
        &plan, &headrot, &blend);
    check(line.find("source_integrity") == std::string::npos,
          "v2 blend entry whose validation passed omits the "
          "integrity marker (CODE-R1-01)");
    check(v2_bare_line.find("\"headrot\"") == std::string::npos &&
              v2_bare_line.find("\"blend\"") == std::string::npos,
          "absent captures serialize as absent blocks");
    check(v2_bare_line.find("\"gate\":\"no_solver\"") !=
              std::string::npos,
          "failing gate serializes by name");

    // Raw-text assertions for the null shapes (a JSON parser
    // cannot distinguish null from a missing key by value).
    check(null_q_line.find("\"q\":null") != std::string::npos,
          "invalid hmd orientation serializes as \"q\":null");
    check(null_hmd_line.find("\"hmd\":null") !=
              std::string::npos,
          "absent hmd serializes as \"hmd\":null");

    // Write every line to the output file.
    {
        std::ofstream out(out_path,
                          std::ios::out | std::ios::trunc);
        if (!out)
        {
            std::cout << "FAIL: cannot open output file "
                      << out_path << std::endl;
            return 1;
        }
        for (const std::string& l : lines)
        {
            out << l;
        }
    }

    // Read the file back and parse EVERY line from disk.
    std::vector<std::string> disk_lines;
    {
        std::ifstream in(out_path);
        if (!in)
        {
            std::cout << "FAIL: cannot reopen output file"
                      << std::endl;
            return 1;
        }
        std::string l;
        while (std::getline(in, l))
        {
            disk_lines.push_back(l);
        }
    }
    check(disk_lines.size() == lines.size(),
          "file line count matches produced line count");
    std::vector<boost::property_tree::ptree> trees(
        disk_lines.size());
    for (size_t i = 0; i < disk_lines.size(); ++i)
    {
        check(parseLine(disk_lines[i], trees[i]),
              "line " + std::to_string(i) + " parses as JSON");
    }
    if (g_failures > 0)
    {
        std::cout << "BONELOG_FORMAT_TEST: " << g_failures
                  << " FAILURE(S) in parse phase" << std::endl;
        return 1;
    }

    namespace pt = boost::property_tree;

    // Header assertions: names round-trip byte-exact,
    // including the hostile one.
    {
        const pt::ptree& header = trees[0];
        check(header.get<bool>("header", false),
              "header flag present");
        check(header.get<long long>("t_ms", 0) == 123456789LL,
              "header t_ms");
        const pt::ptree& joints = header.get_child("joints");
        check(joints.size() == joint_names.size(),
              "header joint count");
        size_t i = 0;
        for (const pt::ptree::value_type& entry : joints)
        {
            check(entry.second.get_value<std::string>() ==
                      joint_names[i],
                  "joint name " + std::to_string(i) +
                      " round-trips byte-exact");
            ++i;
        }
    }

    // Event assertions.
    check(trees[1].get<std::string>("event", "") ==
              "calibrate_requested",
          "calibrate_requested event name");
    check(trees[2].get<std::string>("event", "") ==
              "sync_executed",
          "sync_executed event name");
    check(trees[2].get<unsigned long long>("cycle", 0) == 4242,
          "sync_executed cycle token");
    {
        const pt::ptree& neutral =
            trees[2].get_child("neutral_cornea_root");
        auto it = neutral.begin();
        check(near(ptreeFloat(it->second), -0.1f) &&
                  near(ptreeFloat((++it)->second), 0.2f) &&
                  near(ptreeFloat((++it)->second), -0.3f),
              "sync_executed neutral components");
    }
    check(trees[3].get<std::string>("event", "") ==
              "bonelog_drops",
          "drops event name");
    check(trees[3].get<unsigned long long>("dropped", 0) == 7,
          "drops total");

    // Full-sample assertions.
    {
        const pt::ptree& sample = trees[4];
        check(sample.get<unsigned long long>("cycle", 0) ==
                  4243,
              "sample cycle token");
        const pt::ptree& hmd_q =
            sample.get_child("hmd").get_child("q");
        auto qit = hmd_q.begin();
        check(near(ptreeFloat(qit->second), 0.f) &&
                  near(ptreeFloat((++qit)->second), 0.f) &&
                  near(ptreeFloat((++qit)->second), 0.f) &&
                  near(ptreeFloat((++qit)->second), 1.f),
              "sample hmd quaternion components");
        const pt::ptree& bones = sample.get_child("bones");
        check(bones.size() == 3, "sample bone count");
        auto bit = bones.begin();
        ++bit; // bone 1
        const pt::ptree& p1 = bit->second.get_child("p");
        auto pit = p1.begin();
        check(near(ptreeFloat(pit->second), 1.25f) &&
                  near(ptreeFloat((++pit)->second), -2.5f) &&
                  near(ptreeFloat((++pit)->second), 3.75f),
              "sample bone 1 position");
        const pt::ptree& tvo =
            sample.get_child("target_vs_observed");
        check(near(static_cast<F32>(std::stod(
                        tvo.get<std::string>("observed_err_m"))),
                    0.0125f),
              "target_vs_observed observed_err_m");
    }

    // Null-q sample: parses (already proven) and still
    // carries the hmd position.
    {
        const pt::ptree& hmd = trees[5].get_child("hmd");
        const pt::ptree& p = hmd.get_child("p");
        auto pit = p.begin();
        check(near(ptreeFloat(pit->second), 0.125f),
              "null-q sample keeps hmd position");
    }

    // DIAG-1 v2 header assertions (trees[7]).
    {
        const pt::ptree& header = trees[7];
        check(header.get<S32>("schema", 0) == 2,
              "v2 header carries schema 2");
        const pt::ptree& pj = header.get_child("plan_joints");
        check(pj.size() == plan_names.size(),
              "v2 header plan_joints count");
        size_t i = 0;
        for (const pt::ptree::value_type& entry : pj)
        {
            check(entry.second.get_value<std::string>() ==
                      plan_names[i],
                  "plan joint name " + std::to_string(i) +
                      " round-trips");
            ++i;
        }
        // Plan-name uniqueness (brief section 2.1).
        std::vector<std::string> sorted = plan_names;
        std::sort(sorted.begin(), sorted.end());
        check(std::adjacent_find(sorted.begin(), sorted.end()) ==
                  sorted.end(),
              "plan joint names are unique");
    }

    // DIAG-1 v2 sample assertions (trees[8]).
    {
        const pt::ptree& sample = trees[8];
        const pt::ptree& plan_t = sample.get_child("plan");
        check(plan_t.get_child("base_locals").size() ==
                      plan_names.size() &&
                  plan_t.get_child("base_worlds").size() ==
                      plan_names.size() &&
                  plan_t.get_child("resolved_locals").size() ==
                      plan_names.size() &&
                  plan_t.get_child("resolved_worlds").size() ==
                      plan_names.size() &&
                  plan_t.get_child("domains").size() ==
                      plan_names.size(),
              "v2 plan arrays match the header plan-joint list "
              "length");
        check(plan_t.get<bool>("applied", false),
              "v2 plan applied flag");
        check(disk_lines[8].find("\"gate\":null") !=
                  std::string::npos,
              "v2 plan gate is null when no gate failed");
        check(plan_t.get<unsigned>("writes", 0) == 5,
              "v2 plan writes count");
        auto dit = plan_t.get_child("domains").begin();
        ++dit; // joint 1: spine
        check(dit->second.get<std::string>("domain", "") ==
                  "spine",
              "v2 plan domain serializes by name");
        const pt::ptree& hr = sample.get_child("headrot");
        check(hr.get<unsigned long long>("generation", 0) == 17,
              "v2 headrot generation");
        const pt::ptree& bl = sample.get_child("blend");
        check(bl.count("mNeck") == 1 && bl.count("mHead") == 0,
              "v2 blend carries only the joints actually blended");
        const pt::ptree& slots =
            bl.get_child("mNeck").get_child("slots");
        check(slots.size() == 2, "v2 blend slot count");
        auto sit = slots.begin();
        check(sit->second.get<std::string>("source", "") ==
                  "headrot",
              "v2 blend slot 0 is the headrot-attributed slot");
        ++sit;
        check(sit->second.count("source") == 0,
              "v2 blend slot 1 carries no source marker");
        check(sit->second.get_child_optional("rot_sum_after") &&
                  !sit->second.get_child_optional("delta"),
              "v2 non-additive slot carries rot_sum_after, "
              "not delta");
    }

    // DIAG-1 bare v2 sample assertions (trees[9]): plan
    // block present, headrot/blend absent, non-finite plan
    // value serialized as null.
    {
        const pt::ptree& sample = trees[9];
        check(sample.count("plan") == 1,
              "bare v2 sample still carries the plan block");
        check(sample.count("headrot") == 0 &&
                  sample.count("blend") == 0,
              "bare v2 sample omits headrot and blend blocks");
        check(disk_lines[9].find("null") != std::string::npos,
              "non-finite plan value serialized as null");
    }

    if (g_failures == 0)
    {
        std::cout << "BONELOG_FORMAT_TEST: ALL " << g_checks
                  << " CHECKS PASSED (" << disk_lines.size()
                  << " lines written and parsed)" << std::endl;
        return 0;
    }
    std::cout << "BONELOG_FORMAT_TEST: " << g_failures
              << " FAILURE(S) of " << g_checks << " checks"
              << std::endl;
    return 1;
}
