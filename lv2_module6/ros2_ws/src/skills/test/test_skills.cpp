// skill 노드 공통 — 정책 폴더(meta.yaml) 읽기 · 잡기 판정 · 저장소의 정책 폴더가 다 읽히는지 (colcon test --packages-select skills)
#include <gtest/gtest.h>

#include <filesystem>
#include <sstream>

#include "ml/arm_obs.hpp"
#include "ml/policy.hpp"
#include "skills/arm_io.hpp"
#include "skills/model.hpp"

TEST(SkillModel, ParsesFlatMeta) {
  std::istringstream in("# 주석\nskill: load_slot\nuse: true   # 씀\nsim_policy: 99.0\n  들여쓴: 무시\nwhy: \"따옴표\"\n");
  const auto m = skills::parse_meta(in);
  EXPECT_EQ(m.at("skill"), "load_slot");
  EXPECT_EQ(m.at("use"), "true");
  EXPECT_EQ(m.at("sim_policy"), "99.0");
  EXPECT_EQ(m.at("why"), "따옴표");
  EXPECT_EQ(m.count("들여쓴"), 0u);
}

TEST(SkillModel, MissingFolderIsNotUsed) {
  std::string why;
  const auto m = skills::load_skill_model("/nonexistent", "dock_box", why);
  EXPECT_FALSE(m.use);
  EXPECT_FALSE(why.empty());
}

// 저장소의 skill 정책 폴더마다: meta가 있고, use: true면 정책 파일(.onnx · runtime: policy면 .policy)이 있고 kind·layout·입력 크기가 맞음
TEST(SkillModel, RepoFoldersAreConsistent) {
  int n = 0;
  for (const auto & e : std::filesystem::directory_iterator(MODEL_DIR)) {
    if (!e.is_directory()) continue;
    const auto name = e.path().filename().string();
    std::string why;
    const auto m = skills::load_skill_model(MODEL_DIR, name, why);
    ASSERT_TRUE(m.meta.count("kind")) << name << ": " << why;
    EXPECT_EQ(m.meta.at("skill"), name);
    if (m.meta.at("use") != "true") { ++n; continue; }
    ASSERT_TRUE(m.use) << name << ": " << why;
    if (ml::OnnxPolicy::runtime_version() == "none" && !(m.meta.count("runtime") && m.meta.at("runtime") == "policy")) { ++n; continue; }  // ONNX 없이 빌드
    ml::Policy p;
    ASSERT_TRUE(p.load(m.policy, why)) << why;
    if (m.kind == "base") {
      EXPECT_EQ(p.inputs(), 13u) << name;
      EXPECT_EQ(p.outputs(), 2u) << name;
    } else if (m.kind == "hand") {  // 세우기 방식 C: ml/stand_c_obs.hpp 28 → 손 속도 3
      EXPECT_EQ(p.inputs(), 28u) << name;
      EXPECT_EQ(p.outputs(), 3u) << name;
    } else {
      ml::ArmLayout l;
      ASSERT_TRUE(ml::arm_layout_from(m.layout, l)) << name << " " << m.layout;
      EXPECT_EQ(p.inputs(), ml::arm_obs_size(l)) << name << " " << m.layout;
      EXPECT_EQ(p.outputs(), 5u) << name;
    }
    ++n;
  }
  EXPECT_GE(n, 6);
}

TEST(GraspHeld, PositionAndCurrent) {
  EXPECT_FALSE(skills::grasp_held(0.001, 0.4, 0.003, 0.25));   // 끝까지 닫힘
  EXPECT_FALSE(skills::grasp_held(0.006, 0.1, 0.003, 0.25));   // 전류 약함
  EXPECT_TRUE(skills::grasp_held(0.006, -0.4, 0.003, 0.25));   // 부호 상관없음
  EXPECT_TRUE(skills::grasp_held(0.006, std::nan(""), 0.003, 0.25));  // 전류 모름 → 위치만
}

TEST(ArmIo, GripState) {  // 위치형 쥐기 판정
  using skills::grip_state;
  EXPECT_EQ(grip_state(0.007, 0.009, 0.3, 1.0, 0.007, 0.003, 4.0), 1);   // 멈춤 + 전류 = 쥠
  EXPECT_EQ(grip_state(0.002, 0.009, 0.3, 1.0, 0.007, 0.003, 4.0), 0);   // 다 닫힘 = 빈손 (전류와 무관)
  EXPECT_EQ(grip_state(0.007, 0.009, 0.1, 1.0, 0.007, 0.003, 4.0), 2);   // 아직 움직임
  EXPECT_EQ(grip_state(0.007, 0.002, 0.3, 1.0, 0.007, 0.003, 4.0), 2);   // 전류 부족
  EXPECT_EQ(grip_state(0.007, 0.002, 0.3, 4.1, 0.007, 0.003, 4.0), -1);  // 시간 초과
}

TEST(ArmIo, HandYaw) {  // ψ ↔ 위→아래 손 쿼터니언 (grasp_scorer top_down)
  geometry_msgs::msg::Quaternion q;
  q.x = std::cos(0.3);
  q.y = std::sin(0.3);
  q.z = 0.0;
  q.w = 0.0;
  double psi;
  ASSERT_TRUE(skills::hand_yaw(q, psi));
  EXPECT_NEAR(psi, 0.6, 1e-9);
  q.x = q.y = 0.0;
  q.w = 1.0;  // 단위 쿼터니언 — 손 방향 아님
  EXPECT_FALSE(skills::hand_yaw(q, psi));
}
