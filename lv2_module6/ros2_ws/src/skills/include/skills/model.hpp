#pragma once

// skill 정책 폴더 — models/skills/<skill>/ 하나가 skill 하나의 학습 결과 (skill 노드가 갖고 관리)
//   <skill>.policy  로봇이 읽는 MLP (ml/scripts/export_policy.py — C++ 직접 계산, 파일 안 'test' 줄로 검산)
//   <skill>.onnx    원본 (Isaac Lab 학습 산출물, 보관·다른 도구용)
//   meta.yaml       한 줄에 "키: 값" — kind(arm|base) · layout(팔 정책 입력 모양) · use(true면 씀) · 시뮬 성공률 · 출처 · 주행 판정값
//   use: false면 정책을 안 씀 (Δ = 0, 기준 동작만) — 기준선보다 못하거나 로봇 입력이 아직 없을 때
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>

namespace skills {

struct SkillModel {
  std::string name, dir, kind, layout, policy;  // policy = .policy 절대 경로 (없으면 "")
  bool use = false;                             // meta use: true이고 .policy가 있을 때만 true
  std::map<std::string, std::string> meta;      // meta.yaml 전부 (진단·판정값)
  double num(const std::string & key, double fallback) const {
    const auto it = meta.find(key);
    if (it == meta.end()) return fallback;
    char * end = nullptr;
    const double v = std::strtod(it->second.c_str(), &end);
    return end == it->second.c_str() ? fallback : v;
  }
};

// "키: 값" 줄만 (주석 #, 빈 줄, 들여쓴 줄은 건너뜀 — 평평한 meta.yaml). 따옴표는 벗김
inline std::map<std::string, std::string> parse_meta(std::istream & in) {
  std::map<std::string, std::string> out;
  std::string line;
  const auto trim = [](std::string s) {
    const auto a = s.find_first_not_of(" \t\r"), b = s.find_last_not_of(" \t\r");
    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
  };
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#' || line[0] == ' ' || line[0] == '\t') continue;
    const auto c = line.find(':');
    if (c == std::string::npos) continue;
    std::string v = line.substr(c + 1);
    if (const auto h = v.find(" #"); h != std::string::npos) v = v.substr(0, h);
    v = trim(v);
    if (v.size() >= 2 && (v.front() == '"' || v.front() == '\'') && v.back() == v.front()) v = v.substr(1, v.size() - 2);
    out[trim(line.substr(0, c))] = v;
  }
  return out;
}

// dir/<name>/meta.yaml을 읽음. 폴더·meta가 없으면 use=false (why에 이유)
inline SkillModel load_skill_model(const std::string & root, const std::string & name, std::string & why) {
  SkillModel m;
  m.name = name;
  m.dir = root + "/" + name;
  std::ifstream f(m.dir + "/meta.yaml");
  if (!f) { why = "meta.yaml 없음 (" + m.dir + ")"; return m; }
  m.meta = parse_meta(f);
  m.kind = m.meta["kind"];
  m.layout = m.meta["layout"];
  const std::string policy = m.dir + "/" + name + ".policy";
  const bool have = static_cast<bool>(std::ifstream(policy));
  if (have) m.policy = policy;
  m.use = m.meta["use"] == "true" && have;
  if (m.meta["use"] == "true" && !have) why = name + ".policy 없음";
  else if (m.meta["use"] != "true") why = "use: " + m.meta["use"] + (m.meta.count("why") ? " — " + m.meta["why"] : "");
  return m;
}

}  // namespace skills
