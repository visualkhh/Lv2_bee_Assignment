#include "ManualActors.hh"

#include <QCoreApplication>
#include <QCursor>
#include <QMenu>
#include <QStringList>
#include <QTimer>

#include <cstdlib>
#include <chrono>
#include <cmath>
#include <thread>

#include <fstream>
#include <regex>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Helpers.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/plugin/Register.hh>

namespace beebot
{
/////////////////////////////////////////////////
ManualActors::ManualActors() = default;

/////////////////////////////////////////////////
ManualActors::~ManualActors()
{
  delete this->contextMenu_;
}

/////////////////////////////////////////////////
void ManualActors::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty())
    this->title = "수동 액터";

  const auto worlds = gz::gui::worldNames();
  if (!worlds.isEmpty())
    this->worldName = worlds[0].toStdString();

  const auto env_name = [](const char *key) -> std::string {
    const char *v = std::getenv(key);
    if (!v || std::string(v).empty() || std::string(v) == "none")
      return "none";
    return std::string(v);
  };
  this->floorName_ = env_name("GZ_FLOOR_TEXTURE");
  this->wallName_ = env_name("GZ_WALL_TEXTURE");
  this->backgroundName_ = env_name("GZ_BACKGROUND");

  auto *mainWindow = gz::gui::App()->findChild<gz::gui::MainWindow *>();
  if (mainWindow)
    mainWindow->installEventFilter(this);

  // QMenu::exec()를 이벤트 필터 안에서 호출하면 마우스 release 이벤트와
  // 겹쳐 메뉴가 즉시 닫힐 수 있다. 메뉴를 유지하고 popup()으로 띄운다.
  this->contextMenu_ = new QMenu();
  auto add = [this](const char *label, int kind) {
    QAction *action = this->contextMenu_->addAction(QString::fromUtf8(label));
    QObject::connect(action, &QAction::triggered, this, [this, kind]() {
      this->RunContextAction(kind);
    });
  };
  add("사람 소환", 0);
  add("박스 장애물 소환", 1);
  this->contextMenu_->addSeparator();
  add("빨강 퍽", 2);
  add("주황 퍽", 3);
  add("초록 퍽", 4);
  this->contextMenu_->addSeparator();
  add("가로형 신호등", 5);
  this->contextMenu_->addSeparator();
  add("수동 모델 전체 삭제", 6);
  this->deleteSelectedAction_ = this->contextMenu_->addAction("선택된 물체 삭제");
  QObject::connect(this->deleteSelectedAction_, &QAction::triggered, this,
                   [this]() { this->RemoveSelectedEntities(); });

  this->manualTimer_ = new QTimer(this);
  QObject::connect(this->manualTimer_, &QTimer::timeout, this, [this]() {
    this->TickManual();
  });
  this->manualTimer_->start(200);
}

/////////////////////////////////////////////////
std::string ManualActors::HumanSdf() const
{
  // dynamic_obstacles.py의 사람 형상과 같은 비율. 모델 원점은 몸 중심이고,
  // 발이 바닥에 닿도록 z=0.31에서 시작한다. Spawn 플러그인이 클릭 위치로 옮긴다.
  return R"(
<sdf version="1.8">
  <model name="manual_human">
    <pose>0 0 0.31 0 0 0</pose>
    <static>false</static>
    <self_collide>false</self_collide>
    <link name="link">
      <inertial>
        <mass>3.5</mass>
        <inertia><ixx>0.1</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.1</iyy><iyz>0</iyz><izz>0.03</izz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><cylinder><radius>0.075</radius><length>0.49</length></cylinder></geometry>
      </collision>
      <visual name="torso">
        <pose>0 0 0.03 0 0 0</pose>
        <geometry><cylinder><radius>0.075</radius><length>0.24</length></cylinder></geometry>
        <material><ambient>0.80 0.20 0.15 1</ambient><diffuse>0.80 0.20 0.15 1</diffuse></material>
      </visual>
      <visual name="head">
        <pose>0 0 0.225 0 0 0</pose>
        <geometry><sphere><radius>0.073</radius></sphere></geometry>
        <material><ambient>0.93 0.76 0.60 1</ambient><diffuse>0.93 0.76 0.60 1</diffuse></material>
      </visual>
      <visual name="leg_left">
        <pose>-0.055 0 -0.141 0 0 0</pose>
        <geometry><cylinder><radius>0.018</radius><length>0.318</length></cylinder></geometry>
        <material><ambient>0.12 0.15 0.24 1</ambient><diffuse>0.12 0.15 0.24 1</diffuse></material>
      </visual>
      <visual name="leg_right">
        <pose>0.055 0 -0.141 0 0 0</pose>
        <geometry><cylinder><radius>0.018</radius><length>0.318</length></cylinder></geometry>
        <material><ambient>0.12 0.15 0.24 1</ambient><diffuse>0.12 0.15 0.24 1</diffuse></material>
      </visual>
      <visual name="arm_left">
        <pose>-0.095 0 0.08 0 0 0</pose>
        <geometry><cylinder><radius>0.014</radius><length>0.22</length></cylinder></geometry>
        <material><ambient>0.80 0.20 0.15 1</ambient><diffuse>0.80 0.20 0.15 1</diffuse></material>
      </visual>
      <visual name="arm_right">
        <pose>0.095 0 0.08 0 0 0</pose>
        <geometry><cylinder><radius>0.014</radius><length>0.22</length></cylinder></geometry>
        <material><ambient>0.80 0.20 0.15 1</ambient><diffuse>0.80 0.20 0.15 1</diffuse></material>
      </visual>
      <visual name="neck">
        <pose>0 0 0.134 0 0 0</pose>
        <geometry><cylinder><radius>0.024</radius><length>0.036</length></cylinder></geometry>
        <material><ambient>0.93 0.76 0.60 1</ambient><diffuse>0.93 0.76 0.60 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>)";
}

/////////////////////////////////////////////////
std::string ManualActors::BoxSdf() const
{
  return R"(
<sdf version="1.8">
  <model name="manual_box">
    <pose>0 0 0.10 0 0 0</pose>
    <static>false</static>
    <self_collide>false</self_collide>
    <link name="link">
      <inertial>
        <mass>1.5</mass>
        <inertia><ixx>0.02</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.02</iyy><iyz>0</iyz><izz>0.02</izz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>0.25 0.25 0.20</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.25 0.25 0.20</size></box></geometry>
        <material><ambient>0.75 0.45 0.10 1</ambient><diffuse>0.75 0.45 0.10 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>)";
}

/////////////////////////////////////////////////
std::vector<std::pair<double, double>> ManualActors::SpawnPoints() const
{
  std::vector<std::pair<double, double>> points;
  std::ifstream input("/tmp/beebot_scene.json");
  if (input)
  {
    const std::string text((std::istreambuf_iterator<char>(input)),
                           std::istreambuf_iterator<char>());
    const auto begin = text.find("\"spawn_points\"");
    if (begin != std::string::npos)
    {
      const auto end = text.find(']', begin);
      const std::string block = text.substr(begin, end - begin);
      const std::regex point(R"(\{\s*\"x\"\s*:\s*([-0-9.]+)\s*,\s*\"y\"\s*:\s*([-0-9.]+)\s*\})");
      for (std::sregex_iterator it(block.begin(), block.end(), point), last;
           it != last; ++it)
      {
        points.emplace_back(std::stod((*it)[1]), std::stod((*it)[2]));
      }
    }
  }
  if (points.empty())
    points = {{0.0, 0.4}, {0.7, 0.4}, {-0.7, 0.4}};
  return points;
}

/////////////////////////////////////////////////
bool ManualActors::CreateAt(const std::string &_sdf, double _z)
{
  const auto points = this->SpawnPoints();
  const auto &point = points[this->spawnCounter++ % points.size()];
  return this->CreateAtPoint(_sdf, point.first, point.second, _z);
}

/////////////////////////////////////////////////
bool ManualActors::CreateAtPoint(const std::string &_sdf, double _x, double _y, double _z,
                                const std::string &_name)
{
  std::string sdf = _sdf;
  if (!_name.empty())
  {
    const std::string marker = "<model name=\"";
    const auto p = sdf.find(marker);
    if (p != std::string::npos)
    {
      const auto q = sdf.find('\"', p + marker.size());
      if (q != std::string::npos)
        sdf.replace(p + marker.size(), q - (p + marker.size()), _name);
    }
  }
  gz::msgs::EntityFactory request;
  request.set_sdf(sdf);
  request.set_allow_renaming(_name.empty());
  auto *pose = request.mutable_pose();
  pose->mutable_position()->set_x(_x);
  pose->mutable_position()->set_y(_y);
  pose->mutable_position()->set_z(_z);
  pose->mutable_orientation()->set_w(1.0);

  gz::msgs::Boolean reply;
  bool result = false;
  const std::string service = "/world/" + this->worldName + "/create";
  if (!this->node.Request(service, request, 1500, reply, result) || !result || !reply.data())
  {
    gzerr << "[ManualActors] create failed: " << service << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
bool ManualActors::CreateFromTemplate(const std::string &_name, double _z)
{
  const auto points = this->SpawnPoints();
  const auto &point = points[this->spawnCounter++ % points.size()];
  return this->CreateTemplateAtPoint(_name, point.first, point.second, _z);
}

/////////////////////////////////////////////////
bool ManualActors::CreateTemplateAtPoint(const std::string &_name, double _x,
                                        double _y, double _z)
{
  const char *dir = std::getenv("BEEBOT_SPAWN_DIR");
  const std::string base = dir ? dir : "/tmp/beebot_spawn_models";
  const std::string path = base + "/" + _name + ".sdf";
  {
    std::ifstream check(path);
    if (!check)
    {
      gzerr << "[ManualActors] template not found: " << path << std::endl;
      return false;
    }
  }

  gz::msgs::EntityFactory request;
  request.set_sdf_filename(path);
  request.set_allow_renaming(true);
  auto *pose = request.mutable_pose();
  pose->mutable_position()->set_x(_x);
  pose->mutable_position()->set_y(_y);
  pose->mutable_position()->set_z(_z);
  if (_name == "traffic_light")
  {
    // generate_world.py의 가로형 신호등과 같은 자세 (roll=+90°, yaw=-90°).
    pose->mutable_orientation()->set_x(0.5);
    pose->mutable_orientation()->set_y(-0.5);
    pose->mutable_orientation()->set_z(-0.5);
    pose->mutable_orientation()->set_w(0.5);
  }
  else
  {
    pose->mutable_orientation()->set_w(1.0);
  }

  gz::msgs::Boolean reply;
  bool result = false;
  const std::string service = "/world/" + this->worldName + "/create";
  if (!this->node.Request(service, request, 1500, reply, result) || !result || !reply.data())
  {
    gzerr << "[ManualActors] template spawn failed: " << path << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
void ManualActors::SpawnHuman()
{
  const auto points = this->SpawnPoints();
  if (points.empty())
    return;
  const auto &point = points[this->spawnCounter++ % points.size()];
  this->CreateManualHuman(point.first, point.second);
}

/////////////////////////////////////////////////
void ManualActors::CreateManualHuman(double _x, double _y)
{
  const std::string name = "manual_human_" + std::to_string(this->manualSeq_++);
  if (!this->CreateAtPoint(this->HumanSdf(), _x, _y, 0.31, name))
    return;
  ManualHuman human;
  human.name = name;
  human.x = _x;
  human.y = _y;
  human.z = 0.31;
  const auto points = this->SpawnPoints();
  if (!points.empty())
  {
    const auto &target = points[this->spawnCounter++ % points.size()];
    human.tx = target.first;
    human.ty = target.second;
  }
  else
  {
    human.tx = _x + 0.5;
    human.ty = _y;
  }
  this->humans_.push_back(human);
}

/////////////////////////////////////////////////
bool ManualActors::SetModelPose(const std::string &_name, double _x, double _y,
                                double _z, double _yaw)
{
  gz::msgs::Pose pose;
  pose.set_name(_name);
  pose.mutable_position()->set_x(_x);
  pose.mutable_position()->set_y(_y);
  pose.mutable_position()->set_z(_z);
  pose.mutable_orientation()->set_z(std::sin(_yaw / 2.0));
  pose.mutable_orientation()->set_w(std::cos(_yaw / 2.0));
  gz::msgs::Boolean reply;
  bool result = false;
  return this->node.Request("/world/" + this->worldName + "/set_pose", pose, 500, reply, result) &&
         result && reply.data();
}

/////////////////////////////////////////////////
void ManualActors::TickManual()
{
  const auto points = this->SpawnPoints();
  for (auto &human : this->humans_)
  {
    if (std::hypot(human.tx - human.x, human.ty - human.y) < 0.04)
    {
      if (points.empty())
        continue;
      const auto &target = points[this->spawnCounter++ % points.size()];
      human.tx = target.first;
      human.ty = target.second;
    }
    const double dx = human.tx - human.x;
    const double dy = human.ty - human.y;
    const double distance = std::hypot(dx, dy);
    if (distance < 1e-6)
      continue;
    const double step = std::min(distance, 0.12 * 0.2);  // 약 0.12 m/s
    human.x += dx / distance * step;
    human.y += dy / distance * step;
    human.yaw = std::atan2(dy, dx);
    this->SetModelPose(human.name, human.x, human.y, human.z, human.yaw);
  }
}

/////////////////////////////////////////////////
void ManualActors::SpawnBox()
{
  this->CreateAt(this->BoxSdf(), 0.10);
}

/////////////////////////////////////////////////
void ManualActors::SpawnPuckRed()
{
  this->CreateFromTemplate("puck_red", 0.031);
}

/////////////////////////////////////////////////
void ManualActors::SpawnPuckOrange()
{
  this->CreateFromTemplate("puck_orange", 0.031);
}

/////////////////////////////////////////////////
void ManualActors::SpawnPuckGreen()
{
  this->CreateFromTemplate("puck_green", 0.031);
}

/////////////////////////////////////////////////
void ManualActors::SpawnTrafficLight()
{
  this->CreateFromTemplate("traffic_light", 0.65);
}

/////////////////////////////////////////////////
void ManualActors::RemoveModel(const std::string &_name)
{
  gz::msgs::Entity entity;
  entity.set_name(_name);
  entity.set_type(gz::msgs::Entity::MODEL);
  gz::msgs::Boolean reply;
  bool result = false;
  this->node.Request("/world/" + this->worldName + "/remove", entity, 500, reply, result);
}

/////////////////////////////////////////////////
std::string ManualActors::EnvironmentModelDir() const
{
  const char *dir = std::getenv("BEEBOT_ENV_MODEL_DIR");
  return dir ? std::string(dir) : std::string("/tmp/beebot_environment_models");
}

/////////////////////////////////////////////////
bool ManualActors::CreateModelFile(const std::string &_path)
{
  std::ifstream check(_path);
  if (!check)
  {
    gzerr << "[ManualActors] environment template not found: " << _path << std::endl;
    return false;
  }
  gz::msgs::EntityFactory request;
  request.set_sdf_filename(_path);
  request.set_allow_renaming(false);
  auto *pose = request.mutable_pose();
  pose->mutable_position()->set_x(0.0);
  pose->mutable_position()->set_y(0.0);
  pose->mutable_position()->set_z(0.0);
  pose->mutable_orientation()->set_w(1.0);
  const std::string service = "/world/" + this->worldName + "/create";
  for (int attempt = 0; attempt < 4; ++attempt)
  {
    gz::msgs::Boolean reply;
    bool result = false;
    if (this->node.Request(service, request, 2000, reply, result) && result && reply.data())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }
  gzerr << "[ManualActors] environment create failed: " << _path << std::endl;
  return false;
}

/////////////////////////////////////////////////
void ManualActors::ApplyEnvironment(const QString &_floor, const QString &_wall,
                                    const QString &_background)
{
  const std::string base = this->EnvironmentModelDir();
  const std::string floor = _floor.toStdString();
  const std::string wall = _wall.toStdString();
  const std::string background = _background.toStdString();

  // 변경된 항목만 교체한다. 벽만 바꿀 때 바닥/로봇을 불필요하게 제거하지 않는다.
  if (floor != this->floorName_)
  {
    this->RemoveModel("textured_floor");
    this->RemoveModel("ground");
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    if (this->CreateModelFile(base + "/floor/" + floor + "/model.sdf"))
      this->floorName_ = floor;
  }

  if (wall != this->wallName_)
  {
    this->RemoveModel("twin_closed_maze");
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    if (this->CreateModelFile(base + "/wall/" + wall + "/model.sdf"))
      this->wallName_ = wall;
  }

  if (background != this->backgroundName_)
  {
    this->RemoveModel("panorama_background");
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    if (background == "none")
      this->backgroundName_ = "none";
    else if (this->CreateModelFile(base + "/background/" + background + "/model.sdf"))
      this->backgroundName_ = background;
  }
}

/////////////////////////////////////////////////
bool ManualActors::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::RightClickToScene::kType)
  {
    auto *event = static_cast<gz::gui::events::RightClickToScene *>(_event);
    this->ShowContextMenu(event->Point());
    return true;
  }
  if (_event->type() == gz::sim::gui::events::EntitiesSelected::kType)
  {
    auto *event = static_cast<gz::sim::gui::events::EntitiesSelected *>(_event);
    this->selectedEntities_ = event->Data();
    if (this->deleteSelectedAction_)
      this->deleteSelectedAction_->setEnabled(!this->selectedEntities_.empty());
    return false;
  }
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void ManualActors::ShowContextMenu(const gz::math::Vector3d &_point)
{
  this->contextPoint_ = _point;
  if (this->deleteSelectedAction_)
    this->deleteSelectedAction_->setEnabled(!this->selectedEntities_.empty());
  QTimer::singleShot(0, this, [this]() {
    if (this->contextMenu_)
      this->contextMenu_->exec(QCursor::pos());
  });
}

/////////////////////////////////////////////////
void ManualActors::RemoveSelectedEntities()
{
  for (const auto id : this->selectedEntities_)
  {
    gz::msgs::Entity entity;
    entity.set_id(id);
    entity.set_type(gz::msgs::Entity::MODEL);
    gz::msgs::Boolean reply;
    bool result = false;
    this->node.Request("/world/" + this->worldName + "/remove", entity, 1000, reply, result);
  }
  this->selectedEntities_.clear();
  if (this->deleteSelectedAction_)
    this->deleteSelectedAction_->setEnabled(false);
}

/////////////////////////////////////////////////
void ManualActors::RunContextAction(int _kind)
{
  const double x = this->contextPoint_.X();
  const double y = this->contextPoint_.Y();
  switch (_kind)
  {
    case 0: this->CreateManualHuman(x, y); break;
    case 1: this->CreateAtPoint(this->BoxSdf(), x, y, 0.10); break;
    case 2: this->CreateTemplateAtPoint("puck_red", x, y, 0.031); break;
    case 3: this->CreateTemplateAtPoint("puck_orange", x, y, 0.031); break;
    case 4: this->CreateTemplateAtPoint("puck_green", x, y, 0.031); break;
    case 5: this->CreateTemplateAtPoint("traffic_light", x, y, 0.65); break;
    case 6: this->ClearManualModels(); break;
    case 7: this->RemoveSelectedEntities(); break;
    default: break;
  }
}

/////////////////////////////////////////////////
void ManualActors::ClearManualModels()
{
  this->humans_.clear();
  const std::string sceneService = "/world/" + this->worldName + "/scene/info";
  const std::string removeService = "/world/" + this->worldName + "/remove";

  gz::msgs::Empty empty;
  gz::msgs::Scene scene;
  bool result = false;
  if (!this->node.Request(sceneService, empty, 1000, scene, result) || !result)
  {
    gzerr << "[ManualActors] scene service failed: " << sceneService << std::endl;
    return;
  }

  for (const auto &model : scene.model())
  {
    if (model.name().rfind("manual_", 0) != 0)
      continue;
    gz::msgs::Entity entity;
    entity.set_name(model.name());
    entity.set_type(gz::msgs::Entity::MODEL);
    gz::msgs::Boolean reply;
    bool removed = false;
    if (!this->node.Request(removeService, entity, 1000, reply, removed) || !removed)
    {
      gzerr << "[ManualActors] remove failed: " << model.name() << std::endl;
    }
  }
}

}  // namespace beebot

GZ_ADD_PLUGIN(beebot::ManualActors, gz::gui::Plugin)
