#ifndef BEEBOT_MANUAL_ACTORS_HH_
#define BEEBOT_MANUAL_ACTORS_HH_

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <QEvent>

#include <vector>

class QMenu;
class QTimer;
#include <gz/gui/Plugin.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/gui/GuiEvents.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

namespace beebot
{
/// \brief Gazebo GUI에서 수동 미션용 사람/장애물을 소환하는 버튼 패널.
class ManualActors : public gz::gui::Plugin
{
  Q_OBJECT

  /// \brief Constructor
  public: ManualActors();

  /// \brief Destructor
  public: ~ManualActors() override;

  /// \brief Plugin configuration. Sets the short panel title.
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief Right-click 3D scene context menu.
  protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

  /// \brief Spawn a human at the next free point and move it automatically.
  public: Q_INVOKABLE void SpawnHuman();

  /// \brief Spawn a box obstacle at the next free point.
  public: Q_INVOKABLE void SpawnBox();

  /// \brief Spawn a red/orange/green tagged puck.
  public: Q_INVOKABLE void SpawnPuckRed();
  public: Q_INVOKABLE void SpawnPuckOrange();
  public: Q_INVOKABLE void SpawnPuckGreen();

  /// \brief Spawn a horizontal gantry traffic light.
  public: Q_INVOKABLE void SpawnTrafficLight();

  /// \brief Replace floor/wall/background environment models immediately.
  public: Q_INVOKABLE void ApplyEnvironment(const QString &_floor,
                                             const QString &_wall,
                                             const QString &_background);

  /// \brief Remove all models whose name starts with "manual_".
  public: Q_INVOKABLE void ClearManualModels();

  /// \brief Spawn a model at the next free point from beebot_scene.json.
  private: bool CreateAt(const std::string &_sdf, double _z);

  /// \brief Create a manually tracked walking human at an explicit point.
  private: void CreateManualHuman(double _x, double _y);

  /// \brief Move tracked manual humans every timer tick.
  private: void TickManual();

  /// \brief Spawn a model at an explicit scene point.
  private: bool CreateAtPoint(const std::string &_sdf, double _x, double _y, double _z,
                              const std::string &_name = "");

  /// \brief Spawn a prepared SDF template from BEEBOT_SPAWN_DIR.
  private: bool CreateFromTemplate(const std::string &_name, double _z);

  /// \brief Spawn a prepared template at an explicit scene point.
  private: bool CreateTemplateAtPoint(const std::string &_name, double _x, double _y, double _z);

  /// \brief Set an entity pose for the walking human timer.
  private: bool SetModelPose(const std::string &_name, double _x, double _y,
                             double _z, double _yaw);

  /// \brief Show the right-click spawn menu at a scene point.
  private: void ShowContextMenu(const gz::math::Vector3d &_point);

  /// \brief Execute the selected context-menu action at the stored point.
  private: void RunContextAction(int _kind);

  /// \brief Remove the entities selected in Gazebo's entity tree/scene.
  private: void RemoveSelectedEntities();

  /// \brief Remove one model by name (missing model is ignored).
  private: void RemoveModel(const std::string &_name);

  /// \brief Create an SDF model file at the world origin.
  private: bool CreateModelFile(const std::string &_path);

  /// \brief Environment model catalog directory from BEEBOT_ENV_MODEL_DIR.
  private: std::string EnvironmentModelDir() const;

  /// \brief Read free spawn points from /tmp/beebot_scene.json.
  private: std::vector<std::pair<double, double>> SpawnPoints() const;

  /// \brief Build the humanoid SDF string.
  private: std::string HumanSdf() const;

  /// \brief Build the box SDF string.
  private: std::string BoxSdf() const;

  /// \brief World name (default: default).
  private: std::string worldName{"default"};

  /// \brief Transport node for create/remove/scene services.
  private: gz::transport::Node node;

  /// \brief Spawn point round-robin counter.
  private: std::size_t spawnCounter{0};

  /// \brief Persistent right-click menu (popup, not blocking exec).
  private: QMenu *contextMenu_{nullptr};

  /// \brief "Delete selected entity" menu action.
  private: QAction *deleteSelectedAction_{nullptr};

  /// \brief Currently selected Gazebo entities.
  private: std::vector<gz::sim::Entity> selectedEntities_;

  /// \brief Scene point captured when the context menu opened.
  private: gz::math::Vector3d contextPoint_{0, 0, 0};

  /// \brief Manually spawned humans that should keep walking.
  private: struct ManualHuman
  {
    std::string name;
    double x{0};
    double y{0};
    double z{0.31};
    double yaw{0};
    double tx{0};
    double ty{0};
  };
  private: std::vector<ManualHuman> humans_;
  private: QTimer *manualTimer_{nullptr};
  private: std::size_t manualSeq_{0};

  /// \brief Currently applied environment names (none | stem).
  private: std::string floorName_{"none"};
  private: std::string wallName_{"none"};
  private: std::string backgroundName_{"none"};
};

}  // namespace beebot

#endif  // BEEBOT_MANUAL_ACTORS_HH_
